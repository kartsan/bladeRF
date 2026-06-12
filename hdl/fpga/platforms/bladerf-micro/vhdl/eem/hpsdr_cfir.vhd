-- Copyright (c) 2026 Nuand LLC
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.

-- =============================================================================
-- hpsdr_cfir
--
-- Sinc-compensation FIR ("cFIR") that flattens the 5-stage CIC's sinc^5
-- passband droop.  Applied independently to I and Q at the DDC output rate.
--
-- Why this exists
--   A 5-stage CIC has frequency response |H_CIC(u)| = sinc^5(u) where
--   u = f / fs_out.  Filling the panadapter to the band edge the droop is a
--   ~10..20 dB centre-vs-edge "smile" on the noise floor.  This filter is the
--   inverse of that shape, so CIC * cFIR is flat across the displayed span.
--
-- Coefficient design (15-tap symmetric, least-squares)
--   The earlier 5-tap set was a Taylor (maximally-flat-at-DC) match: flat to
--   ~+/-0.3 of the band but ~10 dB short at the edges, leaving a centre hump
--   on a full-span panadapter.  This 15-tap set is a weighted least-squares
--   fit of the FIR magnitude to 1/sinc^5(u) over |u| <= 0.46, DC gain pinned
--   to exactly 1.000 (sum of taps = 16384).  Result (CIC * cFIR):
--       passband ripple ~= 0.42 dB out to u = 0.46 (92% of Nyquist)
--       peak |H| ~= 7.1x (+17 dB) at the band edge
--   The edge lift IS the flattening (the CIC drops the edge ~17 dB, the cFIR
--   puts it back).  Coefficients were generated offline; see the project
--   history for the design script (LS + DC-gain KKT constraint).  To re-flatten
--   for a different passband or stage count, regenerate the table and the
--   N_TAPS / ACC_W guard below -- the surrounding wiring is unchanged.
--
-- Implementation
--   * Coefficients held as 18-bit signed Q3.14 (1 sign + 3 integer + 14
--     fractional, range +/- 8).  Largest |coef| = 2.546 (centre tap) -> fits.
--   * Per-channel: N_TAPS-1 sample-delay registers, N_TAPS 24x18 multipliers,
--     adder tree, single arithmetic right shift by 14.  15 taps x I/Q = 30
--     multipliers -- watch DSP-block usage in the fitter (fold the symmetric
--     pairs into pre-adders to halve it if the device runs short).
--   * Combinational FIR sum + adder tree, one rx_clock at 12.288 MHz (81 ns
--     period -- very relaxed; pipeline only if timing ever fails).
--   * Latency: 1 rx_clock from in_valid -> out_valid.
--   * Reset is rx_reset only (NOT ddc_fifo_aclr / host_run), so the SR
--     stays warm between Thetis reconnects.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_cfir is
    generic (
        WIDTH : positive := 24
    );
    port (
        clock     : in  std_logic;
        reset     : in  std_logic;

        in_i      : in  signed(WIDTH-1 downto 0);
        in_q      : in  signed(WIDTH-1 downto 0);
        in_valid  : in  std_logic;

        out_i     : out signed(WIDTH-1 downto 0);
        out_q     : out signed(WIDTH-1 downto 0);
        out_valid : out std_logic
    );
end entity;

architecture rtl of hpsdr_cfir is

    -- 5-tap symmetric, Q3.14 fixed-point.  See header comment for design.
    -- Values: [0.0505, -0.410, 1.719, -0.410, 0.0505] * 2^14, rounded.
    constant N_TAPS         : positive := 15;
    constant COEF_W         : positive := 18;
    constant COEF_FRAC_BITS : positive := 14;

    -- 15-tap symmetric inverse-sinc^5 compensator, Q3.14 (scale = 2^14).
    -- Designed by weighted least-squares fit of the FIR magnitude to
    -- 1/sinc^5(u) over the passband |u| <= 0.46 (u = f/Fs_out), with the DC
    -- gain constrained to exactly 1.000 (sum of taps = 16384).  Combined with
    -- the CIC this is flat to within ~0.42 dB out to 92% of the output Nyquist
    -- (vs the old 5-tap set, which under-compensated ~10 dB at the band edges
    -- and left the panadapter floor humped at centre).  Peak |H| ~= 7.1x
    -- (+17 dB) at the band edge -- that edge lift is what flattens the noise
    -- floor; strong signals right at the edge clip via sat_width (benign).
    -- Centre tap 41722/16384 = 2.546 fits the 3 integer bits of Q3.14.
    type coef_array_t is array (0 to N_TAPS-1) of signed(COEF_W-1 downto 0);
    constant COEFS : coef_array_t := (
         0 => to_signed(  -371, COEF_W),
         1 => to_signed(   798, COEF_W),
         2 => to_signed( -1429, COEF_W),
         3 => to_signed(  2612, COEF_W),
         4 => to_signed( -4699, COEF_W),
         5 => to_signed(  8993, COEF_W),
         6 => to_signed(-18573, COEF_W),
         7 => to_signed( 41722, COEF_W),
         8 => to_signed(-18573, COEF_W),
         9 => to_signed(  8993, COEF_W),
        10 => to_signed( -4699, COEF_W),
        11 => to_signed(  2612, COEF_W),
        12 => to_signed( -1429, COEF_W),
        13 => to_signed(   798, COEF_W),
        14 => to_signed(  -371, COEF_W)
    );

    -- Sample delay line.  sr(k) holds the sample from k cycles back; the
    -- current input is convolved as tap 0 combinationally below.
    type sample_array_t is array (0 to N_TAPS-2) of signed(WIDTH-1 downto 0);
    signal sr_i : sample_array_t := (others => (others => '0'));
    signal sr_q : sample_array_t := (others => (others => '0'));

    -- Output registers.
    signal out_i_r     : signed(WIDTH-1 downto 0) := (others => '0');
    signal out_q_r     : signed(WIDTH-1 downto 0) := (others => '0');
    signal out_valid_r : std_logic                := '0';

    -- Symmetric (folded) implementation: the FIR is linear-phase, so mirror
    -- taps share a coefficient.  Pre-add each mirror pair, then one multiply per
    -- pair -> HALF_TAPS+1 = 8 multiplies/channel instead of 15.  This both
    -- halves the DSP-block count and, crucially, keeps the inferred DSP MAC
    -- cascade at 8 (the device caps chains at 11; an unfolded 15-tap linear
    -- accumulate forms a length-15 chain and fails to fit).
    constant HALF_TAPS : natural := (N_TAPS - 1) / 2;   -- = 7 for N_TAPS=15

    -- Product/accumulator widths.  Pre-add of two WIDTH-bit samples is WIDTH+1
    -- bits; product = (WIDTH+1) + COEF_W = 25 + 18 = 43.  The 8-way sum needs
    -- ceil(log2(8)) = 3 guard bits -> 46-bit accumulator.
    constant PROD_W : positive := (WIDTH + 1) + COEF_W;
    constant ACC_W  : positive := PROD_W + 3;

    -- Saturating cast to WIDTH-bit signed.  The cFIR's ~2.64x peak gain can
    -- push a near-full-scale CIC sample past the 24-bit output range; a plain
    -- resize() would two's-complement WRAP, turning a strong band-edge signal
    -- into a violent sign-flipped spike (which also defeats a downstream DC
    -- blocker).  Clip to full-scale instead -- benign vs a wrap.
    function sat_width(v : signed) return signed is
        constant HI : integer := 2**(WIDTH-1) - 1;
        constant LO : integer := -(2**(WIDTH-1));
    begin
        if    v > to_signed(HI, v'length) then
            return to_signed(HI, WIDTH);
        elsif v < to_signed(LO, v'length) then
            return to_signed(LO, WIDTH);
        else
            return resize(v, WIDTH);
        end if;
    end function;

begin

    out_i     <= out_i_r;
    out_q     <= out_q_r;
    out_valid <= out_valid_r;

    process(clock, reset)
        -- Window of the N_TAPS samples convolved this cycle: w(0) = current
        -- input, w(k) = sr(k-1) = sample from k cycles back.
        type window_t is array (0 to N_TAPS-1) of signed(WIDTH-1 downto 0);
        variable wi, wq         : window_t;
        variable pre_i, pre_q   : signed(WIDTH downto 0);      -- WIDTH+1 bits
        variable prod_i, prod_q : signed(PROD_W-1 downto 0);
        variable acc_i, acc_q   : signed(ACC_W-1 downto 0);
    begin
        if reset = '1' then
            sr_i        <= (others => (others => '0'));
            sr_q        <= (others => (others => '0'));
            out_i_r     <= (others => '0');
            out_q_r     <= (others => '0');
            out_valid_r <= '0';
        elsif rising_edge(clock) then
            out_valid_r <= '0';

            if in_valid = '1' then
                -- Assemble the sample window: tap 0 is the current input, the
                -- rest come from the delay line.
                wi(0) := in_i;
                wq(0) := in_q;
                for k in 1 to N_TAPS-1 loop
                    wi(k) := sr_i(k-1);
                    wq(k) := sr_q(k-1);
                end loop;

                acc_i := (others => '0');
                acc_q := (others => '0');

                -- Folded symmetric pairs: w(j) and w(N-1-j) share COEFS(j).
                -- Pre-add the pair, then one multiply -> 8 mul/channel, and an
                -- 8-long MAC cascade (within the device's 11 limit).
                for j in 0 to HALF_TAPS-1 loop
                    pre_i  := resize(wi(j), WIDTH+1) + resize(wi(N_TAPS-1-j), WIDTH+1);
                    pre_q  := resize(wq(j), WIDTH+1) + resize(wq(N_TAPS-1-j), WIDTH+1);
                    prod_i := pre_i * COEFS(j);
                    prod_q := pre_q * COEFS(j);
                    acc_i  := acc_i + resize(prod_i, ACC_W);
                    acc_q  := acc_q + resize(prod_q, ACC_W);
                end loop;

                -- Centre tap (no mirror): COEFS(HALF_TAPS) * w(HALF_TAPS).
                prod_i := resize(wi(HALF_TAPS), WIDTH+1) * COEFS(HALF_TAPS);
                prod_q := resize(wq(HALF_TAPS), WIDTH+1) * COEFS(HALF_TAPS);
                acc_i  := acc_i + resize(prod_i, ACC_W);
                acc_q  := acc_q + resize(prod_q, ACC_W);

                -- Advance the delay line: sr(0) <= in, sr(k) <= sr(k-1).
                sr_i(0) <= in_i;
                sr_q(0) <= in_q;
                for k in 1 to N_TAPS-2 loop
                    sr_i(k) <= sr_i(k-1);
                    sr_q(k) <= sr_q(k-1);
                end loop;

                -- Drop the Q3.14 fractional bits to return to input scale,
                -- then saturate: the ~7.1x peak gain on a full-scale CIC sample
                -- can exceed 24-bit range, and we run the CIC at full scale
                -- (shift_left(...,4) on a full 16-bit input), so a plain
                -- resize() could wrap.  sat_width() clips instead.
                out_i_r     <= sat_width(shift_right(acc_i, COEF_FRAC_BITS));
                out_q_r     <= sat_width(shift_right(acc_q, COEF_FRAC_BITS));
                out_valid_r <= '1';
            end if;
        end if;
    end process;

end architecture;
