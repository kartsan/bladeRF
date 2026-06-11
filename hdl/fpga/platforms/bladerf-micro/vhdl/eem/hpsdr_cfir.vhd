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
--   u = f / fs_out.  Across a panadapter window that fills ~80% of the
--   output Nyquist the droop is visible as a ~10 dB centre-vs-edge "smile"
--   on the noise floor.  This filter is the inverse of that shape.
--
-- Coefficient design (5-tap symmetric, fully parallel)
--   The frequency response of a 5-tap symmetric FIR is
--       H(omega) = a + 2 b cos(omega) + 2 c cos(2 omega)
--   Expand to Taylor order omega^4 and match the leading terms of
--   1 / sinc^5(omega/2pi):
--       1/sinc^5(u) ~= 1 + 0.208 omega^2 + 0.0332 omega^4 + ...
--   The match gives a = 1.719, b = -0.410, c = 0.0505.  DC gain is
--   exactly 1.000.  Passband lift:
--       u = 0.0  ->  0.00 dB    (centre, perfect)
--       u = 0.2  -> +2.83 dB    (target: +2.93 dB)
--       u = 0.3  -> +6.03 dB    (target: +6.63 dB)
--       u = 0.4  -> +7.66 dB    (target: +12.1 dB) -- under-compensates
--       u = 0.5  -> +8.43 dB    (Nyquist; doesn't matter, CIC kills it -19 dB)
--   Net visible-flatness: <1 dB across u = +/- 0.3 of the band; degrades
--   gracefully beyond that.  If you ever want true flatness all the way
--   to the edges, swap in a longer (21-tap) frequency-sampled set without
--   touching the surrounding wiring.
--
-- Implementation
--   * Coefficients held as 18-bit signed Q3.14 (1 sign + 3 integer + 14
--     fractional, range +/- 4).  The largest |coef| is 1.719 -> fits.
--   * Per-channel: 5 sample-delay registers, 5 parallel 24x18 multipliers,
--     5-input adder tree, single arithmetic right shift by 14.
--   * Combinational FIR sum: ~5 multipliers + adder tree, fully within
--     one rx_clock at 12.288 MHz (worst-case path << 80 ns).
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
    constant N_TAPS         : positive := 5;
    constant COEF_W         : positive := 18;
    constant COEF_FRAC_BITS : positive := 14;

    type coef_array_t is array (0 to N_TAPS-1) of signed(COEF_W-1 downto 0);
    constant COEFS : coef_array_t := (
        0 => to_signed(   827, COEF_W),  --  0.0505
        1 => to_signed( -6717, COEF_W),  -- -0.410
        2 => to_signed( 28164, COEF_W),  --  1.719
        3 => to_signed( -6717, COEF_W),  -- -0.410
        4 => to_signed(   827, COEF_W)   --  0.0505
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

    -- Product/accumulator widths.  Product: WIDTH + COEF_W = 24 + 18 = 42.
    -- 5-way sum needs ceil(log2(5)) = 3 guard bits -> 45-bit accumulator.
    constant PROD_W : positive := WIDTH + COEF_W;
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
        variable acc_i, acc_q   : signed(ACC_W-1 downto 0);
        variable prod_i, prod_q : signed(PROD_W-1 downto 0);
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
                acc_i := (others => '0');
                acc_q := (others => '0');

                -- Tap 0: current input.
                prod_i := in_i * COEFS(0);
                prod_q := in_q * COEFS(0);
                acc_i  := acc_i + resize(prod_i, ACC_W);
                acc_q  := acc_q + resize(prod_q, ACC_W);

                -- Taps 1..N_TAPS-1: delayed samples from the shift register.
                for k in 1 to N_TAPS-1 loop
                    prod_i := sr_i(k-1) * COEFS(k);
                    prod_q := sr_q(k-1) * COEFS(k);
                    acc_i  := acc_i + resize(prod_i, ACC_W);
                    acc_q  := acc_q + resize(prod_q, ACC_W);
                end loop;

                -- Advance the delay line: sr(0) <= in, sr(k) <= sr(k-1).
                sr_i(0) <= in_i;
                sr_q(0) <= in_q;
                for k in 1 to N_TAPS-2 loop
                    sr_i(k) <= sr_i(k-1);
                    sr_q(k) <= sr_q(k-1);
                end loop;

                -- Drop the Q3.14 fractional bits to return to input scale,
                -- then saturate: the ~2.64x peak gain on a full-scale CIC
                -- sample can exceed 24-bit range, and we run the CIC at full
                -- scale (shift_left(...,4) on a full 16-bit input), so the
                -- old plain resize() could wrap.  sat_width() clips instead.
                out_i_r     <= sat_width(shift_right(acc_i, COEF_FRAC_BITS));
                out_q_r     <= sat_width(shift_right(acc_q, COEF_FRAC_BITS));
                out_valid_r <= '1';
            end if;
        end if;
    end process;

end architecture;
