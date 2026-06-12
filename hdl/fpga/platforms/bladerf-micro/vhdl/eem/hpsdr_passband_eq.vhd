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
-- hpsdr_passband_eq
--
-- AD9361 RX passband-rolloff equalizer.  A short symmetric (linear-phase) FIR
-- applied to the complex baseband at the AD9361 NATIVE rate (12.288 MSPS),
-- BEFORE the Fs/4 mixer.
--
-- Why this exists
--   Fs/4 offset tuning (hpsdr_fs4_mixer) buries the zero-IF DC spike, but it
--   makes the panadapter view an OFF-CENTRE slice of the AD9361 baseband
--   (~2.3..3.8 MHz from the chip LO on the widest 1536 kHz span).  The AD9361's
--   own analog+digital passband rolls off across that slice, so the noise floor
--   tilts ~12 dB high-to-low on the widest span (narrow spans stay flat -- the
--   slice is tiny).  This FIR flattens that tilt.
--
--   It runs at the native rate (before the mixer) on purpose: the AD9361
--   response is symmetric about the chip LO = native DC, so the inverse is a
--   plain REAL symmetric FIR (after the mixer it would be asymmetric about the
--   recentred DC and need complex taps).  Being in absolute Hz at the fixed
--   native rate, one filter corrects every HPSDR rate at once.
--
-- Coefficient design (15-tap symmetric, attenuation-only)
--   The AD9361 rolloff was measured off the panadapter noise floor (1536 kHz
--   span), the known CIC*cFIR shape removed, and the inverse fit by weighted
--   least-squares over the viewed band f_ad in [2.30, 3.84] MHz.  The target is
--   normalised so the filter only ever ATTENUATES (peak |H| ~= 1.03, ~0 dB):
--   it pulls the higher (LO-side) part of the slice DOWN to the level of the
--   far side rather than boosting -- no noise/alias amplification, no extra
--   saturation.  Residual floor flatness after correction: ~0.7 dB across the
--   widest span (down from ~12 dB).  Coefficients Q1.16 (scale 2^16); largest
--   |coef| = 0.668 (centre tap).  Generated offline (LS fit, host gcc).
--
--   ORIENTATION (HW-determined): the conjugating I/Q swap sits between this EQ
--   and the panadapter, so the EQ's frequency axis is MIRRORED vs the display:
--   native f maps to display = 3072 - f (kHz), NOT display + 3072.  The first
--   build used the un-mirrored mapping and steepened the tilt; this table uses
--   f = 3072 - display, so the EQ attenuates HIGH native f (a lowpass shape) =
--   the display LO-side that was too high.  (A symmetric FIR can't tell +f from
--   -f, but the attenuation lands on the wrong display side if the axis sense
--   is wrong -- hence the mirror.)
--
--   The fit is from one hardware measurement; if the floor still tilts, re-read
--   the 1536 kHz noise floor and regenerate the table -- wiring is unchanged.
--
-- Implementation
--   * Folded symmetric FIR: mirror taps share a coefficient, pre-added then one
--     multiply per pair -> HALF_TAPS+1 = 8 multiplies/channel (8-long MAC
--     cascade, within the device's 11 limit), 16 DSP for I/Q.
--   * Output saturates (sat_width): the ~1.03x peak can nudge a full-scale
--     sample just past the WIDTH range; clip rather than wrap.
--   * Latency: 1 clock from in_valid -> out_valid.  Reset is rx_reset only.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_passband_eq is
    generic (
        WIDTH : positive := 16
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

architecture rtl of hpsdr_passband_eq is

    constant N_TAPS    : positive := 15;
    constant COEF_W    : positive := 18;
    constant FRAC_BITS : positive := 16;   -- Q1.16
    constant HALF_TAPS : natural  := (N_TAPS - 1) / 2;   -- = 7

    type coef_array_t is array (0 to N_TAPS-1) of signed(COEF_W-1 downto 0);
    constant COEFS : coef_array_t := (
         0 => to_signed(  -1053, COEF_W),
         1 => to_signed(   -607, COEF_W),
         2 => to_signed(   2065, COEF_W),
         3 => to_signed(    724, COEF_W),
         4 => to_signed(  -4201, COEF_W),
         5 => to_signed(   -819, COEF_W),
         6 => to_signed(  14007, COEF_W),
         7 => to_signed(  44090, COEF_W),
         8 => to_signed(  14007, COEF_W),
         9 => to_signed(   -819, COEF_W),
        10 => to_signed(  -4201, COEF_W),
        11 => to_signed(    724, COEF_W),
        12 => to_signed(   2065, COEF_W),
        13 => to_signed(   -607, COEF_W),
        14 => to_signed(  -1053, COEF_W)
    );

    type sample_array_t is array (0 to N_TAPS-2) of signed(WIDTH-1 downto 0);
    signal sr_i : sample_array_t := (others => (others => '0'));
    signal sr_q : sample_array_t := (others => (others => '0'));

    signal out_i_r     : signed(WIDTH-1 downto 0) := (others => '0');
    signal out_q_r     : signed(WIDTH-1 downto 0) := (others => '0');
    signal out_valid_r : std_logic                := '0';

    -- Pre-add of two WIDTH-bit samples is WIDTH+1 bits; product = (WIDTH+1) +
    -- COEF_W.  8-way folded sum needs ceil(log2(8)) = 3 guard bits.
    constant PROD_W : positive := (WIDTH + 1) + COEF_W;
    constant ACC_W  : positive := PROD_W + 3;

    -- Saturating cast to WIDTH-bit signed (peak gain ~1.03 can just exceed
    -- range; clip instead of wrap).
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
        type window_t is array (0 to N_TAPS-1) of signed(WIDTH-1 downto 0);
        variable wi, wq         : window_t;
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
                wi(0) := in_i;
                wq(0) := in_q;
                for k in 1 to N_TAPS-1 loop
                    wi(k) := sr_i(k-1);
                    wq(k) := sr_q(k-1);
                end loop;

                acc_i := (others => '0');
                acc_q := (others => '0');

                -- Folded symmetric pairs.
                for j in 0 to HALF_TAPS-1 loop
                    prod_i := (resize(wi(j), WIDTH+1) + resize(wi(N_TAPS-1-j), WIDTH+1)) * COEFS(j);
                    prod_q := (resize(wq(j), WIDTH+1) + resize(wq(N_TAPS-1-j), WIDTH+1)) * COEFS(j);
                    acc_i  := acc_i + resize(prod_i, ACC_W);
                    acc_q  := acc_q + resize(prod_q, ACC_W);
                end loop;

                -- Centre tap.
                prod_i := resize(wi(HALF_TAPS), WIDTH+1) * COEFS(HALF_TAPS);
                prod_q := resize(wq(HALF_TAPS), WIDTH+1) * COEFS(HALF_TAPS);
                acc_i  := acc_i + resize(prod_i, ACC_W);
                acc_q  := acc_q + resize(prod_q, ACC_W);

                -- Advance the delay line.
                sr_i(0) <= in_i;
                sr_q(0) <= in_q;
                for k in 1 to N_TAPS-2 loop
                    sr_i(k) <= sr_i(k-1);
                    sr_q(k) <= sr_q(k-1);
                end loop;

                out_i_r     <= sat_width(shift_right(acc_i, FRAC_BITS));
                out_q_r     <= sat_width(shift_right(acc_q, FRAC_BITS));
                out_valid_r <= '1';
            end if;
        end if;
    end process;

end architecture;
