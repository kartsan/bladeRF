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
-- hpsdr_fs4_mixer
--
-- Trivial complex frequency shift by exactly Fs/4 (a quarter of the sample
-- rate it is clocked at -- here the AD9361 native 12.288 MSPS, so +/-3.072 MHz).
--
-- Why this exists -- offset tuning to kill the zero-IF centre spike
--   This radio tunes the AD9361 analog LO directly onto the VFO, so the LO
--   self-mixing / leakage DC spike sits exactly on the tuned signal at the
--   panadapter centre.  It is NOT a static DC offset (the AD9361's own BB/RF
--   DC tracking can't remove it -- confirmed on HW), it is the LO leakage with
--   close-in drift skirts.  The fix is the classic offset-tuning trick used by
--   real HPSDR hardware: the NIOS parks the RX LO a fixed Fs/4 ABOVE the VFO,
--   so the wanted signal lands at -Fs/4 in the complex baseband while the DC
--   spike stays at 0.  This block then shifts everything UP by Fs/4: the signal
--   moves to centre (0 Hz) and the spike moves to +Fs/4.
--   +Fs/4 = Fs_native/4 is an exact transmission ZERO of the downstream CIC
--   (sin(pi*R/4) = 0 for every HPSDR decimation R, all multiples of 4), so the
--   spike is annihilated by the CIC rather than merely relocated -- no notch,
--   no signal loss.
--
-- The multiply is free: e^{+/-j*pi*n/2} cycles through {1, +/-j, -1, -/+j}, so
-- each output is just one of the inputs, possibly swapped I<->Q and/or negated.
-- A 2-bit phase counter (one step per in_valid) selects the case.  No CORDIC,
-- no multiplier, no LUT.
--
--   MIX_UP = true   ->  multiply by e^{+j*pi*n/2} (shift UP by Fs/4)
--   MIX_UP = false  ->  multiply by e^{-j*pi*n/2} (shift DOWN by Fs/4)
--
-- If the signal lands off-centre on the panadapter (at +/-Fs/4 instead of 0)
-- the baseband inversion sense is opposite to what is assumed: flip MIX_UP
-- here and/or negate the NIOS LO offset (HPSDR_RX_FS4_OFFSET_HZ).  Either flip
-- re-centres it; the spike stays out of band either way.
--
-- Latency: 1 clock from in_valid -> out_valid.  Reset is the rx domain reset.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_fs4_mixer is
    generic (
        WIDTH  : positive := 16;
        -- true: shift UP by Fs/4 (e^{+j*pi*n/2}); false: shift DOWN.
        MIX_UP : boolean  := true
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

architecture rtl of hpsdr_fs4_mixer is

    signal phase   : unsigned(1 downto 0)      := (others => '0');
    signal out_i_r : signed(WIDTH-1 downto 0)  := (others => '0');
    signal out_q_r : signed(WIDTH-1 downto 0)  := (others => '0');
    signal out_v_r : std_logic                 := '0';

    -- Saturating negate.  Negating the most-negative value (-2^(W-1)) overflows
    -- the W-bit range; clamp it to the most-positive (+2^(W-1)-1) instead of
    -- wrapping (a wrap would put a full-scale sign-flip spike into the CIC).
    -- The only overflow case is x = most-negative, where -x wraps back to a
    -- still-negative value (MSB stays '1'); every other negation clears the MSB.
    function neg_sat(x : signed) return signed is
        variable nx : signed(x'length-1 downto 0);
    begin
        nx := -x;
        if x(x'high) = '1' and nx(nx'high) = '1' then
            nx := (others => '1');   -- 0111..1 = +2^(W-1)-1
            nx(nx'high) := '0';
        end if;
        return nx;
    end function;

begin

    out_i     <= out_i_r;
    out_q     <= out_q_r;
    out_valid <= out_v_r;

    process(clock, reset)
    begin
        if reset = '1' then
            phase   <= (others => '0');
            out_i_r <= (others => '0');
            out_q_r <= (others => '0');
            out_v_r <= '0';
        elsif rising_edge(clock) then
            out_v_r <= '0';

            if in_valid = '1' then
                if MIX_UP then
                    -- multiply (I + jQ) by e^{+j*pi*n/2} = {1, j, -1, -j}
                    case phase is
                        when "00"   => out_i_r <= in_i;          out_q_r <= in_q;
                        when "01"   => out_i_r <= neg_sat(in_q); out_q_r <= in_i;
                        when "10"   => out_i_r <= neg_sat(in_i); out_q_r <= neg_sat(in_q);
                        when others => out_i_r <= in_q;          out_q_r <= neg_sat(in_i);
                    end case;
                else
                    -- multiply (I + jQ) by e^{-j*pi*n/2} = {1, -j, -1, j}
                    case phase is
                        when "00"   => out_i_r <= in_i;          out_q_r <= in_q;
                        when "01"   => out_i_r <= in_q;          out_q_r <= neg_sat(in_i);
                        when "10"   => out_i_r <= neg_sat(in_i); out_q_r <= neg_sat(in_q);
                        when others => out_i_r <= neg_sat(in_q); out_q_r <= in_i;
                    end case;
                end if;

                phase   <= phase + 1;
                out_v_r <= '1';
            end if;
        end if;
    end process;

end architecture;
