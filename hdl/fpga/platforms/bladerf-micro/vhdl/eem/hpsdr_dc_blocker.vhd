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
-- hpsdr_dc_blocker
--
-- Classic one-pole IIR DC blocker, applied independently to I and Q.
-- Removes the DC / LO-leakage spike that direct-conversion receivers like
-- the AD9361 leave at the panadapter centre bin.
--
--   y[n] = x[n] - x[n-1] + R * y[n-1]            R = 1 - 2^-K
--
-- Pole at z=R (just inside the unit circle), zero at z=1 (perfect DC null).
-- Corner frequency f_c ~ fs / (2*pi * 2^K).  At fs = 48 kHz with K = 11
-- (R = 0.9995), f_c ~= 3.7 Hz -- inaudible / invisible for any HPSDR use,
-- and well below the smallest CW filter bandwidth.
--
-- Implementation notes
--   * State is held at WIDTH+K+1 bits so the "leak" term (y_state >> K)
--     never truncates to zero for sub-LSB DC residuals.  This is the
--     same trick as the canonical fixed-point DC blocker in DSP texts.
--   * R = 1 - 2^-K is enforced by the shift, so no multiplier is inferred
--     -- the whole block is adders + shifters (~150 ALMs per channel @
--     WIDTH=24, K=11).
--   * Settling time ~ 5 / (1-R) samples ~ 5 * 2^K.  At K=11 / 48 kSPS that
--     is ~10000 samples ~= 0.2 s.  We deliberately do NOT reset on
--     hpsdr_host_run changes so the IIR stays converged between connects
--     and the first IQ packet after engagement is already DC-clean.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_dc_blocker is
    generic (
        -- Sample width (bits) for I and Q.  hpsdr_ddc emits signed 24.
        WIDTH : positive := 24;
        -- Pole at R = 1 - 2^-K.  Larger K -> lower corner, longer settle.
        -- K=11 at 48 kHz -> ~3.7 Hz corner, ~0.2 s settle.
        K     : positive := 11
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

architecture rtl of hpsdr_dc_blocker is

    -- Internal state width: WIDTH bits of input range, K bits of fractional
    -- precision below the LSB so the leak term resolves sub-LSB DC, plus
    -- one guard bit for the transient at the start of a step.
    constant STATE_W : integer := WIDTH + K + 1;

    signal x_prev_i   : signed(WIDTH-1 downto 0)   := (others => '0');
    signal x_prev_q   : signed(WIDTH-1 downto 0)   := (others => '0');
    signal y_state_i  : signed(STATE_W-1 downto 0) := (others => '0');
    signal y_state_q  : signed(STATE_W-1 downto 0) := (others => '0');

    signal out_i_r    : signed(WIDTH-1 downto 0)   := (others => '0');
    signal out_q_r    : signed(WIDTH-1 downto 0)   := (others => '0');
    signal out_valid_r: std_logic                  := '0';

begin

    out_i     <= out_i_r;
    out_q     <= out_q_r;
    out_valid <= out_valid_r;

    process(clock, reset)
        variable in_scaled_i, in_scaled_q     : signed(STATE_W-1 downto 0);
        variable prev_scaled_i, prev_scaled_q : signed(STATE_W-1 downto 0);
        variable diff_i, diff_q               : signed(STATE_W-1 downto 0);
        variable leak_i, leak_q               : signed(STATE_W-1 downto 0);
        variable next_y_i, next_y_q           : signed(STATE_W-1 downto 0);
    begin
        if reset = '1' then
            x_prev_i    <= (others => '0');
            x_prev_q    <= (others => '0');
            y_state_i   <= (others => '0');
            y_state_q   <= (others => '0');
            out_i_r     <= (others => '0');
            out_q_r     <= (others => '0');
            out_valid_r <= '0';
        elsif rising_edge(clock) then
            out_valid_r <= '0';

            if in_valid = '1' then
                -- Scale x up by 2^K so the recursive leak term has K bits
                -- of fractional headroom and resolves residual DC down to
                -- well under 1 LSB at the output.
                in_scaled_i   := shift_left(resize(in_i,    STATE_W), K);
                in_scaled_q   := shift_left(resize(in_q,    STATE_W), K);
                prev_scaled_i := shift_left(resize(x_prev_i, STATE_W), K);
                prev_scaled_q := shift_left(resize(x_prev_q, STATE_W), K);

                diff_i        := in_scaled_i - prev_scaled_i;
                diff_q        := in_scaled_q - prev_scaled_q;

                -- R * y[n-1] == y[n-1] - (y[n-1] >> K); cheap, no DSP.
                leak_i        := shift_right(y_state_i, K);
                leak_q        := shift_right(y_state_q, K);

                next_y_i      := (y_state_i - leak_i) + diff_i;
                next_y_q      := (y_state_q - leak_q) + diff_q;

                y_state_i     <= next_y_i;
                y_state_q     <= next_y_q;

                x_prev_i      <= in_i;
                x_prev_q      <= in_q;

                -- Output: the integer-scale slice of the state.  |y| stays
                -- bounded by |x| (HP filter gain <= 1 across the band), so
                -- a straight slice without saturation is safe.
                out_i_r       <= next_y_i(K + WIDTH - 1 downto K);
                out_q_r       <= next_y_q(K + WIDTH - 1 downto K);
                out_valid_r   <= '1';
            end if;
        end if;
    end process;

end architecture;
