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
-- hpsdr_hp_cmd_handler
--
-- OpenHPSDR Protocol 2 "High Priority Command" receiver (host -> radio,
-- UDP dst port 1027), modelled on the upstream Orion / Orion MkII
-- reference firmware's High_Priority_CC.v
-- (Y:\Ilkka\ham\bladerf\orion2\High_Priority_CC.v).  Consumes the
-- udp_rx_handler.hpsdr_hp_cmd_* byte stream (UDP header already
-- stripped) and latches the engagement gate (`host_run`) and PTT bit
-- (`host_ptt0`) so the periodic radio->host TX producers
-- (hpsdr_hp_status_sender, future DDC IQ / Mic) can start streaming.
--
-- HP Command payload layout (1444 bytes total, BE numerics):
--   byte 0..3   : Sequence number (BE; we don't track for now)
--   byte 4      : bit 0 = run, bit 1 = PTT0, bits 2..4 = PTT1..PTT3
--   byte 5      : bit 0 = CWX, bit 1 = Dot,  bit 2 = Dash
--   bytes 9..12 : RX0 frequency/phase (BE)
--   bytes 13..16: RX1 frequency/phase
--   ...
--   bytes 329..332: Tx0 frequency
--   byte 345    : Tx0 drive level (0..255)
--   bytes 1428..1435: Alex Tx/Rx filter data
--   bytes 1442,1443: Attenuator1, Attenuator0 (5 bits each)
--
-- This iteration handles ONLY the engagement-critical bits at byte 4
-- (run, PTT0) -- pure scaffold to unblock the HP Status heartbeat.
-- Frequency / drive / Alex / attenuator decoding is deferred until
-- there's a real DDC IQ streamer to consume the values.
--
-- Sequencing
-- ----------
-- Walk a byte counter from 0 on rx_sop; latch run/PTT0 directly when
-- the counter hits 4 (matching Orion2's "write through" approach -- no
-- temp+commit pattern, no check that the packet completes successfully).
-- A truncated HP Command with at least 5 valid bytes would still update
-- run/PTT0 -- harmless given Thetis sends the same bits at ~10 Hz, the
-- next valid packet corrects any glitch.
--
-- host_run / host_ptt0 reset to '0' on bladeRF reset and are
-- monotonically driven from the host's most recent HP Command thereafter.
-- We do NOT implement Orion2's HW_timeout that forces run=0 after a long
-- silence -- the bladeRF USB stack resets the FPGA on reconnect, which
-- restores `run='0'` cleanly enough for the bring-up phase.
--
-- hp_cmd_pulse fires once per fully-consumed HP Command (rx_eop) so the
-- LED chain / SignalTap can confirm we're seeing the host's command stream.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_hp_cmd_handler is
    port (
        clock        : in  std_logic;
        reset        : in  std_logic;

        -- HP Command byte stream input (from udp_rx_handler
        -- hpsdr_hp_cmd_* channel, UDP header already stripped).
        rx_data      : in  std_logic_vector(7 downto 0);
        rx_valid     : in  std_logic;
        rx_sop       : in  std_logic;
        rx_eop       : in  std_logic;
        rx_length    : in  std_logic_vector(13 downto 0);  -- UDP payload bytes

        -- Engagement state, latched from byte 4 of the most recent
        -- accepted HP Command and held until the next.  '0' from reset.
        host_run     : out std_logic;
        host_ptt0    : out std_logic;

        -- Observability: one cycle when the final byte of an HP Command
        -- is consumed (~10 Hz steady-state once Thetis is engaged).
        hp_cmd_pulse : out std_logic
    );
end entity;

architecture arch of hpsdr_hp_cmd_handler is

    -- 11 bits cover byte 0..1443 with margin (HP Command is 1444 bytes).
    signal byte_idx     : unsigned(10 downto 0) := (others => '0');

    signal host_run_r   : std_logic := '0';
    signal host_ptt0_r  : std_logic := '0';
    signal hp_cmd_pulse_r : std_logic := '0';

begin

    host_run     <= host_run_r;
    host_ptt0    <= host_ptt0_r;
    hp_cmd_pulse <= hp_cmd_pulse_r;

    fsm : process(clock, reset)
        variable n_byte_idx : unsigned(10 downto 0);
    begin
        if reset = '1' then
            byte_idx       <= (others => '0');
            host_run_r     <= '0';
            host_ptt0_r    <= '0';
            hp_cmd_pulse_r <= '0';
        elsif rising_edge(clock) then
            hp_cmd_pulse_r <= '0';

            if rx_valid = '1' then
                n_byte_idx := byte_idx;
                if rx_sop = '1' then
                    n_byte_idx := (others => '0');
                end if;

                -- Latch run / PTT0 at byte 4 (Orion2 High_Priority_CC.v
                -- line 175: `run <= udp_rx_data[0]; PC_PTT <= udp_rx_data[1];`).
                if n_byte_idx = to_unsigned(4, n_byte_idx'length) then
                    host_run_r  <= rx_data(0);
                    host_ptt0_r <= rx_data(1);
                end if;

                if rx_eop = '1' then
                    hp_cmd_pulse_r <= '1';
                    n_byte_idx := (others => '0');
                else
                    n_byte_idx := n_byte_idx + 1;
                end if;

                byte_idx <= n_byte_idx;
            end if;
        end if;
    end process fsm;

end architecture;
