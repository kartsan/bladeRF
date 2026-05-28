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
-- stripped) and latches:
--
--   * `host_run`   (byte 4 bit 0) - engagement gate for every radio->host
--                                   TX producer per Orion2 sdr_send.v:195
--   * `host_ptt0`  (byte 4 bit 1) - PTT0; reserved for future Tx-path use
--   * `host_port`  (UDP source port of the HP Command itself) - the dst
--                                   port subsequent HP Status replies must
--                                   target.  Thetis binds its receive
--                                   socket to whatever ephemeral it used
--                                   for sendto() of the HP Command, NOT to
--                                   the discovery probe's ephemeral nor to
--                                   1025.  Confirmed by tcpdump of
--                                   hpsdr_sim engagement:
--                                       pelto.1025 > kissa.50138
--                                   where 50138 is Thetis's HP Command
--                                   source port, distinct from the
--                                   discovery probe's source (e.g. 51411).
--                                   See [[feedback_hpsdr_reply_udp_dst_port]]
--                                   for the analogous discovery rule.
--   * `host_drive_level` (byte 345) - 0..255 drive level, echoed in HP
--                                     Status payload byte 7 to satisfy
--                                     Thetis's "echo my drive back to me"
--                                     check.
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
-- This iteration handles the engagement-critical bits (byte 4 run/PTT0,
-- byte 345 drive_level) and the addressing required for HP Status to land
-- in the host's listening socket.  Frequency / Alex / attenuator decoding
-- is deferred until there's a real DDC IQ streamer to consume them.
--
-- Sequencing
-- ----------
-- Walk a byte counter from 0 on rx_sop; latch each interesting field
-- directly the cycle the counter hits its index (matching Orion2's
-- "write through" approach -- no temp+commit pattern, no check that the
-- packet completes successfully).  A truncated HP Command with at least
-- 5 valid bytes would still update run/PTT0 -- harmless given Thetis
-- sends the same bits at ~10 Hz, the next valid packet corrects any
-- glitch.
--
-- `udp_src_port` is latched the cycle rx_sop=1 (the udp_rx_handler holds
-- src_port stable from header parse onwards) so the very first HP Status
-- packet emitted after engagement targets the correct host ephemeral.
--
-- host_run / host_ptt0 / host_port / host_drive_level reset on bladeRF
-- reset and are driven from the host's most recent HP Command thereafter.
--
-- Watchdog
-- --------
-- Thetis silently disconnects (stops sending HP Commands and General
-- Packets) on power-off / app close / network drop, but doesn't signal
-- the radio.  Without a timeout the radio would keep streaming forever,
-- saturating USB and the EEM pipe.  We implement Orion2's HW_timeout
-- behaviour (High_Priority_CC.v `HW_timeout_cnt` -> `run`-clear) here:
--
--   * `watchdog_r` counts down from WATCHDOG_CYCLES.
--   * Every accepted HP Command (rx_eop) resets it to WATCHDOG_CYCLES.
--   * When it reaches 0, host_run / host_ptt0 are forced to '0'.
--
-- WATCHDOG_CYCLES = 200 ms at 100 MHz fx3_pclk_pll.  Thetis's HP Command
-- rate is ~30 Hz (~33 ms inter-packet), so 200 ms tolerates 5 missed
-- packets before declaring the host gone.  When the host comes back the
-- first HP Command with run=1 resumes streaming cleanly (host_valid from
-- the discovery responder stays latched throughout, so no re-discovery
-- is required).
--
-- hp_cmd_pulse fires once per fully-consumed HP Command (rx_eop) so the
-- LED chain / SignalTap can confirm we're seeing the host's command stream.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_hp_cmd_handler is
    port (
        clock            : in  std_logic;
        reset            : in  std_logic;

        -- HP Command byte stream input (from udp_rx_handler
        -- hpsdr_hp_cmd_* channel, UDP header already stripped).
        rx_data          : in  std_logic_vector(7 downto 0);
        rx_valid         : in  std_logic;
        rx_sop           : in  std_logic;
        rx_eop           : in  std_logic;
        rx_length        : in  std_logic_vector(13 downto 0);  -- UDP payload bytes

        -- UDP source port of the inbound HP Command, latched at rx_sop and
        -- republished on `host_port`.  Driven from udp_rx_handler.src_port,
        -- which holds the value stable across the whole packet.
        udp_src_port     : in  std_logic_vector(15 downto 0);

        -- Engagement state, latched from byte 4 of the most recent
        -- accepted HP Command and held until the next.  '0' from reset.
        host_run         : out std_logic;
        host_ptt0        : out std_logic;

        -- HP Command's UDP source port, latched at rx_sop.  Subsequent
        -- HP Status replies must use this as their UDP dst (Thetis binds
        -- its receive socket to its HP Command sendto() source).
        host_port        : out std_logic_vector(15 downto 0);

        -- Tx0 drive level (payload byte 345), 0..255.  Echoed by
        -- hpsdr_hp_status_sender in payload byte 7.
        host_drive_level : out std_logic_vector(7 downto 0);

        -- RX0 LO frequency.  Latched big-endian from HP Command payload
        -- bytes 9..12.  Interpretation (Hz vs NCO phase word) follows the
        -- value advertised by hpsdr_discovery_responder's FREQ_PHASE byte
        -- (byte 21 of the discovery reply); current FREQ_PHASE = 0x00
        -- means "Hz".  Consumed by the NIOS mailbox poller, which calls
        -- rfic_command_write_immed(... FREQUENCY ... RX(0) ...) when it
        -- detects a stable change.
        host_rx0_freq    : out std_logic_vector(31 downto 0);

        -- Observability: one cycle when the final byte of an HP Command
        -- is consumed (~10 Hz steady-state once Thetis is engaged).
        hp_cmd_pulse     : out std_logic
    );
end entity;

architecture arch of hpsdr_hp_cmd_handler is

    -- 11 bits cover byte 0..1443 with margin (HP Command is 1444 bytes).
    signal byte_idx        : unsigned(10 downto 0) := (others => '0');

    signal host_run_r       : std_logic                    := '0';
    signal host_ptt0_r      : std_logic                    := '0';
    signal host_port_r      : std_logic_vector(15 downto 0):= (others => '0');
    signal host_drive_r     : std_logic_vector(7 downto 0) := (others => '0');
    signal host_rx0_freq_r  : std_logic_vector(31 downto 0):= (others => '0');
    signal hp_cmd_pulse_r   : std_logic                    := '0';

    -- Watchdog: clears host_run / host_ptt0 if no HP Command has arrived
    -- in WATCHDOG_CYCLES.  Reloaded on every rx_eop.
    constant WATCHDOG_CYCLES : natural := 20_000_000;  -- 200 ms @ 100 MHz
    signal watchdog_r        : unsigned(24 downto 0)
        := (others => '0');

begin

    host_run         <= host_run_r;
    host_ptt0        <= host_ptt0_r;
    host_port        <= host_port_r;
    host_drive_level <= host_drive_r;
    host_rx0_freq    <= host_rx0_freq_r;
    hp_cmd_pulse     <= hp_cmd_pulse_r;

    fsm : process(clock, reset)
        variable n_byte_idx : unsigned(10 downto 0);
    begin
        if reset = '1' then
            byte_idx        <= (others => '0');
            host_run_r      <= '0';
            host_ptt0_r     <= '0';
            host_port_r     <= (others => '0');
            host_drive_r    <= (others => '0');
            host_rx0_freq_r <= (others => '0');
            hp_cmd_pulse_r  <= '0';
            watchdog_r      <= (others => '0');
        elsif rising_edge(clock) then
            hp_cmd_pulse_r <= '0';

            -- Watchdog: clear run/ptt0 if the host stops talking to us.
            -- The reload on rx_eop below races with this decrement; in
            -- VHDL the later assignment wins, so a packet arriving on the
            -- same cycle the watchdog would expire keeps us alive.
            if watchdog_r = 0 then
                host_run_r  <= '0';
                host_ptt0_r <= '0';
            else
                watchdog_r <= watchdog_r - 1;
            end if;

            if rx_valid = '1' then
                n_byte_idx := byte_idx;
                if rx_sop = '1' then
                    n_byte_idx  := (others => '0');
                    -- udp_rx_handler holds src_port stable from header parse,
                    -- so it's already settled by the time the first payload
                    -- byte (rx_sop=1) arrives here.
                    host_port_r <= udp_src_port;
                end if;

                -- Latch run / PTT0 at byte 4 (Orion2 High_Priority_CC.v
                -- line 175: `run <= udp_rx_data[0]; PC_PTT <= udp_rx_data[1];`).
                if n_byte_idx = to_unsigned(4, n_byte_idx'length) then
                    host_run_r  <= rx_data(0);
                    host_ptt0_r <= rx_data(1);
                end if;

                -- Shift-and-load RX0 frequency over bytes 9..12 (BE).  One
                -- shift-register update per byte is significantly cheaper
                -- than four separate byte-select assignments.  Bytes 13..16
                -- carry RX1 freq; we don't expose that yet.
                if n_byte_idx >= to_unsigned(9, n_byte_idx'length) and
                   n_byte_idx <= to_unsigned(12, n_byte_idx'length) then
                    host_rx0_freq_r <= host_rx0_freq_r(23 downto 0) & rx_data;
                end if;

                -- Latch Tx0 drive level (byte 345) for HP Status payload
                -- byte 7 echo.  Orion2 High_Priority_CC.v stores this in
                -- the `Drive_Level` register at the corresponding index.
                if n_byte_idx = to_unsigned(345, n_byte_idx'length) then
                    host_drive_r <= rx_data;
                end if;

                if rx_eop = '1' then
                    hp_cmd_pulse_r <= '1';
                    n_byte_idx := (others => '0');
                    -- Reload the watchdog -- this assignment runs after the
                    -- decrement above and takes precedence on this cycle.
                    watchdog_r <= to_unsigned(WATCHDOG_CYCLES,
                                              watchdog_r'length);
                else
                    n_byte_idx := n_byte_idx + 1;
                end if;

                byte_idx <= n_byte_idx;
            end if;
        end if;
    end process fsm;

end architecture;
