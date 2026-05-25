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
-- hpsdr_hp_command_receiver
--
-- Parses the OpenHPSDR Protocol 2 "High Priority Command from PC to Hardware"
-- packet (V4.4 spec page 32) arriving on UDP/1027 dst port (= the hpcmd_*
-- channel from udp_rx_handler, Eth/IP/UDP headers already stripped).  RX-only
-- entity: no TX side, no eem_tx_framer attachment.
--
-- Thetis (and piHPSDR) send this packet at ~30 Hz once the radio is in the
-- run state, carrying the host's intent for run/PTT, per-DDC tuning
-- frequency, drive level, ALEX control, etc.  For this iteration we extract
-- just the three fields that gate downstream behaviour:
--
--   * host_run        : bit [0] of payload byte 4 -- "host wants IQ streaming"
--   * host_ptt(3:0)   : bits [4:1] of payload byte 4 -- per-channel PTT requests
--   * host_freq_ddc0  : payload bytes 9..12 (big-endian) -- DDC0 tuning word
--
-- Payload layout (1444 bytes total = 4-byte seq + 1440 bytes of HP data,
-- per V4.4 spec page 32):
--   byte 0..3  : sequence number (BE, host-set; we don't validate)
--   byte 4     : [0]=run, [1]=PTT0, [2]=PTT1, [3]=PTT2, [4]=PTT3, [5..7]=rsvd
--   byte 5..8  : CWX0..3 (dot/dash; ignored for now)
--   byte 9..12 : Frequency/phase word DDC0 (BE) -- consumed
--   byte 13..16: Frequency/phase word DDC1            -- (future)
--   ...
--   byte 329..332: Frequency/phase word DUC0          -- (future)
--   ...
--   byte 1443  : tail (currently unused per spec)
--
-- Commit model
-- ------------
-- We commit each field byte-by-byte as it arrives (no per-packet snapshot
-- buffer).  The only validation is "byte 4 actually arrived before eop";
-- a runt packet that truncates before byte 4 leaves host_run / host_ptt
-- at their previous values and skips cmd_pulse / cmd_seen.  Note this is
-- the same trust-the-host model the discovery responder uses (we don't
-- validate UDP checksums either way); a bit-flip on a single command will
-- produce one bad 33 ms PTT pulse, recovered by the next correct command
-- 33 ms later.
--
-- cmd_seen
-- --------
-- One-shot latched on the first accepted command and held until reset.
-- Reserved as a future gating hook for hpsdr_hp_status_sender (hpsdr_sim
-- only starts emitting status after receiving the first HP command; we
-- don't gate today but want the signal visible if Thetis behavior reveals
-- a need).
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_hp_command_receiver is
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        -- HP Command byte stream input (from udp_rx_handler hpcmd_* channel,
        -- Eth/IP/UDP headers already stripped; byte 0 of rx_* is byte 0 of
        -- the 1444-byte HP Command payload).
        rx_data       : in  std_logic_vector(7 downto 0);
        rx_valid      : in  std_logic;
        rx_sop        : in  std_logic;
        rx_eop        : in  std_logic;

        -- Latched host intent.  All outputs hold their last-good value
        -- until the next valid command refreshes them.  Initial state
        -- (post-reset, before any command seen) is all-zero / all-clear.
        host_run      : out std_logic;
        host_ptt      : out std_logic_vector(3 downto 0);
        host_freq_ddc0: out std_logic_vector(31 downto 0);

        -- One cycle per accepted command (eop reached after at least
        -- byte 4 arrived).  Useful as LED feedback and as a "host_run
        -- just got refreshed" indicator distinct from edges on host_run
        -- itself (a steady run=1 stream gives a steady 30 Hz pulse train).
        cmd_pulse     : out std_logic;

        -- Sticky one-shot: latched high on the first accepted command and
        -- held until reset.  Reserved for future gating of HP status
        -- emission (hpsdr_sim pattern); not consumed today.
        cmd_seen      : out std_logic
    );
end entity;

architecture arch of hpsdr_hp_command_receiver is

    -- Walk-counter wide enough for the 1444-byte payload (~11 bits).  We
    -- pin at 13 bits to match the other RX consumers and leave headroom
    -- for any future spec expansion of the HP Command size.
    signal byte_idx     : unsigned(12 downto 0)         := (others => '0');

    -- Per-packet "we got far enough to commit" flag.  Cleared on rx_sop,
    -- set when byte_idx reaches 4 with a valid beat.  Gates cmd_pulse /
    -- cmd_seen at eop so a runt packet that closed before byte 4 doesn't
    -- count as a command observation.
    signal got_byte4_r  : std_logic                     := '0';

    -- Committed outputs
    signal host_run_r   : std_logic                     := '0';
    signal host_ptt_r   : std_logic_vector(3 downto 0)  := (others => '0');
    signal host_freq_r  : std_logic_vector(31 downto 0) := (others => '0');

    signal cmd_pulse_r  : std_logic                     := '0';
    signal cmd_seen_r   : std_logic                     := '0';

begin

    host_run       <= host_run_r;
    host_ptt       <= host_ptt_r;
    host_freq_ddc0 <= host_freq_r;
    cmd_pulse      <= cmd_pulse_r;
    cmd_seen       <= cmd_seen_r;

    rx_proc : process(clock, reset)
        variable n_byte_idx : unsigned(12 downto 0);
    begin
        if reset = '1' then
            byte_idx    <= (others => '0');
            got_byte4_r <= '0';
            host_run_r  <= '0';
            host_ptt_r  <= (others => '0');
            host_freq_r <= (others => '0');
            cmd_pulse_r <= '0';
            cmd_seen_r  <= '0';
        elsif rising_edge(clock) then
            cmd_pulse_r <= '0';   -- one-cycle default

            if rx_valid = '1' then
                n_byte_idx := byte_idx;

                if rx_sop = '1' then
                    -- New packet: clear the per-packet "got byte 4" flag.
                    -- Committed outputs keep their last-good values
                    -- through any new packet's RX walk -- only this
                    -- packet's own commits will replace them.
                    n_byte_idx  := (others => '0');
                    got_byte4_r <= '0';
                end if;

                case to_integer(n_byte_idx) is
                    when 4 =>
                        host_run_r  <= rx_data(0);
                        host_ptt_r  <= rx_data(4 downto 1);
                        got_byte4_r <= '1';
                    -- DDC0 frequency, big-endian, bytes 9..12
                    when  9 => host_freq_r(31 downto 24) <= rx_data;
                    when 10 => host_freq_r(23 downto 16) <= rx_data;
                    when 11 => host_freq_r(15 downto  8) <= rx_data;
                    when 12 => host_freq_r( 7 downto  0) <= rx_data;
                    when others =>
                        null;
                end case;

                if rx_eop = '1' then
                    -- Pulse / latch only if this packet was long enough
                    -- to have observed the command byte.  Note we OR in
                    -- the same-cycle case (byte_idx=4 with rx_eop=1):
                    -- in that case got_byte4_r is still '0' from prior
                    -- cycles, but the case branch above scheduled the
                    -- host_run / host_ptt commits AND the saw bit -- the
                    -- numeric comparison catches it without needing to
                    -- read the freshly-scheduled signal value.
                    if got_byte4_r = '1' or
                       n_byte_idx >= to_unsigned(4, n_byte_idx'length) then
                        cmd_pulse_r <= '1';
                        cmd_seen_r  <= '1';
                    end if;
                    n_byte_idx := (others => '0');
                else
                    n_byte_idx := n_byte_idx + 1;
                end if;

                byte_idx <= n_byte_idx;
            end if;
        end if;
    end process rx_proc;

end architecture;
