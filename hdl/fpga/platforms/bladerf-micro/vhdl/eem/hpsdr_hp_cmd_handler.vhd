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
-- UDP/1027), per Orion2 High_Priority_CC.v.  Consumes udp_rx_handler's
-- hpsdr_hp_cmd_* stream (UDP header stripped) and latches:
--   * host_run      (byte 4 bit 0) - engagement gate for every radio->host
--                                    producer (Orion2 sdr_send.v:195)
--   * host_ptt0     (byte 4 bit 1) - reserved for the Tx path
--   * host_rx0_freq (bytes 9..12, big-endian Hz) - DDC0 receive frequency
--                    (Orion2 High_Priority_CC.v: RX0 = byte9[31:24]..byte12[7:0]).
--                    Assembled in a shift register and committed at rx_eop so a
--                    partial big-endian value is never published.  Crosses to
--                    the Nios via hpsdr_cmd_mux + the hpsdr_cmd_* PIOs;
--                    firmware retunes RX0.
--   * host_rx0_atten (byte 1443, 0..31 dB) - 0-31dB step attenuator before
--                    ADC0 (V4.4 spec p.35: Thetis "RX1 attenuator").
--                    Single byte, low 5 bits = attenuation in dB.  Also
--                    committed at rx_eop and republished as host_rx0_atten;
--                    hpsdr_cmd_mux maps it to BLADERF_RFIC_COMMAND_GAIN with
--                    value = 60 - atten.
--   * host_port     (HP Command's UDP source ephemeral, latched at rx_sop) -
--                    the dst port HP Status / DDC IQ replies must target.
--                    Thetis binds its receive socket to its HP Command sendto()
--                    source, NOT to the discovery probe's port nor 1025.
--
-- Watchdog: host_run is forced back to 0 if no C&C packet arrives within
-- RUN_TIMEOUT_CYCLES, so a host that vanishes without sending run=0 stops
-- the radio's streams (Orion2 HW_timeout equivalent).  Per V4.4 spec p.7-8
-- ("any C&C packet ... must be sent ... at least every second"),
-- cc_activity_pulse - driven by udp_rx_handler for every recognised HPSDR
-- C&C dst port (1024/1025/1026/1027/1029) - refreshes the timer alongside
-- a fully decoded HP Command.  Thetis sends HP Commands at ~10 Hz once
-- engaged, but the slower per-port cadences (e.g. DDC Specific resends or
-- DUC I&Q streaming) are now sufficient on their own.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_hp_cmd_handler is
    generic (
        -- Cycles of HP Command silence before host_run is forced low.
        -- Default 200e6 at 100 MHz = 2 s.  Must fit in the 28-bit counter
        -- below (<= ~2.68 s at 100 MHz); widen wd_counter to go longer.
        RUN_TIMEOUT_CYCLES : natural := 200_000_000
    );
    port (
        clock            : in  std_logic;
        reset            : in  std_logic;

        -- HP Command byte stream in (UDP/1027 payload, header stripped).
        rx_data          : in  std_logic_vector(7 downto 0);
        rx_valid         : in  std_logic;
        rx_sop           : in  std_logic;
        rx_eop           : in  std_logic;
        rx_length        : in  std_logic_vector(13 downto 0);

        -- UDP source port of the HP Command (held stable by udp_rx_handler);
        -- latched at rx_sop and republished on host_port.
        udp_src_port     : in  std_logic_vector(15 downto 0);

        -- C&C activity pulse from udp_rx_handler: one cycle per recognised
        -- HPSDR C&C dst port (1024/1025/1026/1027/1029).  Refreshes the
        -- watchdog so non-1027 traffic keeps host_run alive (V4.4 spec
        -- p.7-8 "any C&C packet").  Tie to '0' to retain HP-Command-only
        -- behaviour.
        cc_activity_pulse: in  std_logic;

        host_run         : out std_logic;
        host_ptt0        : out std_logic;
        host_port        : out std_logic_vector(15 downto 0);

        -- DDC0 receive frequency in Hz (HP Command bytes 9..12, big-endian),
        -- committed at rx_eop.  Held across packets until the next command.
        host_rx0_freq    : out std_logic_vector(31 downto 0);

        -- DDC0 step attenuator in dB (HP Command byte 1443, low 5 bits),
        -- committed at rx_eop.  Held across packets until the next command.
        host_rx0_atten   : out std_logic_vector(4 downto 0);

        -- One cycle per fully-consumed HP Command (~10 Hz once engaged).
        hp_cmd_pulse     : out std_logic
    );
end entity;

architecture arch of hpsdr_hp_cmd_handler is

    -- 11 bits cover byte 0..1443.
    signal byte_idx       : unsigned(10 downto 0) := (others => '0');

    signal host_run_r     : std_logic                     := '0';
    signal host_ptt0_r    : std_logic                     := '0';
    signal host_port_r    : std_logic_vector(15 downto 0) := (others => '0');
    signal hp_cmd_pulse_r : std_logic                     := '0';

    -- RX0 frequency: shift register assembles bytes 9..12 big-endian
    -- (freq_sr); committed to host_rx0_freq_r at rx_eop.
    signal freq_sr        : std_logic_vector(31 downto 0) := (others => '0');
    signal host_rx0_freq_r: std_logic_vector(31 downto 0) := (others => '0');

    -- RX0 step attenuator: single-byte capture at byte 1443 (atten_latched);
    -- committed to host_rx0_atten_r at rx_eop alongside the frequency.
    signal atten_latched   : std_logic_vector(4 downto 0) := (others => '0');
    signal host_rx0_atten_r: std_logic_vector(4 downto 0) := (others => '0');

    -- Watchdog (see RUN_TIMEOUT_CYCLES).
    signal wd_counter     : unsigned(27 downto 0) := (others => '0');

begin

    host_run       <= host_run_r;
    host_ptt0      <= host_ptt0_r;
    host_port      <= host_port_r;
    host_rx0_freq  <= host_rx0_freq_r;
    host_rx0_atten <= host_rx0_atten_r;
    hp_cmd_pulse   <= hp_cmd_pulse_r;

    fsm : process(clock, reset)
        variable n_byte_idx : unsigned(10 downto 0);
    begin
        if reset = '1' then
            byte_idx        <= (others => '0');
            host_run_r      <= '0';
            host_ptt0_r     <= '0';
            host_port_r     <= (others => '0');
            freq_sr         <= (others => '0');
            host_rx0_freq_r <= (others => '0');
            atten_latched   <= (others => '0');
            host_rx0_atten_r<= (others => '0');
            hp_cmd_pulse_r  <= '0';
            wd_counter      <= (others => '0');
        elsif rising_edge(clock) then
            hp_cmd_pulse_r <= '0';

            -- Watchdog ages while engaged; an arriving C&C packet (any
            -- HPSDR port, via cc_activity_pulse) or a fully-decoded HP
            -- Command (rx_eop below) refreshes it.  Later assignments
            -- in this process win, so the order here is fine.
            if host_run_r = '1' then
                if wd_counter = to_unsigned(RUN_TIMEOUT_CYCLES - 1,
                                            wd_counter'length) then
                    host_run_r <= '0';
                    wd_counter <= (others => '0');
                else
                    wd_counter <= wd_counter + 1;
                end if;
            else
                wd_counter <= (others => '0');
            end if;

            -- Any C&C packet refreshes the watchdog (V4.4 spec p.7-8).
            if cc_activity_pulse = '1' then
                wd_counter <= (others => '0');
            end if;

            if rx_valid = '1' then
                n_byte_idx := byte_idx;
                if rx_sop = '1' then
                    n_byte_idx  := (others => '0');
                    host_port_r <= udp_src_port;
                end if;

                -- Orion2 High_Priority_CC.v: run <= byte4[0], PC_PTT <= byte4[1].
                if n_byte_idx = to_unsigned(4, n_byte_idx'length) then
                    host_run_r  <= rx_data(0);
                    host_ptt0_r <= rx_data(1);
                end if;

                -- Orion2 High_Priority_CC.v: RX0 frequency bytes 9..12,
                -- big-endian (byte 9 = [31:24] MSB).  Shift in MSB-first; a
                -- byte-9..12 window leaves freq_sr holding the full 32-bit
                -- value, committed below at rx_eop.
                if (n_byte_idx >= to_unsigned(9, n_byte_idx'length)) and
                   (n_byte_idx <= to_unsigned(12, n_byte_idx'length)) then
                    freq_sr <= freq_sr(23 downto 0) & rx_data;
                end if;

                -- V4.4 spec p.35: byte 1443 = 0-31dB step attenuator before
                -- ADC0 (Thetis "RX1").  Only low 5 bits are defined.
                if n_byte_idx = to_unsigned(1443, n_byte_idx'length) then
                    atten_latched <= rx_data(4 downto 0);
                end if;

                if rx_eop = '1' then
                    host_rx0_freq_r  <= freq_sr;          -- commit assembled value
                    host_rx0_atten_r <= atten_latched;    -- commit at packet boundary
                    hp_cmd_pulse_r   <= '1';
                    wd_counter       <= (others => '0');  -- refresh on each command
                    n_byte_idx := (others => '0');
                else
                    n_byte_idx := n_byte_idx + 1;
                end if;

                byte_idx <= n_byte_idx;
            end if;
        end if;
    end process;

end architecture;
