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
--   * host_rx0_freq (NCO phase word) - the active receive frequency.  We
--                    emulate Hermes (single DDC0 receiver), but different P2
--                    clients put the main RX on different DDC slots: Hermes/
--                    Thetis use DDC0 (bytes 9..12), while deskHPSDR's Orion2
--                    personality uses DDC2 (bytes 17..20).  Each 4-byte slot is
--                    assembled big-endian in its own shift register; at rx_eop
--                    we publish DDC0 if it is non-zero, else DDC2 -- DDC0 wins
--                    when both are present (the Hermes-preferred mapping).
--                    Crosses to the Nios via hpsdr_cmd_mux + the hpsdr_cmd_*
--                    PIOs (which expand the phase word to Hz); firmware
--                    retunes RX0.
--   * host_rx0_atten (byte 1443, 0..31 dB) - 0-31dB step attenuator before
--                    ADC0 (V4.4 spec p.35: Thetis "RX1 attenuator").
--                    Single byte, low 5 bits = attenuation in dB.  Also
--                    committed at rx_eop and republished as host_rx0_atten;
--                    hpsdr_cmd_mux maps it to BLADERF_RFIC_COMMAND_GAIN with
--                    value = 60 - atten.
--   * host_band_index (byte 1401 bits [7:2], 0..63) - virtual-band selector
--                    repurposed from V4.4 spec p.34's open-collector enables.
--                    NIOS derives the LO offset as (70 + X*150) MHz so each
--                    band sits in the AD9361's 70 MHz..6 GHz range; X=0 -> 70
--                    MHz, the bladeRF RX floor.  Committed at rx_eop with the
--                    other fields, exposed via its own PIO (NIOS reads it
--                    inside the FREQUENCY dispatch).  hpsdr_cmd_mux also
--                    re-issues a FREQUENCY op when only this byte changes, so
--                    a band switch retunes the radio even if the Thetis dial
--                    didn't move.
--   * host_rx_biastee (byte 1401 bit [1]) - RX1 bias-tee power enable, also
--                    repurposed from the V4.4 open-collector byte.  NIOS
--                    drives the RFFE control register bit
--                    RFFE_CONTROL_RX_BIAS_EN (=5) to match.  Sideband PIO,
--                    polled by NIOS every idle iteration; the cmd mailbox
--                    is not involved because bias-tee is a GPIO, not an
--                    RFIC API command.
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

        -- DUC0 transmit frequency in Hz (HP Command bytes 329..332,
        -- big-endian), committed at rx_eop.  Orion2 High_Priority_CC.v:
        -- Tx0_frequency = byte329[31:24]..byte332[7:0].  Held across packets
        -- until the next command.  Decoded + exposed for the (deferred) NIOS
        -- TX LO bring-up; no fabric consumer in the TX datapath phase.
        host_duc0_freq   : out std_logic_vector(31 downto 0);

        -- DDC0 step attenuator in dB (HP Command byte 1443, low 5 bits),
        -- committed at rx_eop.  Held across packets until the next command.
        host_rx0_atten   : out std_logic_vector(4 downto 0);

        -- DUC0 (TX0) drive level (HP Command byte 345, 0..255; 255 = max
        -- power).  Orion2 High_Priority_CC.v: drive_level <= byte 345.
        -- Committed at rx_eop; hpsdr_cmd_mux maps it to an AD9361 TX
        -- attenuation (GAIN op on the TX0 channel).
        host_tx_drive    : out std_logic_vector(7 downto 0);

        -- Virtual-band index (HP Command byte 1401 bits [7:2], 0..63).  NIOS
        -- converts to LO offset (70 + X*150) MHz.  Committed at rx_eop.
        host_band_index  : out std_logic_vector(5 downto 0);

        -- RX1 bias-tee power enable (HP Command byte 1401 bit [1]).  NIOS
        -- drives RFFE_CONTROL_RX_BIAS_EN to match.  Committed at rx_eop.
        host_rx_biastee  : out std_logic;

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

    -- RX0 frequency: two candidate shift registers assembled big-endian -
    -- DDC0 (bytes 9..12, ddc0_sr) and DDC2 (bytes 17..20, ddc2_sr).  At rx_eop
    -- host_rx0_freq_r takes DDC0 if non-zero, else DDC2 (DDC0 preferred when
    -- both are sent).  Each window writes its full 4 bytes every packet, so
    -- both registers always reflect the current command.
    signal ddc0_sr        : std_logic_vector(31 downto 0) := (others => '0');
    signal ddc2_sr        : std_logic_vector(31 downto 0) := (others => '0');
    signal host_rx0_freq_r: std_logic_vector(31 downto 0) := (others => '0');

    -- DUC0 (TX) frequency: shift register assembles bytes 329..332 big-endian
    -- (duc0_freq_sr); committed to host_duc0_freq_r at rx_eop.
    signal duc0_freq_sr    : std_logic_vector(31 downto 0) := (others => '0');
    signal host_duc0_freq_r: std_logic_vector(31 downto 0) := (others => '0');

    -- RX0 step attenuator: single-byte capture at byte 1443 (atten_latched);
    -- committed to host_rx0_atten_r at rx_eop alongside the frequency.
    signal atten_latched   : std_logic_vector(4 downto 0) := (others => '0');
    signal host_rx0_atten_r: std_logic_vector(4 downto 0) := (others => '0');

    -- DUC0 (TX0) drive level: single-byte capture at byte 345 (drive_latched);
    -- committed to host_tx_drive_r at rx_eop alongside the other fields.
    signal drive_latched   : std_logic_vector(7 downto 0) := (others => '0');
    signal host_tx_drive_r : std_logic_vector(7 downto 0) := (others => '0');

    -- Virtual-band index: byte 1401 bits [7:2], captured mid-packet and
    -- committed to host_band_index_r at rx_eop.
    signal band_latched     : std_logic_vector(5 downto 0) := (others => '0');
    signal host_band_index_r: std_logic_vector(5 downto 0) := (others => '0');

    -- RX bias-tee enable: byte 1401 bit [1], captured mid-packet and
    -- committed to host_rx_biastee_r at rx_eop.
    signal biastee_latched     : std_logic := '0';
    signal host_rx_biastee_r   : std_logic := '0';

    -- Watchdog (see RUN_TIMEOUT_CYCLES).
    signal wd_counter     : unsigned(27 downto 0) := (others => '0');

begin

    host_run        <= host_run_r;
    host_ptt0       <= host_ptt0_r;
    host_port       <= host_port_r;
    host_rx0_freq   <= host_rx0_freq_r;
    host_duc0_freq  <= host_duc0_freq_r;
    host_rx0_atten  <= host_rx0_atten_r;
    host_tx_drive   <= host_tx_drive_r;
    host_band_index <= host_band_index_r;
    host_rx_biastee <= host_rx_biastee_r;
    hp_cmd_pulse    <= hp_cmd_pulse_r;

    fsm : process(clock, reset)
        variable n_byte_idx : unsigned(10 downto 0);
    begin
        if reset = '1' then
            byte_idx          <= (others => '0');
            host_run_r        <= '0';
            host_ptt0_r       <= '0';
            host_port_r       <= (others => '0');
            ddc0_sr           <= (others => '0');
            ddc2_sr           <= (others => '0');
            host_rx0_freq_r   <= (others => '0');
            duc0_freq_sr      <= (others => '0');
            host_duc0_freq_r  <= (others => '0');
            atten_latched     <= (others => '0');
            host_rx0_atten_r  <= (others => '0');
            drive_latched     <= (others => '0');
            host_tx_drive_r   <= (others => '0');
            band_latched      <= (others => '0');
            host_band_index_r <= (others => '0');
            biastee_latched   <= '0';
            host_rx_biastee_r <= '0';
            hp_cmd_pulse_r    <= '0';
            wd_counter        <= (others => '0');
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

                -- DDC0 frequency bytes 9..12, big-endian (byte 9 = [31:24]
                -- MSB).  Shift in MSB-first; the byte-9..12 window leaves
                -- ddc0_sr holding the full 32-bit value, selected below at
                -- rx_eop.  (Hermes/Thetis main RX.)
                if (n_byte_idx >= to_unsigned(9, n_byte_idx'length)) and
                   (n_byte_idx <= to_unsigned(12, n_byte_idx'length)) then
                    ddc0_sr <= ddc0_sr(23 downto 0) & rx_data;
                end if;

                -- DDC2 frequency bytes 17..20, big-endian.  deskHPSDR's Orion2
                -- personality carries the main RX here instead of DDC0; used
                -- as the fallback when DDC0 is zero.
                if (n_byte_idx >= to_unsigned(17, n_byte_idx'length)) and
                   (n_byte_idx <= to_unsigned(20, n_byte_idx'length)) then
                    ddc2_sr <= ddc2_sr(23 downto 0) & rx_data;
                end if;

                -- Orion2 High_Priority_CC.v: DUC0 (Tx0) frequency bytes
                -- 329..332, big-endian (byte 329 = [31:24] MSB).
                if (n_byte_idx >= to_unsigned(329, n_byte_idx'length)) and
                   (n_byte_idx <= to_unsigned(332, n_byte_idx'length)) then
                    duc0_freq_sr <= duc0_freq_sr(23 downto 0) & rx_data;
                end if;

                -- Orion2 High_Priority_CC.v / V4.4 spec p.34: byte 345 = DUC0
                -- (Tx0) drive level, 0..255 (255 = max power).
                if n_byte_idx = to_unsigned(345, n_byte_idx'length) then
                    drive_latched <= rx_data;
                end if;

                -- V4.4 spec p.35: byte 1443 = 0-31dB step attenuator before
                -- ADC0 (Thetis "RX1").  Only low 5 bits are defined.
                if n_byte_idx = to_unsigned(1443, n_byte_idx'length) then
                    atten_latched <= rx_data(4 downto 0);
                end if;

                -- V4.4 spec p.34: byte 1401 = open-collector enables.  We
                -- repurpose:
                --   bits [7:2] -> 6-bit virtual-band index (LO offset)
                --   bit  [1]   -> RX1 bias-tee power enable (RFFE bit)
                --   bit  [0]   -> PA enable (out of scope here)
                if n_byte_idx = to_unsigned(1401, n_byte_idx'length) then
                    band_latched    <= rx_data(7 downto 2);
                    biastee_latched <= rx_data(1);
                end if;

                if rx_eop = '1' then
                    -- Prefer DDC0; fall back to DDC2 when DDC0 is zero.
                    if ddc0_sr /= x"00000000" then
                        host_rx0_freq_r <= ddc0_sr;
                    else
                        host_rx0_freq_r <= ddc2_sr;
                    end if;
                    host_duc0_freq_r  <= duc0_freq_sr;     -- commit TX freq too
                    host_rx0_atten_r  <= atten_latched;    -- commit at packet boundary
                    host_tx_drive_r   <= drive_latched;    -- commit TX drive level
                    host_band_index_r <= band_latched;
                    host_rx_biastee_r <= biastee_latched;
                    hp_cmd_pulse_r    <= '1';
                    wd_counter        <= (others => '0');  -- refresh on each command
                    n_byte_idx := (others => '0');
                else
                    n_byte_idx := n_byte_idx + 1;
                end if;

                byte_idx <= n_byte_idx;
            end if;
        end if;
    end process;

end architecture;
