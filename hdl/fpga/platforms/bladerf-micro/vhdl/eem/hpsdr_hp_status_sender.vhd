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
-- hpsdr_hp_status_sender
--
-- Periodic transmitter for OpenHPSDR Protocol 2 "High Priority Status from
-- Radio to Host" packets, modelled on the Orion / Orion MkII reference
-- firmware's CC_encoder.v + sdr_send.v CC_SEND path
-- (Y:\Ilkka\ham\bladerf\orion2).  Sent from UDP src port 1025 (the radio's
-- HP Status port per V4.4 spec page 41).  This is the heartbeat Thetis (and
-- piHPSDR) watch for to declare a discovered radio "alive": after the
-- discovery handshake completes Thetis would otherwise time the radio out
-- after ~3 s of silence in the radio->host direction.
--
-- Payload (60 bytes, all zero in the idle case)
-- ---------------------------------------------
-- Per Orion2's CC_encoder.v, when the radio is idle (no PTT, no ADC
-- overload, no analogue sources connected) every byte of the 56-byte
-- C&C payload (= bytes 4..59 of the UDP payload after the 4-byte
-- sequence prefix) is zero.  Specifically:
--   byte 4: PTT/Dot/Dash/keyout = 0
--   byte 5: ADC overload bits = 0
--   byte 7: Exciter Power 0 [7:0] = 0
--   byte 49: Supply Volts [15:8] = 0 (unless ADC sensor connected)
--   ...etc, all-zero
--
-- Previous iterations of this file sent byte 7 = 0x8b and byte 49 = 0x3f
-- mirroring hpsdr_sim's fake "running radio" pattern; investigation of
-- the real Orion2 firmware shows those are NOT spec values, they're
-- simulator artifacts.  We omit them here -- if a clean-disconnect
-- regression appears we'll re-add them as generics (with Orion2-faithful
-- 0x00 defaults).  See [[project_orion2_reference_analysis]] and
-- [[feedback_hpsdr_thetis_disconnect_bytes]] for the history.
--
-- Rate (5 Hz idle, matching Orion2)
-- ---------------------------------
-- Orion2's CC_encoder.v ([CC_encoder.v:174](Y:/Ilkka/ham/bladerf/orion2/CC_encoder.v#L174))
-- counts 25_000_000 cycles at 125 MHz tx_clock = 200 ms (= 5 Hz) when
-- idle, switching to 125_000 cycles (= 1 ms / 1 kHz) when transmitting
-- or on ADC overload.  Plus immediate "send now" on memory[0] (PTT/Dot/
-- Dash/keyout) or memory[55] (User_IO) changes.
--
-- We currently only implement the idle 5 Hz rate (TICK_CYCLES = 20_000_000
-- at 100 MHz fx3_pclk_pll = 200 ms).  Bumping to 1 kHz requires plumbing
-- a TX-active signal from an eventual HP Command receiver -- deferred.
--
-- Gating on host_run (CRITICAL -- silence between discovery and engagement)
-- ------------------------------------------------------------------------
-- Orion2's sdr_send.v ([sdr_send.v:195](Y:/Ilkka/ham/bladerf/orion2/Ethernet/sdr_send.v#L195))
-- gates the entire CC_SEND path on `run && CC_data_ready` -- the radio is
-- SILENT between discovery and the host sending HP Command with run=1.
-- A live tcpdump of hpsdr_sim confirms: discovery reply then nothing,
-- until the host engages.  Our prior unconditional 5 Hz heartbeat was a
-- misreading of Orion2's "5 Hz idle" rate (which is the post-engagement
-- non-PTT cadence, not a from-boot heartbeat).  Sending unsolicited HP
-- Status packets to the host's ephemeral port before any HP Command has
-- arrived appears to be why Thetis lists but won't engage with our radio.
--
-- Until an HP Command receiver exists, the top level wires host_run='0'
-- and this sender stays in S_IDLE forever.  Once the receiver lands,
-- host_run reflects byte-4 bit-0 of the last received HP Command.
-- See [[project_orion2_reference_analysis]] for the full Orion2 TX-gating
-- analysis.
--
-- Addressing
-- ----------
-- {Eth dst, IP dst} = the host snapshot the discovery responder captured
-- at the last successful discovery (host_mac, host_ip).
--
-- UDP dst port: **hardcoded 1025** (= the default `High_Priority_to_PC_port`
-- per Orion2 General_CC.v + V4.4 spec).  The "reply to ephemeral source"
-- rule from [[feedback_hpsdr_reply_udp_dst_port]] only applies to the
-- discovery reply -- post-engagement traffic goes to the host-declared
-- ports per General Packet, and Thetis sends the Orion2 defaults
-- (confirmed in [[project_hpsdr_thetis_engagement]] connect dump:
-- General Packet bytes 11..12 = `04 01` = 1025).
--
-- The `host_port` input is kept on the entity for compatibility but is
-- unused in the byte mux -- synthesis will optimise the connection away.
-- When a General Packet receiver lands, we'll wire the
-- (potentially-reconfigured) `hp_to_pc_port` value here instead.
--
-- We gate everything on (host_valid='1' AND host_run='1') -- before
-- discovery completes host_mac/ip/port are all zero, and before HP Command
-- run=1 the radio is supposed to stay quiet per Orion2.
--
-- IP checksum
-- -----------
-- Identical two-stage register cascade to the discovery responder.  Stage 1
-- folds in our_ip, stage 2 folds in host_ip and inverts.  Same IP_CONST_PART
-- value (same 88-byte total length, same TTL, same proto, same flags).
--
-- UDP checksum = 0 (legal IPv4 "not computed", same as Orion2's udp_send.v
-- and our discovery responder).
--
-- FSM
-- ---
--   S_IDLE: tx_valid='0'.  Count tick_counter while host_valid='1'; on
--           TICK_CYCLES reached, increment seq, reset counter, go to S_TX.
--   S_TX:   tx_valid='1'.  Walk tx_byte_idx 0..FRAME_BYTES-1; advance only
--           when tx_ready='1'.  On the final byte, pulse send_pulse and
--           return to S_IDLE.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.bladerf_p.all;

entity hpsdr_hp_status_sender is
    generic (
        -- Inter-frame idle period in clock cycles.  Default 20_000_000 at
        -- 100 MHz fx3_pclk_pll = 200 ms = 5 Hz, matching Orion2's idle
        -- rate from CC_encoder.v.  Real Orion2 switches to 1 kHz during
        -- TX/overload -- not implemented here pending HP Command receiver.
        TICK_CYCLES         : natural := 20_000_000
    );
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        -- Local addresses
        our_mac       : in  std_logic_vector(47 downto 0);
        our_ip        : in  std_logic_vector(31 downto 0);

        -- Discovered host's address triple, snapshotted by
        -- hpsdr_discovery_responder.  host_valid='1' gates the whole
        -- sender; the three address fields are stable while it's high.
        host_mac      : in  std_logic_vector(47 downto 0);
        host_ip       : in  std_logic_vector(31 downto 0);
        host_port     : in  std_logic_vector(15 downto 0);
        host_valid    : in  std_logic;

        -- Engagement gate, driven by byte-4 bit-0 of the last HP Command
        -- received (per Orion2 sdr_send.v line 195).  '0' = radio idle ->
        -- silence the heartbeat; '1' = host has engaged -> emit at the
        -- TICK_CYCLES rate.  Tie to '0' at the top level until an HP
        -- Command receiver lands and can latch the real bit.
        host_run      : in  std_logic;

        -- TX byte stream output (to tx_arbiter -> eem_tx_framer).
        tx_data       : out std_logic_vector(7 downto 0);
        tx_valid      : out std_logic;
        tx_sop        : out std_logic;
        tx_eop        : out std_logic;
        tx_length     : out unsigned(13 downto 0);
        tx_ready      : in  std_logic;

        -- Observability: one cycle when the final byte of each status
        -- packet is emitted (~5 Hz steady-state once host_valid='1').
        send_pulse    : out std_logic
    );
end entity;

architecture arch of hpsdr_hp_status_sender is

    -- ------------------------------------------------------------------
    -- Geometry
    -- ------------------------------------------------------------------
    constant ETH_HDR_BYTES : natural := 14;
    constant IP_HDR_BYTES  : natural := 20;
    constant UDP_HDR_BYTES : natural := 8;
    constant P2_PAYLOAD    : natural := 60;
    constant FRAME_BYTES   : natural := ETH_HDR_BYTES + IP_HDR_BYTES
                                      + UDP_HDR_BYTES + P2_PAYLOAD;        -- 102
    constant IP_TOTAL_LEN  : natural := IP_HDR_BYTES + UDP_HDR_BYTES + P2_PAYLOAD; -- 88
    constant UDP_LEN       : natural := UDP_HDR_BYTES + P2_PAYLOAD;        -- 68

    constant IP_TLEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(IP_TOTAL_LEN, 16));
    constant UDP_LEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(UDP_LEN, 16));

    -- UDP/1025 = HPSDR HP Status (radio's source port).
    constant HPSDR_HP_PORT_HI : std_logic_vector(7 downto 0) := x"04";
    constant HPSDR_HP_PORT_LO : std_logic_vector(7 downto 0) := x"01";

    -- ------------------------------------------------------------------
    -- One's-complement 16-bit addition (matches the other responders).
    -- ------------------------------------------------------------------
    function oc_add(a, b : unsigned(15 downto 0)) return unsigned is
        variable sum : unsigned(16 downto 0);
    begin
        sum := ('0' & a) + ('0' & b);
        if sum(16) = '1' then
            return sum(15 downto 0) + 1;
        else
            return sum(15 downto 0);
        end if;
    end function;

    function calc_ip_const_part return unsigned is
        variable s : unsigned(15 downto 0);
    begin
        s := oc_add(to_unsigned(16#4500#, 16), to_unsigned(IP_TOTAL_LEN, 16));
        s := oc_add(s, to_unsigned(16#4000#, 16));
        s := oc_add(s, to_unsigned(16#4011#, 16));
        return s;
    end function;

    constant IP_CONST_PART : unsigned(15 downto 0) := calc_ip_const_part;

    signal ip_fixed_part_r : unsigned(15 downto 0)         := (others => '0');
    signal ip_chk_r        : std_logic_vector(15 downto 0) := (others => '0');

    -- ------------------------------------------------------------------
    -- State
    -- ------------------------------------------------------------------
    type state_t is (S_IDLE, S_TX);
    signal state : state_t := S_IDLE;

    -- 25-bit counter covers the default 20_000_000 ticks with headroom.
    signal tick_counter    : unsigned(24 downto 0) := (others => '0');

    -- Sequence number (incremented per packet, wraps after 2^32-1).
    signal seq_r           : unsigned(31 downto 0) := (others => '0');

    signal tx_byte_idx     : unsigned(13 downto 0) := (others => '0');

    signal send_pulse_r    : std_logic := '0';

    -- ------------------------------------------------------------------
    -- Combinational byte lookup for the 102-byte status frame.  Payload
    -- bytes 4..59 (idx 46..101) are all zero in this Orion2-faithful
    -- idle implementation; only the sequence number (bytes 0..3 of
    -- payload = idx 42..45) carries non-zero data.
    -- ------------------------------------------------------------------
    function status_byte_at(
        idx       : natural;
        host_mac  : std_logic_vector(47 downto 0);
        our_mac   : std_logic_vector(47 downto 0);
        our_ip    : std_logic_vector(31 downto 0);
        host_ip   : std_logic_vector(31 downto 0);
        host_port : std_logic_vector(15 downto 0);
        ip_chk    : std_logic_vector(15 downto 0);
        seq       : unsigned(31 downto 0)
    ) return std_logic_vector is
    begin
        case idx is
            -- ---- Ethernet header ----
            when  0 => return host_mac(47 downto 40);
            when  1 => return host_mac(39 downto 32);
            when  2 => return host_mac(31 downto 24);
            when  3 => return host_mac(23 downto 16);
            when  4 => return host_mac(15 downto  8);
            when  5 => return host_mac( 7 downto  0);
            when  6 => return our_mac(47 downto 40);
            when  7 => return our_mac(39 downto 32);
            when  8 => return our_mac(31 downto 24);
            when  9 => return our_mac(23 downto 16);
            when 10 => return our_mac(15 downto  8);
            when 11 => return our_mac( 7 downto  0);
            when 12 => return x"08";
            when 13 => return x"00";

            -- ---- IPv4 header ----
            when 14 => return x"45";
            when 15 => return x"00";
            when 16 => return IP_TLEN_VEC(15 downto 8);
            when 17 => return IP_TLEN_VEC( 7 downto 0);
            when 18 | 19 => return x"00";
            when 20 => return x"40";
            when 21 => return x"00";
            when 22 => return x"40";
            when 23 => return x"11";
            when 24 => return ip_chk(15 downto 8);
            when 25 => return ip_chk( 7 downto 0);
            when 26 => return our_ip(31 downto 24);
            when 27 => return our_ip(23 downto 16);
            when 28 => return our_ip(15 downto  8);
            when 29 => return our_ip( 7 downto  0);
            when 30 => return host_ip(31 downto 24);
            when 31 => return host_ip(23 downto 16);
            when 32 => return host_ip(15 downto  8);
            when 33 => return host_ip( 7 downto  0);

            -- ---- UDP header ----
            when 34 => return HPSDR_HP_PORT_HI;   -- UDP src = 1025
            when 35 => return HPSDR_HP_PORT_LO;
            -- UDP dst = 1025 hardcoded (= High_Priority_to_PC_port default
            -- per Orion2; the value Thetis configures in its General Packet).
            -- host_port is the discovery probe's ephemeral and is NOT used
            -- here -- that rule applies only to the discovery reply.
            when 36 => return HPSDR_HP_PORT_HI;
            when 37 => return HPSDR_HP_PORT_LO;
            when 38 => return UDP_LEN_VEC(15 downto 8);
            when 39 => return UDP_LEN_VEC( 7 downto 0);
            when 40 | 41 => return x"00";          -- UDP checksum = 0

            -- ---- HPSDR P2 HP Status payload (60 bytes) ----
            -- bytes 0..3: sequence (BE)
            when 42 => return std_logic_vector(seq(31 downto 24));
            when 43 => return std_logic_vector(seq(23 downto 16));
            when 44 => return std_logic_vector(seq(15 downto  8));
            when 45 => return std_logic_vector(seq( 7 downto  0));
            -- bytes 4..59 of payload (idx 46..101): all zero, matching
            -- Orion2's idle behaviour (no PTT, no overload, no analogue
            -- sources).  Real Orion2's CC_encoder.v populates these when
            -- the upstream signals go active; we have no upstream sources
            -- in this iteration so all stay zero.
            when others => return x"00";
        end case;
    end function;

begin

    -- ----------------------------------------------------------------------
    -- Output drivers
    -- ----------------------------------------------------------------------
    tx_data_mux : process(state, tx_byte_idx, host_mac, our_mac,
                          our_ip, host_ip, host_port, ip_chk_r, seq_r)
    begin
        if state = S_TX then
            tx_data <= status_byte_at(to_integer(tx_byte_idx),
                                      host_mac, our_mac,
                                      our_ip, host_ip, host_port,
                                      ip_chk_r, seq_r);
        else
            tx_data <= (others => '0');
        end if;
    end process tx_data_mux;

    tx_valid   <= '1' when state = S_TX else '0';
    tx_sop     <= '1' when (state = S_TX and tx_byte_idx = 0) else '0';
    tx_eop     <= '1' when (state = S_TX and
                            tx_byte_idx = to_unsigned(FRAME_BYTES - 1,
                                                      tx_byte_idx'length))
                      else '0';
    tx_length  <= to_unsigned(FRAME_BYTES, tx_length'length);
    send_pulse <= send_pulse_r;

    -- ----------------------------------------------------------------------
    -- Registered IP checksum -- two-stage cascade.
    -- ----------------------------------------------------------------------
    ip_fixed_part_proc : process(clock, reset)
        variable s : unsigned(15 downto 0);
    begin
        if reset = '1' then
            ip_fixed_part_r <= (others => '0');
        elsif rising_edge(clock) then
            s := IP_CONST_PART;
            s := oc_add(s, unsigned(our_ip(31 downto 16)));
            s := oc_add(s, unsigned(our_ip(15 downto  0)));
            ip_fixed_part_r <= s;
        end if;
    end process ip_fixed_part_proc;

    ip_chk_proc : process(clock, reset)
        variable s : unsigned(15 downto 0);
    begin
        if reset = '1' then
            ip_chk_r <= (others => '0');
        elsif rising_edge(clock) then
            s := ip_fixed_part_r;
            s := oc_add(s, unsigned(host_ip(31 downto 16)));
            s := oc_add(s, unsigned(host_ip(15 downto  0)));
            ip_chk_r <= std_logic_vector(not s);
        end if;
    end process ip_chk_proc;

    -- ----------------------------------------------------------------------
    -- Main FSM with rate timer
    -- ----------------------------------------------------------------------
    fsm : process(clock, reset)
    begin
        if reset = '1' then
            state         <= S_IDLE;
            tick_counter  <= (others => '0');
            seq_r         <= (others => '0');
            tx_byte_idx   <= (others => '0');
            send_pulse_r  <= '0';
        elsif rising_edge(clock) then
            send_pulse_r <= '0';

            case state is

            when S_IDLE =>
                -- Both gates must be high: a valid discovered host AND
                -- the host has engaged (HP Command run=1).  Either being
                -- low resets the tick counter so engagement starts the
                -- TICK_CYCLES interval cleanly rather than firing as
                -- soon as run=1 if the counter was mid-period.
                if host_valid = '1' and host_run = '1' then
                    if tick_counter = to_unsigned(TICK_CYCLES - 1,
                                                  tick_counter'length) then
                        tick_counter <= (others => '0');
                        tx_byte_idx  <= (others => '0');
                        seq_r        <= seq_r + 1;
                        state        <= S_TX;
                    else
                        tick_counter <= tick_counter + 1;
                    end if;
                else
                    tick_counter <= (others => '0');
                end if;

            when S_TX =>
                if tx_ready = '1' then
                    if tx_byte_idx = to_unsigned(FRAME_BYTES - 1,
                                                 tx_byte_idx'length) then
                        send_pulse_r <= '1';
                        state        <= S_IDLE;
                        tx_byte_idx  <= (others => '0');
                    else
                        tx_byte_idx <= tx_byte_idx + 1;
                    end if;
                end if;

            end case;
        end if;
    end process fsm;

end architecture;
