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
-- hpsdr_discovery_responder
--
-- OpenHPSDR Protocol 2 (V4.4 spec) discovery responder, modelled byte-for-byte
-- on the upstream Orion / Orion MkII reference firmware
-- (Y:\Ilkka\ham\bladerf\orion2\Orion.v + Ethernet\sdr_send.v).  The goal is to
-- present to Thetis (and piHPSDR) as a real Orion MkII (ANAN-7000/8000DLE)
-- radio.  Listens on udp_rx_handler's hpsdr_* channel (UDP/1024 payload),
-- recognises the General-Packet "discovery request" by command byte 4 == 0x02,
-- and emits a single 102-byte Ethernet frame back through eem_tx_framer.
--
-- Orion2 discovery reply packing (from sdr_send.v line 179):
--   {32'd0, (8'd02 + run), local_mac, board_type, protocol_version,
--    code_version, 48'd0, number_Rx, 8'd1, 8'd0, beta_version}
--
-- Decoded (60-byte UDP payload):
--   byte 0..3 : sequence number = 0
--   byte 4    : 0x02 idle / 0x03 running (hard-coded 0x02 here -- no HP
--               Command receiver in this iteration)
--   byte 5..10: our MAC, network order
--   byte 11   : BOARD_TYPE generic (default 0x05 = ORION Mk II per V4.4 spec)
--   byte 12   : PROTOCOL_VERSION generic (default 0x2C = 44 = "v4.4")
--   byte 13   : CODE_VERSION generic (default 0x16 = 22 = Orion firmware "v2.2")
--   byte 14..19: zero (Mercury/Penny/Metis sub-board code versions; Atlas only)
--   byte 20   : NUMBER_DDCS generic (default 0x01 = 1 receiver for our bladeRF)
--   byte 21   : FREQ_PHASE generic (default 0x01 = phase-word mode, per Orion2)
--   byte 22   : 0x00 (Available Endian modes; 0 = Big-Endian + 3-byte IQ default)
--   byte 23   : BETA_VERSION generic (default 0x0A = 10, matches Orion v2.2)
--   byte 24..59: zero pad
--
-- Note vs prior hpsdr_sim mirror: hpsdr_sim sends 0x26/0x13 at bytes 12/13 and
-- 0x02/0x01/0x03/0x00/0x00 at 19/20/21/22/23.  Orion2 (which this responder
-- models) sends 0x2C/0x16/0x00/0x01/0x01/0x00/0x0A.  Thetis recognises both
-- as Orion MkII because the classification key is byte 11 alone; the other
-- bytes affect downstream behaviour (protocol version, feature gating).
-- See [[project_orion2_reference_analysis]].
--
-- Unicast reply
-- -------------
-- Eth dst = inbound src MAC, IP dst = inbound src IP, UDP dst port = inbound
-- src UDP port.  All three sidebands are held stable by the upstream
-- demuxes/handlers for the full duration of the inbound payload, so a
-- single snapshot at rx_sop captures the matching triple.  IP src =
-- effective_ip (DHCP-leased post-lease, static EEM_OUR_IP pre-lease).
--
-- host_mac / host_ip / host_port / host_valid expose the committed client
-- identity for downstream HPSDR producers (currently just hpsdr_hp_status_
-- sender; future DDC IQ streamers).  host_valid latches '1' on first
-- successful discovery and stays high.
--
-- IP checksum strategy
-- --------------------
-- Two-stage register cascade.  Stage 1 folds in our_ip; stage 2 folds in
-- peer_ip and inverts.  Each oc_add chain stays at 3 levels max so the
-- 100 MHz fx3_pclk_pll closes comfortably.  peer_ip_r changes only on
-- rx_sop, ~60 cycles before rx_eop -> S_TX, so ip_chk_r has plenty of
-- time to settle before the first reply byte goes out.
--
-- UDP checksum = 0 (legal IPv4 "not computed", same as Orion2's udp_send.v
-- and our dhcp_client / hpsdr senders).
--
-- FSM
-- ---
--   S_RX: walk inbound bytes, latch peer_mac/ip/port at sop, set
--         is_discovery_r at byte 4 if command = 0x02.  On rx_eop with a
--         valid discovery probe, commit host_*_r and transition to S_TX.
--   S_TX: combinational byte mux walks tx_byte_idx 0..101, advances only
--         when tx_ready='1'.  On the final byte, pulse reply_pulse and
--         return to S_RX.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.bladerf_p.all;

entity hpsdr_discovery_responder is
    generic (
        -- Byte 11 of the reply payload.  V4.4 spec page 44 board ID table:
        --   0x00 = ATLAS, 0x01 = HERMES (ANAN-10/100), 0x02 = HERMES (ANAN-10E/100B),
        --   0x03 = ANGELIA, 0x04 = ORION (ANAN-200D), 0x05 = ORION Mk II
        --   (ANAN-7/8000DLE), 0x06 = Hermes Lite, 0x0A = SATURN (ANAN-G2).
        -- We advertise as Orion MkII (0x05) to match the upstream Orion2
        -- reference firmware that Thetis was designed against.
        BOARD_TYPE       : std_logic_vector(7 downto 0) := x"05";

        -- Byte 12: openHPSDR Protocol version supported.  Thetis interprets
        -- as a decimal fraction: 0x2C = 44 -> "v4.4" (matches the published
        -- V4.4 spec document; same value Orion2's Orion.v hardcodes).
        PROTOCOL_VERSION : std_logic_vector(7 downto 0) := x"2C";

        -- Byte 13: Firmware Code Version.  Decimal interpretation:
        -- 0x16 = 22 -> "Orion firmware v2.2" (matches Orion2's Orion_version).
        CODE_VERSION     : std_logic_vector(7 downto 0) := x"16";

        -- Byte 20: Number of DDCs implemented.  bladeRF currently has 1
        -- DDC; bump if/when we add more receivers.
        NUMBER_DDCS      : std_logic_vector(7 downto 0) := x"01";

        -- Byte 21: Frequency or phase word advertisement
        -- (0 = frequency in Hz, 1 = phase word).  Orion2 uses phase mode
        -- (= 1) since its HP Command's RX frequency bytes carry pre-computed
        -- 32-bit phase increments at the radio's 122.88 MHz reference clock.
        -- We advertise Hz mode (= 0) instead: Thetis then sends raw Hz in
        -- HP Command bytes 9..12, which hpsdr_hp_cmd_handler latches and
        -- forwards to NIOS via the xb_gpio mailbox -> ad9361_set_rx_lo_freq.
        -- Avoids needing a phase-word -> Hz conversion in NIOS.
        FREQ_PHASE       : std_logic_vector(7 downto 0) := x"00";

        -- Byte 23: Beta version tag (0 = official release, non-zero = beta
        -- number).  0x0A matches Orion firmware v2.2's beta_version.
        BETA_VERSION     : std_logic_vector(7 downto 0) := x"0A"
    );
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        -- Local addresses
        our_mac       : in  std_logic_vector(47 downto 0);
        our_ip        : in  std_logic_vector(31 downto 0);

        -- Probing host's L2/L3/L4 addresses, snooped at the Eth/IP/UDP
        -- layers and held stable across the full payload duration.
        peer_mac      : in  std_logic_vector(47 downto 0);
        peer_ip       : in  std_logic_vector(31 downto 0);
        peer_port     : in  std_logic_vector(15 downto 0);

        -- HPSDR byte stream input (from udp_rx_handler hpsdr_* channel,
        -- Eth/IP/UDP headers already stripped; byte 0 of rx_* is byte 0
        -- of the 60-byte General Packet).
        rx_data       : in  std_logic_vector(7 downto 0);
        rx_valid      : in  std_logic;
        rx_sop        : in  std_logic;
        rx_eop        : in  std_logic;
        rx_length     : in  std_logic_vector(13 downto 0);   -- UDP payload bytes

        -- TX byte stream output (to tx_arbiter -> eem_tx_framer).
        tx_data       : out std_logic_vector(7 downto 0);
        tx_valid      : out std_logic;
        tx_sop        : out std_logic;
        tx_eop        : out std_logic;
        tx_length     : out unsigned(13 downto 0);
        tx_ready      : in  std_logic;

        -- Committed HPSDR client identity, captured at the moment we
        -- accept a discovery probe.  Future producers (HP status sender,
        -- DDC IQ streamers) read these to address their unsolicited
        -- transmissions to the last discovered host.
        host_mac      : out std_logic_vector(47 downto 0);
        host_ip       : out std_logic_vector(31 downto 0);
        host_port     : out std_logic_vector(15 downto 0);
        host_valid    : out std_logic;

        -- Observability: one cycle when the final reply byte is emitted.
        reply_pulse   : out std_logic
    );
end entity;

architecture arch of hpsdr_discovery_responder is

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

    -- HPSDR Protocol 2 General Packet command byte values relevant here.
    constant P2_CMD_DISCOVERY : std_logic_vector(7 downto 0) := x"02";

    -- UDP/1024 = HPSDR General-Packet plane (radio's src port for the
    -- discovery reply, per V4.4 spec page 43).
    constant HPSDR_PORT_HI : std_logic_vector(7 downto 0) := x"04";
    constant HPSDR_PORT_LO : std_logic_vector(7 downto 0) := x"00";

    -- ------------------------------------------------------------------
    -- One's-complement 16-bit addition (matches icmp_responder /
    -- dhcp_client).
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

    -- Precomputed contribution to the IPv4-header checksum from the
    -- truly compile-time-constant fields of the reply:
    --   0x4500 (V/IHL/DSCP), IP_TOTAL_LEN, 0x4000 (DF), 0x4011 (TTL=64/UDP).
    -- ID (0) and the placeholder checksum word contribute 0 and are omitted.
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
    type state_t is (S_RX, S_TX);
    signal state : state_t := S_RX;

    signal rx_byte_idx     : unsigned(13 downto 0) := (others => '0');
    signal is_discovery_r  : std_logic             := '0';

    signal peer_mac_r      : std_logic_vector(47 downto 0) := (others => '0');
    signal peer_ip_r       : std_logic_vector(31 downto 0) := (others => '0');
    signal peer_port_r     : std_logic_vector(15 downto 0) := (others => '0');

    signal host_mac_r      : std_logic_vector(47 downto 0) := (others => '0');
    signal host_ip_r       : std_logic_vector(31 downto 0) := (others => '0');
    signal host_port_r     : std_logic_vector(15 downto 0) := (others => '0');
    signal host_valid_r    : std_logic                     := '0';

    signal tx_byte_idx     : unsigned(13 downto 0) := (others => '0');

    signal reply_pulse_r   : std_logic := '0';

    -- ------------------------------------------------------------------
    -- Combinational byte lookup for the 102-byte reply frame.
    -- ------------------------------------------------------------------
    function disc_byte_at(
        idx       : natural;
        peer_mac  : std_logic_vector(47 downto 0);
        our_mac   : std_logic_vector(47 downto 0);
        our_ip    : std_logic_vector(31 downto 0);
        peer_ip   : std_logic_vector(31 downto 0);
        peer_port : std_logic_vector(15 downto 0);
        ip_chk    : std_logic_vector(15 downto 0)
    ) return std_logic_vector is
    begin
        case idx is
            -- ---- Ethernet header ----
            when  0 => return peer_mac(47 downto 40);
            when  1 => return peer_mac(39 downto 32);
            when  2 => return peer_mac(31 downto 24);
            when  3 => return peer_mac(23 downto 16);
            when  4 => return peer_mac(15 downto  8);
            when  5 => return peer_mac( 7 downto  0);
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
            when 18 | 19 => return x"00";          -- IP identification = 0
            when 20 => return x"40";                -- IP flags = DF
            when 21 => return x"00";
            when 22 => return x"40";                -- IP TTL = 64
            when 23 => return x"11";                -- IP protocol = UDP
            when 24 => return ip_chk(15 downto 8);
            when 25 => return ip_chk( 7 downto 0);
            when 26 => return our_ip(31 downto 24); -- IP src = us
            when 27 => return our_ip(23 downto 16);
            when 28 => return our_ip(15 downto  8);
            when 29 => return our_ip( 7 downto  0);
            when 30 => return peer_ip(31 downto 24);-- IP dst = host
            when 31 => return peer_ip(23 downto 16);
            when 32 => return peer_ip(15 downto  8);
            when 33 => return peer_ip( 7 downto  0);

            -- ---- UDP header ----
            when 34 => return HPSDR_PORT_HI;        -- UDP src = 1024
            when 35 => return HPSDR_PORT_LO;
            when 36 => return peer_port(15 downto 8);  -- UDP dst = host's ephemeral
            when 37 => return peer_port( 7 downto 0);
            when 38 => return UDP_LEN_VEC(15 downto 8);
            when 39 => return UDP_LEN_VEC( 7 downto 0);
            when 40 | 41 => return x"00";           -- UDP checksum = 0

            -- ---- HPSDR P2 discovery reply payload (60 bytes) ----
            -- bytes 0..3: sequence (always 0)
            when 42 | 43 | 44 | 45 => return x"00";
            -- byte 4: status = 0x02 (idle).  Will become 0x03 ("running and
            -- connected to a different host") once a HP Command receiver
            -- and host_run plumbing exist.  Orion2 derives this from
            -- `(8'd02 + run)`; we hardcode 0x02 in this iteration.
            when 46 => return x"02";
            -- bytes 5..10 of payload (idx 47..52): our MAC
            when 47 => return our_mac(47 downto 40);
            when 48 => return our_mac(39 downto 32);
            when 49 => return our_mac(31 downto 24);
            when 50 => return our_mac(23 downto 16);
            when 51 => return our_mac(15 downto  8);
            when 52 => return our_mac( 7 downto  0);
            -- byte 11 of payload (idx 53): Board Type
            when 53 => return BOARD_TYPE;
            -- byte 12 of payload (idx 54): openHPSDR Protocol Version
            when 54 => return PROTOCOL_VERSION;
            -- byte 13 of payload (idx 55): Firmware Code Version
            when 55 => return CODE_VERSION;
            -- bytes 14..19 of payload (idx 56..61): zero
            -- (Mercury 0..3 / Penny / Metis sub-board code versions for
            -- Atlas-based systems; zero for non-Atlas like Orion).
            -- byte 20 of payload (idx 62): Number of DDCs implemented
            when 62 => return NUMBER_DDCS;
            -- byte 21 of payload (idx 63): Frequency-or-phase-word advertisement
            when 63 => return FREQ_PHASE;
            -- byte 22 of payload (idx 64): Available Endian modes
            -- (0 = Big-Endian + 3-byte IQ default, per V4.4 spec page 45)
            when 64 => return x"00";
            -- byte 23 of payload (idx 65): Beta version
            when 65 => return BETA_VERSION;
            -- bytes 24..59 of payload (idx 66..101): zero pad
            when others => return x"00";
        end case;
    end function;

begin

    -- ----------------------------------------------------------------------
    -- Output drivers
    -- ----------------------------------------------------------------------
    tx_data_mux : process(state, tx_byte_idx, peer_mac_r, our_mac,
                          our_ip, peer_ip_r, peer_port_r, ip_chk_r)
    begin
        if state = S_TX then
            tx_data <= disc_byte_at(to_integer(tx_byte_idx),
                                    peer_mac_r, our_mac,
                                    our_ip, peer_ip_r, peer_port_r,
                                    ip_chk_r);
        else
            tx_data <= (others => '0');
        end if;
    end process tx_data_mux;

    tx_valid    <= '1' when state = S_TX else '0';
    tx_sop      <= '1' when (state = S_TX and tx_byte_idx = 0) else '0';
    tx_eop      <= '1' when (state = S_TX and
                             tx_byte_idx = to_unsigned(FRAME_BYTES - 1,
                                                       tx_byte_idx'length))
                       else '0';
    tx_length   <= to_unsigned(FRAME_BYTES, tx_length'length);
    reply_pulse <= reply_pulse_r;

    host_mac    <= host_mac_r;
    host_ip     <= host_ip_r;
    host_port   <= host_port_r;
    host_valid  <= host_valid_r;

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
            s := oc_add(s, unsigned(peer_ip_r(31 downto 16)));
            s := oc_add(s, unsigned(peer_ip_r(15 downto  0)));
            ip_chk_r <= std_logic_vector(not s);
        end if;
    end process ip_chk_proc;

    -- ----------------------------------------------------------------------
    -- Main FSM
    -- ----------------------------------------------------------------------
    fsm : process(clock, reset)
        variable n_byte_idx : unsigned(13 downto 0);
        variable n_is_disc  : std_logic;
    begin
        if reset = '1' then
            state          <= S_RX;
            rx_byte_idx    <= (others => '0');
            is_discovery_r <= '0';
            peer_mac_r     <= (others => '0');
            peer_ip_r      <= (others => '0');
            peer_port_r    <= (others => '0');
            host_mac_r     <= (others => '0');
            host_ip_r      <= (others => '0');
            host_port_r    <= (others => '0');
            host_valid_r   <= '0';
            tx_byte_idx    <= (others => '0');
            reply_pulse_r  <= '0';
        elsif rising_edge(clock) then
            reply_pulse_r <= '0';

            case state is

            when S_RX =>
                n_byte_idx := rx_byte_idx;
                n_is_disc  := is_discovery_r;

                if rx_valid = '1' then
                    if rx_sop = '1' then
                        n_byte_idx := (others => '0');
                        n_is_disc  := '0';
                        peer_mac_r  <= peer_mac;
                        peer_ip_r   <= peer_ip;
                        peer_port_r <= peer_port;
                    end if;

                    if n_byte_idx = to_unsigned(4, n_byte_idx'length) then
                        if rx_data = P2_CMD_DISCOVERY then
                            n_is_disc := '1';
                        end if;
                    end if;

                    if rx_eop = '1' then
                        if n_is_disc = '1' and
                           n_byte_idx >= to_unsigned(4, n_byte_idx'length) then
                            tx_byte_idx  <= (others => '0');
                            state        <= S_TX;
                            host_mac_r   <= peer_mac_r;
                            host_ip_r    <= peer_ip_r;
                            host_port_r  <= peer_port_r;
                            host_valid_r <= '1';
                        end if;
                        n_byte_idx := (others => '0');
                        n_is_disc  := '0';
                    else
                        n_byte_idx := n_byte_idx + 1;
                    end if;
                end if;

                rx_byte_idx    <= n_byte_idx;
                is_discovery_r <= n_is_disc;

            when S_TX =>
                if tx_ready = '1' then
                    if tx_byte_idx = to_unsigned(FRAME_BYTES - 1,
                                                 tx_byte_idx'length) then
                        reply_pulse_r <= '1';
                        state         <= S_RX;
                        tx_byte_idx   <= (others => '0');
                    else
                        tx_byte_idx <= tx_byte_idx + 1;
                    end if;
                end if;

            end case;
        end if;
    end process fsm;

end architecture;
