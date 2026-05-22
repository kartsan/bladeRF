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
-- Minimal OpenHPSDR Protocol 2 discovery responder.  Listens on the
-- udp_rx_handler hpsdr_* channel (UDP/1024 payload, Eth/IP/UDP stripped),
-- recognises the General-Packet "discovery request" by command byte 4 == 0x02,
-- and emits a single 102-byte Ethernet frame back through eem_tx_framer
-- advertising this device as a Hermes-class HPSDR P2 radio.
--
-- Protocol 2 General Packet layout (host -> radio, 60-byte UDP payload)
-- ---------------------------------------------------------------------
--   byte 0..3 : sequence number (BE, host-set; we ignore)
--   byte 4    : command  (0x02 = discovery; 0x00/0x04 = start/stop; ...)
--   byte 5..59: command-specific (zero for discovery)
--
-- Discovery reply (radio -> host, 60-byte UDP payload)
-- ----------------------------------------------------
--   byte 0..3 : sequence number = 0 (spec is implementation-defined; piHPSDR
--               ignores)
--   byte 4    : status        (0x02 = idle, 0x03 = sending IQ)
--   byte 5..10: our MAC (6 bytes, big-endian / network order)
--   byte 11   : board type ID (BOARD_TYPE generic; default 0x06 = Hermes)
--   byte 12   : gateware major version (GW_MAJOR generic; default 0x01)
--   byte 13   : gateware minor / protocol revision tag (GW_MINOR generic;
--               default 0x02 = "P2")
--   byte 14..59: zero pad (device-specific fields ignored by piHPSDR for
--               minimal "device present" advertisement)
--
-- Unicast reply
-- -------------
-- The reply is unicast back to the host that sent the probe: Eth dst =
-- inbound src MAC, IP dst = inbound src IP.  Most reference HPSDR
-- servers (Hermes-Lite 2 firmware, openHPSDR FPGA) do this; some clients
-- (notably one piHPSDR variant) outright reject broadcast replies, and
-- the L2 dst on a unicast reply is correct anyway since the inbound
-- frame told us the exact L2/L3 binding (src MAC <-> src IP) of a
-- known-reachable neighbor -- no ARP lookup needed.
--
-- peer_mac comes from eth_rx_demux's src_mac sideband; peer_ip from
-- ip_rx_handler's src_ip sideband.  Both are latched at classify time
-- by their respective stages and held stable for the full duration of
-- the payload emission through udp_rx_handler's hpsdr_* forward.  We
-- snapshot them locally on rx_sop and hold for the entire reply burst.
--
-- The same snapshot is exposed as host_mac / host_ip / host_valid so
-- subsequent HPSDR producers (high-priority status, IQ streamers) can
-- target the same client without re-snooping the inbound side.
--
-- IP checksum strategy
-- --------------------
-- IPv4-header fields split into three classes:
--   * truly constant         {V/IHL/DSCP, total_length, flags, TTL/proto}
--                            -> compile-time `IP_CONST_PART`
--   * varies with our_ip     {src halves}
--                            -> registered `ip_fixed_part_r` =
--                               IP_CONST_PART + our_ip_hi + our_ip_lo
--   * varies with peer_ip    {dst halves}
--                            -> registered `ip_chk_r` =
--                               ~(ip_fixed_part_r + peer_ip_hi + peer_ip_lo)
-- Two register stages keep each oc_add chain shallow (3 levels).  The
-- per-byte TX mux at idx 24..25 just reads ip_chk_r -- no per-packet
-- arithmetic on the emission path.  peer_ip_r changes only at rx_sop,
-- so by the ~60-cycle rx_eop->S_TX transition ip_chk_r has settled
-- well before the first reply byte.
--
-- UDP checksum = 0 (legal "no checksum" in IPv4) -- avoids pseudo-header
-- math across a 60-byte payload.  dhcp_client uses the same trick.
--
-- FSM
-- ---
--   S_RX: walk the incoming hpsdr_* payload bytes.  Track byte index;
--         at byte 4, latch is_discovery := (rx_data = 0x02).  On rx_eop,
--         if is_discovery and we reached at least byte 4, transition to
--         S_TX.  Otherwise stay in S_RX.
--   S_TX: combinational byte mux from disc_byte_at(tx_byte_idx, ...);
--         advance tx_byte_idx only when tx_ready='1'.  On the final byte
--         (tx_byte_idx = FRAME_BYTES - 1), pulse reply_pulse and return
--         to S_RX.
--
-- Concurrent requests during S_TX are dropped (hpsdr_* bytes aren't
-- consumed in that state); host retransmit timer (~2-3 s in piHPSDR)
-- recovers cleanly since we're well under 1 ms per reply.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.bladerf_p.all;

entity hpsdr_discovery_responder is
    generic (
        -- Advertised board type (byte 11 of the reply payload).  0x06 =
        -- Hermes; piHPSDR will display the radio as a Hermes-class
        -- single-RX device.  Switch to 0x07 (Angelia) once the dual-DDC
        -- IQ packetizers are in place and piHPSDR's "dual receiver"
        -- features should be exposed.
        BOARD_TYPE  : std_logic_vector(7 downto 0) := x"06";

        -- Gateware version (byte 12) and protocol-revision tag (byte 13).
        -- Both are advisory -- piHPSDR's discovery dialog displays them
        -- but doesn't gate behaviour on them.
        GW_MAJOR    : std_logic_vector(7 downto 0) := x"01";
        GW_MINOR    : std_logic_vector(7 downto 0) := x"02";

        -- Status byte (byte 4).  0x02 = idle (no IQ flowing), 0x03 = active.
        -- Hard-coded idle for now; will become a runtime input once the
        -- DDC packetizers exist.
        STATUS_BYTE : std_logic_vector(7 downto 0) := x"02"
    );
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        -- Local addresses
        our_mac       : in  std_logic_vector(47 downto 0);
        our_ip        : in  std_logic_vector(31 downto 0);

        -- Probing host's L2/L3 addresses, snooped at the Eth/IP layers
        -- and held stable across the full payload duration.  Latched
        -- locally on rx_sop into peer_mac_r / peer_ip_r so subsequent
        -- bytes of the probe (or a fresh probe arriving during S_TX)
        -- can't disturb the in-flight reply.
        peer_mac      : in  std_logic_vector(47 downto 0);
        peer_ip       : in  std_logic_vector(31 downto 0);

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

        -- Known HPSDR client identity, committed at the moment we
        -- accept a discovery probe and begin emitting its reply.  Future
        -- HPSDR producers (high-priority status, DDC IQ streamers, ...)
        -- read these to address their unsolicited transmissions to the
        -- last discovered host without re-snooping eth_rx_demux /
        -- ip_rx_handler.  host_valid latches '1' on first successful
        -- discovery and stays high; host_mac / host_ip track the most
        -- recent successful discoverer (in case another client probes
        -- after the first selection).
        host_mac      : out std_logic_vector(47 downto 0);
        host_ip       : out std_logic_vector(31 downto 0);
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

    -- UDP/1024 = HPSDR control plane.
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
    -- Both src (our_ip) and dst (peer_ip) are added at register stages
    -- below; neither is folded in here.
    function calc_ip_const_part return unsigned is
        variable s : unsigned(15 downto 0);
    begin
        s := oc_add(to_unsigned(16#4500#, 16), to_unsigned(IP_TOTAL_LEN, 16));
        s := oc_add(s, to_unsigned(16#4000#, 16));
        s := oc_add(s, to_unsigned(16#4011#, 16));
        return s;
    end function;

    constant IP_CONST_PART : unsigned(15 downto 0) := calc_ip_const_part;

    -- Stage-1 register: IP_CONST_PART + our_ip halves.  Updated one
    -- cycle after `our_ip` changes (typically once, at DHCP-ACK).
    signal ip_fixed_part_r : unsigned(15 downto 0)         := (others => '0');

    -- Stage-2 register: ~(ip_fixed_part_r + peer_ip_r halves).  Updated
    -- one cycle after peer_ip_r changes (latched on each rx_sop).  The
    -- per-byte TX mux reads it directly at idx 24..25.
    signal ip_chk_r        : std_logic_vector(15 downto 0) := (others => '0');

    -- ------------------------------------------------------------------
    -- State
    -- ------------------------------------------------------------------
    type state_t is (S_RX, S_TX);
    signal state : state_t := S_RX;

    -- RX-side tracking
    signal rx_byte_idx     : unsigned(13 downto 0) := (others => '0');
    signal is_discovery_r  : std_logic             := '0';

    -- Snapshot of the probing host's addresses at rx_sop.  Held stable
    -- across the entire reply emission (S_TX consumes no RX bytes, so
    -- a fresh probe arriving during S_TX can't disturb in-flight data).
    signal peer_mac_r      : std_logic_vector(47 downto 0) := (others => '0');
    signal peer_ip_r       : std_logic_vector(31 downto 0) := (others => '0');

    -- Committed host identity, exported via host_* ports.  Latched at
    -- the S_RX -> S_TX transition (where is_discovery has already been
    -- proven '1' and peer_*_r are guaranteed populated).
    signal host_mac_r      : std_logic_vector(47 downto 0) := (others => '0');
    signal host_ip_r       : std_logic_vector(31 downto 0) := (others => '0');
    signal host_valid_r    : std_logic                     := '0';

    -- TX-side counter (walks 0..FRAME_BYTES-1)
    signal tx_byte_idx     : unsigned(13 downto 0) := (others => '0');

    signal reply_pulse_r   : std_logic := '0';

    -- ------------------------------------------------------------------
    -- Combinational byte lookup for the 102-byte reply frame.  Pure
    -- function of inputs; called from the tx_data mux below.
    -- ------------------------------------------------------------------
    function disc_byte_at(
        idx       : natural;
        peer_mac  : std_logic_vector(47 downto 0);
        our_mac   : std_logic_vector(47 downto 0);
        our_ip    : std_logic_vector(31 downto 0);
        peer_ip   : std_logic_vector(31 downto 0);
        ip_chk    : std_logic_vector(15 downto 0)
    ) return std_logic_vector is
    begin
        case idx is
            -- ---- Ethernet header ----
            -- Eth dst MAC = the probing host
            when  0 => return peer_mac(47 downto 40);
            when  1 => return peer_mac(39 downto 32);
            when  2 => return peer_mac(31 downto 24);
            when  3 => return peer_mac(23 downto 16);
            when  4 => return peer_mac(15 downto  8);
            when  5 => return peer_mac( 7 downto  0);
            -- Eth src MAC = us
            when  6 => return our_mac(47 downto 40);
            when  7 => return our_mac(39 downto 32);
            when  8 => return our_mac(31 downto 24);
            when  9 => return our_mac(23 downto 16);
            when 10 => return our_mac(15 downto  8);
            when 11 => return our_mac( 7 downto  0);
            -- Ethertype = IPv4
            when 12 => return x"08";
            when 13 => return x"00";

            -- ---- IPv4 header ----
            -- V/IHL=0x45, DSCP/ECN=0x00
            when 14 => return x"45";
            when 15 => return x"00";
            -- IP total length = 88
            when 16 => return IP_TLEN_VEC(15 downto 8);
            when 17 => return IP_TLEN_VEC( 7 downto 0);
            -- IP identification = 0
            when 18 | 19 => return x"00";
            -- IP flags + frag offset = 0x4000 (DF)
            when 20 => return x"40";
            when 21 => return x"00";
            -- IP TTL=64, protocol=UDP (0x11)
            when 22 => return x"40";
            when 23 => return x"11";
            -- IP header checksum (two-stage register cascade, folds in
            -- our_ip at stage 1 and peer_ip at stage 2; see ip_chk_proc)
            when 24 => return ip_chk(15 downto 8);
            when 25 => return ip_chk( 7 downto 0);
            -- IP src = our_ip
            when 26 => return our_ip(31 downto 24);
            when 27 => return our_ip(23 downto 16);
            when 28 => return our_ip(15 downto  8);
            when 29 => return our_ip( 7 downto  0);
            -- IP dst = the probing host
            when 30 => return peer_ip(31 downto 24);
            when 31 => return peer_ip(23 downto 16);
            when 32 => return peer_ip(15 downto  8);
            when 33 => return peer_ip( 7 downto  0);

            -- ---- UDP header ----
            -- UDP src port = 1024 (HPSDR control plane)
            when 34 => return HPSDR_PORT_HI;
            when 35 => return HPSDR_PORT_LO;
            -- UDP dst port = 1024 (host's HPSDR listener)
            when 36 => return HPSDR_PORT_HI;
            when 37 => return HPSDR_PORT_LO;
            -- UDP length = 68
            when 38 => return UDP_LEN_VEC(15 downto 8);
            when 39 => return UDP_LEN_VEC( 7 downto 0);
            -- UDP checksum = 0 (no checksum)
            when 40 | 41 => return x"00";

            -- ---- HPSDR P2 General Packet payload (60 bytes) ----
            -- bytes 0..3 of payload (idx 42..45): sequence number = 0
            when 42 | 43 | 44 | 45 => return x"00";
            -- byte 4 of payload (idx 46): status (idle/active)
            when 46 => return STATUS_BYTE;
            -- bytes 5..10 of payload (idx 47..52): our MAC, network order
            when 47 => return our_mac(47 downto 40);
            when 48 => return our_mac(39 downto 32);
            when 49 => return our_mac(31 downto 24);
            when 50 => return our_mac(23 downto 16);
            when 51 => return our_mac(15 downto  8);
            when 52 => return our_mac( 7 downto  0);
            -- byte 11 of payload (idx 53): board type
            when 53 => return BOARD_TYPE;
            -- byte 12 of payload (idx 54): gateware major version
            when 54 => return GW_MAJOR;
            -- byte 13 of payload (idx 55): gateware minor / P2 tag
            when 55 => return GW_MINOR;
            -- bytes 14..59 of payload (idx 56..101): zero pad
            -- (device-specific fields; piHPSDR tolerates zeros for a
            -- minimal "device present" advertisement)
            when others => return x"00";
        end case;
    end function;

begin

    -- ----------------------------------------------------------------------
    -- Output drivers
    -- ----------------------------------------------------------------------
    tx_data_mux : process(state, tx_byte_idx, peer_mac_r, our_mac,
                          our_ip, peer_ip_r, ip_chk_r)
    begin
        if state = S_TX then
            tx_data <= disc_byte_at(to_integer(tx_byte_idx),
                                    peer_mac_r, our_mac,
                                    our_ip, peer_ip_r, ip_chk_r);
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
    host_valid  <= host_valid_r;

    -- ----------------------------------------------------------------------
    -- Registered IP checksum -- two-stage cascade.
    --
    -- Stage 1 (`ip_fixed_part_r`) folds in our_ip.  Recomputed every
    -- cycle as a pure function of our_ip; converges within one clock
    -- of any our_ip change (typically once, at DHCP-ACK).
    --
    -- Stage 2 (`ip_chk_r`) folds in peer_ip_r.  Recomputed every cycle
    -- as a pure function of ip_fixed_part_r and peer_ip_r; converges
    -- within one clock of any peer_ip_r change.  peer_ip_r only changes
    -- on rx_sop, which fires ~60 cycles before S_RX -> S_TX, so by the
    -- time the first reply byte goes out ip_chk_r is well-settled.
    --
    -- Splitting into two register stages keeps each oc_add chain at
    -- 3 levels (16-bit values) -- no Fmax pressure.
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
            state         <= S_RX;
            rx_byte_idx   <= (others => '0');
            is_discovery_r <= '0';
            peer_mac_r    <= (others => '0');
            peer_ip_r     <= (others => '0');
            host_mac_r    <= (others => '0');
            host_ip_r     <= (others => '0');
            host_valid_r  <= '0';
            tx_byte_idx   <= (others => '0');
            reply_pulse_r <= '0';
        elsif rising_edge(clock) then
            reply_pulse_r <= '0';

            case state is

            -- ----------------------------------------------------------------
            -- Receive: walk the General Packet payload byte-by-byte.  Capture
            -- the command byte at offset 4 to decide whether this is a
            -- discovery request.  On rx_eop, if it was, transition to S_TX.
            -- All other commands (start/stop/config writes/...) are silently
            -- dropped at this stage -- those will get their own consumers as
            -- HPSDR functionality grows.
            -- ----------------------------------------------------------------
            when S_RX =>
                n_byte_idx := rx_byte_idx;
                n_is_disc  := is_discovery_r;

                if rx_valid = '1' then
                    if rx_sop = '1' then
                        n_byte_idx := (others => '0');
                        n_is_disc  := '0';
                        -- Latch the probing host's addresses.  The
                        -- eth_rx_demux src_mac and ip_rx_handler src_ip
                        -- sidebands are held stable for the full frame
                        -- duration, so capturing on sop is safe.
                        peer_mac_r <= peer_mac;
                        peer_ip_r  <= peer_ip;
                    end if;

                    if n_byte_idx = to_unsigned(4, n_byte_idx'length) then
                        if rx_data = P2_CMD_DISCOVERY then
                            n_is_disc := '1';
                        end if;
                    end if;

                    if rx_eop = '1' then
                        -- Require we actually saw the command byte.  A
                        -- truncated request (< 5 bytes) is malformed; drop
                        -- regardless of n_is_disc's contents.
                        if n_is_disc = '1' and
                           n_byte_idx >= to_unsigned(4, n_byte_idx'length) then
                            tx_byte_idx <= (others => '0');
                            state       <= S_TX;
                            -- Publish committed host identity for any
                            -- downstream HPSDR producer (high-priority
                            -- status, IQ streamers, ...).  peer_*_r are
                            -- guaranteed populated by the rx_sop latch
                            -- earlier in this same packet.
                            host_mac_r   <= peer_mac_r;
                            host_ip_r    <= peer_ip_r;
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

            -- ----------------------------------------------------------------
            -- Transmit: walk tx_byte_idx 0..FRAME_BYTES-1 emitting reply
            -- bytes via the combinational tx_data mux.  tx_byte_idx advances
            -- only when tx_ready='1', which holds the sop beat stable for
            -- eem_tx_framer's 2-cycle S_HDR backpressure (same contract
            -- the other responders rely on).
            -- ----------------------------------------------------------------
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
    end process;

end architecture;
