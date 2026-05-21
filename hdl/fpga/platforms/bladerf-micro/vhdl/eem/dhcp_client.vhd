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
-- dhcp_client
--
-- Minimal RFC 2131 DHCPv4 client: DISCOVER -> OFFER -> REQUEST -> ACK and
-- then sit BOUND.  No renewal / rebinding (T1/T2 ignored for now); reset
-- restarts the whole acquisition.
--
-- TX path
-- -------
-- A single 342-byte Ethernet frame template covers both DHCPDISCOVER and
-- DHCPREQUEST.  Most of it (Eth + IP + UDP + the 240-byte BOOTP fixed
-- header + the magic cookie) is byte-identical between the two messages;
-- only the options section (offsets 282..297) differs:
--
--   DHCPDISCOVER options:
--     282 .35 .01 .01     (Option 53: DHCP Message Type = DISCOVER)
--     285 .FF             (End)
--     286+                (zero padding to 300-byte payload)
--
--   DHCPREQUEST options:
--     282 .35 .01 .03     (Option 53: DHCP Message Type = REQUEST)
--     285 .32 .04 yyyy    (Option 50: Requested IP Address = offered yiaddr)
--     291 .36 .04 ssss    (Option 54: Server Identifier      = OFFER's source)
--     297 .FF             (End)
--     298+                (zero padding)
--
-- IP checksum precomputed at elaboration: every IP-header field is a
-- compile-time constant (src=0.0.0.0, dst=255.255.255.255, total_length=328
-- always), so the runtime byte mux is a plain case statement with zero
-- arithmetic.
--
-- UDP checksum = 0 (legal "no checksum" in IPv4).  Saves us the trouble of
-- computing the IPv4 pseudo-header sum across a 300-byte payload.
--
-- Broadcast bit (BOOTP flags = 0x8000) is set so the server broadcasts
-- OFFER/ACK back to us; otherwise the server would try to unicast to our
-- offered IP, which won't work since our NIC doesn't yet have an IP.
--
-- RX path
-- -------
-- DHCP replies arrive header-stripped of Ethernet (14B), IP (20B), and
-- UDP (8B), so byte 0 of rx_* is the BOOTP `op` byte.  A two-phase parser
-- walks the 240-byte BOOTP fixed header (validating op/htype/hlen, xid,
-- chaddr, and the magic cookie; capturing yiaddr) and then a TLV
-- sub-parser walks the options.  We capture Option 53 (message type) and
-- Option 54 (server identifier); other options are skipped by their
-- declared length.
--
-- On rx_eop, if the message type is OFFER and we're in S_WAIT_OFFER, the
-- FSM advances to S_TX_REQUEST.  If it's ACK and we're in S_WAIT_ACK, we
-- latch our_ip <- yiaddr and advance to S_BOUND, raising our_ip_valid.
--
-- Timeouts
-- --------
-- BOOT_DELAY_TICKS  : delay after reset before first DISCOVER (so the
--                     host's dnsmasq has time to start, and chip_id_mac
--                     has settled).  Default 2 s @ 100 MHz.
-- TIMEOUT_TICKS     : OFFER / ACK wait timeout.  On timeout the FSM
--                     restarts from S_TX_DISCOVER.  Default 4 s.
--
-- xid
-- ---
-- Latched as our_mac(31..0) at the first S_INIT -> S_TX_DISCOVER
-- transition and held for the whole session.  Different per board (since
-- our_mac is chip-ID-derived) so two bladeRFs on the same network don't
-- collide.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.bladerf_p.all;

entity dhcp_client is
    generic (
        -- Delay after reset before first DISCOVER, in clock ticks.
        -- Default 2 s @ 100 MHz fx3_pclk_pll.
        BOOT_DELAY_TICKS : natural := 200_000_000;

        -- OFFER / ACK wait timeout, in clock ticks.  On timeout restart
        -- from DISCOVER.  Default 4 s.
        TIMEOUT_TICKS    : natural := 400_000_000
    );
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        -- Local MAC (from chip_id_mac).
        our_mac       : in  std_logic_vector(47 downto 0);

        -- DHCP byte stream input (from udp_rx_handler dhcp_* channel,
        -- header-stripped of Eth/IP/UDP).
        rx_data       : in  std_logic_vector(7 downto 0);
        rx_valid      : in  std_logic;
        rx_sop        : in  std_logic;
        rx_eop        : in  std_logic;
        rx_length     : in  std_logic_vector(13 downto 0);

        -- TX byte stream out (to tx_arbiter port D -> eem_tx_framer).
        tx_data       : out std_logic_vector(7 downto 0);
        tx_valid      : out std_logic;
        tx_sop        : out std_logic;
        tx_eop        : out std_logic;
        tx_length     : out unsigned(13 downto 0);
        tx_ready      : in  std_logic;

        -- Leased L3 outputs.  our_ip_valid rises when DHCPACK arrives and
        -- stays high until reset.  server_ip is the DHCP server's IP
        -- (Option 54), needed for unicast renewal once implemented.
        our_ip        : out std_logic_vector(31 downto 0);
        our_ip_valid  : out std_logic;
        server_ip     : out std_logic_vector(31 downto 0);

        -- Observability
        send_pulse    : out std_logic;   -- one cycle per emitted TX packet
        bound_pulse   : out std_logic    -- one cycle when entering S_BOUND
    );
end entity;

architecture arch of dhcp_client is

    -- ------------------------------------------------------------------
    -- Geometry
    -- ------------------------------------------------------------------
    constant ETH_HDR_BYTES : natural := 14;
    constant IP_HDR_BYTES  : natural := 20;
    constant UDP_HDR_BYTES : natural := 8;
    constant DHCP_PAYLOAD  : natural := 300;
    constant FRAME_BYTES   : natural := ETH_HDR_BYTES + IP_HDR_BYTES
                                      + UDP_HDR_BYTES + DHCP_PAYLOAD;        -- 342
    constant IP_TOTAL_LEN  : natural := IP_HDR_BYTES + UDP_HDR_BYTES + DHCP_PAYLOAD; -- 328
    constant UDP_LEN       : natural := UDP_HDR_BYTES + DHCP_PAYLOAD;        -- 308

    constant IP_TLEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(IP_TOTAL_LEN, 16));
    constant UDP_LEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(UDP_LEN, 16));

    -- BOOTP magic cookie 99.130.83.99
    constant MAGIC0 : std_logic_vector(7 downto 0) := x"63";
    constant MAGIC1 : std_logic_vector(7 downto 0) := x"82";
    constant MAGIC2 : std_logic_vector(7 downto 0) := x"53";
    constant MAGIC3 : std_logic_vector(7 downto 0) := x"63";

    -- DHCP message types
    constant DHCPDISCOVER : std_logic_vector(7 downto 0) := x"01";
    constant DHCPOFFER    : std_logic_vector(7 downto 0) := x"02";
    constant DHCPREQUEST  : std_logic_vector(7 downto 0) := x"03";
    constant DHCPACK      : std_logic_vector(7 downto 0) := x"05";

    -- ------------------------------------------------------------------
    -- One's-complement 16-bit addition (matches icmp_responder /
    -- udp_tx_injector).
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

    -- IPv4 header checksum: every contributing word is a compile-time
    -- constant for DHCP TX (src=0.0.0.0, dst=255.255.255.255, flags=DF).
    function calc_ip_chk return std_logic_vector is
        variable s : unsigned(15 downto 0);
    begin
        s := oc_add(to_unsigned(16#4500#, 16),
                    to_unsigned(IP_TOTAL_LEN, 16));        -- V/IHL/DSCP + total_len
        s := oc_add(s, to_unsigned(0, 16));                -- ID
        s := oc_add(s, to_unsigned(16#4000#, 16));         -- DF + frag offset
        s := oc_add(s, to_unsigned(16#4011#, 16));         -- TTL=64, proto=UDP
        s := oc_add(s, to_unsigned(0, 16));                -- src hi
        s := oc_add(s, to_unsigned(0, 16));                -- src lo
        s := oc_add(s, to_unsigned(16#FFFF#, 16));         -- dst hi
        s := oc_add(s, to_unsigned(16#FFFF#, 16));         -- dst lo
        return std_logic_vector(not s);
    end function;

    constant IP_CHK : std_logic_vector(15 downto 0) := calc_ip_chk;

    -- ------------------------------------------------------------------
    -- Per-byte mux for the 342-byte TX frame.  Pure function of inputs.
    -- ------------------------------------------------------------------
    function dhcp_byte_at(
        idx       : natural;
        our_mac   : std_logic_vector(47 downto 0);
        xid       : std_logic_vector(31 downto 0);
        yiaddr    : std_logic_vector(31 downto 0);
        server_id : std_logic_vector(31 downto 0);
        msg_type  : std_logic_vector(7 downto 0)
    ) return std_logic_vector is
        variable is_request : boolean;
    begin
        is_request := (msg_type = DHCPREQUEST);

        case idx is
            -- Eth dst MAC = broadcast
            when 0 | 1 | 2 | 3 | 4 | 5 => return x"FF";
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
            -- IP V/IHL=0x45, DSCP=0x00
            when 14 => return x"45";
            when 15 => return x"00";
            -- IP total length
            when 16 => return IP_TLEN_VEC(15 downto 8);
            when 17 => return IP_TLEN_VEC( 7 downto 0);
            -- IP identification = 0
            when 18 | 19 => return x"00";
            -- IP flags + frag offset = 0x4000 (DF)
            when 20 => return x"40";
            when 21 => return x"00";
            -- IP TTL=64, protocol=UDP
            when 22 => return x"40";
            when 23 => return x"11";
            -- IP header checksum (precomputed)
            when 24 => return IP_CHK(15 downto 8);
            when 25 => return IP_CHK( 7 downto 0);
            -- IP src = 0.0.0.0
            when 26 | 27 | 28 | 29 => return x"00";
            -- IP dst = 255.255.255.255
            when 30 | 31 | 32 | 33 => return x"FF";
            -- UDP src port = 68 (BOOTPC)
            when 34 => return x"00";
            when 35 => return x"44";
            -- UDP dst port = 67 (BOOTPS)
            when 36 => return x"00";
            when 37 => return x"43";
            -- UDP length
            when 38 => return UDP_LEN_VEC(15 downto 8);
            when 39 => return UDP_LEN_VEC( 7 downto 0);
            -- UDP checksum = 0 (no checksum)
            when 40 | 41 => return x"00";
            -- BOOTP op = BOOTREQUEST (0x01)
            when 42 => return x"01";
            -- BOOTP htype = Ethernet (0x01)
            when 43 => return x"01";
            -- BOOTP hlen = 6
            when 44 => return x"06";
            -- BOOTP hops = 0
            when 45 => return x"00";
            -- xid (32-bit, big-endian)
            when 46 => return xid(31 downto 24);
            when 47 => return xid(23 downto 16);
            when 48 => return xid(15 downto  8);
            when 49 => return xid( 7 downto  0);
            -- secs = 0
            when 50 | 51 => return x"00";
            -- flags = 0x8000 (broadcast bit set)
            when 52 => return x"80";
            when 53 => return x"00";
            -- ciaddr (4), yiaddr (4), siaddr (4), giaddr (4) all zero
            when 54 | 55 | 56 | 57 | 58 | 59 | 60 | 61
               | 62 | 63 | 64 | 65 | 66 | 67 | 68 | 69 => return x"00";
            -- chaddr first 6 bytes = our MAC
            when 70 => return our_mac(47 downto 40);
            when 71 => return our_mac(39 downto 32);
            when 72 => return our_mac(31 downto 24);
            when 73 => return our_mac(23 downto 16);
            when 74 => return our_mac(15 downto  8);
            when 75 => return our_mac( 7 downto  0);
            -- BOOTP magic cookie
            when 278 => return MAGIC0;
            when 279 => return MAGIC1;
            when 280 => return MAGIC2;
            when 281 => return MAGIC3;
            -- Option 53: DHCP Message Type
            when 282 => return x"35";
            when 283 => return x"01";
            when 284 => return msg_type;
            -- 285..297: REQUEST-only options (DISCOVER puts End at 285 and
            -- zero-pads the rest).
            when 285 =>
                if is_request then return x"32"; else return x"FF"; end if;
            when 286 =>
                if is_request then return x"04"; else return x"00"; end if;
            when 287 =>
                if is_request then return yiaddr(31 downto 24); else return x"00"; end if;
            when 288 =>
                if is_request then return yiaddr(23 downto 16); else return x"00"; end if;
            when 289 =>
                if is_request then return yiaddr(15 downto  8); else return x"00"; end if;
            when 290 =>
                if is_request then return yiaddr( 7 downto  0); else return x"00"; end if;
            when 291 =>
                if is_request then return x"36"; else return x"00"; end if;
            when 292 =>
                if is_request then return x"04"; else return x"00"; end if;
            when 293 =>
                if is_request then return server_id(31 downto 24); else return x"00"; end if;
            when 294 =>
                if is_request then return server_id(23 downto 16); else return x"00"; end if;
            when 295 =>
                if is_request then return server_id(15 downto  8); else return x"00"; end if;
            when 296 =>
                if is_request then return server_id( 7 downto  0); else return x"00"; end if;
            when 297 =>
                if is_request then return x"FF"; else return x"00"; end if;
            -- Everything else (chaddr padding 76..85, sname 86..149,
            -- file 150..277, option padding 298..341) = zero.
            when others => return x"00";
        end case;
    end function;

    -- ------------------------------------------------------------------
    -- State
    -- ------------------------------------------------------------------
    type state_t is (S_INIT, S_TX_DISCOVER, S_WAIT_OFFER,
                     S_TX_REQUEST, S_WAIT_ACK, S_BOUND);
    signal state : state_t := S_INIT;

    -- 29 bits cover TIMEOUT_TICKS = 400M.
    signal delay_count    : unsigned(28 downto 0) := (others => '0');

    -- TX byte index walks 0 .. FRAME_BYTES-1 = 0..341.  9 bits.
    signal tx_byte_idx    : unsigned(8 downto 0)  := (others => '0');

    -- Captured / latched session state
    signal xid_r          : std_logic_vector(31 downto 0) := (others => '0');
    signal offered_ip_r   : std_logic_vector(31 downto 0) := (others => '0');
    signal server_id_r    : std_logic_vector(31 downto 0) := (others => '0');
    signal our_ip_r       : std_logic_vector(31 downto 0) := (others => '0');
    signal our_ip_valid_r : std_logic                     := '0';
    signal tx_msg_type_r  : std_logic_vector(7 downto 0)  := DHCPDISCOVER;

    signal send_pulse_r   : std_logic := '0';
    signal bound_pulse_r  : std_logic := '0';

    -- ------------------------------------------------------------------
    -- RX parser sub-state
    -- ------------------------------------------------------------------
    type rx_state_t is (RX_FIXED, RX_OPT_CODE, RX_OPT_LEN, RX_OPT_VAL);
    signal rx_state         : rx_state_t            := RX_FIXED;
    signal rx_byte_count    : unsigned(13 downto 0) := (others => '0');
    signal rx_bad           : std_logic             := '0';
    signal rx_yiaddr_r      : std_logic_vector(31 downto 0) := (others => '0');
    signal rx_opt_code_r    : std_logic_vector(7 downto 0)  := (others => '0');
    signal rx_opt_remain    : unsigned(7 downto 0)          := (others => '0');
    signal rx_msg_type_r    : std_logic_vector(7 downto 0)  := (others => '0');
    signal rx_has_msg_type  : std_logic := '0';
    signal rx_server_id_r   : std_logic_vector(31 downto 0) := (others => '0');

    -- Per-eop validated capture, consumed by main FSM.
    signal rx_done_pulse    : std_logic                     := '0';
    signal rx_done_msg_type : std_logic_vector(7 downto 0)  := (others => '0');
    signal rx_done_yiaddr   : std_logic_vector(31 downto 0) := (others => '0');
    signal rx_done_srvid    : std_logic_vector(31 downto 0) := (others => '0');

begin

    -- ------------------------------------------------------------------
    -- Combinational TX outputs.  Same sop-hold-until-tx_ready pattern as
    -- arp_responder / udp_tx_injector.
    -- ------------------------------------------------------------------
    tx_data <= dhcp_byte_at(to_integer(tx_byte_idx),
                            our_mac, xid_r,
                            offered_ip_r, server_id_r,
                            tx_msg_type_r)
                 when (state = S_TX_DISCOVER or state = S_TX_REQUEST)
                 else (others => '0');
    tx_valid <= '1' when (state = S_TX_DISCOVER or state = S_TX_REQUEST)
                else '0';
    tx_sop   <= '1' when ((state = S_TX_DISCOVER or state = S_TX_REQUEST)
                          and tx_byte_idx = 0)
                else '0';
    tx_eop   <= '1' when ((state = S_TX_DISCOVER or state = S_TX_REQUEST)
                          and tx_byte_idx = to_unsigned(FRAME_BYTES-1, tx_byte_idx'length))
                else '0';
    tx_length <= to_unsigned(FRAME_BYTES, tx_length'length);

    our_ip       <= our_ip_r;
    our_ip_valid <= our_ip_valid_r;
    server_ip    <= server_id_r;
    send_pulse   <= send_pulse_r;
    bound_pulse  <= bound_pulse_r;

    -- ------------------------------------------------------------------
    -- RX parser: walks the BOOTP fixed header (0..239) then the options
    -- TLV stream (240..end).  Validates op/htype/hlen, xid, chaddr,
    -- magic cookie; captures yiaddr (bytes 16..19), Option 53 (msg type),
    -- and Option 54 (server identifier).  At eop, if no validation
    -- failure and a msg_type was captured, raise rx_done_pulse and
    -- snapshot the captured fields.
    -- ------------------------------------------------------------------
    rx_parser : process(clock, reset)
        variable n_state    : rx_state_t;
        variable n_bcount   : unsigned(13 downto 0);
        variable n_bad      : std_logic;
        variable n_yiaddr   : std_logic_vector(31 downto 0);
        variable n_opt_code : std_logic_vector(7 downto 0);
        variable n_opt_rem  : unsigned(7 downto 0);
        variable n_msg_type : std_logic_vector(7 downto 0);
        variable n_has_msg  : std_logic;
        variable n_srvid    : std_logic_vector(31 downto 0);
    begin
        if reset = '1' then
            rx_state         <= RX_FIXED;
            rx_byte_count    <= (others => '0');
            rx_bad           <= '0';
            rx_yiaddr_r      <= (others => '0');
            rx_opt_code_r    <= (others => '0');
            rx_opt_remain    <= (others => '0');
            rx_msg_type_r    <= (others => '0');
            rx_has_msg_type  <= '0';
            rx_server_id_r   <= (others => '0');
            rx_done_pulse    <= '0';
            rx_done_msg_type <= (others => '0');
            rx_done_yiaddr   <= (others => '0');
            rx_done_srvid    <= (others => '0');
        elsif rising_edge(clock) then
            rx_done_pulse <= '0';

            n_state    := rx_state;
            n_bcount   := rx_byte_count;
            n_bad      := rx_bad;
            n_yiaddr   := rx_yiaddr_r;
            n_opt_code := rx_opt_code_r;
            n_opt_rem  := rx_opt_remain;
            n_msg_type := rx_msg_type_r;
            n_has_msg  := rx_has_msg_type;
            n_srvid    := rx_server_id_r;

            if rx_valid = '1' then
                if rx_sop = '1' then
                    n_state    := RX_FIXED;
                    n_bcount   := (others => '0');
                    n_bad      := '0';
                    n_yiaddr   := (others => '0');
                    n_opt_code := (others => '0');
                    n_opt_rem  := (others => '0');
                    n_msg_type := (others => '0');
                    n_has_msg  := '0';
                    n_srvid    := (others => '0');
                end if;

                case n_state is

                when RX_FIXED =>
                    case to_integer(n_bcount) is
                        when 0  => if rx_data /= x"02" then n_bad := '1'; end if; -- op = BOOTREPLY
                        when 1  => if rx_data /= x"01" then n_bad := '1'; end if; -- htype
                        when 2  => if rx_data /= x"06" then n_bad := '1'; end if; -- hlen
                        when 4  => if rx_data /= xid_r(31 downto 24) then n_bad := '1'; end if;
                        when 5  => if rx_data /= xid_r(23 downto 16) then n_bad := '1'; end if;
                        when 6  => if rx_data /= xid_r(15 downto  8) then n_bad := '1'; end if;
                        when 7  => if rx_data /= xid_r( 7 downto  0) then n_bad := '1'; end if;
                        when 16 => n_yiaddr(31 downto 24) := rx_data;
                        when 17 => n_yiaddr(23 downto 16) := rx_data;
                        when 18 => n_yiaddr(15 downto  8) := rx_data;
                        when 19 => n_yiaddr( 7 downto  0) := rx_data;
                        when 28 => if rx_data /= our_mac(47 downto 40) then n_bad := '1'; end if;
                        when 29 => if rx_data /= our_mac(39 downto 32) then n_bad := '1'; end if;
                        when 30 => if rx_data /= our_mac(31 downto 24) then n_bad := '1'; end if;
                        when 31 => if rx_data /= our_mac(23 downto 16) then n_bad := '1'; end if;
                        when 32 => if rx_data /= our_mac(15 downto  8) then n_bad := '1'; end if;
                        when 33 => if rx_data /= our_mac( 7 downto  0) then n_bad := '1'; end if;
                        when 236 => if rx_data /= MAGIC0 then n_bad := '1'; end if;
                        when 237 => if rx_data /= MAGIC1 then n_bad := '1'; end if;
                        when 238 => if rx_data /= MAGIC2 then n_bad := '1'; end if;
                        when 239 => if rx_data /= MAGIC3 then n_bad := '1'; end if;
                        when others => null;
                    end case;
                    if n_bcount = to_unsigned(239, n_bcount'length) then
                        n_state := RX_OPT_CODE;
                    end if;

                when RX_OPT_CODE =>
                    if rx_data = x"00" then
                        -- Pad, single byte; stay in RX_OPT_CODE.
                        null;
                    elsif rx_data = x"FF" then
                        -- End of options.  Remain in RX_OPT_CODE; subsequent
                        -- bytes (if any -- usually zero padding) are ignored
                        -- and the parser idles here until rx_eop.
                        null;
                    else
                        n_opt_code := rx_data;
                        n_state    := RX_OPT_LEN;
                    end if;

                when RX_OPT_LEN =>
                    n_opt_rem := unsigned(rx_data);
                    if unsigned(rx_data) = 0 then
                        -- Zero-length option (uncommon); back to code.
                        n_state := RX_OPT_CODE;
                    else
                        -- Reset 4-byte shift register if this is the server-ID
                        -- option, so a malformed prior option can't leak into
                        -- our capture.
                        if rx_data = x"04" and n_opt_code = x"36" then
                            n_srvid := (others => '0');
                        end if;
                        n_state := RX_OPT_VAL;
                    end if;

                when RX_OPT_VAL =>
                    if n_opt_code = x"35" then
                        -- Option 53: DHCP Message Type (1 byte expected).
                        n_msg_type := rx_data;
                        n_has_msg  := '1';
                    elsif n_opt_code = x"36" then
                        -- Option 54: Server Identifier (4 bytes expected).
                        -- Shift in MSB-first.
                        n_srvid := n_srvid(23 downto 0) & rx_data;
                    end if;
                    -- Other options are read for length but their bytes
                    -- discarded.
                    if n_opt_rem = to_unsigned(1, n_opt_rem'length) then
                        n_state := RX_OPT_CODE;
                    end if;
                    n_opt_rem := n_opt_rem - 1;

                end case;

                n_bcount := n_bcount + 1;

                if rx_eop = '1' then
                    if n_bad = '0' and n_has_msg = '1' then
                        rx_done_pulse    <= '1';
                        rx_done_msg_type <= n_msg_type;
                        rx_done_yiaddr   <= n_yiaddr;
                        rx_done_srvid    <= n_srvid;
                    end if;
                    -- Reset parser state for the next frame.
                    n_state    := RX_FIXED;
                    n_bcount   := (others => '0');
                    n_bad      := '0';
                    n_yiaddr   := (others => '0');
                    n_opt_code := (others => '0');
                    n_opt_rem  := (others => '0');
                    n_msg_type := (others => '0');
                    n_has_msg  := '0';
                    n_srvid    := (others => '0');
                end if;
            end if;

            rx_state         <= n_state;
            rx_byte_count    <= n_bcount;
            rx_bad           <= n_bad;
            rx_yiaddr_r      <= n_yiaddr;
            rx_opt_code_r    <= n_opt_code;
            rx_opt_remain    <= n_opt_rem;
            rx_msg_type_r    <= n_msg_type;
            rx_has_msg_type  <= n_has_msg;
            rx_server_id_r   <= n_srvid;
        end if;
    end process rx_parser;

    -- ------------------------------------------------------------------
    -- Main FSM: walks DISCOVER -> WAIT_OFFER -> REQUEST -> WAIT_ACK -> BOUND.
    -- Delay counter doubles for the boot delay and OFFER/ACK timeouts.
    -- ------------------------------------------------------------------
    fsm : process(clock, reset)
    begin
        if reset = '1' then
            state          <= S_INIT;
            delay_count    <= to_unsigned(BOOT_DELAY_TICKS, delay_count'length);
            tx_byte_idx    <= (others => '0');
            xid_r          <= (others => '0');
            offered_ip_r   <= (others => '0');
            server_id_r    <= (others => '0');
            our_ip_r       <= (others => '0');
            our_ip_valid_r <= '0';
            tx_msg_type_r  <= DHCPDISCOVER;
            send_pulse_r   <= '0';
            bound_pulse_r  <= '0';
        elsif rising_edge(clock) then
            send_pulse_r  <= '0';
            bound_pulse_r <= '0';

            case state is

            when S_INIT =>
                if delay_count = to_unsigned(0, delay_count'length) then
                    -- Snapshot xid for the whole acquisition session.
                    xid_r         <= our_mac(31 downto 0);
                    tx_msg_type_r <= DHCPDISCOVER;
                    tx_byte_idx   <= (others => '0');
                    state         <= S_TX_DISCOVER;
                else
                    delay_count <= delay_count - 1;
                end if;

            when S_TX_DISCOVER =>
                if tx_ready = '1' then
                    if tx_byte_idx = to_unsigned(FRAME_BYTES-1, tx_byte_idx'length) then
                        send_pulse_r <= '1';
                        delay_count  <= to_unsigned(TIMEOUT_TICKS, delay_count'length);
                        state        <= S_WAIT_OFFER;
                    else
                        tx_byte_idx <= tx_byte_idx + 1;
                    end if;
                end if;

            when S_WAIT_OFFER =>
                if rx_done_pulse = '1' and rx_done_msg_type = DHCPOFFER then
                    offered_ip_r  <= rx_done_yiaddr;
                    server_id_r   <= rx_done_srvid;
                    tx_msg_type_r <= DHCPREQUEST;
                    tx_byte_idx   <= (others => '0');
                    state         <= S_TX_REQUEST;
                elsif delay_count = to_unsigned(0, delay_count'length) then
                    -- OFFER didn't arrive; restart from DISCOVER.
                    tx_msg_type_r <= DHCPDISCOVER;
                    tx_byte_idx   <= (others => '0');
                    state         <= S_TX_DISCOVER;
                else
                    delay_count <= delay_count - 1;
                end if;

            when S_TX_REQUEST =>
                if tx_ready = '1' then
                    if tx_byte_idx = to_unsigned(FRAME_BYTES-1, tx_byte_idx'length) then
                        send_pulse_r <= '1';
                        delay_count  <= to_unsigned(TIMEOUT_TICKS, delay_count'length);
                        state        <= S_WAIT_ACK;
                    else
                        tx_byte_idx <= tx_byte_idx + 1;
                    end if;
                end if;

            when S_WAIT_ACK =>
                if rx_done_pulse = '1' and rx_done_msg_type = DHCPACK then
                    our_ip_r       <= rx_done_yiaddr;
                    our_ip_valid_r <= '1';
                    -- server_id was already captured at OFFER; refresh just
                    -- in case the ACK comes from a different relay.
                    server_id_r    <= rx_done_srvid;
                    bound_pulse_r  <= '1';
                    state          <= S_BOUND;
                elsif delay_count = to_unsigned(0, delay_count'length) then
                    -- ACK didn't arrive; restart the whole acquisition.
                    tx_msg_type_r <= DHCPDISCOVER;
                    tx_byte_idx   <= (others => '0');
                    state         <= S_TX_DISCOVER;
                else
                    delay_count <= delay_count - 1;
                end if;

            when S_BOUND =>
                -- Stay here forever (no renewal yet).  our_ip_valid stays
                -- high until reset.
                null;

            end case;
        end if;
    end process fsm;

end architecture;
