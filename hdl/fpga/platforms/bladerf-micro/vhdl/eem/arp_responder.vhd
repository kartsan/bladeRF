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
-- arp_responder
--
-- Minimal RFC 826 ARP responder for IPv4-over-Ethernet.  Listens on the
-- eth_rx_demux ARP channel (28-byte ARP body, header already stripped) for
-- Request operations targeting OUR_IP; replies with our local MAC by
-- streaming a 42-byte ARP Reply Ethernet frame into eem_tx_framer's byte
-- input.
--
-- Validation
-- ----------
-- Drops the request (no reply generated) on any of:
--   * htype  != 0x0001            (not Ethernet)
--   * ptype  != 0x0800            (not IPv4)
--   * hlen   != 0x06              (MAC length wrong)
--   * plen   != 0x04              (IPv4 length wrong)
--   * oper   != 0x0001            (not Request)
--   * tpa    != OUR_IP            (not asking about us)
--   * body length < 28 bytes      (truncated)
--
-- The target hardware address (THA) in a Request is conventionally zero or
-- unspecified, so it is captured but not validated.  Trailing bytes past
-- the 28-byte ARP body are silently ignored: Linux pads short Ethernet
-- frames to a 60-byte minimum, so a real ARP Request arrives here as 28
-- ARP bytes followed by ~16 zero padding bytes.
--
-- Local MAC source
-- ----------------
-- `our_mac` is driven by chip_id_mac in bladerf-hosted.vhd.  Its reset
-- value is a fallback locally-administered unicast MAC, so the responder
-- can reply correctly even in the brief window before cv_chip_id_reader
-- has finished shifting out the per-board chip ID (~65 clocks).  After
-- that, our_mac switches to the chip-ID-derived value and subsequent
-- replies carry the real per-board address; the host's ARP cache simply
-- updates.
--
-- TX-side handshake
-- -----------------
-- eem_tx_framer requires the sop beat held stable until frame_in_ready
-- goes high (the framer needs 2 clocks after sop to inject its EEM
-- header bytes).  We satisfy that by keeping tx_byte_idx at 0 and the
-- output combinational on tx_byte_idx; the byte index advances only
-- when tx_ready='1'.
--
-- New requests received while we're emitting a reply are dropped: the
-- demux keeps forwarding bytes, but rx_valid is only honored in S_RX.
-- ARP probe rates from Linux are << 1 Hz so a 42-byte burst (~50 clocks
-- at 100 MHz pclk = 500 ns) never overlaps in practice.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.bladerf_p.all;

entity arp_responder is
    generic (
        -- IPv4 address we respond to ARP Requests for.  Default is the
        -- package constant; override per-instance if multiple responders
        -- ever coexist on the same fabric.
        OUR_IP : std_logic_vector(31 downto 0) := EEM_OUR_IP
    );
    port (
        clock       : in  std_logic;
        reset       : in  std_logic;

        -- Local MAC (from chip_id_mac)
        our_mac     : in  std_logic_vector(47 downto 0);

        -- ARP request byte stream in (from eth_rx_demux arp_* channel)
        rx_data     : in  std_logic_vector(7 downto 0);
        rx_valid    : in  std_logic;
        rx_sop      : in  std_logic;
        rx_eop      : in  std_logic;

        -- ARP reply byte stream out (to eem_tx_framer frame_in_* port)
        tx_data     : out std_logic_vector(7 downto 0);
        tx_valid    : out std_logic;
        tx_sop      : out std_logic;
        tx_eop      : out std_logic;
        tx_length   : out unsigned(13 downto 0);
        tx_ready    : in  std_logic;

        -- Observability: pulse on every reply emitted
        reply_pulse : out std_logic
    );
end entity;

architecture arch of arp_responder is

    -- ARP body is exactly 28 bytes.
    constant ARP_BODY_BYTES  : natural := 28;
    -- ARP reply Ethernet frame is exactly 42 bytes (14 eth + 28 arp).
    constant ARP_REPLY_BYTES : natural := 42;

    -- Expected fixed-value bytes in an IPv4 ARP Request.
    constant EXP_HTYPE_HI : std_logic_vector(7 downto 0) := x"00";
    constant EXP_HTYPE_LO : std_logic_vector(7 downto 0) := x"01";
    constant EXP_PTYPE_HI : std_logic_vector(7 downto 0) := x"08";
    constant EXP_PTYPE_LO : std_logic_vector(7 downto 0) := x"00";
    constant EXP_HLEN     : std_logic_vector(7 downto 0) := x"06";
    constant EXP_PLEN     : std_logic_vector(7 downto 0) := x"04";
    constant EXP_OPER_HI  : std_logic_vector(7 downto 0) := x"00";
    constant EXP_OPER_LO  : std_logic_vector(7 downto 0) := x"01";   -- Request

    type state_t is (S_RX, S_TX);
    signal state : state_t := S_RX;

    -- 5 bits cover rx_byte_idx 0..28.  6 bits cover tx_byte_idx 0..41.
    signal rx_byte_idx : unsigned(4 downto 0) := (others => '0');
    signal tx_byte_idx : unsigned(5 downto 0) := (others => '0');

    -- Captured request fields
    signal sender_mac_r : std_logic_vector(47 downto 0) := (others => '0');
    signal sender_ip_r  : std_logic_vector(31 downto 0) := (others => '0');
    signal target_ip_r  : std_logic_vector(31 downto 0) := (others => '0');

    -- Parsing flags
    signal parsing      : std_logic := '0';
    signal bad_field    : std_logic := '0';

    signal reply_pulse_r : std_logic := '0';

    -- Build a single reply byte by index 0..41 from captured/constant data.
    function arp_reply_byte (
        idx        : natural;
        sender_mac : std_logic_vector(47 downto 0);
        sender_ip  : std_logic_vector(31 downto 0);
        our_mac    : std_logic_vector(47 downto 0);
        our_ip     : std_logic_vector(31 downto 0)
    ) return std_logic_vector is
    begin
        case idx is
            -- Eth dst MAC: the original requester (= our reply's target)
            when  0 => return sender_mac(47 downto 40);
            when  1 => return sender_mac(39 downto 32);
            when  2 => return sender_mac(31 downto 24);
            when  3 => return sender_mac(23 downto 16);
            when  4 => return sender_mac(15 downto  8);
            when  5 => return sender_mac( 7 downto  0);
            -- Eth src MAC: us
            when  6 => return our_mac(47 downto 40);
            when  7 => return our_mac(39 downto 32);
            when  8 => return our_mac(31 downto 24);
            when  9 => return our_mac(23 downto 16);
            when 10 => return our_mac(15 downto  8);
            when 11 => return our_mac( 7 downto  0);
            -- Eth type = 0x0806 (ARP)
            when 12 => return x"08";
            when 13 => return x"06";
            -- ARP htype = 0x0001 (Ethernet)
            when 14 => return x"00";
            when 15 => return x"01";
            -- ARP ptype = 0x0800 (IPv4)
            when 16 => return x"08";
            when 17 => return x"00";
            -- ARP hlen = 6, plen = 4
            when 18 => return x"06";
            when 19 => return x"04";
            -- ARP oper = 0x0002 (Reply)
            when 20 => return x"00";
            when 21 => return x"02";
            -- ARP SHA: us
            when 22 => return our_mac(47 downto 40);
            when 23 => return our_mac(39 downto 32);
            when 24 => return our_mac(31 downto 24);
            when 25 => return our_mac(23 downto 16);
            when 26 => return our_mac(15 downto  8);
            when 27 => return our_mac( 7 downto  0);
            -- ARP SPA: our IP
            when 28 => return our_ip(31 downto 24);
            when 29 => return our_ip(23 downto 16);
            when 30 => return our_ip(15 downto  8);
            when 31 => return our_ip( 7 downto  0);
            -- ARP THA: the original requester
            when 32 => return sender_mac(47 downto 40);
            when 33 => return sender_mac(39 downto 32);
            when 34 => return sender_mac(31 downto 24);
            when 35 => return sender_mac(23 downto 16);
            when 36 => return sender_mac(15 downto  8);
            when 37 => return sender_mac( 7 downto  0);
            -- ARP TPA: the original requester's IP
            when 38 => return sender_ip(31 downto 24);
            when 39 => return sender_ip(23 downto 16);
            when 40 => return sender_ip(15 downto  8);
            when 41 => return sender_ip( 7 downto  0);
            when others => return x"00";
        end case;
    end function;

begin

    -- Combinational outputs derived from current state.  This keeps the
    -- sop beat stable while tx_ready is low (eem_tx_framer's required
    -- 2-cycle backpressure at sop).
    tx_data  <= arp_reply_byte(to_integer(tx_byte_idx),
                               sender_mac_r, sender_ip_r,
                               our_mac, OUR_IP)
                  when state = S_TX else (others => '0');
    tx_valid <= '1' when state = S_TX else '0';
    tx_sop   <= '1' when (state = S_TX and tx_byte_idx = 0) else '0';
    tx_eop   <= '1' when (state = S_TX and tx_byte_idx = to_unsigned(ARP_REPLY_BYTES-1, tx_byte_idx'length))
                       else '0';
    tx_length    <= to_unsigned(ARP_REPLY_BYTES, tx_length'length);
    reply_pulse  <= reply_pulse_r;

    fsm : process (clock, reset)
        variable n_byte_idx     : unsigned(4 downto 0);
        variable n_bad          : std_logic;
        variable n_sender_mac   : std_logic_vector(47 downto 0);
        variable n_sender_ip    : std_logic_vector(31 downto 0);
        variable n_target_ip    : std_logic_vector(31 downto 0);
        variable n_parsing      : std_logic;
        variable will_process   : boolean;
    begin
        if reset = '1' then
            state          <= S_RX;
            rx_byte_idx    <= (others => '0');
            tx_byte_idx    <= (others => '0');
            sender_mac_r   <= (others => '0');
            sender_ip_r    <= (others => '0');
            target_ip_r    <= (others => '0');
            parsing        <= '0';
            bad_field      <= '0';
            reply_pulse_r  <= '0';
        elsif rising_edge(clock) then
            reply_pulse_r <= '0';

            case state is

            -- --------------------------------------------------------------
            -- Listen and parse.  On sop we reset all parsing state, then
            -- (still in this same cycle) process byte 0 with the fresh
            -- state.  On subsequent valid bytes we keep parsing while
            -- `parsing='1'`.  After eop with a fully-valid Request whose
            -- TPA matches OUR_IP, transition to S_TX.
            -- --------------------------------------------------------------
            when S_RX =>
                -- Default: hold current registers.
                n_byte_idx   := rx_byte_idx;
                n_bad        := bad_field;
                n_sender_mac := sender_mac_r;
                n_sender_ip  := sender_ip_r;
                n_target_ip  := target_ip_r;
                n_parsing    := parsing;

                if rx_valid = '1' then
                    if rx_sop = '1' then
                        -- New frame -- discard any in-progress (we shouldn't
                        -- be mid-parse here, but be defensive) and start
                        -- fresh from byte 0.
                        n_byte_idx   := (others => '0');
                        n_bad        := '0';
                        n_sender_mac := (others => '0');
                        n_sender_ip  := (others => '0');
                        n_target_ip  := (others => '0');
                        n_parsing    := '1';
                    end if;

                    will_process := (n_parsing = '1');

                    -- Only parse/count the first 28 bytes; any trailing
                    -- bytes are Linux's Ethernet-minimum padding and are
                    -- silently dropped here.  Saturating the counter at
                    -- 28 also prevents the 5-bit n_byte_idx from wrapping
                    -- 31 -> 0 on long padded frames.
                    if will_process
                       and n_byte_idx < to_unsigned(ARP_BODY_BYTES, n_byte_idx'length)
                    then
                        case to_integer(n_byte_idx) is
                            when 0 => if rx_data /= EXP_HTYPE_HI then n_bad := '1'; end if;
                            when 1 => if rx_data /= EXP_HTYPE_LO then n_bad := '1'; end if;
                            when 2 => if rx_data /= EXP_PTYPE_HI then n_bad := '1'; end if;
                            when 3 => if rx_data /= EXP_PTYPE_LO then n_bad := '1'; end if;
                            when 4 => if rx_data /= EXP_HLEN     then n_bad := '1'; end if;
                            when 5 => if rx_data /= EXP_PLEN     then n_bad := '1'; end if;
                            when 6 => if rx_data /= EXP_OPER_HI  then n_bad := '1'; end if;
                            when 7 => if rx_data /= EXP_OPER_LO  then n_bad := '1'; end if;
                            when 8 | 9 | 10 | 11 | 12 | 13 =>
                                n_sender_mac := n_sender_mac(39 downto 0) & rx_data;
                            when 14 | 15 | 16 | 17 =>
                                n_sender_ip  := n_sender_ip(23 downto 0)  & rx_data;
                            when 18 | 19 | 20 | 21 | 22 | 23 =>
                                null;  -- THA: ignored on Request
                            when 24 | 25 | 26 | 27 =>
                                n_target_ip  := n_target_ip(23 downto 0)  & rx_data;
                            when others =>
                                null;  -- unreachable: guarded above
                        end case;
                        n_byte_idx := n_byte_idx + 1;
                    end if;

                    if rx_eop = '1' then
                        n_parsing := '0';
                        if will_process
                           and n_bad = '0'
                           and n_byte_idx = to_unsigned(ARP_BODY_BYTES, n_byte_idx'length)
                           and n_target_ip = OUR_IP
                        then
                            -- Valid Request for us -- prep TX.
                            tx_byte_idx <= (others => '0');
                            state       <= S_TX;
                        end if;
                    end if;
                end if;

                rx_byte_idx  <= n_byte_idx;
                bad_field    <= n_bad;
                sender_mac_r <= n_sender_mac;
                sender_ip_r  <= n_sender_ip;
                target_ip_r  <= n_target_ip;
                parsing      <= n_parsing;

            -- --------------------------------------------------------------
            -- Stream the 42-byte reply through eem_tx_framer.  The output
            -- is combinational on tx_byte_idx, so holding tx_byte_idx at 0
            -- (while tx_ready is low for the framer's 2-cycle header
            -- injection) keeps the sop beat presented stably.
            -- --------------------------------------------------------------
            when S_TX =>
                if tx_ready = '1' then
                    if tx_byte_idx = to_unsigned(ARP_REPLY_BYTES-1, tx_byte_idx'length) then
                        reply_pulse_r <= '1';
                        state         <= S_RX;
                    else
                        tx_byte_idx <= tx_byte_idx + 1;
                    end if;
                end if;

            end case;
        end if;
    end process;

end architecture;
