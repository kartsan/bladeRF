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
-- ip_rx_handler
--
-- Walks the 20-byte IPv4 header on every packet coming out of eth_rx_demux's
-- ip channel and routes the payload to the per-protocol output channel.
-- Currently routes:
--
--   protocol = 0x01 (ICMP) -> icmp_* channel
--   protocol = 0x11 (UDP)  -> udp_*  channel
--   anything else          -> silently dropped
--
-- Filter rules (any failure -> S_DISCARD):
--
--   * version != 4                     (not IPv4)
--   * IHL    != 5                      (options present; not supported)
--   * MF=1 or FragmentOffset != 0      (fragmented packet)
--   * dst_ip != OUR_IP and != BROADCAST_IP (not for us; default broadcast
--                                          is the limited 255.255.255.255,
--                                          subnet broadcast NOT accepted)
--   * protocol not in {0x01, 0x11}     (not ICMP or UDP)
--
-- The IP header is stripped from the output -- handlers downstream see only
-- the IP payload bytes (byte 20 .. N-1 of the incoming stream).  The
-- udp_length sideband carries the payload byte count derived from the IP
-- Total Length field (= total_length - 20).  Note that if the upstream
-- Ethernet frame was padded to the 60-byte minimum but the IP total length
-- is smaller, the demux still delivers eop at the padded-frame boundary so
-- the forwarded payload includes a few trailing zero bytes.  UDP's own
-- length field tells the UDP layer how to discard them.
--
-- Source and destination IPs from the most recent classified header are
-- exposed as sideband signals, held stable until the next classify event.
-- A UDP responder typically latches src_ip on udp_sop to know who to
-- reply to.
--
-- No IP header checksum validation (yet).  Linux drops bad-checksum
-- packets before they reach the wire, so this is fine for arping/HPSDR/
-- DHCP bring-up; revisit if we ever see corrupted traffic.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.bladerf_p.all;

entity ip_rx_handler is
    generic (
        -- Broadcast destination accepted in addition to our_ip.  Default is
        -- the limited broadcast 255.255.255.255 (DHCP-style).  Subnet
        -- broadcast (e.g. 192.168.1.255) is intentionally NOT accepted --
        -- pass the subnet broadcast value here if you want both, or extend
        -- the handler with a netmask generic.
        BROADCAST_IP : std_logic_vector(31 downto 0) := x"FF_FF_FF_FF"
    );
    port (
        clock      : in  std_logic;
        reset      : in  std_logic;

        -- IPv4 address we accept unicast packets for.  Driven at runtime
        -- so the DHCP-leased address can be substituted post-ACK without
        -- recompiling; bladerf-hosted muxes between leased_ip and the
        -- static EEM_OUR_IP fallback.
        our_ip     : in  std_logic_vector(31 downto 0);

        -- IPv4 byte stream input (from eth_rx_demux ip_* channel, header
        -- already stripped of the 14-byte Ethernet preamble).
        rx_data    : in  std_logic_vector(7 downto 0);
        rx_valid   : in  std_logic;
        rx_sop     : in  std_logic;
        rx_eop     : in  std_logic;
        rx_length  : in  std_logic_vector(13 downto 0);   -- eth payload length

        -- UDP byte stream output (IP payload only, IP header stripped).
        udp_data    : out std_logic_vector(7 downto 0);
        udp_valid   : out std_logic;
        udp_sop     : out std_logic;
        udp_eop     : out std_logic;
        udp_length  : out std_logic_vector(13 downto 0);  -- = ip_total_length - 20

        -- ICMP byte stream output (IP payload only, IP header stripped).
        icmp_data   : out std_logic_vector(7 downto 0);
        icmp_valid  : out std_logic;
        icmp_sop    : out std_logic;
        icmp_eop    : out std_logic;
        icmp_length : out std_logic_vector(13 downto 0);  -- = ip_total_length - 20

        -- Sidebands: latched at classify (byte 19), held until the next
        -- classified header.
        src_ip     : out std_logic_vector(31 downto 0);
        dst_ip     : out std_logic_vector(31 downto 0);

        -- Observability: pulses on every accepted IP packet (= one that
        -- was routed to UDP or ICMP).
        rx_pulse   : out std_logic
    );
end entity;

architecture arch of ip_rx_handler is

    constant IP_HDR_BYTES    : natural := 20;

    -- Expected fixed-value bytes
    constant EXP_VERSION_IHL : std_logic_vector(7 downto 0) := x"45"; -- v4, IHL=5
    constant IPPROTO_ICMP    : std_logic_vector(7 downto 0) := x"01"; -- 1
    constant IPPROTO_UDP     : std_logic_vector(7 downto 0) := x"11"; -- 17

    type state_t is (S_HDR, S_FWD_UDP, S_FWD_ICMP, S_DISCARD);
    signal state : state_t := S_HDR;

    -- 5 bits cover hdr_byte_idx 0..19 plus a bit of margin.
    signal hdr_byte_idx : unsigned(4 downto 0)         := (others => '0');
    signal bad_field    : std_logic                    := '0';

    -- Captured header fields
    signal protocol_r   : std_logic_vector(7 downto 0) := (others => '0');
    signal total_len_r  : unsigned(15 downto 0)        := (others => '0');
    signal src_ip_r     : std_logic_vector(31 downto 0) := (others => '0');
    signal dst_ip_r     : std_logic_vector(31 downto 0) := (others => '0');
    signal udp_length_r : std_logic_vector(13 downto 0) := (others => '0');

    signal sop_pending  : std_logic := '0';

    -- Registered outputs
    signal udp_data_r    : std_logic_vector(7 downto 0)  := (others => '0');
    signal udp_valid_r   : std_logic                     := '0';
    signal udp_sop_r     : std_logic                     := '0';
    signal udp_eop_r     : std_logic                     := '0';

    signal icmp_data_r   : std_logic_vector(7 downto 0)  := (others => '0');
    signal icmp_valid_r  : std_logic                     := '0';
    signal icmp_sop_r    : std_logic                     := '0';
    signal icmp_eop_r    : std_logic                     := '0';

    signal rx_pulse_r    : std_logic                     := '0';

begin

    udp_data    <= udp_data_r;
    udp_valid   <= udp_valid_r;
    udp_sop     <= udp_sop_r;
    udp_eop     <= udp_eop_r;
    udp_length  <= udp_length_r;

    icmp_data   <= icmp_data_r;
    icmp_valid  <= icmp_valid_r;
    icmp_sop    <= icmp_sop_r;
    icmp_eop    <= icmp_eop_r;
    icmp_length <= udp_length_r;  -- same derivation: ip_total_length - 20

    src_ip      <= src_ip_r;
    dst_ip      <= dst_ip_r;
    rx_pulse    <= rx_pulse_r;

    fsm : process(clock, reset)
        variable n_byte_idx  : unsigned(4 downto 0);
        variable n_bad       : std_logic;
        variable n_proto     : std_logic_vector(7 downto 0);
        variable n_src_ip    : std_logic_vector(31 downto 0);
        variable n_dst_ip    : std_logic_vector(31 downto 0);
        variable n_total_len : unsigned(15 downto 0);
        variable dst_match   : boolean;
    begin
        if reset = '1' then
            state         <= S_HDR;
            hdr_byte_idx  <= (others => '0');
            bad_field     <= '0';
            protocol_r    <= (others => '0');
            total_len_r   <= (others => '0');
            src_ip_r      <= (others => '0');
            dst_ip_r      <= (others => '0');
            udp_length_r  <= (others => '0');
            sop_pending   <= '0';
            udp_data_r    <= (others => '0');
            udp_valid_r   <= '0';
            udp_sop_r     <= '0';
            udp_eop_r     <= '0';
            rx_pulse_r    <= '0';
        elsif rising_edge(clock) then
            -- One-cycle defaults
            udp_valid_r  <= '0';
            udp_sop_r    <= '0';
            udp_eop_r    <= '0';
            icmp_valid_r <= '0';
            icmp_sop_r   <= '0';
            icmp_eop_r   <= '0';
            rx_pulse_r   <= '0';

            case state is

            -- --------------------------------------------------------------
            -- Walk the 20-byte IPv4 header, validating fixed fields and
            -- capturing protocol / total_length / src_ip / dst_ip.  At
            -- byte 19, classify: if dst matches us-or-broadcast AND
            -- protocol == UDP AND no bad field, transition to S_FWD_UDP;
            -- otherwise S_DISCARD.
            -- --------------------------------------------------------------
            when S_HDR =>
                n_byte_idx  := hdr_byte_idx;
                n_bad       := bad_field;
                n_proto     := protocol_r;
                n_src_ip    := src_ip_r;
                n_dst_ip    := dst_ip_r;
                n_total_len := total_len_r;

                if rx_valid = '1' then
                    if rx_sop = '1' then
                        -- New frame: discard any half-parsed prior state.
                        n_byte_idx  := (others => '0');
                        n_bad       := '0';
                        n_proto     := (others => '0');
                        n_src_ip    := (others => '0');
                        n_dst_ip    := (others => '0');
                        n_total_len := (others => '0');
                    end if;

                    case to_integer(n_byte_idx) is
                        -- byte 0: Version (high nibble) + IHL (low nibble).
                        -- Strict 0x45 = version 4, IHL=5 (20-byte header,
                        -- no options).
                        when 0 =>
                            if rx_data /= EXP_VERSION_IHL then
                                n_bad := '1';
                            end if;

                        -- byte 1: DSCP/ECN -- ignored.
                        when 1 =>
                            null;

                        -- bytes 2..3: Total Length (big-endian).
                        when 2 =>
                            n_total_len := unsigned(rx_data) & n_total_len(7 downto 0);
                        when 3 =>
                            n_total_len := n_total_len(15 downto 8) & unsigned(rx_data);

                        -- bytes 4..5: Identification -- ignored.
                        when 4 | 5 =>
                            null;

                        -- byte 6: Flags (top 3 bits) + Fragment Offset hi (low 5 bits).
                        --   bit 7 = reserved (should be 0)
                        --   bit 6 = DF (don't care)
                        --   bit 5 = MF (must be 0 -- no fragments)
                        --   bits 4..0 = fragment offset high 5 bits (must be 0)
                        when 6 =>
                            if rx_data(5) = '1' or rx_data(4 downto 0) /= "00000" then
                                n_bad := '1';
                            end if;

                        -- byte 7: Fragment Offset low 8 bits (must be 0).
                        when 7 =>
                            if rx_data /= x"00" then
                                n_bad := '1';
                            end if;

                        -- byte 8: TTL -- ignored.
                        when 8 =>
                            null;

                        -- byte 9: Protocol.
                        when 9 =>
                            n_proto := rx_data;

                        -- bytes 10..11: Header Checksum -- not validated.
                        when 10 | 11 =>
                            null;

                        -- bytes 12..15: Source IP (big-endian).
                        when 12 | 13 | 14 | 15 =>
                            n_src_ip := n_src_ip(23 downto 0) & rx_data;

                        -- bytes 16..19: Destination IP (big-endian).
                        when 16 | 17 | 18 | 19 =>
                            n_dst_ip := n_dst_ip(23 downto 0) & rx_data;

                        when others =>
                            null;  -- unreachable: byte_idx walks 0..19 only
                    end case;

                    -- At byte 19 (last header byte), classify and transition.
                    if n_byte_idx = to_unsigned(19, n_byte_idx'length) then
                        dst_match := (n_dst_ip = our_ip) or (n_dst_ip = BROADCAST_IP);
                        if rx_eop = '1' then
                            -- Header was the whole frame; no payload to forward.
                            state      <= S_HDR;
                            n_byte_idx := (others => '0');
                        elsif n_bad = '0' and dst_match
                              and (n_proto = IPPROTO_UDP or n_proto = IPPROTO_ICMP) then
                            if n_proto = IPPROTO_UDP then
                                state <= S_FWD_UDP;
                            else
                                state <= S_FWD_ICMP;
                            end if;
                            sop_pending <= '1';
                            if n_total_len >= to_unsigned(IP_HDR_BYTES, n_total_len'length) then
                                udp_length_r <= std_logic_vector(
                                    resize(n_total_len - IP_HDR_BYTES, udp_length_r'length));
                            else
                                udp_length_r <= (others => '0');
                            end if;
                            rx_pulse_r <= '1';
                            n_byte_idx := (others => '0');
                        else
                            state      <= S_DISCARD;
                            n_byte_idx := (others => '0');
                        end if;
                    elsif rx_eop = '1' then
                        -- Frame ended before header completed; bail out.
                        state      <= S_HDR;
                        n_byte_idx := (others => '0');
                    else
                        n_byte_idx := n_byte_idx + 1;
                    end if;
                end if;

                hdr_byte_idx <= n_byte_idx;
                bad_field    <= n_bad;
                protocol_r   <= n_proto;
                src_ip_r     <= n_src_ip;
                dst_ip_r     <= n_dst_ip;
                total_len_r  <= n_total_len;

            -- --------------------------------------------------------------
            -- Forward bytes to UDP channel.  First emitted byte gets sop;
            -- in_eop becomes our udp_eop.  Return to S_HDR on eop so the
            -- next frame's sop hits the header parser cleanly.
            -- --------------------------------------------------------------
            when S_FWD_UDP =>
                if rx_valid = '1' then
                    udp_data_r  <= rx_data;
                    udp_valid_r <= '1';
                    if sop_pending = '1' then
                        udp_sop_r   <= '1';
                        sop_pending <= '0';
                    end if;
                    if rx_eop = '1' then
                        udp_eop_r <= '1';
                        state     <= S_HDR;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Forward bytes to ICMP channel (mirror of S_FWD_UDP for the
            -- icmp_* port set).
            -- --------------------------------------------------------------
            when S_FWD_ICMP =>
                if rx_valid = '1' then
                    icmp_data_r  <= rx_data;
                    icmp_valid_r <= '1';
                    if sop_pending = '1' then
                        icmp_sop_r  <= '1';
                        sop_pending <= '0';
                    end if;
                    if rx_eop = '1' then
                        icmp_eop_r <= '1';
                        state      <= S_HDR;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Drop bytes until eop (rejected at classify: not for us,
            -- protocol not in {ICMP,UDP}, bad version/IHL, or fragmented).
            -- --------------------------------------------------------------
            when S_DISCARD =>
                if rx_valid = '1' and rx_eop = '1' then
                    state <= S_HDR;
                end if;

            end case;
        end if;
    end process;

end architecture;
