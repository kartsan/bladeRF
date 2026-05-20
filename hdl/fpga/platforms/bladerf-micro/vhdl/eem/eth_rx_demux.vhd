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
-- eth_rx_demux
--
-- Parses the 14-byte Ethernet header at the head of each incoming frame from
-- eem_rx_consumer and routes the remaining payload bytes to a per-ethertype
-- output channel.  Currently demultiplexes:
--
--   ethertype 0x0806 (ARP)  -> arp_*  channel
--   ethertype 0x0800 (IPv4) -> ip_*   channel    (consumer not yet built)
--   anything else            -> silently dropped
--
-- A frame is accepted only if its dst MAC matches the broadcast address
-- FF:FF:FF:FF:FF:FF or the local MAC supplied on our_mac.  Any other dst
-- MAC (multicast group memberships we never joined, or unicast to a
-- different station) sends the frame to the discard sink without
-- consuming any handler bandwidth.
--
-- The Ethernet header is stripped from the output stream -- handlers see
-- only the payload bytes (byte 14 .. N-1), with arp_length / ip_length
-- carrying the payload byte count (= consumer's eth_length - 14).
--
-- Source MAC and ethertype are exposed as sideband signals, held stable
-- for the full duration of the payload emission and through the
-- inter-frame gap until the next sop.  Handlers that need them (e.g. an
-- IP layer mapping eth-src-MAC to ARP cache entries) can latch on sop or
-- read at any time.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity eth_rx_demux is
    port (
        clock        : in  std_logic;
        reset        : in  std_logic;

        -- Local MAC (from chip_id_mac), used for dst-MAC filtering.
        our_mac      : in  std_logic_vector(47 downto 0);

        -- Byte-stream input from eem_rx_consumer (raw Ethernet frame,
        -- header included, FCS already stripped).
        in_data      : in  std_logic_vector(7 downto 0);
        in_valid     : in  std_logic;
        in_sop       : in  std_logic;
        in_eop       : in  std_logic;
        in_length    : in  std_logic_vector(13 downto 0);   -- = Ethernet frame length, hdr included

        -- ARP channel (ethertype 0x0806).  Payload only (eth header stripped).
        arp_data     : out std_logic_vector(7 downto 0);
        arp_valid    : out std_logic;
        arp_sop      : out std_logic;
        arp_eop      : out std_logic;
        arp_length   : out std_logic_vector(13 downto 0);   -- payload byte count

        -- IPv4 channel (ethertype 0x0800).  Same shape as ARP channel.
        ip_data      : out std_logic_vector(7 downto 0);
        ip_valid     : out std_logic;
        ip_sop       : out std_logic;
        ip_eop       : out std_logic;
        ip_length    : out std_logic_vector(13 downto 0);

        -- Sidebands -- latched on the cycle that the ethertype field is
        -- decoded (byte 13), held stable until the next frame is decoded.
        src_mac      : out std_logic_vector(47 downto 0);
        ethertype    : out std_logic_vector(15 downto 0)
    );
end entity;

architecture arch of eth_rx_demux is

    constant ETH_HDR_BYTES : natural := 14;

    constant ETHERTYPE_ARP  : std_logic_vector(15 downto 0) := x"0806";
    constant ETHERTYPE_IPV4 : std_logic_vector(15 downto 0) := x"0800";
    constant MAC_BROADCAST  : std_logic_vector(47 downto 0) := (others => '1');

    type state_t is (S_IDLE, S_HDR, S_FWD_ARP, S_FWD_IP, S_DISCARD);
    signal state : state_t := S_IDLE;

    -- 4 bits cover hdr_byte_idx 0..13.
    signal hdr_byte_idx : unsigned(3 downto 0) := (others => '0');

    signal dst_mac_r   : std_logic_vector(47 downto 0) := (others => '0');
    signal src_mac_r   : std_logic_vector(47 downto 0) := (others => '0');
    signal ethertype_r : std_logic_vector(15 downto 0) := (others => '0');
    signal payload_len : unsigned(13 downto 0)         := (others => '0');
    signal sop_pending : std_logic                     := '0';

    -- Registered outputs
    signal arp_data_r  : std_logic_vector(7 downto 0)  := (others => '0');
    signal arp_valid_r : std_logic                     := '0';
    signal arp_sop_r   : std_logic                     := '0';
    signal arp_eop_r   : std_logic                     := '0';

    signal ip_data_r   : std_logic_vector(7 downto 0)  := (others => '0');
    signal ip_valid_r  : std_logic                     := '0';
    signal ip_sop_r    : std_logic                     := '0';
    signal ip_eop_r    : std_logic                     := '0';

    -- Sideband length register (per-channel arp_length / ip_length are
    -- both driven from this; only one channel is active per frame).
    signal payload_len_r : std_logic_vector(13 downto 0) := (others => '0');

begin

    arp_data   <= arp_data_r;
    arp_valid  <= arp_valid_r;
    arp_sop    <= arp_sop_r;
    arp_eop    <= arp_eop_r;
    arp_length <= payload_len_r;

    ip_data    <= ip_data_r;
    ip_valid   <= ip_valid_r;
    ip_sop     <= ip_sop_r;
    ip_eop     <= ip_eop_r;
    ip_length  <= payload_len_r;

    src_mac    <= src_mac_r;
    ethertype  <= ethertype_r;

    fsm : process(clock, reset)
        variable next_dst        : std_logic_vector(47 downto 0);
        variable next_src        : std_logic_vector(47 downto 0);
        variable next_ethertype  : std_logic_vector(15 downto 0);
        variable dst_match_us    : boolean;
        variable dst_match_bcast : boolean;
        variable in_len_u        : unsigned(13 downto 0);
    begin
        if reset = '1' then
            state         <= S_IDLE;
            hdr_byte_idx  <= (others => '0');
            dst_mac_r     <= (others => '0');
            src_mac_r     <= (others => '0');
            ethertype_r   <= (others => '0');
            payload_len   <= (others => '0');
            payload_len_r <= (others => '0');
            sop_pending   <= '0';
            arp_data_r    <= (others => '0');
            arp_valid_r   <= '0';
            arp_sop_r     <= '0';
            arp_eop_r     <= '0';
            ip_data_r     <= (others => '0');
            ip_valid_r    <= '0';
            ip_sop_r      <= '0';
            ip_eop_r      <= '0';
        elsif rising_edge(clock) then
            -- One-cycle defaults
            arp_valid_r <= '0';
            arp_sop_r   <= '0';
            arp_eop_r   <= '0';
            ip_valid_r  <= '0';
            ip_sop_r    <= '0';
            ip_eop_r    <= '0';

            case state is

            -- --------------------------------------------------------------
            -- Wait for the first byte of a new Ethernet frame.  Capture it
            -- into dst_mac_r[47:40] and latch the frame length so we can
            -- compute the payload length later.
            -- --------------------------------------------------------------
            when S_IDLE =>
                if in_valid = '1' and in_sop = '1' then
                    -- Shift-left-by-8 with the new byte at the LSB; after 6
                    -- such shifts dst_mac_r[47:40] holds byte 0 (= MAC's
                    -- first-on-wire / most-significant octet) and
                    -- dst_mac_r[7:0] holds byte 5.  Same pattern used for
                    -- src_mac_r and ethertype_r in S_HDR below.
                    dst_mac_r    <= dst_mac_r(39 downto 0) & in_data;
                    hdr_byte_idx <= to_unsigned(1, hdr_byte_idx'length);
                    in_len_u     := unsigned(in_length);
                    if in_len_u >= to_unsigned(ETH_HDR_BYTES, in_len_u'length) then
                        payload_len <= in_len_u - ETH_HDR_BYTES;
                    else
                        payload_len <= (others => '0');
                    end if;
                    state <= S_HDR;
                end if;

            -- --------------------------------------------------------------
            -- Walk through bytes 1..13 of the Ethernet header, packing them
            -- into dst_mac_r / src_mac_r / ethertype_r.
            --
            -- Byte ordering: shift-left by 8 with the new byte arriving in
            -- the low 8 bits.  After 6 bytes dst_mac_r[47:40] holds the
            -- first byte received (= MAC's most-significant octet, the one
            -- on the wire first), and dst_mac_r[7:0] holds the last.
            --
            -- On byte 13 (= second byte of ethertype) classify the frame
            -- and branch to the chosen handler's forward state, or to
            -- discard.
            -- --------------------------------------------------------------
            when S_HDR =>
                if in_eop = '1' and in_valid = '1' then
                    -- Frame ended before the header completed.  Anomalous;
                    -- treat as if we just walked off the end.
                    state <= S_IDLE;
                elsif in_valid = '1' then
                    if hdr_byte_idx <= to_unsigned(5, hdr_byte_idx'length) then
                        dst_mac_r <= dst_mac_r(39 downto 0) & in_data;
                    elsif hdr_byte_idx <= to_unsigned(11, hdr_byte_idx'length) then
                        src_mac_r <= src_mac_r(39 downto 0) & in_data;
                    else
                        ethertype_r <= ethertype_r(7 downto 0) & in_data;
                    end if;

                    if hdr_byte_idx = to_unsigned(13, hdr_byte_idx'length) then
                        -- Classify on the byte we just packed in.
                        next_ethertype := ethertype_r(7 downto 0) & in_data;
                        ethertype_r    <= next_ethertype;

                        dst_match_us    := (dst_mac_r = our_mac);
                        dst_match_bcast := (dst_mac_r = MAC_BROADCAST);

                        payload_len_r <= std_logic_vector(payload_len);
                        sop_pending   <= '1';

                        if not (dst_match_us or dst_match_bcast) then
                            state <= S_DISCARD;
                        elsif next_ethertype = ETHERTYPE_ARP then
                            state <= S_FWD_ARP;
                        elsif next_ethertype = ETHERTYPE_IPV4 then
                            state <= S_FWD_IP;
                        else
                            state <= S_DISCARD;
                        end if;
                    else
                        hdr_byte_idx <= hdr_byte_idx + 1;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Forward bytes to the ARP channel.  First emitted byte gets
            -- sop; the consumer's in_eop becomes our arp_eop.
            -- --------------------------------------------------------------
            when S_FWD_ARP =>
                if in_valid = '1' then
                    arp_data_r  <= in_data;
                    arp_valid_r <= '1';
                    if sop_pending = '1' then
                        arp_sop_r   <= '1';
                        sop_pending <= '0';
                    end if;
                    if in_eop = '1' then
                        arp_eop_r <= '1';
                        state     <= S_IDLE;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Symmetric IPv4 forward path -- unused until ip_rx_handler lands.
            -- --------------------------------------------------------------
            when S_FWD_IP =>
                if in_valid = '1' then
                    ip_data_r  <= in_data;
                    ip_valid_r <= '1';
                    if sop_pending = '1' then
                        ip_sop_r    <= '1';
                        sop_pending <= '0';
                    end if;
                    if in_eop = '1' then
                        ip_eop_r <= '1';
                        state    <= S_IDLE;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Drop bytes until in_eop (unmatched dst MAC, unsupported
            -- ethertype, or anomalous short frame).
            -- --------------------------------------------------------------
            when S_DISCARD =>
                if in_valid = '1' and in_eop = '1' then
                    state <= S_IDLE;
                end if;

            end case;
        end if;
    end process;

end architecture;
