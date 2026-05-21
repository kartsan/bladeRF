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
-- icmp_responder
--
-- Minimal ICMP Echo (ping) responder for IPv4.  Listens on the
-- ip_rx_handler ICMP channel for type=8/code=0 Echo Requests targeting us,
-- and streams a complete Ethernet+IP+ICMP Echo Reply frame into
-- eem_tx_framer (mirroring the request's identifier/sequence/data so the
-- host's ping client matches the reply to the request).
--
-- RX path
-- -------
-- ICMP bytes arrive header-stripped of Ethernet (14B) and IP (20B) -- so
-- byte 0 is the ICMP Type field.  We buffer bytes 4 onwards
-- (identifier+sequence+data) into a small register array and capture the
-- original checksum from bytes 2..3.  Bytes 0..1 are validated against
-- Echo Request (0x08, 0x00); any mismatch sends the packet to the drop
-- path.  If the ICMP packet is longer than BUF_BYTES+4 it is truncated
-- (silently dropped) -- BUF_BYTES=256 covers the standard 56-byte Linux
-- ping payload with margin.
--
-- Checksum update (RFC 1624)
-- --------------------------
-- The only field that changes between Echo Request and Reply is the Type
-- byte (8 -> 0); since Type+Code share the same 16-bit word, the
-- old/new field pair is 0x0800 / 0x0000.  We use the incremental
-- checksum update:
--   new_chk = ~( ~old_chk + ~old_field + new_field )
--   new_chk = ~( ~old_chk + 0xF7FF )
-- This avoids recomputing the sum over the entire (potentially large)
-- packet.
--
-- IP header checksum
-- ------------------
-- Recomputed from scratch since most of the IP header differs from any
-- particular incoming packet.  The fixed contribution from V/IHL/DSCP,
-- Flags, TTL/Protocol, and OUR_IP is precomputed at elaboration time
-- (IP_FIXED_PART constant); at TX prep we add total_length and the
-- peer's two 16-bit IP halves.
--
-- TX path
-- -------
-- Single byte-index counter tx_byte_idx walks the reply byte sequence:
--   0..13   : Ethernet header (dst=peer_mac, src=our_mac, type=0x0800)
--   14..33  : IPv4 header (V/IHL=0x45, ..., src=OUR_IP, dst=peer_ip)
--   34..37  : ICMP header (type=0x00, code=0x00, new_checksum)
--   38..N-1 : buffered identifier+sequence+data from the request
-- where N = 42 + buf_count.  Outputs are combinational on
-- (state, tx_byte_idx) so that the sop beat (idx=0) stays stable across
-- the framer's two-cycle S_HDR backpressure.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.bladerf_p.all;

entity icmp_responder is
    generic (
        BUF_BYTES : natural := 256
    );
    port (
        clock        : in  std_logic;
        reset        : in  std_logic;

        -- IPv4 address we respond to ICMP Echo for.  Driven at runtime
        -- (DHCP-leased IP post-lease, static EEM_OUR_IP fallback pre-lease).
        our_ip       : in  std_logic_vector(31 downto 0);

        -- Local addresses
        our_mac      : in  std_logic_vector(47 downto 0);

        -- Peer addresses (sidebands from eth_rx_demux / ip_rx_handler;
        -- latched on rx_sop locally so they don't drift mid-frame).
        peer_mac     : in  std_logic_vector(47 downto 0);
        peer_ip      : in  std_logic_vector(31 downto 0);

        -- ICMP byte stream input (from ip_rx_handler icmp_* channel)
        rx_data      : in  std_logic_vector(7 downto 0);
        rx_valid     : in  std_logic;
        rx_sop       : in  std_logic;
        rx_eop       : in  std_logic;
        rx_length    : in  std_logic_vector(13 downto 0);  -- ICMP packet length

        -- TX byte stream output (to tx_arbiter -> eem_tx_framer)
        tx_data      : out std_logic_vector(7 downto 0);
        tx_valid     : out std_logic;
        tx_sop       : out std_logic;
        tx_eop       : out std_logic;
        tx_length    : out unsigned(13 downto 0);
        tx_ready     : in  std_logic;

        -- Observability
        reply_pulse  : out std_logic
    );
end entity;

architecture arch of icmp_responder is

    -- Fixed sequence offsets in the reply frame.  Note FIXED_HDRS_BYTES
    -- includes only Type/Code/Checksum of the ICMP header (4 bytes);
    -- the remaining 4 bytes (identifier+sequence) come from the buffer
    -- at positions 0..3, and ICMP data follows at positions 4..N-1.
    constant ETH_HDR_BYTES    : natural := 14;
    constant IP_HDR_BYTES     : natural := 20;
    constant ICMP_FIXED_BYTES : natural := 4;   -- type+code+checksum
    constant FIXED_HDRS_BYTES : natural := ETH_HDR_BYTES + IP_HDR_BYTES + ICMP_FIXED_BYTES;
        -- = 38; index where buffered bytes (identifier/sequence/data) begin

    -- Expected fixed-value bytes for an Echo Request
    constant EXP_TYPE_ECHO_REQ : std_logic_vector(7 downto 0) := x"08";
    constant EXP_CODE_ZERO     : std_logic_vector(7 downto 0) := x"00";

    -- ----------------------------------------------------------------------
    -- One's-complement 16-bit addition with end-around carry.  Used for IP
    -- and ICMP checksum arithmetic; pure combinational.
    -- ----------------------------------------------------------------------
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
    -- fields that are true compile-time constants in an Echo Reply:
    --   0x4500 (V/IHL/DSCP), 0x4000 (DF), 0x4001 (TTL/Proto = 64 / ICMP).
    -- ID and the placeholder Checksum word contribute 0 and are omitted.
    function calc_ip_const_part return unsigned is
        variable s : unsigned(15 downto 0);
    begin
        s := oc_add(to_unsigned(16#4500#, 16), to_unsigned(16#4000#, 16));
        s := oc_add(s, to_unsigned(16#4001#, 16));
        return s;
    end function;

    constant IP_CONST_PART : unsigned(15 downto 0) := calc_ip_const_part;

    -- Runtime fixed-part: IP_CONST_PART + our_ip halves.  Registered to
    -- keep the FSM's per-eop chain of checksum adders shallow (4 levels
    -- below: ip_fixed_part_r + total_len + peer_ip_hi + peer_ip_lo).
    signal ip_fixed_part_r : unsigned(15 downto 0) := (others => '0');

    -- ----------------------------------------------------------------------
    -- State
    -- ----------------------------------------------------------------------
    type state_t is (S_RX, S_TX);
    signal state : state_t := S_RX;

    -- Per-frame buffer for bytes 4..N-1 of the incoming ICMP packet
    -- (identifier + sequence + data).  Reflected verbatim in the reply.
    type buf_t is array (0 to BUF_BYTES-1) of std_logic_vector(7 downto 0);
    signal buf : buf_t := (others => (others => '0'));

    -- RX-side tracking
    signal rx_byte_idx     : unsigned(13 downto 0) := (others => '0');
    signal buf_count       : unsigned(13 downto 0) := (others => '0');
    signal bad_field       : std_logic             := '0';
    signal original_chk_r  : std_logic_vector(15 downto 0) := (others => '0');
    signal peer_mac_r      : std_logic_vector(47 downto 0) := (others => '0');
    signal peer_ip_r       : std_logic_vector(31 downto 0) := (others => '0');

    -- TX-side derived fields, latched at end-of-RX
    signal frame_len_r     : unsigned(13 downto 0)        := (others => '0');
    signal new_icmp_chk_r  : std_logic_vector(15 downto 0):= (others => '0');
    signal new_ip_chk_r    : std_logic_vector(15 downto 0):= (others => '0');

    -- TX-side counter
    signal tx_byte_idx     : unsigned(13 downto 0) := (others => '0');

    signal reply_pulse_r   : std_logic := '0';

    -- ----------------------------------------------------------------------
    -- Combinational byte lookup for the reply.  Reads from constants /
    -- captured registers for indices 0..41 and from the buffer for 42+.
    -- ----------------------------------------------------------------------
    function reply_fixed_byte(
        idx        : natural;
        peer_mac   : std_logic_vector(47 downto 0);
        our_mac    : std_logic_vector(47 downto 0);
        our_ip     : std_logic_vector(31 downto 0);
        peer_ip    : std_logic_vector(31 downto 0);
        tot_len    : unsigned(15 downto 0);
        ip_chk     : std_logic_vector(15 downto 0);
        icmp_chk   : std_logic_vector(15 downto 0)
    ) return std_logic_vector is
    begin
        case idx is
            -- Eth dst = peer MAC
            when  0 => return peer_mac(47 downto 40);
            when  1 => return peer_mac(39 downto 32);
            when  2 => return peer_mac(31 downto 24);
            when  3 => return peer_mac(23 downto 16);
            when  4 => return peer_mac(15 downto  8);
            when  5 => return peer_mac( 7 downto  0);
            -- Eth src = our MAC
            when  6 => return our_mac(47 downto 40);
            when  7 => return our_mac(39 downto 32);
            when  8 => return our_mac(31 downto 24);
            when  9 => return our_mac(23 downto 16);
            when 10 => return our_mac(15 downto  8);
            when 11 => return our_mac( 7 downto  0);
            -- Ethertype = IPv4
            when 12 => return x"08";
            when 13 => return x"00";
            -- IP V/IHL, DSCP/ECN
            when 14 => return x"45";
            when 15 => return x"00";
            -- IP total length (big-endian)
            when 16 => return std_logic_vector(tot_len(15 downto 8));
            when 17 => return std_logic_vector(tot_len( 7 downto 0));
            -- IP identification (don't care, DF=1 so we never fragment)
            when 18 => return x"00";
            when 19 => return x"00";
            -- IP flags + frag offset = 0x4000 (DF set)
            when 20 => return x"40";
            when 21 => return x"00";
            -- IP TTL, protocol
            when 22 => return x"40";  -- TTL = 64
            when 23 => return x"01";  -- ICMP
            -- IP header checksum
            when 24 => return ip_chk(15 downto 8);
            when 25 => return ip_chk( 7 downto 0);
            -- IP src = OUR_IP
            when 26 => return our_ip(31 downto 24);
            when 27 => return our_ip(23 downto 16);
            when 28 => return our_ip(15 downto  8);
            when 29 => return our_ip( 7 downto  0);
            -- IP dst = peer IP (the original sender)
            when 30 => return peer_ip(31 downto 24);
            when 31 => return peer_ip(23 downto 16);
            when 32 => return peer_ip(15 downto  8);
            when 33 => return peer_ip( 7 downto  0);
            -- ICMP type / code (Echo Reply)
            when 34 => return x"00";
            when 35 => return x"00";
            -- ICMP checksum (delta-updated from request)
            when 36 => return icmp_chk(15 downto 8);
            when 37 => return icmp_chk( 7 downto 0);
            -- ICMP identifier + sequence (bytes 38..41) are buffered;
            -- this function is only called for idx < 38.
            when others => return x"00";
        end case;
    end function;

begin

    -- Combinational byte mux: function-based lookup for fixed header
    -- bytes (idx 0..37), buffer-based lookup for bytes 38+.  Using a
    -- process here so the buffer read uses a guarded index and never
    -- triggers an out-of-range simulation error when idx is small.
    tx_data_mux : process(state, tx_byte_idx, peer_mac_r, peer_ip_r,
                          our_mac, frame_len_r,
                          new_ip_chk_r, new_icmp_chk_r, buf)
        variable buf_addr : unsigned(13 downto 0);
    begin
        if state = S_TX then
            if tx_byte_idx < to_unsigned(FIXED_HDRS_BYTES, tx_byte_idx'length) then
                tx_data <= reply_fixed_byte(to_integer(tx_byte_idx),
                                            peer_mac_r, our_mac,
                                            our_ip, peer_ip_r,
                                            resize(frame_len_r - ETH_HDR_BYTES, 16),
                                            new_ip_chk_r, new_icmp_chk_r);
            else
                buf_addr := tx_byte_idx - to_unsigned(FIXED_HDRS_BYTES, tx_byte_idx'length);
                if buf_addr < to_unsigned(BUF_BYTES, buf_addr'length) then
                    tx_data <= buf(to_integer(buf_addr));
                else
                    tx_data <= (others => '0');
                end if;
            end if;
        else
            tx_data <= (others => '0');
        end if;
    end process tx_data_mux;

    tx_valid <= '1' when state = S_TX else '0';
    tx_sop   <= '1' when (state = S_TX and tx_byte_idx = 0) else '0';
    tx_eop   <= '1' when (state = S_TX and (tx_byte_idx + 1) = frame_len_r) else '0';
    tx_length    <= frame_len_r;
    reply_pulse  <= reply_pulse_r;

    -- ----------------------------------------------------------------------
    -- Registered fixed-part of the IP checksum.  IP_CONST_PART covers
    -- the truly compile-time fields (V/IHL/DSCP, DF, TTL/Proto); our_ip
    -- halves are added here whenever our_ip changes (typically once, on
    -- the DHCP-ACK transition).  Pre-registering keeps the FSM's per-eop
    -- checksum chain at 4 oc_add levels, same as the original elaboration-
    -- time IP_FIXED_PART path.
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

    fsm : process(clock, reset)
        variable n_byte_idx : unsigned(13 downto 0);
        variable n_bad      : std_logic;
        variable n_orig_chk : std_logic_vector(15 downto 0);
        variable buf_addr   : unsigned(13 downto 0);
        variable icmp_sum   : unsigned(15 downto 0);
        variable ip_sum     : unsigned(15 downto 0);
        variable total_len  : unsigned(15 downto 0);
    begin
        if reset = '1' then
            state           <= S_RX;
            rx_byte_idx     <= (others => '0');
            buf_count       <= (others => '0');
            bad_field       <= '0';
            original_chk_r  <= (others => '0');
            peer_mac_r      <= (others => '0');
            peer_ip_r       <= (others => '0');
            frame_len_r     <= (others => '0');
            new_icmp_chk_r  <= (others => '0');
            new_ip_chk_r    <= (others => '0');
            tx_byte_idx     <= (others => '0');
            reply_pulse_r   <= '0';
        elsif rising_edge(clock) then
            reply_pulse_r <= '0';

            case state is

            -- ----------------------------------------------------------------
            -- Receive the ICMP request: validate type/code, capture original
            -- checksum, buffer bytes 4+.  On eop, if valid, latch peer MAC/IP
            -- and the computed reply fields, then move to S_TX.
            -- ----------------------------------------------------------------
            when S_RX =>
                n_byte_idx := rx_byte_idx;
                n_bad      := bad_field;
                n_orig_chk := original_chk_r;

                if rx_valid = '1' then
                    if rx_sop = '1' then
                        n_byte_idx := (others => '0');
                        n_bad      := '0';
                        n_orig_chk := (others => '0');
                        -- Latch peer addresses at sop.  The eth_rx_demux
                        -- src_mac sideband is held stable for the whole
                        -- frame so this captures the right value.
                        peer_mac_r <= peer_mac;
                        peer_ip_r  <= peer_ip;
                    end if;

                    case to_integer(n_byte_idx) is
                        when 0 =>
                            if rx_data /= EXP_TYPE_ECHO_REQ then n_bad := '1'; end if;
                        when 1 =>
                            if rx_data /= EXP_CODE_ZERO     then n_bad := '1'; end if;
                        when 2 =>
                            n_orig_chk(15 downto 8) := rx_data;
                        when 3 =>
                            n_orig_chk( 7 downto 0) := rx_data;
                        when others =>
                            -- Buffer bytes 4..(BUF_BYTES+3) into buf[0..BUF_BYTES-1].
                            buf_addr := n_byte_idx - 4;
                            if buf_addr < to_unsigned(BUF_BYTES, buf_addr'length) then
                                buf(to_integer(buf_addr)) <= rx_data;
                            else
                                -- Packet exceeded buffer size; drop the reply.
                                n_bad := '1';
                            end if;
                    end case;

                    if rx_eop = '1' then
                        -- End of request.  buf_count = number of bytes
                        -- stored in buf (= n_byte_idx - 3, since bytes
                        -- 0..3 are header type/code/checksum and
                        -- n_byte_idx is the index of the LAST byte).
                        if n_bad = '0' and n_byte_idx >= to_unsigned(4, n_byte_idx'length) then
                            buf_count <= n_byte_idx - 3;
                            -- IP total_length = ICMP packet bytes (n_byte_idx+1)
                            -- + IP header (20).  Resize first to avoid 14-bit
                            -- overflow on the intermediate.
                            total_len := resize(n_byte_idx, 16)
                                         + to_unsigned(1 + IP_HDR_BYTES, 16);
                            frame_len_r <= resize(total_len
                                                  + to_unsigned(ETH_HDR_BYTES, 16),
                                                  frame_len_r'length);

                            -- ICMP checksum incremental update (RFC 1624):
                            --   new_chk = ~( ~old_chk + 0xF7FF )
                            icmp_sum := oc_add(unsigned(not n_orig_chk),
                                               to_unsigned(16#F7FF#, 16));
                            new_icmp_chk_r <= std_logic_vector(not icmp_sum);

                            -- IP checksum: registered fixed part (constants
                            -- + our_ip halves) + total_length + peer_ip halves.
                            ip_sum := oc_add(ip_fixed_part_r, total_len);
                            ip_sum := oc_add(ip_sum, unsigned(peer_ip(31 downto 16)));
                            ip_sum := oc_add(ip_sum, unsigned(peer_ip(15 downto 0)));
                            new_ip_chk_r <= std_logic_vector(not ip_sum);

                            tx_byte_idx <= (others => '0');
                            state       <= S_TX;
                        end if;
                        n_byte_idx := (others => '0');
                        n_bad      := '0';
                    else
                        n_byte_idx := n_byte_idx + 1;
                    end if;
                end if;

                rx_byte_idx    <= n_byte_idx;
                bad_field      <= n_bad;
                original_chk_r <= n_orig_chk;

            -- ----------------------------------------------------------------
            -- Stream the reply through eem_tx_framer.  Outputs are
            -- combinational on (state, tx_byte_idx); tx_byte_idx advances
            -- only when tx_ready='1', which holds the sop beat stable for
            -- the framer's 2-cycle header injection.
            -- ----------------------------------------------------------------
            when S_TX =>
                if tx_ready = '1' then
                    if (tx_byte_idx + 1) = frame_len_r then
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
