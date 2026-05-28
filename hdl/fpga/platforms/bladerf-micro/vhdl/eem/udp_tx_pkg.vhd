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

-- Shared helpers for FPGA-originated Ethernet/IPv4/UDP frames.  The first 42
-- bytes of every such frame are an identical Eth(14)+IP(20)+UDP(8) header;
-- eth_ip_udp_hdr_byte builds them so the HPSDR senders only implement payload.

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

package udp_tx_pkg is

    constant ETH_HDR_BYTES : natural := 14;
    constant IP_HDR_BYTES  : natural := 20;
    constant UDP_HDR_BYTES : natural := 8;
    constant L2L3L4_BYTES  : natural := ETH_HDR_BYTES + IP_HDR_BYTES
                                      + UDP_HDR_BYTES;                  -- 42

    -- One's-complement 16-bit add (IPv4 checksum arithmetic).
    function oc_add(a, b : unsigned(15 downto 0)) return unsigned;

    -- Byte idx (0..41) of the Eth/IPv4/UDP header.  IPv4: TTL=64, proto=UDP,
    -- DF set, ID=0; UDP checksum=0 (legal "not computed").
    function eth_ip_udp_hdr_byte(
        idx      : natural;
        dst_mac  : std_logic_vector(47 downto 0);
        src_mac  : std_logic_vector(47 downto 0);
        src_ip   : std_logic_vector(31 downto 0);
        dst_ip   : std_logic_vector(31 downto 0);
        src_port : std_logic_vector(15 downto 0);
        dst_port : std_logic_vector(15 downto 0);
        ip_tlen  : std_logic_vector(15 downto 0);
        udp_len  : std_logic_vector(15 downto 0);
        ip_chk   : std_logic_vector(15 downto 0)
    ) return std_logic_vector;

end package;

package body udp_tx_pkg is

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

    function eth_ip_udp_hdr_byte(
        idx      : natural;
        dst_mac  : std_logic_vector(47 downto 0);
        src_mac  : std_logic_vector(47 downto 0);
        src_ip   : std_logic_vector(31 downto 0);
        dst_ip   : std_logic_vector(31 downto 0);
        src_port : std_logic_vector(15 downto 0);
        dst_port : std_logic_vector(15 downto 0);
        ip_tlen  : std_logic_vector(15 downto 0);
        udp_len  : std_logic_vector(15 downto 0);
        ip_chk   : std_logic_vector(15 downto 0)
    ) return std_logic_vector is
    begin
        case idx is
            -- Ethernet
            when  0 => return dst_mac(47 downto 40);
            when  1 => return dst_mac(39 downto 32);
            when  2 => return dst_mac(31 downto 24);
            when  3 => return dst_mac(23 downto 16);
            when  4 => return dst_mac(15 downto  8);
            when  5 => return dst_mac( 7 downto  0);
            when  6 => return src_mac(47 downto 40);
            when  7 => return src_mac(39 downto 32);
            when  8 => return src_mac(31 downto 24);
            when  9 => return src_mac(23 downto 16);
            when 10 => return src_mac(15 downto  8);
            when 11 => return src_mac( 7 downto  0);
            when 12 => return x"08";                  -- ethertype = IPv4
            when 13 => return x"00";
            -- IPv4
            when 14 => return x"45";                  -- v4, IHL=5
            when 15 => return x"00";                  -- DSCP/ECN
            when 16 => return ip_tlen(15 downto 8);
            when 17 => return ip_tlen( 7 downto 0);
            when 18 | 19 => return x"00";             -- ID
            when 20 => return x"40";                  -- flags = DF
            when 21 => return x"00";                  -- frag offset
            when 22 => return x"40";                  -- TTL = 64
            when 23 => return x"11";                  -- proto = UDP
            when 24 => return ip_chk(15 downto 8);
            when 25 => return ip_chk( 7 downto 0);
            when 26 => return src_ip(31 downto 24);
            when 27 => return src_ip(23 downto 16);
            when 28 => return src_ip(15 downto  8);
            when 29 => return src_ip( 7 downto  0);
            when 30 => return dst_ip(31 downto 24);
            when 31 => return dst_ip(23 downto 16);
            when 32 => return dst_ip(15 downto  8);
            when 33 => return dst_ip( 7 downto  0);
            -- UDP
            when 34 => return src_port(15 downto 8);
            when 35 => return src_port( 7 downto 0);
            when 36 => return dst_port(15 downto 8);
            when 37 => return dst_port( 7 downto 0);
            when 38 => return udp_len(15 downto 8);
            when 39 => return udp_len( 7 downto 0);
            when others => return x"00";              -- 40,41 = UDP checksum 0
        end case;
    end function;

end package body;
