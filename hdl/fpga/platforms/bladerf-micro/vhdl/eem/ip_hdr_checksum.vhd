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

-- Registered IPv4 header checksum for a fixed-length frame.  Two-stage
-- one's-complement cascade (our_ip folded in stage 1, peer_ip in stage 2,
-- then inverted) keeps each oc_add chain short enough to close at 100 MHz.
-- Constant header fields are pre-summed at elaboration; ID and the checksum
-- field contribute zero.  our_ip/peer_ip are stable long before transmit, so
-- the 2-cycle latency is immaterial.

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.udp_tx_pkg.all;

entity ip_hdr_checksum is
    generic (
        IP_TOTAL_LEN : natural
    );
    port (
        clock    : in  std_logic;
        reset    : in  std_logic;
        our_ip   : in  std_logic_vector(31 downto 0);
        peer_ip  : in  std_logic_vector(31 downto 0);
        checksum : out std_logic_vector(15 downto 0)
    );
end entity;

architecture arch of ip_hdr_checksum is

    -- 0x4500 (v/IHL/DSCP) + total length + 0x4000 (DF) + 0x4011 (TTL/UDP).
    function const_part return unsigned is
        variable s : unsigned(15 downto 0);
    begin
        s := oc_add(to_unsigned(16#4500#, 16), to_unsigned(IP_TOTAL_LEN, 16));
        s := oc_add(s, to_unsigned(16#4000#, 16));
        s := oc_add(s, to_unsigned(16#4011#, 16));
        return s;
    end function;

    constant IP_CONST_PART : unsigned(15 downto 0) := const_part;

    signal stage1 : unsigned(15 downto 0) := (others => '0');

begin

    process(clock, reset)
        variable s : unsigned(15 downto 0);
    begin
        if reset = '1' then
            stage1   <= (others => '0');
            checksum <= (others => '0');
        elsif rising_edge(clock) then
            s := IP_CONST_PART;
            s := oc_add(s, unsigned(our_ip(31 downto 16)));
            s := oc_add(s, unsigned(our_ip(15 downto  0)));
            stage1 <= s;

            s := stage1;                                  -- previous-cycle value
            s := oc_add(s, unsigned(peer_ip(31 downto 16)));
            s := oc_add(s, unsigned(peer_ip(15 downto  0)));
            checksum <= std_logic_vector(not s);
        end if;
    end process;

end architecture;
