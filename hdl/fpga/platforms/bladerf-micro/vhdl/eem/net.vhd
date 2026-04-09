-- Copyright (c) 2017 Nuand LLC
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
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
-- THE SOFTWARE.

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity net is
    port (
        clock       : in  std_logic;
        reset       : in  std_logic;
        data_in     : in  std_logic_vector(31 downto 0);
        packet_start : in std_logic;
        data_valid  : in  std_logic;
        data_out    : out std_logic_vector(31 downto 0);
        packet_type : out std_logic_vector(1 downto 0)  -- 00: OTHER, 01: ARP, 10: UDP
    );
end entity;

architecture arch of net is

    type state_t is (IDLE, READ_ETH_DST, READ_ETH_SRC, READ_ETH_TYPE, READ_IP_VER, READ_IP_PROTO, DONE);
    signal state : state_t := IDLE;

    signal ethertype : std_logic_vector(15 downto 0);
    signal ip_proto  : std_logic_vector(7 downto 0);

    constant ETH_ARP  : std_logic_vector(15 downto 0) := x"0806";
    constant ETH_IP   : std_logic_vector(15 downto 0) := x"0800";
    constant IP_UDP   : std_logic_vector(7 downto 0) := x"11";

begin

    -- Pass through data_in to data_out
    data_out <= data_in;

    process(clock, reset)
    begin
        if reset = '1' then
            state <= IDLE;
            packet_type <= "00";
            ethertype <= (others => '0');
            ip_proto <= (others => '0');
        elsif rising_edge(clock) then
            if packet_start = '1' and data_valid = '1' then
                state <= READ_ETH_SRC;
                packet_type <= "00";
            else
            case state is
                when IDLE =>
                    if data_valid = '1' then
                        state <= READ_ETH_DST;
                    end if;
                    packet_type <= "00";

                when READ_ETH_DST =>
                    -- Skip DST MAC (6 bytes, but since 32-bit, this is first word)
                    state <= READ_ETH_SRC;

                when READ_ETH_SRC =>
                    -- Skip SRC MAC (next 2 words for 6 bytes)
                    state <= READ_ETH_TYPE;

                when READ_ETH_TYPE =>
                    -- Read EtherType (last 2 bytes of this word)
                    ethertype <= data_in(15 downto 0);
                    if data_in(15 downto 0) = ETH_ARP then
                        packet_type <= "01";
                        state <= DONE;
                    elsif data_in(15 downto 0) = ETH_IP then
                        state <= READ_IP_VER;
                    else
                        packet_type <= "00";
                        state <= DONE;
                    end if;

                when READ_IP_VER =>
                    -- Skip IP header fields, read protocol (byte 9 in IP header)
                    -- Assuming standard IP header length
                    ip_proto <= data_in(7 downto 0);  -- Protocol field
                    if data_in(7 downto 0) = IP_UDP then
                        packet_type <= "10";
                    else
                        packet_type <= "00";
                    end if;
                    state <= DONE;

                when DONE =>
                    if packet_start = '1' then
                        state <= READ_ETH_SRC;
                        packet_type <= "00";
                    end if;

                when others =>
                    state <= IDLE;
            end case;
            end if;
        end if;
    end process;

end architecture;
