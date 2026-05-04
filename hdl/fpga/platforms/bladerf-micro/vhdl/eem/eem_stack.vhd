-- eem_stack.vhd
--
-- EEM-only stack variant for fast bring-up/debug.
-- RX bytes from eem_deframer are looped back
-- directly into eem_tx_framer (no HPSDR protocol stack).

library ieee;
    use ieee.std_logic_1164.all;

entity eem_stack is
    generic (
        LOCAL_IP   : std_logic_vector(31 downto 0) := x"C0A8_010A";
        LOCAL_PORT : std_logic_vector(15 downto 0) := x"0400"
    );
    port (
        clock              : in  std_logic;
        reset              : in  std_logic;
        local_mac          : in  std_logic_vector(47 downto 0);

        -- EEM RX (host?FPGA, from eem_deframer)
        eth_data_in        : in  std_logic_vector(31 downto 0);
        eth_data_valid     : in  std_logic;
        eth_packet_start   : in  std_logic;
        eth_packet_end     : in  std_logic;
        eth_packet_empty   : in  std_logic_vector(1 downto 0);

        -- EEM TX (FPGA?host, to eem_rx_fifo / RX1 GPIF path)
        eem_fifo_write     : out std_logic;
        eem_fifo_full      : in  std_logic;
        eem_fifo_data      : out std_logic_vector(31 downto 0);

        -- Status (unused in EEM-only mode)
        broadcast          : out std_logic;
        dst_unreachable    : out std_logic
    );
end entity;

architecture arch of eem_stack is

    signal rx_data   : std_logic_vector(7 downto 0);
    signal rx_enable : std_logic;

begin

    -- Keep outputs deterministic in EEM-only mode.
    broadcast       <= '0';
    dst_unreachable <= '0';


    U_eem_rx_adapter : entity work.eem_rx_adapter
        port map (
            clock            => clock,
            reset            => reset,
            eth_data_in      => eth_data_in,
            eth_data_valid   => eth_data_valid,
            eth_packet_start => eth_packet_start,
            eth_packet_end   => eth_packet_end,
            eth_packet_empty => eth_packet_empty,
            rx_data          => rx_data,
            rx_enable        => rx_enable
        );

    -- Direct loopback through EEM framer for transport-only validation.
    U_eem_tx_framer : entity work.eem_tx_framer
        port map (
            clock      => clock,
            reset      => reset,
            mac_active => rx_enable,
            mac_data   => rx_data,
            fifo_write => eem_fifo_write,
            fifo_full  => eem_fifo_full,
            fifo_data  => eem_fifo_data
        );

end architecture;
