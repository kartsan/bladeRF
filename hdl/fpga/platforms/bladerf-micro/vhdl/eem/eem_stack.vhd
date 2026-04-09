-- eem_stack.vhd
--
-- Top-level EEM networking stack for bladeRF hosted FPGA.
--
-- Data flow (RX — host to FPGA):
--   FX3 GPIF TX2 → eem_deframer → eem_rx_adapter → mac_recv
--                                                 → ip_recv → udp_recv
--                                                           → icmp
--                                                 → arp (RX side)
--
-- Data flow (TX — FPGA to host):
--   arp (TX side) ──┐
--   icmp (TX side)  ├─→ TX arbiter → ip_send → mac_send → eem_tx_framer
--   udp_send ───────┘                                    → FX3 GPIF RX1
--
-- All logic runs in one clock domain (fx3_pclk_pll).  The HPSDR sync.v
-- module is instantiated with rx_clock = tx_clock = clock, so it acts
-- as a simple 2-FF synchroniser with no functional crossing needed.
--
-- Parameters
--   LOCAL_MAC  : 48-bit MAC address for the bladeRF device
--   LOCAL_IP   : 32-bit IPv4 address (static, default 192.168.1.10)
--   LOCAL_PORT : 16-bit UDP port the application listens on (default 1024)

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity eem_stack is
    generic (
        LOCAL_MAC  : std_logic_vector(47 downto 0) := x"12_22_33_44_55_66";
        LOCAL_IP   : std_logic_vector(31 downto 0) := x"C0A8_010A";  -- 192.168.1.10
        LOCAL_PORT : std_logic_vector(15 downto 0) := x"0400"        -- 1024
    );
    port (
        clock              : in  std_logic;
        reset              : in  std_logic;

        -- EEM RX (host→FPGA, from eem_deframer)
        eth_data_in        : in  std_logic_vector(31 downto 0);
        eth_data_valid     : in  std_logic;
        eth_packet_start   : in  std_logic;
        eth_packet_end     : in  std_logic;
        eth_packet_empty   : in  std_logic_vector(1 downto 0);

        -- EEM TX (FPGA→host, to eem_rx_fifo / RX1 GPIF path)
        eem_fifo_write     : out std_logic;
        eem_fifo_full      : in  std_logic;
        eem_fifo_data      : out std_logic_vector(31 downto 0);

        -- Application UDP RX interface (payload bytes, one per clock)
        udp_rx_data        : out std_logic_vector(7 downto 0);
        udp_rx_active      : out std_logic;

        -- Application UDP TX interface
        udp_tx_data        : in  std_logic_vector(7 downto 0);
        udp_tx_length      : in  std_logic_vector(15 downto 0);
        udp_tx_enable      : in  std_logic;
        udp_tx_active      : out std_logic;
        udp_tx_port        : in  std_logic_vector(15 downto 0);

        -- Status
        broadcast          : out std_logic;
        dst_unreachable    : out std_logic
    );
end entity;

architecture arch of eem_stack is

    -- -----------------------------------------------------------------------
    -- Signals driven from generics (Quartus requires signals, not constants
    -- or generics, when connecting to Verilog component ports)
    -- -----------------------------------------------------------------------
    signal MAC_SLV       : std_logic_vector(47 downto 0);
    signal IP_SLV        : std_logic_vector(31 downto 0);
    signal LOCAL_PORT_SIG : std_logic_vector(15 downto 0);

    -- -----------------------------------------------------------------------
    -- Component declarations for Verilog modules (mixed-language instantiation)
    -- -----------------------------------------------------------------------
    component mac_recv is
        port (
            clock      : in  std_logic;
            rx_enable  : in  std_logic;
            data       : in  std_logic_vector(7 downto 0);
            local_mac  : in  std_logic_vector(47 downto 0);
            active     : out std_logic;
            broadcast  : out std_logic;
            is_arp     : out std_logic;
            remote_mac : out std_logic_vector(47 downto 0)
        );
    end component;

    component ip_recv is
        port (
            clock      : in  std_logic;
            rx_enable  : in  std_logic;
            data       : in  std_logic_vector(7 downto 0);
            broadcast  : in  std_logic;
            local_ip   : in  std_logic_vector(31 downto 0);
            active     : out std_logic;
            is_icmp    : out std_logic;
            remote_ip  : out std_logic_vector(31 downto 0);
            to_ip      : out std_logic_vector(31 downto 0)
        );
    end component;

    component udp_recv is
        port (
            clock                : in  std_logic;
            rx_enable            : in  std_logic;
            data                 : in  std_logic_vector(7 downto 0);
            to_ip                : in  std_logic_vector(31 downto 0);
            broadcast            : in  std_logic;
            remote_mac           : in  std_logic_vector(47 downto 0);
            remote_ip            : in  std_logic_vector(31 downto 0);
            local_ip             : in  std_logic_vector(31 downto 0);
            active               : out std_logic;
            dhcp_active          : out std_logic;
            to_port              : out std_logic_vector(15 downto 0);
            udp_destination_ip   : out std_logic_vector(31 downto 0);
            udp_destination_mac  : out std_logic_vector(47 downto 0);
            udp_destination_port : out std_logic_vector(15 downto 0)
        );
    end component;

    component arp is
        port (
            reset           : in  std_logic;
            rx_clock        : in  std_logic;
            rx_enable       : in  std_logic;
            rx_data         : in  std_logic_vector(7 downto 0);
            tx_clock        : in  std_logic;
            local_mac       : in  std_logic_vector(47 downto 0);
            local_ip        : in  std_logic_vector(31 downto 0);
            remote_mac      : in  std_logic_vector(47 downto 0);
            tx_enable       : in  std_logic;
            tx_data         : out std_logic_vector(7 downto 0);
            destination_mac : out std_logic_vector(47 downto 0);
            tx_request      : out std_logic;
            tx_active       : out std_logic
        );
    end component;

    component icmp is
        port (
            reset           : in  std_logic;
            rx_clock        : in  std_logic;
            rx_enable       : in  std_logic;
            rx_data         : in  std_logic_vector(7 downto 0);
            tx_clock        : in  std_logic;
            tx_enable       : in  std_logic;
            remote_mac      : in  std_logic_vector(47 downto 0);
            remote_ip       : in  std_logic_vector(31 downto 0);
            dst_unreachable : out std_logic;
            tx_request      : out std_logic;
            tx_active       : out std_logic;
            tx_data         : out std_logic_vector(7 downto 0);
            length          : out std_logic_vector(15 downto 0);
            destination_mac : out std_logic_vector(47 downto 0);
            destination_ip  : out std_logic_vector(31 downto 0)
        );
    end component;

    component udp_send is
        port (
            reset            : in  std_logic;
            clock            : in  std_logic;
            tx_enable        : in  std_logic;
            data_in          : in  std_logic_vector(7 downto 0);
            length_in        : in  std_logic_vector(15 downto 0);
            local_port       : in  std_logic_vector(15 downto 0);
            destination_port : in  std_logic_vector(15 downto 0);
            port_ID          : in  std_logic_vector(7 downto 0);
            active           : out std_logic;
            data_out         : out std_logic_vector(7 downto 0);
            length_out       : out std_logic_vector(15 downto 0)
        );
    end component;

    component ip_send is
        port (
            reset          : in  std_logic;
            clock          : in  std_logic;
            tx_enable      : in  std_logic;
            active         : out std_logic;
            data_in        : in  std_logic_vector(7 downto 0);
            data_out       : out std_logic_vector(7 downto 0);
            is_icmp        : in  std_logic;
            length         : in  std_logic_vector(15 downto 0);
            local_ip       : in  std_logic_vector(31 downto 0);
            destination_ip : in  std_logic_vector(31 downto 0)
        );
    end component;

    component mac_send is
        port (
            clock           : in  std_logic;
            reset           : in  std_logic;
            tx_enable       : in  std_logic;
            active          : out std_logic;
            data_in         : in  std_logic_vector(7 downto 0);
            data_out        : out std_logic_vector(7 downto 0);
            local_mac       : in  std_logic_vector(47 downto 0);
            destination_mac : in  std_logic_vector(47 downto 0)
        );
    end component;

    -- -----------------------------------------------------------------------
    -- Intermediate signals
    -- -----------------------------------------------------------------------

    -- eem_rx_adapter → mac_recv
    signal rx_data          : std_logic_vector(7 downto 0);
    signal rx_enable        : std_logic;

    -- mac_recv outputs
    signal mac_active       : std_logic;
    signal mac_broadcast    : std_logic;
    signal mac_is_arp       : std_logic;
    signal mac_remote_mac   : std_logic_vector(47 downto 0);

    -- ip_recv outputs
    signal ip_active        : std_logic;
    signal ip_is_icmp       : std_logic;
    signal ip_remote_ip     : std_logic_vector(31 downto 0);
    signal ip_to_ip         : std_logic_vector(31 downto 0);

    -- udp_recv outputs
    signal udp_active_i     : std_logic;
    signal udp_dhcp_active  : std_logic;
    signal udp_to_port      : std_logic_vector(15 downto 0);
    signal udp_dst_ip       : std_logic_vector(31 downto 0);
    signal udp_dst_mac      : std_logic_vector(47 downto 0);
    signal udp_dst_port     : std_logic_vector(15 downto 0);

    -- arp outputs
    signal arp_tx_data      : std_logic_vector(7 downto 0);
    signal arp_dst_mac      : std_logic_vector(47 downto 0);
    signal arp_tx_request   : std_logic;
    signal arp_tx_active    : std_logic;

    -- icmp outputs
    signal icmp_tx_data     : std_logic_vector(7 downto 0);
    signal icmp_length      : std_logic_vector(15 downto 0);
    signal icmp_dst_mac     : std_logic_vector(47 downto 0);
    signal icmp_dst_ip      : std_logic_vector(31 downto 0);
    signal icmp_tx_request  : std_logic;
    signal icmp_tx_active   : std_logic;
    signal icmp_dst_unreach : std_logic;

    -- udp_send outputs
    signal udp_send_data    : std_logic_vector(7 downto 0);
    signal udp_send_active  : std_logic;
    signal udp_send_length  : std_logic_vector(15 downto 0);

    -- TX arbiter → ip_send
    signal tx_data_mux      : std_logic_vector(7 downto 0);
    signal tx_enable_mux    : std_logic;
    signal tx_is_icmp       : std_logic;
    signal tx_length_mux    : std_logic_vector(15 downto 0);
    signal tx_dst_ip        : std_logic_vector(31 downto 0);
    signal tx_dst_mac       : std_logic_vector(47 downto 0);

    -- ip_send outputs
    signal ips_data         : std_logic_vector(7 downto 0);
    signal ips_active       : std_logic;

    -- mac_send outputs
    signal macs_data        : std_logic_vector(7 downto 0);
    signal macs_active      : std_logic;

begin

    MAC_SLV        <= LOCAL_MAC;
    IP_SLV         <= LOCAL_IP;
    LOCAL_PORT_SIG <= LOCAL_PORT;

    broadcast       <= mac_broadcast;
    dst_unreachable <= icmp_dst_unreach;
    udp_rx_data     <= rx_data;
    udp_rx_active   <= udp_active_i;
    udp_tx_active   <= udp_send_active;

    -- -----------------------------------------------------------------------
    -- EEM RX adapter: 32-bit word stream → byte serial
    -- -----------------------------------------------------------------------
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

    -- -----------------------------------------------------------------------
    -- HPSDR mac_recv (Verilog)
    -- -----------------------------------------------------------------------
    U_mac_recv : mac_recv
        port map (
            clock       => clock,
            rx_enable   => rx_enable,
            data        => rx_data,
            local_mac   => MAC_SLV,
            active      => mac_active,
            broadcast   => mac_broadcast,
            is_arp      => mac_is_arp,
            remote_mac  => mac_remote_mac
        );

    -- -----------------------------------------------------------------------
    -- HPSDR ip_recv (Verilog) — driven when mac says payload is IP
    -- -----------------------------------------------------------------------
    U_ip_recv : ip_recv
        port map (
            clock       => clock,
            rx_enable   => mac_active,
            data        => rx_data,
            broadcast   => mac_broadcast,
            local_ip    => IP_SLV,
            active      => ip_active,
            is_icmp     => ip_is_icmp,
            remote_ip   => ip_remote_ip,
            to_ip       => ip_to_ip
        );

    -- -----------------------------------------------------------------------
    -- HPSDR udp_recv (Verilog)
    -- -----------------------------------------------------------------------
    U_udp_recv : udp_recv
        port map (
            clock               => clock,
            rx_enable           => ip_active,
            data                => rx_data,
            to_ip               => ip_to_ip,
            broadcast           => mac_broadcast,
            remote_mac          => mac_remote_mac,
            remote_ip           => ip_remote_ip,
            local_ip            => IP_SLV,
            active              => udp_active_i,
            dhcp_active         => udp_dhcp_active,
            to_port             => udp_to_port,
            udp_destination_ip  => udp_dst_ip,
            udp_destination_mac => udp_dst_mac,
            udp_destination_port => udp_dst_port
        );

    -- -----------------------------------------------------------------------
    -- HPSDR arp (Verilog)
    -- -----------------------------------------------------------------------
    U_arp : arp
        port map (
            reset           => reset,
            rx_clock        => clock,
            rx_enable       => mac_is_arp,  -- mac_recv raises is_arp for ARP frames
            rx_data         => rx_data,
            tx_clock        => clock,
            local_mac       => MAC_SLV,
            local_ip        => IP_SLV,
            remote_mac      => mac_remote_mac,
            tx_enable       => arp_tx_request,
            tx_data         => arp_tx_data,
            destination_mac => arp_dst_mac,
            tx_request      => arp_tx_request,
            tx_active       => arp_tx_active
        );

    -- -----------------------------------------------------------------------
    -- HPSDR icmp (Verilog)
    -- -----------------------------------------------------------------------
    U_icmp : icmp
        port map (
            reset           => reset,
            rx_clock        => clock,
            rx_enable       => ip_active,   -- icmp checks is_icmp internally via first byte
            rx_data         => rx_data,
            tx_clock        => clock,
            tx_enable       => icmp_tx_request,
            remote_mac      => mac_remote_mac,
            remote_ip       => ip_remote_ip,
            dst_unreachable => icmp_dst_unreach,
            tx_request      => icmp_tx_request,
            tx_active       => icmp_tx_active,
            tx_data         => icmp_tx_data,
            length          => icmp_length,
            destination_mac => icmp_dst_mac,
            destination_ip  => icmp_dst_ip
        );

    -- -----------------------------------------------------------------------
    -- HPSDR udp_send (Verilog)
    -- -----------------------------------------------------------------------
    U_udp_send : udp_send
        port map (
            reset            => reset,
            clock            => clock,
            tx_enable        => udp_tx_enable,
            data_in          => udp_tx_data,
            length_in        => udp_tx_length,
            local_port       => LOCAL_PORT_SIG,
            destination_port => udp_tx_port,
            port_ID          => x"00",
            active           => udp_send_active,
            data_out         => udp_send_data,
            length_out       => udp_send_length
        );

    -- -----------------------------------------------------------------------
    -- TX arbiter: ARP has priority, then ICMP, then UDP
    -- Simple combinational priority mux — only one source is active at a time
    -- in normal operation because HPSDR modules serialise requests.
    -- -----------------------------------------------------------------------
    process(arp_tx_active, arp_tx_data, arp_dst_mac,
            icmp_tx_active, icmp_tx_data, icmp_dst_ip, icmp_dst_mac, icmp_length,
            udp_send_active, udp_send_data, udp_send_length,
            udp_dst_ip, udp_dst_mac,
            ip_remote_ip, ip_is_icmp,
            IP_SLV)
    begin
        if arp_tx_active = '1' then
            tx_data_mux   <= arp_tx_data;
            tx_enable_mux <= arp_tx_active;
            tx_is_icmp    <= '0';
            tx_length_mux <= (others => '0');
            tx_dst_ip     <= (others => '0');
            tx_dst_mac    <= arp_dst_mac;
        elsif icmp_tx_active = '1' then
            tx_data_mux   <= icmp_tx_data;
            tx_enable_mux <= icmp_tx_active;
            tx_is_icmp    <= '1';
            tx_length_mux <= icmp_length;
            tx_dst_ip     <= icmp_dst_ip;
            tx_dst_mac    <= icmp_dst_mac;
        else
            tx_data_mux   <= udp_send_data;
            tx_enable_mux <= udp_send_active;
            tx_is_icmp    <= '0';
            tx_length_mux <= udp_send_length;
            tx_dst_ip     <= udp_dst_ip;
            tx_dst_mac    <= udp_dst_mac;
        end if;
    end process;

    -- -----------------------------------------------------------------------
    -- HPSDR ip_send (Verilog)
    -- Note: ARP replies bypass ip_send; the arbiter feeds ip_send.tx_enable
    -- only when a non-ARP frame is being sent.
    -- -----------------------------------------------------------------------
    U_ip_send : ip_send
        port map (
            reset          => reset,
            clock          => clock,
            tx_enable      => tx_enable_mux,
            active         => ips_active,
            data_in        => tx_data_mux,
            data_out       => ips_data,
            is_icmp        => tx_is_icmp,
            length         => tx_length_mux,
            local_ip       => IP_SLV,
            destination_ip => tx_dst_ip
        );

    -- -----------------------------------------------------------------------
    -- HPSDR mac_send (Verilog)
    -- Feeds either ARP payload (bypasses ip_send) or ip_send output.
    -- -----------------------------------------------------------------------
    U_mac_send : mac_send
        port map (
            clock           => clock,
            reset           => reset,
            tx_enable       => ips_active,
            active          => macs_active,
            data_in         => ips_data,
            data_out        => macs_data,
            local_mac       => MAC_SLV,
            destination_mac => tx_dst_mac
        );

    -- -----------------------------------------------------------------------
    -- EEM TX framer: byte stream → EEM packet → RX1 FIFO
    -- -----------------------------------------------------------------------
    U_eem_tx_framer : entity work.eem_tx_framer
        port map (
            clock        => clock,
            reset        => reset,
            mac_active   => macs_active,
            mac_data     => macs_data,
            fifo_write   => eem_fifo_write,
            fifo_full    => eem_fifo_full,
            fifo_data    => eem_fifo_data
        );

end architecture;
