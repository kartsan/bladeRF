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
    use ieee.math_real.all;
    use ieee.math_complex.all;

library work;
    use work.bladerf;
    use work.bladerf_p.all;
    use work.fifo_readwrite_p.all;

architecture hosted_bladerf of bladerf is

    attribute noprune          : boolean;
    attribute keep             : boolean;

    alias  sys_reset_async     : std_logic is fx3_ctl(7);
    signal sys_reset_pclk      : std_logic;
    signal sys_reset           : std_logic;

    signal sys_clock           : std_logic;
    signal sys_clock_out       : std_logic;
    signal sys_pll_locked      : std_logic;
    signal sys_pll_reset       : std_logic;

    signal fx3_pclk_pll        : std_logic;
    signal fx3_pclk_pll_out    : std_logic;
    signal fx3_pclk_pll_locked : std_logic;
    signal fx3_pclk_pll_reset  : std_logic;

    signal rx_mux_sel             : unsigned(2 downto 0);

    signal nios_xb_gpio_in        : std_logic_vector(31 downto 0) := (others => '0');
    signal nios_xb_gpio_out       : std_logic_vector(31 downto 0) := (others => '0');
    signal nios_xb_gpio_oe        : std_logic_vector(31 downto 0) := (others => '0');

    signal nios_gpio              : nios_gpio_t;
    signal nios_gpo_slv           : std_logic_vector(31 downto 0);

    signal i2c_scl_in             : std_logic;
    signal i2c_scl_out            : std_logic;
    signal i2c_scl_oen            : std_logic;

    signal i2c_sda_in             : std_logic;
    signal i2c_sda_out            : std_logic;
    signal i2c_sda_oen            : std_logic;

    signal tx_sample_fifo         : tx_fifo_t       := TX_FIFO_T_DEFAULT;
    signal rx_sample_fifo         : rx_fifo_t       := RX_FIFO_T_DEFAULT;
    signal tx_loopback_fifo       : loopback_fifo_t := LOOPBACK_FIFO_T_DEFAULT;

    signal tx_meta_fifo           : meta_fifo_tx_t := META_FIFO_TX_T_DEFAULT;
    signal rx_meta_fifo           : meta_fifo_rx_t := META_FIFO_RX_T_DEFAULT;

    -- EEM RX path (FPGA -> host, GPIF RX1): driven by eem_tx_framer.
    --
    -- The framer presents a show-ahead 32-bit interface from a small
    -- register-array (combinatorial read).  This bypasses the two known
    -- traps of nuand sync_fifo (READ_AHEAD generic unused -> always 1-cycle
    -- latency, simultaneous read+write silently corrupts state) that broke
    -- the first attempt during bringup; see memory notes
    -- project_eem_gpif_link_bringup.md and project_eem_gpif_wrapup_rx1_artifacts.md.
    signal eem_rx_fifo_rreq  : std_logic;
    signal eem_rx_fifo_rdata : std_logic_vector(31 downto 0);
    signal eem_rx_fifo_empty : std_logic;

    -- eem_tx_framer observability (FPGA -> host EEM packet transmission)
    signal eem_tx_pkt_done   : std_logic;
    signal eem_tx_pkt_count  : std_logic_vector(15 downto 0);

    -- EEM TX FIFO: fx3_gpif -> EEM RX logic (host->FPGA, TX2, pclk domain)
    signal eem_tx_fifo_wreq       : std_logic := '0';
    signal eem_tx_fifo_wdata      : std_logic_vector(31 downto 0);
    signal eem_tx_fifo_full       : std_logic;
    signal eem_tx_fifo_rreq       : std_logic := '0';
    signal eem_tx_fifo_rdata      : std_logic_vector(31 downto 0);
    signal eem_tx_fifo_empty      : std_logic;

    -- eem_rx_consumer observability + Ethernet byte stream output
    signal eem_pkt_done_pulse     : std_logic := '0';
    signal eem_pkt_count          : std_logic_vector(15 downto 0) := (others => '0');
    signal eem_last_eth_length    : std_logic_vector(13 downto 0) := (others => '0');
    signal eem_last_bmtype        : std_logic := '0';

    signal eth_rx_data            : std_logic_vector(7 downto 0);
    signal eth_rx_valid           : std_logic;
    signal eth_rx_sop             : std_logic;
    signal eth_rx_eop             : std_logic;
    signal eth_rx_length          : std_logic_vector(13 downto 0);

    -- Local MAC from chip_id_mac (Cyclone V chip ID + fold).
    signal local_mac              : std_logic_vector(47 downto 0);

    -- eth_rx_demux output channels + sidebands
    signal arp_rx_data            : std_logic_vector(7 downto 0);
    signal arp_rx_valid           : std_logic;
    signal arp_rx_sop             : std_logic;
    signal arp_rx_eop             : std_logic;
    signal arp_rx_length          : std_logic_vector(13 downto 0);

    signal ip_rx_data             : std_logic_vector(7 downto 0);
    signal ip_rx_valid            : std_logic;
    signal ip_rx_sop              : std_logic;
    signal ip_rx_eop              : std_logic;
    signal ip_rx_length           : std_logic_vector(13 downto 0);

    signal eth_rx_src_mac         : std_logic_vector(47 downto 0);
    signal eth_rx_ethertype       : std_logic_vector(15 downto 0);

    -- ARP responder -> framer byte stream (replaces the old test source).
    signal arp_tx_data            : std_logic_vector(7 downto 0);
    signal arp_tx_valid           : std_logic;
    signal arp_tx_sop             : std_logic;
    signal arp_tx_eop             : std_logic;
    signal arp_tx_length          : unsigned(13 downto 0);
    signal arp_tx_ready           : std_logic;
    signal arp_reply_pulse        : std_logic;

    -- udp_rx_handler -> dhcp_client byte stream (dst_port=68 classified)
    signal dhcp_rx_data           : std_logic_vector(7 downto 0);
    signal dhcp_rx_valid          : std_logic;
    signal dhcp_rx_sop            : std_logic;
    signal dhcp_rx_eop            : std_logic;
    signal dhcp_rx_length         : std_logic_vector(13 downto 0);
    signal dhcp_rx_pulse          : std_logic;

    -- dhcp_client -> tx_arbiter (port D) byte stream
    signal dhcp_tx_data           : std_logic_vector(7 downto 0);
    signal dhcp_tx_valid          : std_logic;
    signal dhcp_tx_sop            : std_logic;
    signal dhcp_tx_eop            : std_logic;
    signal dhcp_tx_length         : unsigned(13 downto 0);
    signal dhcp_tx_ready          : std_logic;
    signal dhcp_send_pulse        : std_logic;
    signal dhcp_bound_pulse       : std_logic;

    -- Leased L3 surface from dhcp_client + the effective_* muxes that
    -- the downstream IP-stack modules actually consume.  Pre-lease (and
    -- whenever leased_ip_valid='0') effective_ip falls back to the
    -- static EEM_OUR_IP from bladerf_p, and effective_subnet_mask falls
    -- back to EEM_OUR_SUBNET_MASK (a /24 paired with EEM_OUR_IP), so
    -- ARP / ICMP / UDP / subnet-directed broadcast all keep working
    -- during the DHCP acquisition window and as a safe default if DHCP
    -- never completes.  effective_subnet_bcast is the runtime subnet-
    -- directed broadcast (effective_ip OR ~effective_subnet_mask) that
    -- ip_rx_handler accepts in addition to our_ip and 255.255.255.255.
    signal leased_ip              : std_logic_vector(31 downto 0);
    signal leased_ip_valid        : std_logic;
    signal leased_subnet_mask     : std_logic_vector(31 downto 0);
    signal dhcp_server_ip         : std_logic_vector(31 downto 0);
    signal effective_ip           : std_logic_vector(31 downto 0);
    signal effective_subnet_mask  : std_logic_vector(31 downto 0);
    signal effective_subnet_bcast : std_logic_vector(31 downto 0);

    -- dhcp_server_ip isn't consumed downstream yet (kept for future
    -- unicast renewal); the others are now real consumers and don't
    -- need the keep pragma anymore.
    attribute keep of dhcp_server_ip : signal is true;

    -- ip_rx_handler outputs.  UDP side feeds udp_rx_handler; ICMP side
    -- feeds icmp_responder.
    signal udp_rx_data            : std_logic_vector(7 downto 0);
    signal udp_rx_valid           : std_logic;
    signal udp_rx_sop             : std_logic;
    signal udp_rx_eop             : std_logic;
    signal udp_rx_length          : std_logic_vector(13 downto 0);

    signal icmp_rx_data           : std_logic_vector(7 downto 0);
    signal icmp_rx_valid          : std_logic;
    signal icmp_rx_sop            : std_logic;
    signal icmp_rx_eop            : std_logic;
    signal icmp_rx_length         : std_logic_vector(13 downto 0);

    signal ip_rx_src_ip           : std_logic_vector(31 downto 0);
    signal ip_rx_dst_ip           : std_logic_vector(31 downto 0);
    signal ip_rx_pulse            : std_logic;

    -- udp_rx_handler -> hpsdr_discovery_responder byte stream
    -- (dst_port=1024 classified, Eth/IP/UDP stripped).
    signal hpsdr_rx_data          : std_logic_vector(7 downto 0);
    signal hpsdr_rx_valid         : std_logic;
    signal hpsdr_rx_sop           : std_logic;
    signal hpsdr_rx_eop           : std_logic;
    signal hpsdr_rx_length        : std_logic_vector(13 downto 0);
    signal udp_src_port           : std_logic_vector(15 downto 0);
    signal udp_dst_port           : std_logic_vector(15 downto 0);
    signal hpsdr_pulse            : std_logic;

    -- udp_src_port is consumed by hpsdr_discovery_responder (the
    -- reply's UDP dst port must equal the probe's UDP src port -- Thetis
    -- /piHPSDR strictly require it).  udp_dst_port isn't consumed yet
    -- (the responder already knows it answered a port-1024 probe by
    -- virtue of being on the hpsdr_* channel).  Both still keep-pinned
    -- as SignalTap surfaces.  Same for ip_rx_dst_ip / ip_rx_pulse
    -- which are still observability-only.
    attribute keep of udp_src_port    : signal is true;
    attribute keep of udp_dst_port    : signal is true;
    attribute keep of ip_rx_dst_ip    : signal is true;
    attribute keep of ip_rx_pulse     : signal is true;

    -- hpsdr_discovery_responder -> tx_arbiter (port D) byte stream
    signal hpsdr_tx_data          : std_logic_vector(7 downto 0);
    signal hpsdr_tx_valid         : std_logic;
    signal hpsdr_tx_sop           : std_logic;
    signal hpsdr_tx_eop           : std_logic;
    signal hpsdr_tx_length        : unsigned(13 downto 0);
    signal hpsdr_tx_ready         : std_logic;
    signal hpsdr_disc_reply_pulse : std_logic;

    -- Committed HPSDR client identity, captured by
    -- hpsdr_discovery_responder at the moment it accepts a discovery
    -- probe.  Consumed by hpsdr_hp_status_sender to address its ~30 Hz
    -- heartbeat back to the discovered host; will also feed future DDC
    -- IQ streamers etc.  host_port is the host's ephemeral source port
    -- from the probe, = destination for UDP/1025 high-priority status
    -- and IQ streams (NOT 1025 / NOT 1035, regardless of HPSDR
    -- convention -- Thetis/piHPSDR bind their receive socket to the
    -- sendto() source port; see [[feedback_hpsdr_reply_udp_dst_port]]).
    signal hpsdr_host_mac         : std_logic_vector(47 downto 0);
    signal hpsdr_host_ip          : std_logic_vector(31 downto 0);
    signal hpsdr_host_port        : std_logic_vector(15 downto 0);
    signal hpsdr_host_valid       : std_logic;

    -- hpsdr_hp_status_sender -> tx_arbiter (port E) byte stream
    signal hpsdr_hp_tx_data       : std_logic_vector(7 downto 0);
    signal hpsdr_hp_tx_valid      : std_logic;
    signal hpsdr_hp_tx_sop        : std_logic;
    signal hpsdr_hp_tx_eop        : std_logic;
    signal hpsdr_hp_tx_length     : unsigned(13 downto 0);
    signal hpsdr_hp_tx_ready      : std_logic;
    signal hpsdr_hp_send_pulse    : std_logic;

    -- icmp_responder -> tx_arbiter byte stream
    signal icmp_tx_data           : std_logic_vector(7 downto 0);
    signal icmp_tx_valid          : std_logic;
    signal icmp_tx_sop            : std_logic;
    signal icmp_tx_eop            : std_logic;
    signal icmp_tx_length         : unsigned(13 downto 0);
    signal icmp_tx_ready          : std_logic;
    signal icmp_reply_pulse       : std_logic;

    -- tx_arbiter -> eem_tx_framer byte stream
    signal mux_tx_data            : std_logic_vector(7 downto 0);
    signal mux_tx_valid           : std_logic;
    signal mux_tx_sop             : std_logic;
    signal mux_tx_eop             : std_logic;
    signal mux_tx_length          : unsigned(13 downto 0);
    signal mux_tx_ready           : std_logic;

    signal eem_rx_led             : std_logic := '1';
    signal eem_dma_req_led        : std_logic := '1';

    signal usb_speed_pclk         : std_logic;
    signal usb_speed_rx           : std_logic;
    signal usb_speed_tx           : std_logic;

    signal tx_reset               : std_logic;
    signal rx_reset               : std_logic;

    signal tx_enable_pclk         : std_logic;
    signal rx_enable_pclk         : std_logic;

    signal tx_enable              : std_logic;
    signal rx_enable              : std_logic;

    signal meta_en_pclk           : std_logic;
    signal meta_en_tx             : std_logic;
    signal meta_en_rx             : std_logic;

    signal eightbit_en_pclk       : std_logic;
    signal eightbit_en_tx         : std_logic;
    signal eightbit_en_rx         : std_logic;

    signal highly_packed_en_txrx  : std_logic;

    signal packet_en_pclk         : std_logic;
    signal packet_en_tx           : std_logic;
    signal packet_en_rx           : std_logic;

    signal tx_timestamp           : unsigned(63 downto 0);
    signal rx_timestamp           : unsigned(63 downto 0);
    signal timestamp_sync         : std_logic;

    signal tx_loopback_enabled    : std_logic := '0';

    signal fx3_gpif_in            : std_logic_vector(31 downto 0);
    signal fx3_gpif_out           : std_logic_vector(31 downto 0);
    signal fx3_gpif_oe            : std_logic;

    signal fx3_ctl_in             : std_logic_vector(12 downto 0);
    signal fx3_ctl_out            : std_logic_vector(12 downto 0);
    signal fx3_ctl_oe             : std_logic_vector(12 downto 0);

    signal tx_underflow_led       : std_logic := '1';
    signal rx_overflow_led        : std_logic := '1';

    signal led1_blink             : std_logic;

    signal nios_sdo               : std_logic;
    signal nios_sdio              : std_logic;
    signal nios_sclk              : std_logic;
    signal nios_ss_n              : std_logic_vector(1 downto 0);

    signal command_serial_in      : std_logic;
    signal command_serial_out     : std_logic;

    signal timestamp_req          : std_logic;
    signal timestamp_ack          : std_logic;
    signal fx3_timestamp          : unsigned(63 downto 0);

    signal rx_ts_reset            : std_logic;
    signal tx_ts_reset            : std_logic;

    signal rx_trigger_ctl_i       : std_logic_vector(7 downto 0);
    signal rx_trigger_ctl         : trigger_t := TRIGGER_T_DEFAULT;
    alias  rx_trigger_line        : std_logic is mini_exp1;

    signal tx_trigger_ctl_i       : std_logic_vector(7 downto 0);
    signal tx_trigger_ctl         : trigger_t := TRIGGER_T_DEFAULT;
    alias  tx_trigger_line        : std_logic is mini_exp1;

    signal rffe_gpio              : rffe_gpio_t := (
        i => RFFE_GPI_DEFAULT,
        o => pack(RFFE_GPO_DEFAULT)
    );

    signal ad9361                 : mimo_2r2t_t := MIMO_2R2T_T_DEFAULT;
    alias tx_clock  is ad9361.clock;
    alias rx_clock  is ad9361.clock;

    signal mimo_rx_enables        : std_logic_vector(RFFE_GPO_DEFAULT.mimo_rx_en'range) := RFFE_GPO_DEFAULT.mimo_rx_en;
    signal mimo_tx_enables        : std_logic_vector(RFFE_GPO_DEFAULT.mimo_tx_en'range) := RFFE_GPO_DEFAULT.mimo_tx_en;

    signal dac_controls           : sample_controls_t(ad9361.ch'range)    := (others => SAMPLE_CONTROL_DISABLE);
    signal dac_streams            : sample_streams_t(dac_controls'range)  := (others => ZERO_SAMPLE);
    signal adc_controls           : sample_controls_t(ad9361.ch'range)    := (others => SAMPLE_CONTROL_DISABLE);
    signal adc_streams            : sample_streams_t(adc_controls'range)  := (others => ZERO_SAMPLE);
    signal adc_streams_last_v     : std_logic_vector(adc_controls'range)  := (others => '0');

    signal   ps_sync              : std_logic_vector(0 downto 0)          := (others => '0');


    signal tx_packet_control      : packet_control_t ;
    signal rx_packet_control      : packet_control_t := PACKET_CONTROL_DEFAULT ;

    signal rx_packet_ready        : std_logic;

    signal tx_packet_ready        : std_logic;
    signal tx_packet_empty        : std_logic;


    signal wbm_wb_clk_i           : std_logic;
    signal wbm_wb_rst_i           : std_logic;
    signal wbm_wb_adr_o           : std_logic_vector(31 downto 0);
    signal wbm_wb_dat_o           : std_logic_vector(31 downto 0);
    signal wbm_wb_dat_i           : std_logic_vector(31 downto 0);
    signal wbm_wb_we_o            : std_logic;
    signal wbm_wb_sel_o           : std_logic;
    signal wbm_wb_stb_o           : std_logic;
    signal wbm_wb_ack_i           : std_logic;
    signal wbm_wb_cyc_o           : std_logic;
begin

    U_rx_pkt_gen : entity work.rx_packet_generator
        port map(
            rx_clock               => rx_clock,
            rx_reset               => rx_reset,

            rx_packet_ready        => rx_packet_ready,

            rx_enable              => rx_enable,
            rx_packet_enable       => packet_en_rx,

            rx_packet_control      => rx_packet_control
        ) ;


    -- ========================================================================
    -- PLLs
    -- ========================================================================

    -- Create 80 MHz system clock from 38.4 MHz
    U_system_pll : component system_pll
        port map (
            refclk   => c5_clock2,
            rst      => sys_pll_reset,
            outclk_0 => sys_clock_out,
            locked   => sys_pll_locked
        );

    U_system_pll_ctrl : component clk_ctrl
        port map (
            inclk   => sys_clock_out,
            ena     => sys_pll_locked,
            outclk  => sys_clock
        );

    U_pll_reset_pll : entity work.pll_reset
        generic map (
            SYS_CLOCK_FREQ_HZ   => 38_400_000,
            DEVICE_FAMILY       => "Cyclone V"
        )
        port map (
            sys_clock      => c5_clock2,
            pll_locked     => sys_pll_locked,
            pll_reset      => sys_pll_reset
        );

    -- Use PLL to adjust the phase of the FX3 PCLK to
    -- retime the FX3 GPIF interface for timing closure.
    U_fx3_pll : component fx3_pll
        port map (
            refclk   =>  fx3_pclk,
            rst      =>  fx3_pclk_pll_reset,
            outclk_0 =>  fx3_pclk_pll_out,
            locked   =>  fx3_pclk_pll_locked
        );

    U_fx3_pll_ctrl : component clk_ctrl
        port map (
            inclk   => fx3_pclk_pll_out,
            ena     => fx3_pclk_pll_locked,
            outclk  => fx3_pclk_pll
        );

    U_pll_reset_fx3_pll : entity work.pll_reset
        generic map (
            SYS_CLOCK_FREQ_HZ   => 100_000_000,
            DEVICE_FAMILY       => "Cyclone V"
        )
        port map (
            sys_clock      => fx3_pclk,
            pll_locked     => fx3_pclk_pll_locked,
            pll_reset      => fx3_pclk_pll_reset
        );


    -- ========================================================================
    -- POWER SUPPLY SYNCHRONIZATION
    -- ========================================================================

    U_ps_sync : entity work.ps_sync
        generic map (
            OUTPUTS  => 1,
            USE_LFSR => true,
            HOP_LIST => adp2384_sync_divisors( REFCLK_HZ  => 38.4e6,
                                               n_divisors => 7 ),
            HOP_RATE => 100
        )
        port map (
            refclk   => c5_clock2,
            sync     => ps_sync
        );

    ps_sync_1p1 <= ps_sync(0);
    ps_sync_1p8 <= ps_sync(0);

    -- ========================================================================
    -- FX3 GPIF
    -- ========================================================================

    -- FX3 GPIF
    U_fx3_gpif : entity work.fx3_gpif
        port map (
            pclk                =>  fx3_pclk_pll,
            reset               =>  sys_reset_pclk,

            usb_speed           =>  usb_speed_pclk,

            meta_enable         =>  meta_en_pclk,
            packet_enable       =>  packet_en_pclk,
            rx_enable           =>  rx_enable_pclk,
            tx_enable           =>  tx_enable_pclk,

            gpif_in             =>  fx3_gpif_in,
            gpif_out            =>  fx3_gpif_out,
            gpif_oe             =>  fx3_gpif_oe,
            ctl_in              =>  fx3_ctl_in,
            ctl_out             =>  fx3_ctl_out,
            ctl_oe              =>  fx3_ctl_oe,

            tx_fifo_write       =>  tx_sample_fifo.wreq,
            tx_fifo_full        =>  tx_sample_fifo.wfull,
            tx_fifo_empty       =>  tx_sample_fifo.wempty,
            tx_fifo_usedw       =>  tx_sample_fifo.wused,
            tx_fifo_data        =>  tx_sample_fifo.wdata,

            tx_timestamp        =>  fx3_timestamp,
            tx_meta_fifo_write  =>  tx_meta_fifo.wreq,
            tx_meta_fifo_full   =>  tx_meta_fifo.wfull,
            tx_meta_fifo_empty  =>  tx_meta_fifo.wempty,
            tx_meta_fifo_usedw  =>  tx_meta_fifo.wused,
            tx_meta_fifo_data   =>  tx_meta_fifo.wdata,

            rx_fifo_read        =>  rx_sample_fifo.rreq,
            rx_fifo_full        =>  rx_sample_fifo.rfull,
            rx_fifo_empty       =>  rx_sample_fifo.rempty,
            rx_fifo_usedw       =>  rx_sample_fifo.rused,
            rx_fifo_data        =>  rx_sample_fifo.rdata,

            rx_meta_fifo_read   =>  rx_meta_fifo.rreq,
            rx_meta_fifo_full   =>  rx_meta_fifo.rfull,
            rx_meta_fifo_empty  =>  rx_meta_fifo.rempty,
            rx_meta_fifo_usedr  =>  rx_meta_fifo.rused,
            rx_meta_fifo_data   =>  rx_meta_fifo.rdata,

            eem_rx_fifo_read    =>  eem_rx_fifo_rreq,
            eem_rx_fifo_empty   =>  eem_rx_fifo_empty,
            eem_rx_fifo_data    =>  eem_rx_fifo_rdata,

            eem_tx_fifo_write   =>  eem_tx_fifo_wreq,
            eem_tx_fifo_full    =>  eem_tx_fifo_full,
            eem_tx_fifo_data    =>  eem_tx_fifo_wdata
        );

    -- FX3 GPIF bidirectional signal control
    register_gpif : process(sys_reset_pclk, fx3_pclk_pll)
    begin
        if( sys_reset_pclk = '1' ) then
            fx3_gpif    <= (others =>'Z');
            fx3_gpif_in <= (others =>'0');
        elsif( rising_edge(fx3_pclk_pll) ) then
            fx3_gpif_in <= fx3_gpif;
            if( fx3_gpif_oe = '1' ) then
                fx3_gpif <= fx3_gpif_out;
            else
                fx3_gpif <= (others =>'Z');
            end if;
        end if;
    end process;

    -- FX3 CTL bidirectional signal control
    generate_ctl : for i in fx3_ctl'range generate
        fx3_ctl(i) <= fx3_ctl_out(i) when fx3_ctl_oe(i) = '1' else 'Z';
    end generate;

    fx3_ctl_in <= fx3_ctl;

    -- ========================================================================
    -- EEM SYNCHRONOUS FIFOs (pclk domain only, no clock-domain crossing)
    -- ========================================================================

    -- Note: the EEM RX (FPGA->host) path has no FIFO instance. eem_tx_framer
    -- presents a show-ahead 32-bit interface directly from its internal
    -- register-array buffer; see eem_tx_framer.vhd for why (Cyclone V BRAMs
    -- always have a registered output, which breaks the 0-cycle-latency
    -- timing fx3_gpif's SAMPLE_READ requires for RX1).

    -- EEM TX: fx3_gpif writes here via TX2 DMA channel; EEM RX logic reads.
    -- Read side (eem_tx_fifo_rreq / eem_tx_fifo_rdata) connects to EEM RX logic.
    --
    -- Uses work.eem_sync_fifo (EEM-local fork) instead of the shared
    -- nuand.sync_fifo, because the shared version silently corrupts state
    -- on simultaneous read+write -- a collision pattern that's normal for
    -- the EEM TX FIFO (100 MHz fx3_gpif writer + 100 MHz consumer reader
    -- in the same clock domain).  See feedback_sync_fifo_rw_collision.md.
    U_eem_tx_fifo : entity work.eem_sync_fifo
        generic map (
            DEPTH       =>  1024,
            WIDTH       =>  32,
            READ_AHEAD  =>  true
        )
        port map (
            areset      =>  sys_reset_pclk,
            clock       =>  fx3_pclk_pll,
            full        =>  eem_tx_fifo_full,
            empty       =>  eem_tx_fifo_empty,
            used_words  =>  open,
            data_in     =>  eem_tx_fifo_wdata,
            write_en    =>  eem_tx_fifo_wreq,
            data_out    =>  eem_tx_fifo_rdata,
            read_en     =>  eem_tx_fifo_rreq
        );

    -- ========================================================================
    -- Host->FPGA EEM RX path
    -- ------------------------------------------------------------------------
    -- Parses the EEM header on every packet delivered via fx3_gpif's TX2 DMA,
    -- counts packets, and discards the payload (TX-direction bringup only;
    -- payload routing comes later). Drains eem_tx_fifo at one read per two
    -- pclk cycles, well above EEM bandwidth need and keeping eem_tx_fifo_full
    -- low so GPIF TX2 acks stay open.
    -- ========================================================================
    U_eem_rx_consumer : entity work.eem_rx_consumer
        port map (
            clock           => fx3_pclk_pll,
            reset           => sys_reset_pclk,

            fifo_empty      => eem_tx_fifo_empty,
            fifo_rdata      => eem_tx_fifo_rdata,
            fifo_rreq       => eem_tx_fifo_rreq,

            eth_data        => eth_rx_data,
            eth_valid       => eth_rx_valid,
            eth_sop         => eth_rx_sop,
            eth_eop         => eth_rx_eop,
            eth_length      => eth_rx_length,

            pkt_done_pulse  => eem_pkt_done_pulse,
            pkt_count       => eem_pkt_count,
            last_eth_length => eem_last_eth_length,
            last_bmtype     => eem_last_bmtype
        );

    -- ========================================================================
    -- Local MAC source
    -- ------------------------------------------------------------------------
    -- chip_id_mac wraps cv_chip_id_reader (direct Cyclone V chipidblock
    -- primitive instantiation) and folds the 64-bit chip ID to a 40-bit MAC
    -- tail with the IEEE-802 locally-administered unicast prefix 0x02.  Its
    -- reset value is a well-formed fallback MAC, so any ARP reply emitted
    -- in the ~65 clocks before the chip-ID shift completes still has a
    -- valid source MAC; the host's ARP cache just updates when the real
    -- per-board value appears.
    -- ========================================================================
    U_chip_id_mac : entity work.chip_id_mac
        port map (
            clock     => fx3_pclk_pll,
            reset     => sys_reset_pclk,
            local_mac => local_mac
        );

    -- ========================================================================
    -- effective_ip / effective_subnet_mask / effective_subnet_bcast: the
    -- L3 identity that the IP stack (arp_responder, icmp_responder,
    -- ip_rx_handler, and future FPGA-originated senders such as HPSDR)
    -- actually answers / sources on.  Track the DHCP-leased values when
    -- valid; otherwise fall back to the static EEM_OUR_IP
    -- (= 192.168.1.2 / 24) so pre-DHCP traffic and DHCP-failure cases
    -- stay reachable on the bring-up IP.  Pure combinational muxes --
    -- the transition happens in the same cycle that dhcp_client raises
    -- our_ip_valid (= S_BOUND entry).
    --
    -- effective_subnet_bcast = effective_ip OR ~effective_subnet_mask.
    -- For a /24 (mask 0xFFFFFF00) at IP 192.168.1.2 this is 192.168.1.255.
    -- If the DHCP server didn't supply Option 1 then leased_subnet_mask
    -- = 0x00000000, NOT-ing gives 0xFFFFFFFF, and effective_subnet_bcast
    -- collapses to 0xFFFFFFFF (= limited broadcast, which ip_rx_handler
    -- already accepts via BROADCAST_IP -- safe degenerate behaviour).
    -- ========================================================================
    effective_ip          <= leased_ip          when leased_ip_valid = '1'
                                                else EEM_OUR_IP;
    effective_subnet_mask <= leased_subnet_mask when leased_ip_valid = '1'
                                                else EEM_OUR_SUBNET_MASK;
    effective_subnet_bcast <= effective_ip or (not effective_subnet_mask);

    -- ========================================================================
    -- Inbound Ethernet demux: routes by ethertype.
    --   0x0806 -> arp_responder
    --   0x0800 -> (future ip_rx_handler; channel currently unconnected)
    -- Frames with dst MAC != broadcast and != local_mac are silently dropped.
    -- ========================================================================
    U_eth_rx_demux : entity work.eth_rx_demux
        port map (
            clock      => fx3_pclk_pll,
            reset      => sys_reset_pclk,

            our_mac    => local_mac,

            in_data    => eth_rx_data,
            in_valid   => eth_rx_valid,
            in_sop     => eth_rx_sop,
            in_eop     => eth_rx_eop,
            in_length  => eth_rx_length,

            arp_data   => arp_rx_data,
            arp_valid  => arp_rx_valid,
            arp_sop    => arp_rx_sop,
            arp_eop    => arp_rx_eop,
            arp_length => arp_rx_length,

            ip_data    => ip_rx_data,
            ip_valid   => ip_rx_valid,
            ip_sop     => ip_rx_sop,
            ip_eop     => ip_rx_eop,
            ip_length  => ip_rx_length,

            src_mac    => eth_rx_src_mac,
            ethertype  => eth_rx_ethertype
        );

    -- ========================================================================
    -- ARP responder for effective_ip (leased post-DHCP, EEM_OUR_IP
    -- fallback pre-DHCP).  Replies with local_mac to every ARP Request
    -- whose TPA matches us.  Drives tx_arbiter port A.
    -- ========================================================================
    U_arp_responder : entity work.arp_responder
        port map (
            clock       => fx3_pclk_pll,
            reset       => sys_reset_pclk,

            our_ip      => effective_ip,
            our_mac     => local_mac,

            rx_data     => arp_rx_data,
            rx_valid    => arp_rx_valid,
            rx_sop      => arp_rx_sop,
            rx_eop      => arp_rx_eop,

            tx_data        => arp_tx_data,
            tx_valid       => arp_tx_valid,
            tx_sop         => arp_tx_sop,
            tx_eop         => arp_tx_eop,
            tx_length      => arp_tx_length,
            tx_ready       => arp_tx_ready,

            -- Snooped host MAC outputs left unconnected.  They drove
            -- the (now-removed) udp_tx_injector; left in the responder
            -- entity as a cheap debug surface (SignalTap, future use).
            peer_mac       => open,
            peer_mac_valid => open,

            reply_pulse    => arp_reply_pulse
        );

    -- ========================================================================
    -- IPv4 RX handler.  Walks the 20-byte IP header, filters dst-IP for
    -- {effective_ip, 255.255.255.255, effective_subnet_bcast}, drops
    -- fragmented / non-strict-IHL / non-IPv4 / non-{UDP,ICMP} packets,
    -- and routes payloads to the udp_rx_* and icmp_rx_* channels
    -- respectively.  src_ip / dst_ip / rx_pulse are latched at classify
    -- time for downstream use.  The subnet-directed broadcast lets us
    -- receive Thetis/piHPSDR-style discovery probes that target x.x.x.255
    -- on our subnet (the dnsmasq / router-supplied netmask via DHCP
    -- Option 1).
    -- ========================================================================
    U_ip_rx_handler : entity work.ip_rx_handler
        port map (
            clock        => fx3_pclk_pll,
            reset        => sys_reset_pclk,

            our_ip       => effective_ip,
            subnet_bcast => effective_subnet_bcast,

            rx_data     => ip_rx_data,
            rx_valid    => ip_rx_valid,
            rx_sop      => ip_rx_sop,
            rx_eop      => ip_rx_eop,
            rx_length   => ip_rx_length,

            udp_data    => udp_rx_data,
            udp_valid   => udp_rx_valid,
            udp_sop     => udp_rx_sop,
            udp_eop     => udp_rx_eop,
            udp_length  => udp_rx_length,

            icmp_data   => icmp_rx_data,
            icmp_valid  => icmp_rx_valid,
            icmp_sop    => icmp_rx_sop,
            icmp_eop    => icmp_rx_eop,
            icmp_length => icmp_rx_length,

            src_ip      => ip_rx_src_ip,
            dst_ip      => ip_rx_dst_ip,

            rx_pulse    => ip_rx_pulse
        );

    -- ========================================================================
    -- UDP RX handler.  Walks the 8-byte UDP header on every IP-handler
    -- udp_* packet, classifies by dst_port, and routes the payload to the
    -- matching application channel.  Two recognised ports:
    --   dst_port = 1024 -> hpsdr_* (consumed by hpsdr_discovery_responder)
    --   dst_port = 68   -> dhcp_*  (consumed by dhcp_client)
    -- hpsdr_pulse blinks led(3) on every classified port-1024 packet (the
    -- reply itself separately blinks via hpsdr_disc_reply_pulse).
    -- ========================================================================
    U_udp_rx_handler : entity work.udp_rx_handler
        port map (
            clock        => fx3_pclk_pll,
            reset        => sys_reset_pclk,

            rx_data      => udp_rx_data,
            rx_valid     => udp_rx_valid,
            rx_sop       => udp_rx_sop,
            rx_eop       => udp_rx_eop,
            rx_length    => udp_rx_length,

            hpsdr_data   => hpsdr_rx_data,
            hpsdr_valid  => hpsdr_rx_valid,
            hpsdr_sop    => hpsdr_rx_sop,
            hpsdr_eop    => hpsdr_rx_eop,
            hpsdr_length => hpsdr_rx_length,

            dhcp_data    => dhcp_rx_data,
            dhcp_valid   => dhcp_rx_valid,
            dhcp_sop     => dhcp_rx_sop,
            dhcp_eop     => dhcp_rx_eop,
            dhcp_length  => dhcp_rx_length,

            src_port     => udp_src_port,
            dst_port     => udp_dst_port,

            hpsdr_pulse  => hpsdr_pulse,
            dhcp_pulse   => dhcp_rx_pulse
        );

    -- ========================================================================
    -- ICMP Echo (ping) responder.  Sits on ip_rx_handler's icmp_* channel,
    -- mirrors Type-8/Code-0 Echo Requests back as Type-0 Echo Replies with
    -- updated checksums.  Drives the tx_arbiter's B port.
    -- ========================================================================
    U_icmp_responder : entity work.icmp_responder
        port map (
            clock        => fx3_pclk_pll,
            reset        => sys_reset_pclk,

            our_ip       => effective_ip,
            our_mac      => local_mac,
            peer_mac     => eth_rx_src_mac,
            peer_ip      => ip_rx_src_ip,

            rx_data      => icmp_rx_data,
            rx_valid     => icmp_rx_valid,
            rx_sop       => icmp_rx_sop,
            rx_eop       => icmp_rx_eop,
            rx_length    => icmp_rx_length,

            tx_data      => icmp_tx_data,
            tx_valid     => icmp_tx_valid,
            tx_sop       => icmp_tx_sop,
            tx_eop       => icmp_tx_eop,
            tx_length    => icmp_tx_length,
            tx_ready     => icmp_tx_ready,

            reply_pulse  => icmp_reply_pulse
        );

    -- ========================================================================
    -- DHCP client.  Walks DISCOVER -> OFFER -> REQUEST -> ACK against
    -- the host's DHCP server (e.g. the dedicated dnsmasq instance on
    -- usb0 with `--port=0 --interface=usb0 --bind-interfaces --dhcp-range
    -- =192.168.1.10,192.168.1.20`).  On success, exposes the leased IPv4
    -- via `leased_ip` + `leased_ip_valid`; nothing consumes these yet --
    -- they're "keep"-pinned signals visible to SignalTap / journalctl
    -- correlation during bring-up.  Drives tx_arbiter port D (lowest
    -- priority).  RX comes off udp_rx_handler's dhcp_* channel
    -- (dst_port=68).  2 s boot delay before first DISCOVER; 4 s OFFER /
    -- ACK timeouts with restart-from-DISCOVER on failure.
    -- ========================================================================
    U_dhcp_client : entity work.dhcp_client
        port map (
            clock        => fx3_pclk_pll,
            reset        => sys_reset_pclk,

            our_mac      => local_mac,

            rx_data      => dhcp_rx_data,
            rx_valid     => dhcp_rx_valid,
            rx_sop       => dhcp_rx_sop,
            rx_eop       => dhcp_rx_eop,
            rx_length    => dhcp_rx_length,

            tx_data      => dhcp_tx_data,
            tx_valid     => dhcp_tx_valid,
            tx_sop       => dhcp_tx_sop,
            tx_eop       => dhcp_tx_eop,
            tx_length    => dhcp_tx_length,
            tx_ready     => dhcp_tx_ready,

            our_ip       => leased_ip,
            our_ip_valid => leased_ip_valid,
            server_ip    => dhcp_server_ip,
            subnet_mask  => leased_subnet_mask,

            send_pulse   => dhcp_send_pulse,
            bound_pulse  => dhcp_bound_pulse
        );

    -- ========================================================================
    -- HPSDR Protocol 2 discovery responder.  Sits on udp_rx_handler's
    -- hpsdr_* channel (UDP/1024 payload, Eth/IP/UDP stripped); recognises
    -- the General-Packet "discovery request" by payload byte 4 == 0x02
    -- and replies with a 102-byte Ethernet frame advertising this device
    -- as a Hermes-class HPSDR P2 radio (board type 0x06).  Reply is
    -- unicast back to the probing host: Eth dst = eth_rx_demux's src_mac
    -- sideband, IP dst = ip_rx_handler's src_ip sideband, UDP dst port
    -- = udp_rx_handler's src_port sideband (= host's ephemeral port,
    -- NOT 1024 -- Thetis/piHPSDR bind their receive socket to their
    -- sendto() source port).  All three sidebands are held stable
    -- through the inbound payload duration, so a single snapshot at
    -- rx_sop captures the matching triple.  IP src = effective_ip
    -- (DHCP-leased post-lease, static EEM_OUR_IP pre-lease).
    --
    -- host_mac / host_ip / host_port / host_valid expose the committed
    -- client identity for downstream HPSDR producers so they can target
    -- the discovered host without re-snooping the inbound path.  First
    -- consumer is hpsdr_hp_status_sender immediately below; future DDC
    -- IQ streamers will use the same snapshot.
    --
    -- Drives tx_arbiter port D (lowest priority -- piHPSDR retries
    -- every ~2-3 s).
    -- ========================================================================
    U_hpsdr_discovery_responder : entity work.hpsdr_discovery_responder
        port map (
            clock        => fx3_pclk_pll,
            reset        => sys_reset_pclk,

            our_mac      => local_mac,
            our_ip       => effective_ip,
            peer_mac     => eth_rx_src_mac,
            peer_ip      => ip_rx_src_ip,
            peer_port    => udp_src_port,

            rx_data      => hpsdr_rx_data,
            rx_valid     => hpsdr_rx_valid,
            rx_sop       => hpsdr_rx_sop,
            rx_eop       => hpsdr_rx_eop,
            rx_length    => hpsdr_rx_length,

            tx_data      => hpsdr_tx_data,
            tx_valid     => hpsdr_tx_valid,
            tx_sop       => hpsdr_tx_sop,
            tx_eop       => hpsdr_tx_eop,
            tx_length    => hpsdr_tx_length,
            tx_ready     => hpsdr_tx_ready,

            host_mac     => hpsdr_host_mac,
            host_ip      => hpsdr_host_ip,
            host_port    => hpsdr_host_port,
            host_valid   => hpsdr_host_valid,

            reply_pulse  => hpsdr_disc_reply_pulse
        );

    -- ========================================================================
    -- HPSDR Protocol 2 High-Priority Status sender.  Periodic ~30 Hz
    -- heartbeat on UDP/1025 (radio -> host) that Thetis/piHPSDR watch
    -- for to declare the radio "alive"; without it, Thetis sends a few
    -- 1444-byte High-Priority Commands then closes the connection after
    -- ~3 s of silence.  Gated on hpsdr_host_valid='1' so nothing goes
    -- out before discovery completes.  Addressed using the same
    -- {host_mac, host_ip, host_port} snapshot the discovery responder
    -- captured -- the contract that those three signals were exposed for.
    -- Drives tx_arbiter port E (lowest priority; the heartbeat is the
    -- most loss-tolerant traffic in the control plane).
    -- ========================================================================
    U_hpsdr_hp_status_sender : entity work.hpsdr_hp_status_sender
        port map (
            clock        => fx3_pclk_pll,
            reset        => sys_reset_pclk,

            our_mac      => local_mac,
            our_ip       => effective_ip,

            host_mac     => hpsdr_host_mac,
            host_ip      => hpsdr_host_ip,
            host_port    => hpsdr_host_port,
            host_valid   => hpsdr_host_valid,

            tx_data      => hpsdr_hp_tx_data,
            tx_valid     => hpsdr_hp_tx_valid,
            tx_sop       => hpsdr_hp_tx_sop,
            tx_eop       => hpsdr_hp_tx_eop,
            tx_length    => hpsdr_hp_tx_length,
            tx_ready     => hpsdr_hp_tx_ready,

            send_pulse   => hpsdr_hp_send_pulse
        );

    -- ========================================================================
    -- TX arbiter: multiplexes arp_responder (port A, highest priority),
    -- icmp_responder (port B), dhcp_client (port C),
    -- hpsdr_discovery_responder (port D), and hpsdr_hp_status_sender
    -- (port E, lowest priority) onto the single eem_tx_framer input.
    -- Holds the active producer until the framer's pkt_done_pulse fires,
    -- then releases for the next.  Future HPSDR IQ streaming will likely
    -- replace this priority arbiter with a high-rate scheduler (see
    -- tx_arbiter.vhd's header).
    -- ========================================================================
    U_tx_arbiter : entity work.tx_arbiter
        port map (
            clock     => fx3_pclk_pll,
            reset     => sys_reset_pclk,

            a_data    => arp_tx_data,
            a_valid   => arp_tx_valid,
            a_sop     => arp_tx_sop,
            a_eop     => arp_tx_eop,
            a_length  => arp_tx_length,
            a_ready   => arp_tx_ready,

            b_data    => icmp_tx_data,
            b_valid   => icmp_tx_valid,
            b_sop     => icmp_tx_sop,
            b_eop     => icmp_tx_eop,
            b_length  => icmp_tx_length,
            b_ready   => icmp_tx_ready,

            c_data    => dhcp_tx_data,
            c_valid   => dhcp_tx_valid,
            c_sop     => dhcp_tx_sop,
            c_eop     => dhcp_tx_eop,
            c_length  => dhcp_tx_length,
            c_ready   => dhcp_tx_ready,

            d_data    => hpsdr_tx_data,
            d_valid   => hpsdr_tx_valid,
            d_sop     => hpsdr_tx_sop,
            d_eop     => hpsdr_tx_eop,
            d_length  => hpsdr_tx_length,
            d_ready   => hpsdr_tx_ready,

            e_data    => hpsdr_hp_tx_data,
            e_valid   => hpsdr_hp_tx_valid,
            e_sop     => hpsdr_hp_tx_sop,
            e_eop     => hpsdr_hp_tx_eop,
            e_length  => hpsdr_hp_tx_length,
            e_ready   => hpsdr_hp_tx_ready,

            tx_data   => mux_tx_data,
            tx_valid  => mux_tx_valid,
            tx_sop    => mux_tx_sop,
            tx_eop    => mux_tx_eop,
            tx_length => mux_tx_length,
            tx_ready  => mux_tx_ready,

            pkt_done  => eem_tx_pkt_done
        );

    -- ========================================================================
    -- FPGA -> host EEM TX path
    -- ------------------------------------------------------------------------
    -- eem_tx_framer wraps whatever byte stream is presented on its
    -- frame_in_* port in a CDC EEM data packet (2-byte hdr + 4-byte
    -- 0xDEADBEEF FCS sentinel + 4-byte dummy word) and presents it
    -- word-by-word to fx3_gpif's RX1 SAMPLE_READ via a show-ahead FIFO
    -- interface backed by a sync-write / async-read array (Quartus
    -- auto-infers MLAB / LAB-resident LUT-RAM for the storage, no M10K
    -- and no wide fabric read mux).  Driven by tx_arbiter (port A =
    -- arp_responder, port B = icmp_responder, port C = dhcp_client,
    -- port D = hpsdr_discovery_responder, port E = hpsdr_hp_status_sender).
    -- Future HPSDR producers (DDC IQ packetizers, mic/DUC consumers)
    -- extend the arbiter's port list further.
    --
    -- BUF_DEPTH defaults to 512 (= 2 KB) inside the framer entity --
    -- sized for HPSDR Protocol 2 DDC IQ frames (Eth+IP+UDP+1444 = 1486 B
    -- per packet = 372 words) with ~38% headroom.  Comfortably absorbs
    -- everything smaller (ARP 14 B, ICMP 98-298 B, DHCP 342 B, HPSDR
    -- control < 100 B).  MLAB cost ~32 blocks out of ~1000+ on Cyclone V GX.
    -- ========================================================================
    U_eem_tx_framer : entity work.eem_tx_framer
        port map (
            clock           => fx3_pclk_pll,
            reset           => sys_reset_pclk,

            frame_in_data   => mux_tx_data,
            frame_in_valid  => mux_tx_valid,
            frame_in_sop    => mux_tx_sop,
            frame_in_eop    => mux_tx_eop,
            frame_in_length => mux_tx_length,
            frame_in_ready  => mux_tx_ready,

            fifo_empty      => eem_rx_fifo_empty,
            fifo_rdata      => eem_rx_fifo_rdata,
            fifo_rreq       => eem_rx_fifo_rreq,

            pkt_done_pulse  => eem_tx_pkt_done,
            pkt_count       => eem_tx_pkt_count
        );

    -- Light led(3) for 250 ms on every reply emitted by ARP or ICMP
    -- responder, or whenever udp_rx_handler classifies an HPSDR-port
    -- (1024) or DHCP-port (68) UDP packet.  Generic "FPGA saw / replied
    -- to host traffic" beacon -- the exact protocol can be distinguished
    -- by watching the host with tcpdump.  Order-of-magnitude blink
    -- cadence during DHCP bring-up: 1 blip per OFFER + 1 per ACK = ~2
    -- blips per acquisition.
    reply_activity : process(sys_reset_pclk, fx3_pclk_pll)
        variable count : natural range 0 to 25_000_000 := 0;
    begin
        if (sys_reset_pclk = '1') then
            eem_dma_req_led <= '1';
            count := 0;
        elsif (rising_edge(fx3_pclk_pll)) then
            if (arp_reply_pulse        = '1' or
                icmp_reply_pulse       = '1' or
                hpsdr_pulse            = '1' or
                hpsdr_disc_reply_pulse = '1' or
                dhcp_rx_pulse          = '1') then
                eem_dma_req_led <= '0';
                count := 25_000_000;
            elsif (count > 0) then
                count := count - 1;
                if (count = 0) then
                    eem_dma_req_led <= '1';
                end if;
            end if;
        end if;
    end process reply_activity;

    -- Light led(2) for 250 ms on every fully-parsed EEM packet (header + payload
    -- drained from eem_tx_fifo by eem_rx_consumer).  One pulse per host
    -- packet received, so idle traffic blinks at ARP/RS cadence.
    eem_rx_activity : process(sys_reset_pclk, fx3_pclk_pll)
        variable count : natural range 0 to 25_000_000 := 0;
    begin
        if (sys_reset_pclk = '1') then
            eem_rx_led <= '1';
            count := 0;
        elsif (rising_edge(fx3_pclk_pll)) then
            if (eem_pkt_done_pulse = '1') then
                eem_rx_led <= '0';
                count := 25_000_000;
            elsif (count > 0) then
                count := count - 1;
                if (count = 0) then
                    eem_rx_led <= '1';
                end if;
            end if;
        end if;
    end process eem_rx_activity;

    -- Heartbeat on led(1): toggles every 10 M pclks (~ 100 ms at 100 MHz,
    -- giving a 5 Hz blink) so the user can see the FPGA is alive even
    -- when no EEM/ARP traffic is flowing.
    toggle_led1 : process(fx3_pclk_pll)
        variable count : natural range 0 to 10_000_000 := 10_000_000;
    begin
        if( rising_edge(fx3_pclk_pll) ) then
            count := count - 1;
            if( count = 0 ) then
                count := 10_000_000;
                led1_blink <= not led1_blink;
            end if;
        end if;
    end process;


    -- ========================================================================
    -- NIOS SYSTEM
    -- ========================================================================

    U_nios_system : component nios_system
        port map (
            clk_clk                         => sys_clock,
            reset_reset_n                   => '1',
            dac_MISO                        => nios_sdo,
            dac_MOSI                        => nios_sdio,
            dac_SCLK                        => nios_sclk,
            dac_SS_n                        => nios_ss_n,
            spi_MISO                        => adi_spi_sdo,
            spi_MOSI                        => adi_spi_sdi,
            spi_SCLK                        => adi_spi_sclk,
            spi_SS_n                        => adi_spi_csn,
            gpio_in_port                    => pack(nios_gpio.i, '0'),
            gpio_out_port                   => nios_gpo_slv,
            gpio_rffe_0_in_port             => pack(rffe_gpio),
            gpio_rffe_0_out_port            => rffe_gpio.o,
            ad9361_dac_sync_in_sync         => '0',
            ad9361_dac_sync_out_sync        => adi_sync_in,
            ad9361_data_clock_clk           => ad9361.clock, -- out std_logic;
            ad9361_data_reset_reset         => ad9361.reset, -- out std_logic;
            ad9361_device_if_rx_clk_in_p    => adi_rx_clock,
            ad9361_device_if_rx_clk_in_n    => '0',
            ad9361_device_if_rx_frame_in_p  => adi_rx_frame,
            ad9361_device_if_rx_frame_in_n  => '0',
            ad9361_device_if_rx_data_in_p   => adi_rx_data,
            ad9361_device_if_rx_data_in_n   => (others => '0'),
            ad9361_device_if_tx_clk_out_p   => adi_tx_clock,
            ad9361_device_if_tx_clk_out_n   => open,
            ad9361_device_if_tx_frame_out_p => adi_tx_frame,
            ad9361_device_if_tx_frame_out_n => open,
            ad9361_device_if_tx_data_out_p  => adi_tx_data,
            ad9361_device_if_tx_data_out_n  => open,
            ad9361_adc_i0_enable            => ad9361.ch(0).adc.i.enable, -- out sl
            ad9361_adc_i0_valid             => ad9361.ch(0).adc.i.valid,  -- out sl
            ad9361_adc_i0_data              => ad9361.ch(0).adc.i.data,   -- out slv(15:0)
            ad9361_adc_i1_enable            => ad9361.ch(1).adc.i.enable, -- out sl
            ad9361_adc_i1_valid             => ad9361.ch(1).adc.i.valid,  -- out sl
            ad9361_adc_i1_data              => ad9361.ch(1).adc.i.data,   -- out slv(15:0)
            ad9361_adc_overflow_ovf         => ad9361.adc_overflow,       -- in  sl
            ad9361_adc_q0_enable            => ad9361.ch(0).adc.q.enable, -- out sl
            ad9361_adc_q0_valid             => ad9361.ch(0).adc.q.valid,  -- out sl
            ad9361_adc_q0_data              => ad9361.ch(0).adc.q.data,   -- out slv(15:0)
            ad9361_adc_q1_enable            => ad9361.ch(1).adc.q.enable, -- out sl
            ad9361_adc_q1_valid             => ad9361.ch(1).adc.q.valid,  -- out sl
            ad9361_adc_q1_data              => ad9361.ch(1).adc.q.data,   -- out slv(15:0)
            ad9361_adc_underflow_unf        => ad9361.adc_underflow,      -- in  sl
            ad9361_dac_i0_enable            => ad9361.ch(0).dac.i.enable, -- out sl
            ad9361_dac_i0_valid             => ad9361.ch(0).dac.i.valid,  -- out sl
            ad9361_dac_i0_data              => ad9361.ch(0).dac.i.data,   -- in  slv(15:0)
            ad9361_dac_i1_enable            => ad9361.ch(1).dac.i.enable, -- out sl
            ad9361_dac_i1_valid             => ad9361.ch(1).dac.i.valid,  -- out sl
            ad9361_dac_i1_data              => ad9361.ch(1).dac.i.data,   -- in  slv(15:0)
            ad9361_dac_overflow_ovf         => ad9361.dac_overflow,       -- in  sl
            ad9361_dac_q0_enable            => ad9361.ch(0).dac.q.enable, -- out sl
            ad9361_dac_q0_valid             => ad9361.ch(0).dac.q.valid,  -- out sl
            ad9361_dac_q0_data              => ad9361.ch(0).dac.q.data,   -- in  slv(15:0)
            ad9361_dac_q1_enable            => ad9361.ch(1).dac.q.enable, -- out sl
            ad9361_dac_q1_valid             => ad9361.ch(1).dac.q.valid,  -- out sl
            ad9361_dac_q1_data              => ad9361.ch(1).dac.q.data,   -- in  slv(15:0)
            ad9361_dac_underflow_unf        => ad9361.dac_underflow,      -- in  sl
            xb_gpio_in_port                 => nios_xb_gpio_in,
            xb_gpio_out_port                => nios_xb_gpio_out,
            xb_gpio_dir_export              => nios_xb_gpio_oe,
            command_serial_in               => command_serial_in,
            command_serial_out              => command_serial_out,
            oc_i2c_arst_i                   => '0',
            oc_i2c_scl_pad_i                => i2c_scl_in,
            oc_i2c_scl_pad_o                => i2c_scl_out,
            oc_i2c_scl_padoen_o             => i2c_scl_oen,
            oc_i2c_sda_pad_i                => i2c_sda_in,
            oc_i2c_sda_pad_o                => i2c_sda_out,
            oc_i2c_sda_padoen_o             => i2c_sda_oen,
            rx_tamer_ts_sync_in             => '0',
            rx_tamer_ts_sync_out            => open,
            rx_tamer_ts_pps                 => '0',
            rx_tamer_ts_clock               => rx_clock,
            rx_tamer_ts_reset               => rx_ts_reset,
            unsigned(rx_tamer_ts_time)      => rx_timestamp,
            tx_tamer_ts_sync_in             => '0',
            tx_tamer_ts_sync_out            => open,
            tx_tamer_ts_pps                 => '0',
            tx_tamer_ts_clock               => tx_clock,
            tx_tamer_ts_reset               => tx_ts_reset,
            unsigned(tx_tamer_ts_time)      => tx_timestamp,
            rx_trigger_ctl_out_port         => rx_trigger_ctl_i,
            tx_trigger_ctl_out_port         => tx_trigger_ctl_i,
            rx_trigger_ctl_in_port          => pack(rx_trigger_ctl),
            tx_trigger_ctl_in_port          => pack(tx_trigger_ctl),
            wbm_wb_clk_i                    => wbm_wb_clk_i,
            wbm_wb_rst_i                    => wbm_wb_rst_i,
            wbm_wb_adr_o                    => wbm_wb_adr_o,
            wbm_wb_dat_o                    => wbm_wb_dat_o,
            wbm_wb_dat_i                    => wbm_wb_dat_i,
            wbm_wb_we_o                     => wbm_wb_we_o,
            wbm_wb_sel_o                    => wbm_wb_sel_o,
            wbm_wb_stb_o                    => wbm_wb_stb_o,
            wbm_wb_ack_i                    => wbm_wb_ack_i,
            wbm_wb_cyc_o                    => wbm_wb_cyc_o
        );

    -- FX3 UART
    command_serial_in <= fx3_uart_txd       when sys_reset = '0' else '1';
    fx3_uart_rxd      <= command_serial_out when sys_reset = '0' else 'Z';

    -- FX3 UART CTS and Flash SPI CSx are tied to the same signal.
    -- Allow SPI accesses when FPGA is in reset
    fx3_uart_cts      <= '1' when sys_reset_pclk = '0' else 'Z';

    -- Unpack the Nios general-purpose outputs into a record
    nios_gpio.o <= unpack(nios_gpo_slv);

    -- Readback of Nios general-purpose outputs
    nios_gpio.i.gpo_readback <= nios_gpio.o;

    -- RFFE GPIO outputs
    adi_ctrl_in    <= unpack(rffe_gpio.o).ctrl_in;
    adi_tx_spdt2_v <= unpack(rffe_gpio.o).tx_spdt2;
    adi_tx_spdt1_v <= unpack(rffe_gpio.o).tx_spdt1;
    tx_bias_en     <= unpack(rffe_gpio.o).tx_bias_en;
    adi_rx_spdt2_v <= unpack(rffe_gpio.o).rx_spdt2;
    adi_rx_spdt1_v <= unpack(rffe_gpio.o).rx_spdt1;
    rx_bias_en     <= unpack(rffe_gpio.o).rx_bias_en;
    --adi_sync_in    <= unpack(rffe_gpio.o).sync_in;
    adi_en_agc     <= unpack(rffe_gpio.o).en_agc;
    adi_txnrx      <= unpack(rffe_gpio.o).txnrx;
    adi_enable     <= unpack(rffe_gpio.o).enable;
    adi_reset_n    <= unpack(rffe_gpio.o).reset_n;

    -- Unpack trigger GPIO bits into records
    rx_trigger_ctl <= unpack(rx_trigger_ctl_i, rx_trigger_line);
    tx_trigger_ctl <= unpack(tx_trigger_ctl_i, tx_trigger_line);

    -- LEDs
    led(1) <= led1_blink        when nios_gpio.o.led_mode = '0' else not nios_gpio.o.leds(1);
    led(2) <= eem_rx_led        when nios_gpio.o.led_mode = '0' else not nios_gpio.o.leds(2);
    led(3) <= eem_dma_req_led   when nios_gpio.o.led_mode = '0' else not nios_gpio.o.leds(3);

    -- DAC SPI (data latched on falling edge)
    dac_sclk <= not nios_sclk when nios_gpio.o.adf_chip_enable = '0' else '0';
    dac_sdi  <= nios_sdio     when nios_gpio.o.adf_chip_enable = '0' else '0';
    dac_csn  <= nios_ss_n(0)  when nios_gpio.o.adf_chip_enable = '0' else '1';

    -- ADF SPI (data latched on rising edge)
    adf_sclk <= nios_sclk    when nios_gpio.o.adf_chip_enable = '1' else '0';
    adf_sdi  <= nios_sdio    when nios_gpio.o.adf_chip_enable = '1' else '0';
    adf_csn  <= nios_ss_n(1) when nios_gpio.o.adf_chip_enable = '1' else '1';
    adf_ce   <= nios_gpio.o.adf_chip_enable;

    nios_sdo <= adf_muxout when ((nios_ss_n(1) = '0') and (nios_gpio.o.adf_chip_enable = '1'))
                else '0';

    -- Power monitor I2C
    pwr_scl     <= i2c_scl_out when i2c_scl_oen = '0' else 'Z';
    pwr_sda     <= i2c_sda_out when i2c_sda_oen = '0' else 'Z';

    i2c_scl_in  <= pwr_scl;
    i2c_sda_in  <= pwr_sda;

    -- TPS2115A status
    nios_gpio.i.pwr_status <= pwr_status;

    -- SI53304 controls / clock output enables
    si_clock_sel <= nios_gpio.o.si_clock_sel;
    c5_clock2_oe <= '1';
    exp_clock_oe <= exp_present and exp_clock_req;
    ufl_clock_oe <= nios_gpio.o.ufl_clock_oe;

    -- Expansion I2C
    exp_i2c_scl <= 'Z';
    exp_i2c_sda <= 'Z';

    -- Expansion GPIO outputs
    generate_xb_gpio_out : for i in exp_gpio'range generate
        exp_gpio(i) <= nios_xb_gpio_out(i) when nios_xb_gpio_oe(i) = '1' else 'Z';
    end generate;

    tx_packet_ready <= '1';

    -- TX Submodule
    U_tx : entity work.tx
        generic map (
            NUM_STREAMS          => dac_controls'length
        )
        port map (
            tx_reset             => tx_reset,
            tx_clock             => tx_clock,
            tx_enable            => tx_enable,

            meta_en              => meta_en_tx,
            timestamp_reset      => tx_ts_reset,
            usb_speed            => usb_speed_tx,
            tx_underflow_led     => tx_underflow_led,
            tx_timestamp         => tx_timestamp,

            -- Triggering
            trigger_arm          => tx_trigger_ctl.arm,
            trigger_fire         => tx_trigger_ctl.fire,
            trigger_master       => tx_trigger_ctl.master,
            trigger_line         => tx_trigger_line,

            -- Eightbit mode
            eight_bit_mode_en    => eightbit_en_tx,
            highly_packed_mode_en => highly_packed_en_txrx,

            -- Packet FIFO
            packet_en            => packet_en_tx,
            packet_empty         => tx_packet_empty,
            packet_control       => tx_packet_control,
            packet_ready         => tx_packet_ready,

            -- Samples from host via FX3
            sample_fifo_wclock   => fx3_pclk_pll,
            sample_fifo_wreq     => tx_sample_fifo.wreq,
            sample_fifo_wdata    => tx_sample_fifo.wdata,
            sample_fifo_wempty   => tx_sample_fifo.wempty,
            sample_fifo_wfull    => tx_sample_fifo.wfull,
            sample_fifo_wused    => tx_sample_fifo.wused,

            -- Metadata from host via FX3
            meta_fifo_wclock     => fx3_pclk_pll,
            meta_fifo_wreq       => tx_meta_fifo.wreq,
            meta_fifo_wdata      => tx_meta_fifo.wdata,
            meta_fifo_wempty     => tx_meta_fifo.wempty,
            meta_fifo_wfull      => tx_meta_fifo.wfull,
            meta_fifo_wused      => tx_meta_fifo.wused,

            -- Digital Loopback Interface
            loopback_enabled     => tx_loopback_enabled,
            loopback_fifo_wclock => tx_loopback_fifo.wclock,
            loopback_fifo_wdata  => tx_loopback_fifo.wdata,
            loopback_fifo_wreq   => tx_loopback_fifo.wreq,
            loopback_fifo_wfull  => tx_loopback_fifo.wfull,
            loopback_fifo_wused  => tx_loopback_fifo.wused,

            -- RFFE Interface
            dac_controls         => dac_controls,
            dac_streams          => dac_streams
        );

    dac_assignment_proc : process( all )
    begin
        for i in dac_controls'range loop
            dac_controls(i).enable   <= (ad9361.ch(i).dac.i.enable or ad9361.ch(i).dac.q.enable or tx_loopback_enabled) and
                                        mimo_tx_enables(i);
            dac_controls(i).data_req <= (ad9361.ch(i).dac.i.valid  or ad9361.ch(i).dac.q.valid  or tx_loopback_enabled) and
                                        mimo_tx_enables(i);

            if (rising_edge(tx_clock) and dac_streams(i).data_v = '1') then
                ad9361.ch(i).dac.i.data  <= std_logic_vector(dac_streams(i).data_i(11 downto 0)) & "0000";
                ad9361.ch(i).dac.q.data  <= std_logic_vector(dac_streams(i).data_q(11 downto 0)) & "0000";
            end if;
        end loop;
    end process;

    -- RX Submodule
    U_rx : entity work.rx
        generic map (
            NUM_STREAMS            => adc_controls'length
        )
        port map (
            rx_reset               => rx_reset,
            rx_clock               => rx_clock,
            rx_enable              => rx_enable,

            meta_en                => meta_en_rx,
            timestamp_reset        => rx_ts_reset,
            usb_speed              => usb_speed_rx,
            rx_mux_sel             => rx_mux_sel,
            rx_overflow_led        => rx_overflow_led,
            rx_timestamp           => rx_timestamp,

            -- Triggering
            trigger_arm            => rx_trigger_ctl.arm,
            trigger_fire           => rx_trigger_ctl.fire,
            trigger_master         => rx_trigger_ctl.master,
            trigger_line           => rx_trigger_line,

            -- Packed modes
            eight_bit_mode_en      => eightbit_en_rx,
            highly_packed_mode_en  => highly_packed_en_txrx,

            -- Packet FIFO
            packet_en              => packet_en_rx,
            packet_control         => rx_packet_control,
            packet_ready           => rx_packet_ready,

            -- Samples to host via FX3
            sample_fifo_rclock     => fx3_pclk_pll,
            sample_fifo_raclr      => not rx_enable_pclk,
            sample_fifo_rreq       => rx_sample_fifo.rreq,
            sample_fifo_rdata      => rx_sample_fifo.rdata,
            sample_fifo_rempty     => rx_sample_fifo.rempty,
            sample_fifo_rfull      => rx_sample_fifo.rfull,
            sample_fifo_rused      => rx_sample_fifo.rused,

            -- Mini expansion signals
            mini_exp               => mini_exp2 & mini_exp1,

            -- Metadata to host via FX3
            meta_fifo_rclock       => fx3_pclk_pll,
            meta_fifo_raclr        => not rx_enable_pclk,
            meta_fifo_rreq         => rx_meta_fifo.rreq,
            meta_fifo_rdata        => rx_meta_fifo.rdata,
            meta_fifo_rempty       => rx_meta_fifo.rempty,
            meta_fifo_rfull        => rx_meta_fifo.rfull,
            meta_fifo_rused        => rx_meta_fifo.rused,

            -- Digital Loopback Interface
            loopback_fifo_wenabled => tx_loopback_enabled,
            loopback_fifo_wreset   => tx_reset,
            loopback_fifo_wclock   => tx_loopback_fifo.wclock,
            loopback_fifo_wdata    => tx_loopback_fifo.wdata,
            loopback_fifo_wreq     => tx_loopback_fifo.wreq,
            loopback_fifo_wfull    => tx_loopback_fifo.wfull,
            loopback_fifo_wused    => tx_loopback_fifo.wused,

            -- RFFE Interface
            adc_controls           => adc_controls,
            adc_streams            => adc_streams
        );

    adc_assignment_proc : process( all )
    begin
        for i in adc_controls'range loop
            adc_controls(i).enable   <= (ad9361.ch(i).adc.i.enable or ad9361.ch(i).adc.q.enable) and mimo_rx_enables(i);
            adc_controls(i).data_req <= '1';
            adc_streams(i).data_i    <= signed(ad9361.ch(i).adc.i.data);
            adc_streams(i).data_q    <= signed(ad9361.ch(i).adc.q.data);
            adc_streams(i).data_v    <= (ad9361.ch(i).adc.i.valid  or ad9361.ch(i).adc.q.valid) and not adc_streams_last_v(i);
        end loop;
    end process;

    process(rx_clock)
    begin
        if( rx_reset = '1' ) then
            adc_streams_last_v  <= ( others => '0' ) ;
        elsif( rising_edge( rx_clock ) ) then
            for i in adc_controls'range loop
                adc_streams_last_v(i)  <= ad9361.ch(i).adc.i.valid  or ad9361.ch(i).adc.q.valid;
            end loop;
        end if;
    end process;

    -- ========================================================================
    -- RESET SYNCHRONIZERS
    -- ========================================================================

    U_reset_sync_pclk : entity work.reset_synchronizer
        generic map (
            INPUT_LEVEL         =>  '1',
            OUTPUT_LEVEL        =>  '1'
        )
        port map (
            clock               =>  fx3_pclk_pll,
            async               =>  sys_reset_async,
            sync                =>  sys_reset_pclk
        );

    U_reset_sync_sys : entity work.reset_synchronizer
        generic map (
            INPUT_LEVEL         =>  '1',
            OUTPUT_LEVEL        =>  '1'
        )
        port map (
            clock               =>  sys_clock,
            async               =>  sys_reset_async,
            sync                =>  sys_reset
        );

    U_reset_sync_rx : entity work.reset_synchronizer
        generic map (
            INPUT_LEVEL         =>  '1',
            OUTPUT_LEVEL        =>  '1'
        )
        port map (
            clock               =>  rx_clock,
            async               =>  sys_reset_pclk,
            sync                =>  rx_reset
        );

    U_reset_sync_tx : entity work.reset_synchronizer
        generic map (
            INPUT_LEVEL         =>  '1',
            OUTPUT_LEVEL        =>  '1'
        )
        port map (
            clock               =>  tx_clock,
            async               =>  sys_reset_pclk,
            sync                =>  tx_reset
        );


    -- ========================================================================
    -- SYNCHRONIZERS
    -- ========================================================================

    U_sync_usb_speed_pclk : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  fx3_pclk_pll,
            async               =>  nios_gpio.o.usb_speed,
            sync                =>  usb_speed_pclk
        );

    U_sync_usb_speed_rx : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  rx_clock,
            async               =>  nios_gpio.o.usb_speed,
            sync                =>  usb_speed_rx
        );

    U_sync_usb_speed_tx : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  tx_clock,
            async               =>  nios_gpio.o.usb_speed,
            sync                =>  usb_speed_tx
        );


    U_sync_meta_en_pclk : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  fx3_pclk_pll,
            async               =>  nios_gpio.o.meta_sync,
            sync                =>  meta_en_pclk
        );

    U_sync_meta_en_rx : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  rx_clock,
            async               =>  nios_gpio.o.meta_sync,
            sync                =>  meta_en_rx
        );

    U_sync_meta_en_tx : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  tx_clock,
            async               =>  nios_gpio.o.meta_sync,
            sync                =>  meta_en_tx
        );

    U_sync_eightbit_en_pclk : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  fx3_pclk_pll,
            async               =>  nios_gpio.o.eightbit_en,
            sync                =>  eightbit_en_pclk
        );

    U_sync_eightbit_en_rx : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  rx_clock,
            async               =>  nios_gpio.o.eightbit_en,
            sync                =>  eightbit_en_rx
        );

    U_sync_eightbit_en_tx : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  tx_clock,
            async               =>  nios_gpio.o.eightbit_en,
            sync                =>  eightbit_en_tx
        );

    U_sync_highly_packed_en_txrx : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  ad9361.clock,
            async               =>  nios_gpio.o.highly_packed_en,
            sync                =>  highly_packed_en_txrx
        );

    U_sync_packet_en_pclk : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  fx3_pclk_pll,
            async               =>  nios_gpio.o.packet_en,
            sync                =>  packet_en_pclk
        );

    U_sync_packet_en_rx : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  rx_clock,
            async               =>  nios_gpio.o.packet_en,
            sync                =>  packet_en_rx
        );

    U_sync_packet_en_tx : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  tx_clock,
            async               =>  nios_gpio.o.packet_en,
            sync                =>  packet_en_tx
        );

    generate_sync_rx_mux_sel : for i in rx_mux_sel'range generate
        U_sync_rx_mux_sel : entity work.synchronizer
            generic map (
                RESET_LEVEL         =>  '0'
            )
            port map (
                reset               =>  '0',
                clock               =>  rx_clock,
                async               =>  nios_gpio.o.rx_mux_sel(i),
                sync                =>  rx_mux_sel(i)
            );
    end generate;

    generate_sync_mimo_rx_en : for i in mimo_rx_enables'range generate
        U_sync_mimo_rx_en : entity work.synchronizer
            generic map (
                RESET_LEVEL         =>  '0'
                )
            port map (
                reset               =>  '0',
                clock               =>  rx_clock,
                async               =>  unpack(rffe_gpio.o).mimo_rx_en(i),
                sync                =>  mimo_rx_enables(i)
            );
    end generate;

    generate_sync_mimo_tx_en : for i in mimo_tx_enables'range generate
        U_sync_mimo_tx_en : entity work.synchronizer
            generic map (
                RESET_LEVEL         =>  '0'
                )
            port map (
                reset               =>  '0',
                clock               =>  tx_clock,
                async               =>  unpack(rffe_gpio.o).mimo_tx_en(i),
                sync                =>  mimo_tx_enables(i)
            );
    end generate;

    generate_sync_adi_ctrl_out : for i in adi_ctrl_out'range generate
        U_sync_adi_ctrl_out : entity work.synchronizer
            generic map (
                RESET_LEVEL         =>  '0'
            )
            port map (
                reset               =>  '0',
                clock               =>  sys_clock,
                async               =>  adi_ctrl_out(i),
                sync                =>  rffe_gpio.i.ctrl_out(i)
            );
    end generate;

    U_sync_adf_muxout : entity work.synchronizer
        generic map (
            RESET_LEVEL         =>  '0'
        )
        port map (
            reset               =>  '0',
            clock               =>  sys_clock,
            async               =>  adf_muxout,
            sync                =>  rffe_gpio.i.adf_muxout
        );

    generate_sync_xb_gpio_in : for i in exp_gpio'range generate
        U_sync_xb_gpio_in : entity work.synchronizer
          generic map (
            RESET_LEVEL         =>  '0'
          ) port map (
            reset               =>  '0',
            clock               =>  sys_clock,
            async               =>  exp_gpio(i),
            sync                =>  nios_xb_gpio_in(i)
          );
    end generate;

    U_sync_rx_enable : entity work.synchronizer
        generic map (
            RESET_LEVEL =>  '0'
        )
        port map (
            reset       =>  rx_reset,
            clock       =>  rx_clock,
            async       =>  rx_enable_pclk,
            sync        =>  rx_enable
        );

    U_sync_tx_enable : entity work.synchronizer
        generic map (
            RESET_LEVEL =>  '0'
        )
        port map (
            reset       =>  tx_reset,
            clock       =>  tx_clock,
            async       =>  tx_enable_pclk,
            sync        =>  tx_enable
        );


    -- ========================================================================
    -- HANDSHAKES
    -- ========================================================================

    drive_handshake_timestamp : process( fx3_pclk_pll, sys_reset_pclk )
    begin
        if( sys_reset_pclk = '1' ) then
            timestamp_req <= '0';
        elsif( rising_edge(fx3_pclk_pll) ) then
            if( meta_en_pclk = '0' ) then
                timestamp_req <= '0';
            else
                if( timestamp_ack = '0' ) then
                    timestamp_req <= '1';
                elsif( timestamp_ack = '1' ) then
                    timestamp_req <= '0';
                end if;
            end if;
        end if;
    end process;

    U_handshake_timestamp : entity work.handshake
        generic map (
            DATA_WIDTH          =>  tx_timestamp'length
        )
        port map (
            source_clock        =>  tx_clock,
            source_reset        =>  tx_reset,
            source_data         =>  std_logic_vector(tx_timestamp),

            dest_clock          =>  fx3_pclk_pll,
            dest_reset          =>  sys_reset_pclk,
            unsigned(dest_data) =>  fx3_timestamp,
            dest_req            =>  timestamp_req,
            dest_ack            =>  timestamp_ack
        );

end architecture;
