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

    -- EEM RX path (FPGA -> host, GPIF RX1): currently a ROM-style test injector
    -- (see eem_rx_inject block) drives these signals directly. The injector has
    -- true show-ahead semantics: eem_rx_fifo_rdata is combinatorial from a
    -- constant frame array, and eem_rx_fifo_rreq advances the read address.
    --
    -- When a real producer arrives, it will need a real FIFO -- and two traps
    -- in the current nuand sync_fifo to be aware of:
    --
    --   1. The READ_AHEAD generic is declared but unused in the entity body;
    --      sync_fifo always has a registered data_out (1-cycle latency from
    --      read pulse). Combined with the gpif_mux register on gpif_out, that
    --      gives a 2-cycle pipeline that duplicates the first word of every
    --      burst on the GPIF bus.
    --   2. Simultaneous read+write silently corrupts state: both addresses
    --      stall, used count freezes, data_in clobbers ram[old write_addr]
    --      while data_out returns ram[old read_addr] indefinitely.
    --
    -- A real producer therefore needs either a fixed sync_fifo (proper FWFT
    -- when READ_AHEAD=true, correct simultaneous read+write) or a different
    -- FIFO altogether.
    signal eem_rx_fifo_rreq  : std_logic;
    signal eem_rx_fifo_rdata : std_logic_vector(31 downto 0);
    signal eem_rx_fifo_empty : std_logic;

    -- EEM TX FIFO: fx3_gpif -> EEM RX logic (host->FPGA, TX2, pclk domain)
    signal eem_tx_fifo_wreq       : std_logic := '0';
    signal eem_tx_fifo_wdata      : std_logic_vector(31 downto 0);
    signal eem_tx_fifo_full       : std_logic;
    signal eem_tx_fifo_rreq       : std_logic := '0';
    signal eem_tx_fifo_rdata      : std_logic_vector(31 downto 0);
    signal eem_tx_fifo_empty      : std_logic;

    -- eem_rx_consumer observability (host->FPGA EEM packet reception)
    signal eem_pkt_done_pulse     : std_logic := '0';
    signal eem_pkt_count          : std_logic_vector(15 downto 0) := (others => '0');
    signal eem_last_eth_length    : std_logic_vector(13 downto 0) := (others => '0');
    signal eem_last_bmtype        : std_logic := '0';

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

    -- Note: the EEM RX (FPGA->host) FIFO is intentionally absent. The current
    -- bring-up uses a ROM-style test injector (see eem_rx_inject below) that
    -- drives eem_rx_fifo_rdata / eem_rx_fifo_empty directly. Adding a real
    -- producer means re-introducing a FIFO here -- with the caveats called
    -- out on the eem_rx_fifo_* signal declarations above.

    -- EEM TX: fx3_gpif writes here via TX2 DMA channel; EEM RX logic reads.
    -- Read side (eem_tx_fifo_rreq / eem_tx_fifo_rdata) connects to EEM RX logic.
    U_eem_tx_fifo : entity work.sync_fifo
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

            pkt_done_pulse  => eem_pkt_done_pulse,
            pkt_count       => eem_pkt_count,
            last_eth_length => eem_last_eth_length,
            last_bmtype     => eem_last_bmtype
        );

    -- ========================================================================
    -- FPGA->host EEM RX test injector (ROM-style, bypasses sync_fifo)
    -- ------------------------------------------------------------------------
    -- Once per ~1 s, drives a fixed 16-word / 64-byte CDC EEM frame onto the
    -- eem_rx_fifo_* signals that fx3_gpif consumes via the RX1 DMA path.
    -- Exercises the WRAPUP_RX1 / COMMIT(Thread1) plumbing in fx3_gpif.vhd's
    -- SAMPLE_READ. Combined with the matching GPIF II Designer FSM change in
    -- cyfxgpif_RFlink.h, this is the smallest end-to-end test that lands a
    -- valid Ethernet frame on usb0.
    --
    -- Why ROM-style and not a real sync_fifo: two traps in the current
    -- nuand sync_fifo (see comment on eem_rx_fifo_* signal declarations)
    -- duplicate the first word of every burst and limit URB content. The
    -- ROM here is intrinsically show-ahead: eem_rx_fifo_rdata is combinatorial
    -- from FRAME(addr), and eem_rx_fifo_rreq advances addr on the next edge,
    -- so each pclk cycle that fx3_gpif reads, the bus already carries the
    -- next FRAME word. Combined with gpif_mux's one register stage, FX3
    -- sees a clean FRAME(0), FRAME(1), ..., FRAME(15) sequence.
    --
    -- URB sizing: empirically the FX3 GPIF II FSM drops the LAST word of every
    -- burst -- with FRAME = N words the host receives an URB of exactly N-1
    -- words. Observed via tcpdump -XX -i usbmon0:
    --   FRAME = 16 words -> URB = 15 words (FRAME(15) lost)
    --   FRAME = 15 words -> URB = 14 words (FRAME(14) lost)
    -- Without GPIF II Designer to inspect the FSM, the exact mechanism doesn't
    -- matter; the pattern is reliable. We work around it by placing the FCS
    -- sentinel one position from the end and a dummy filler at the very end.
    -- The dummy gets dropped, the FCS makes it through.
    --
    -- Frame contents (56 URB bytes):
    --   dst MAC   02:00:00:0B:1A:DE  (locally-administered unicast)
    --   src MAC   02:00:00:0B:1A:DF
    --   ethertype 0x88B5             (IEEE 802 Local Experimental EtherType 1)
    --   payload   36 bytes of 0x55   (alternating bits, easy to spot)
    --   FCS       0xDEADBEEF         (sentinel; bmCRC=0 in EEM header)
    --
    -- URB layout on the wire (56 bytes = 14 captured words):
    --   off  0..1 : EEM hdr   36 00              (bmType=0 bmCRC=0 len=54)
    --   off  2..7 : dst MAC   02 00 00 0B 1A DE
    --   off  8..13: src MAC   02 00 00 0B 1A DF
    --   off 14..15: ethertype 88 B5
    --   off 16..51: payload   55 ... (36 bytes)
    --   off 52..55: sentinel  DE AD BE EF
    --
    -- 32-bit FIFO words pack URB bytes [b0,b1,b2,b3] as rdata(7..0)=b0 ...
    -- rdata(31..24)=b3, i.e. each table entry below reads x"B3B2B1B0".
    --
    -- INJECT_PERIOD = 100_000_000 -> ~1 s @ 100 MHz pclk. Crank up to silence,
    -- or remove this whole block once a real FPGA-side EEM producer exists.
    -- ========================================================================
    eem_rx_inject : block
        constant INJECT_PERIOD : natural := 100_000_000;

        type word_arr_t is array(natural range <>) of std_logic_vector(31 downto 0);
        constant FRAME : word_arr_t := (
             0 => x"00020036",  -- EEM hdr (36 00, len=54), dst MAC[0..1] (02 00)
             1 => x"DE1A0B00",  -- dst MAC[2..5] (00 0B 1A DE)
             2 => x"0B000002",  -- src MAC[0..3] (02 00 00 0B)
             3 => x"B588DF1A",  -- src MAC[4..5] (1A DF), ethertype (88 B5)
             4 => x"55555555",
             5 => x"55555555",
             6 => x"55555555",
             7 => x"55555555",
             8 => x"55555555",
             9 => x"55555555",
            10 => x"55555555",
            11 => x"55555555",
            12 => x"55555555",
            13 => x"EFBEADDE",  -- FCS sentinel (DE AD BE EF, big-endian per cdc_eem)
            14 => x"DEADCAFE"   -- dummy: FX3 drops the last word, anything goes here
        );

        signal addr   : natural range 0 to FRAME'length-1 := 0;
        signal active : std_logic                          := '0';
        signal count  : natural range 0 to INJECT_PERIOD   := INJECT_PERIOD;
    begin
        -- Combinatorial show-ahead: when active, the current FRAME entry is
        -- already on the bus; when inactive, the FIFO looks empty so the
        -- fx3_gpif FSM never enters IF_RX_1 between bursts.
        eem_rx_fifo_rdata <= FRAME(addr);
        eem_rx_fifo_empty <= not active;

        inject_proc : process(sys_reset_pclk, fx3_pclk_pll)
        begin
            if (sys_reset_pclk = '1') then
                addr   <= 0;
                active <= '0';
                count  <= INJECT_PERIOD;
            elsif (rising_edge(fx3_pclk_pll)) then
                if (active = '0') then
                    -- Idle gap between bursts. Countdown to next injection.
                    if (count = 0) then
                        active <= '1';
                        addr   <= 0;
                        count  <= INJECT_PERIOD;
                    else
                        count <= count - 1;
                    end if;
                else
                    -- Burst in progress. Each fx3_gpif read pulse consumes one
                    -- word: advance addr to the next, or, on the last word,
                    -- drop active so fx3_gpif sees empty and finishes the
                    -- burst (RX_1 -> WRAPUP_RX1 -> COMMIT(Thread1)).
                    if (eem_rx_fifo_rreq = '1') then
                        if (addr = FRAME'length - 1) then
                            active <= '0';
                            addr   <= 0;
                        else
                            addr <= addr + 1;
                        end if;
                    end if;
                end if;
            end if;
        end process inject_proc;
    end block eem_rx_inject;

    -- Light led(3) for 250 ms whenever the FX3 asserts dma2_tx_reqx (ctl_in(10) low):
    -- data is ready on PIB socket 2, i.e. EEM data has arrived from the USB host.
    eem_dma_req_activity : process(sys_reset_pclk, fx3_pclk_pll)
        variable count : natural range 0 to 25_000_000 := 0;
    begin
        if (sys_reset_pclk = '1') then
            eem_dma_req_led <= '1';
            count := 0;
        elsif (rising_edge(fx3_pclk_pll)) then
            if (fx3_ctl_in(6) = '0') then   -- dma2_tx_reqx active low
                eem_dma_req_led <= '0';
                count := 25_000_000;
            elsif (count > 0) then
                count := count - 1;
                if (count = 0) then
                    eem_dma_req_led <= '1';
                end if;
            end if;
        end if;
    end process eem_dma_req_activity;

    -- Light led(2) for 250 ms on every fully-parsed EEM packet (header + payload
    -- drained from eem_tx_fifo by eem_rx_consumer). One pulse per real host
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
