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

library nuand;
    use nuand.fx3_net_gpif_p.all;

entity fx3_net_gpif is
  port (
    -- FX3 control signals
    pclk                :   in  std_logic;
    reset               :   in  std_logic;

    -- USB Speed 0 = SS, 1 = HS
    usb_speed           :   in  std_logic;

    -- FX3 GPIF interface
    gpif_in             :   in  std_logic_vector(31 downto 0);
    gpif_out            :   out std_logic_vector(31 downto 0);
    gpif_oe             :   out std_logic;
    ctl_in              :   in  std_logic_vector(12 downto 0);
    ctl_out             :   out std_logic_vector(12 downto 0);
    ctl_oe              :   out std_logic_vector(12 downto 0);

    -- Packet interface (for UDP/EEM protocol)
    tx_packet_data_in   :   in  std_logic_vector(31 downto 0);
    rx_packet_data_in   :   in  std_logic_vector(31 downto 0);
    packet_tx_ready     :   out std_logic;
    packet_rx_ready     :   out std_logic
  );
end entity;

architecture sample_shuffler of fx3_net_gpif is

    -- max(a, b) returns the greater of a or b
    function max( a : integer ; b : integer ) return integer is
    begin
        if (a > b) then
            return a;
        else
            return b;
        end if;
    end function;

    type state_t        is (IDLE, SETUP_RD, SAMPLE_READ, SETUP_WR,
                            SAMPLE_WRITE, FINISHED);
    type gpif_mode_t    is (IDLE, RX, TX);
    type dma_channel_t  is (RX0, RX1, TX2, TX3);

    -- Packet interface signals (for UDP/EEM protocol)
    signal tx_packet_data        : std_logic_vector(31 downto 0);
    signal rx_packet_data        : std_logic_vector(31 downto 0);
    signal packet_tx_valid       : std_logic;
    signal packet_rx_valid       : std_logic;

    -- Control mapping
    alias dma0_rx_ack   is ctl_out(0);
    alias dma1_rx_ack   is ctl_out(1);
    alias dma2_tx_ack   is ctl_out(2);
    alias dma3_tx_ack   is ctl_out(3);
    alias dma_rx_enable is ctl_in(4);
    alias dma_tx_enable is ctl_in(5);
    alias dma_idle      is ctl_in(6);
    alias dma0_rx_reqx  is ctl_in(8);
    alias dma1_rx_reqx  is ctl_in(12); -- due to 9 being connected to dclk
    alias dma2_tx_reqx  is ctl_in(10);
    alias dma3_tx_reqx  is ctl_in(11);

    -- Control OE (12 downto 0)
    constant CONTROL_OE : std_logic_vector := "0000000001111";

    -- State machine downcounter values
    constant ACK_DOWNCOUNT_READ     :   natural := 2;
    constant ACK_DOWNCOUNT_WRITE    :   natural := 4;
    constant META_DOWNCOUNT_RESET   :   natural := 4;
    constant FINI_DOWNCOUNT_RESET   :   natural := 10;

    type dma_handshake_t is record
        rx0             :   std_logic;
        rx1             :   std_logic;
        tx2             :   std_logic;
        tx3             :   std_logic;
    end record;

    type fsm_t is record
        state           :   state_t;
        gpif_mode       :   gpif_mode_t;
        dma_idle        :   std_logic;
        ack_downcount   :   integer range  0 to max(ACK_DOWNCOUNT_READ, ACK_DOWNCOUNT_WRITE);
        dma_downcount   :   integer range -1 to 65536;
        fini_downcount  :   integer range  0 to FINI_DOWNCOUNT_RESET;
        dma_acks        :   dma_handshake_t;
        rx_current_dma  :   dma_channel_t;
        tx_current_dma  :   dma_channel_t;
    end record;

    constant FSM_RESET_VALUE : fsm_t := (
        state           =>  IDLE,
        gpif_mode       =>  IDLE,
        dma_idle        =>  '0',
        ack_downcount   =>  0,
        dma_downcount   =>  0,
        fini_downcount  =>  0,
        dma_acks        =>  (others => '0'),
        rx_current_dma  =>  RX0,
        tx_current_dma  =>  TX3
    );

    signal current, future      :   fsm_t := FSM_RESET_VALUE;

    signal can_rx               :   boolean;
    signal can_tx               :   boolean;

    signal dma_req              :   dma_handshake_t;

    -- acknowledge(dma_channel) returns a dma_handshake_t with all bits 0
    -- except the bit corresponding to dma_channel
    function acknowledge( chan : dma_channel_t ) return dma_handshake_t is
        variable retval : dma_handshake_t := (others => '0');
    begin
        case (chan) is
            when RX0 => retval.rx0 := '1';
            when RX1 => retval.rx1 := '1';
            when TX2 => retval.tx2 := '1';
            when TX3 => retval.tx3 := '1';
        end case;

        return retval;
    end function;

begin
    -- DMA handshake signals (active-low, so inverting for clarity)
    dma_req.rx0 <= not dma0_rx_reqx;
    dma_req.rx1 <= not dma1_rx_reqx;
    dma_req.tx2 <= not dma2_tx_reqx;
    dma_req.tx3 <= not dma3_tx_reqx;

    -- Set FX3 control OEs and default the unused outputs
    ctl_oe                  <= CONTROL_OE;
    ctl_out(12 downto 4)    <= (others => '0');

    -- Handle gpif_oe and moving data to/from the gpif bus
    gpif_mux : process(reset, pclk)
    begin
        if (reset = '1') then
            gpif_oe             <= '1';
            gpif_out            <= (others => '0');
            tx_packet_data      <= (others => '0');
            packet_tx_valid     <= '0';
            packet_rx_valid     <= '0';
        elsif (rising_edge(pclk)) then
            gpif_oe             <= '0';
            gpif_out            <= (others => '1');
            tx_packet_data      <= gpif_in;
            packet_tx_valid     <= '1' when current.gpif_mode = TX else '0';

            case (current.gpif_mode) is
                when IDLE =>
                    gpif_oe         <= '0';

                when RX =>
                    gpif_oe         <= '1';
                    gpif_out        <= rx_packet_data;
                    packet_rx_valid <= '1';

                when TX =>
                    gpif_oe         <= '0';

            end case;
        end if;
    end process gpif_mux;

    can_and_should : process(pclk, reset)
        variable rx_dma_req : boolean;
        variable tx_dma_req : boolean;
    begin
        if (reset = '1') then
            can_rx      <= false;
            can_tx      <= false;
        elsif (rising_edge(pclk)) then
            rx_dma_req := dma_req.rx0 = '1' or dma_req.rx1 = '1';
            can_rx <= dma_rx_enable = '1' and rx_dma_req;

            tx_dma_req := dma_req.tx2 = '1' or dma_req.tx3 = '1';
            can_tx <= dma_tx_enable = '1' and tx_dma_req;
        end if;
    end process;

    -- Synchronous process for FSM
    fsm_sync_proc : process(reset, pclk) is
    begin
        if (reset = '1') then
            current <= FSM_RESET_VALUE;
        elsif (rising_edge(pclk)) then
            current <= future;
        end if;
    end process fsm_sync_proc;

    -- Combinatorial process for FSM
    fsm_comb_proc : process(all) is
        variable next_state : state_t := IDLE;
    begin
        -- Maintain current state unless otherwise specified
        future              <= current;

        -- Prevents inferred latch by making sure this variable always gets an
        -- assignment. (It is used as a temporary variable during SETUP_RD.)
        next_state          := IDLE;

        -- Deassert DMA acknowledgements
        future.dma_acks     <= (others => '0');

        -- Register incoming signals
        future.dma_idle     <= dma_idle;

        case (current.state) is
            -- =================================================================
            -- Idle State
            -- =================================================================
            when IDLE =>
                -- Set up initial conditions
                future.gpif_mode        <= IDLE;
                future.fini_downcount   <= FINI_DOWNCOUNT_RESET;

                -- Arbitrate RX and TX requests
                if (current.dma_idle = '1') then
                    future.dma_downcount <= 1024;  -- Fixed packet size, adjust as needed
                    if (can_tx) then
                        future.ack_downcount    <= ACK_DOWNCOUNT_WRITE;
                        future.tx_current_dma   <= TX3;
                        future.state            <= SETUP_WR;
                    elsif (can_rx) then
                        future.ack_downcount    <= ACK_DOWNCOUNT_READ;
                        future.rx_current_dma   <= RX0;
                        future.state            <= SETUP_RD;
                    end if;
                end if;

            -- =================================================================
            -- Read Handling (moving data to FX3)
            -- =================================================================
            when SETUP_RD =>
                -- SETUP_RD starts moving data from packet to GPIF
                future.gpif_mode    <= RX;
                next_state          := SAMPLE_READ;

                -- After the first iteration, assert the proper ack.
                if (current.ack_downcount /= ACK_DOWNCOUNT_READ) then
                    future.dma_acks         <= acknowledge(current.rx_current_dma);
                end if;

                -- Remain in this state until we're done acking
                if (current.ack_downcount = 0) then
                    future.state        <= next_state;
                else
                    future.state        <= current.state;
                end if;

                future.ack_downcount    <= max(current.ack_downcount-1, 0);
                future.dma_downcount    <= max(current.dma_downcount-1, -1);

            when SAMPLE_READ =>
                -- Service the packet data.
                future.gpif_mode        <= RX;

                -- Once the DMA countdown is done, conclude this transaction
                if (current.dma_downcount = 0) then
                    future.state        <= FINISHED;
                end if;

                future.dma_downcount    <= max(current.dma_downcount-1, -1);

            -- =================================================================
            -- Write Handling (moving data from FX3)
            -- =================================================================
            when SETUP_WR =>
                -- SETUP_WR acknowledges the DMA request
                future.dma_acks         <= acknowledge(current.tx_current_dma);

                -- Set GPIF mode
                future.gpif_mode    <= TX;

                -- When we're done acking, move to writing
                if (current.ack_downcount = 0) then
                    future.state        <= SAMPLE_WRITE;
                end if;

                future.ack_downcount    <= max(current.ack_downcount-1, 0);

            when SAMPLE_WRITE =>
                -- Move data from GPIF to packet
                future.gpif_mode    <= TX;

                -- Determine when we are finished
                if (current.dma_downcount = 0) then
                    future.state        <= FINISHED;
                end if;

                future.dma_downcount    <= max(current.dma_downcount-1, -1);

            -- =================================================================
            -- End state
            -- =================================================================
            when FINISHED =>
                -- This provides a pause between adjacent transactions.
                future.gpif_mode        <= IDLE;
                future.fini_downcount   <= max(current.fini_downcount-1, 0);

                if (current.fini_downcount = 0) then
                    future.state        <= IDLE;
                end if;

        end case;
    end process fsm_comb_proc;

    -- Output process for FSM
    fsm_output_proc : process(current) is
    begin
        -- DMA acknowledgements
        dma0_rx_ack             <= current.dma_acks.rx0;
        dma1_rx_ack             <= current.dma_acks.rx1;
        dma2_tx_ack             <= current.dma_acks.tx2;
        dma3_tx_ack             <= current.dma_acks.tx3;

        -- Packet interface
        tx_packet_data          <= tx_packet_data_in;
        rx_packet_data          <= rx_packet_data_in;
        packet_tx_ready         <= packet_tx_valid;
        packet_rx_ready         <= packet_rx_valid;
    end process fsm_output_proc;

end architecture sample_shuffler;
