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
-- eem_rx_consumer
--
-- Drains the host->FPGA EEM TX FIFO (data delivered by fx3_gpif's TX2 path,
-- originally written by the host via cdc_eem's bulk OUT endpoint), parses the
-- 2-byte EEM header, and exposes the embedded Ethernet frame to downstream
-- logic as a byte stream.  Per EEM packet:
--
--   * read the header word, decode bmType/bmCRC and EthernetLength;
--   * for data packets (bmType=0): emit EthernetLength - 4 Ethernet payload
--     bytes via (eth_data, eth_valid, eth_sop, eth_eop) -- the trailing
--     4-byte FCS (or 0xDEADBEEF sentinel when bmCRC=0) is dropped before
--     it ever leaves this entity;
--   * for command packets (bmType=1): no emission, just drain the FIFO so
--     subsequent packets see a clean head;
--   * pulse pkt_done_pulse and bump pkt_count.
--
-- URB byte / word packing convention (matches eem_tx_framer's inverse and
-- the proven bringup test injector):
--   URB byte k  ->  fifo_rdata((k mod 4)*8 + 7 downto (k mod 4)*8)
-- So the first word's low 16 bits are the EEM header and the high 16 bits
-- are the first two Ethernet bytes; subsequent words pack four Ethernet
-- bytes each, with byte 0 of every word in bits 7..0.
--
-- Pacing
-- ------
-- One Ethernet byte emitted per clock during an active S_EMIT; one extra
-- clock between successive FIFO words (the S_FETCH gap absorbing the
-- sync_fifo's 1-cycle read latency).  So ~5 clocks per 4 emitted bytes
-- = ~80 MB/s at 100 MHz pclk.  EEM line rate < 10 MB/s, so we never
-- backpressure the GPIF TX2 path.  Downstream (eth_rx_demux) must
-- tolerate idle cycles with eth_valid='0' both inside frames (between
-- words) and between frames.
--
-- Length sideband
-- ---------------
-- eth_length is sampled at sop and is the Ethernet *payload* length
-- (EthernetLength - 4), i.e. what eth_rx_demux and downstream see.  Higher
-- layers needing the original on-wire length can derive it as
-- eth_length + 4.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity eem_rx_consumer is
    port (
        clock           : in  std_logic;
        reset           : in  std_logic;

        -- sync_fifo show-ahead read interface
        fifo_empty      : in  std_logic;
        fifo_rdata      : in  std_logic_vector(31 downto 0);
        fifo_rreq       : out std_logic;

        -- Ethernet payload byte stream (FCS stripped)
        eth_data        : out std_logic_vector(7 downto 0);
        eth_valid       : out std_logic;
        eth_sop         : out std_logic;
        eth_eop         : out std_logic;
        eth_length      : out std_logic_vector(13 downto 0);   -- valid with sop

        -- Observability
        pkt_done_pulse  : out std_logic;                       -- 1-cycle pulse per completed packet
        pkt_count       : out std_logic_vector(15 downto 0);   -- wrap-around packet counter
        last_eth_length : out std_logic_vector(13 downto 0);   -- header bits 13:0 of last packet
        last_bmtype     : out std_logic                        -- header bit 15 of last packet
    );
end entity;

architecture arch of eem_rx_consumer is

    -- words_left counts 32-bit FIFO reads remaining after the header word.
    -- 11 bits cover the worst case (Ethernet max 1518 -> ~380 words).
    constant WORDS_LEFT_W : natural := 11;

    -- FSM stages:
    --   S_IDLE     : wait for non-empty FIFO; issue read of header word
    --   S_HDR      : header word on fifo_rdata; decode and set up emit state
    --   S_EMIT     : emit one Ethernet byte per cycle from word_buf, stepping
    --                byte_lane 0..3 (or 2..3 immediately after S_HDR, since
    --                lanes 0..1 of the header word are the EEM header).
    --                When lane wraps 3->0 either fetch the next word (S_FETCH)
    --                or finish the packet.
    --   S_FETCH    : 1-cycle wait for the next word's data to appear on
    --                fifo_rdata; capture into word_buf and return to S_EMIT.
    --   S_DRAIN    : pop remaining FIFO words without emitting (command
    --                packets, or any tail that doesn't apply).
    --   S_DRAIN_W  : 1-cycle wait for the drained word's data (companion
    --                of S_DRAIN, identical role to S_FETCH).
    type state_t is (S_IDLE, S_HDR, S_EMIT, S_FETCH, S_DRAIN, S_DRAIN_W);

    signal state              : state_t                            := S_IDLE;
    signal words_left         : unsigned(WORDS_LEFT_W-1 downto 0)  := (others => '0');
    signal bytes_remaining    : unsigned(13 downto 0)              := (others => '0');
    signal word_buf           : std_logic_vector(31 downto 0)      := (others => '0');
    signal byte_lane          : unsigned(1 downto 0)               := (others => '0');
    signal sop_pending        : std_logic                          := '0';

    signal pkt_count_reg      : unsigned(15 downto 0)              := (others => '0');
    signal last_eth_length_r  : std_logic_vector(13 downto 0)      := (others => '0');
    signal last_bmtype_r      : std_logic                          := '0';
    signal pkt_done_pulse_r   : std_logic                          := '0';
    signal fifo_rreq_r        : std_logic                          := '0';

    signal eth_data_r         : std_logic_vector(7 downto 0)       := (others => '0');
    signal eth_valid_r        : std_logic                          := '0';
    signal eth_sop_r          : std_logic                          := '0';
    signal eth_eop_r          : std_logic                          := '0';
    signal eth_length_r       : unsigned(13 downto 0)              := (others => '0');

    -- Pick a byte lane from a 32-bit word using the URB-byte ordering
    -- (byte 0 = bits 7..0, byte 3 = bits 31..24).
    function byte_at (word : std_logic_vector(31 downto 0);
                      lane : unsigned(1 downto 0))
        return std_logic_vector is
    begin
        case lane is
            when "00"   => return word(7 downto 0);
            when "01"   => return word(15 downto 8);
            when "10"   => return word(23 downto 16);
            when others => return word(31 downto 24);
        end case;
    end function;

    -- ceil((payload_bytes - 2) / 4), with payload_bytes <= 2 collapsing to 0.
    -- payload_bytes here = EEM EthernetLength field for data packets, or the
    -- derived payload length for command packets.
    function payload_words_after_hdr (payload_bytes : unsigned)
        return unsigned is
        variable rem_bytes : unsigned(payload_bytes'range);
    begin
        if payload_bytes <= to_unsigned(2, payload_bytes'length) then
            return to_unsigned(0, WORDS_LEFT_W);
        else
            rem_bytes := payload_bytes - 2;
            return resize(shift_right(rem_bytes + 3, 2), WORDS_LEFT_W);
        end if;
    end function;

begin

    fifo_rreq       <= fifo_rreq_r;
    pkt_done_pulse  <= pkt_done_pulse_r;
    pkt_count       <= std_logic_vector(pkt_count_reg);
    last_eth_length <= last_eth_length_r;
    last_bmtype     <= last_bmtype_r;

    eth_data        <= eth_data_r;
    eth_valid       <= eth_valid_r;
    eth_sop         <= eth_sop_r;
    eth_eop         <= eth_eop_r;
    eth_length      <= std_logic_vector(eth_length_r);

    fsm : process(clock, reset)
        variable bmtype    : std_logic;
        variable bmcrc     : std_logic;
        variable eth_len   : unsigned(13 downto 0);
        variable cmd_code  : std_logic_vector(2 downto 0);
        variable cmd_param : unsigned(10 downto 0);
        variable payload_b : unsigned(13 downto 0);
        variable wl_init   : unsigned(WORDS_LEFT_W-1 downto 0);
        variable br_init   : unsigned(13 downto 0);
    begin
        if reset = '1' then
            state             <= S_IDLE;
            words_left        <= (others => '0');
            bytes_remaining   <= (others => '0');
            word_buf          <= (others => '0');
            byte_lane         <= (others => '0');
            sop_pending       <= '0';
            pkt_count_reg     <= (others => '0');
            last_eth_length_r <= (others => '0');
            last_bmtype_r     <= '0';
            pkt_done_pulse_r  <= '0';
            fifo_rreq_r       <= '0';
            eth_data_r        <= (others => '0');
            eth_valid_r       <= '0';
            eth_sop_r         <= '0';
            eth_eop_r         <= '0';
            eth_length_r      <= (others => '0');
        elsif rising_edge(clock) then
            -- One-cycle-pulse defaults
            pkt_done_pulse_r <= '0';
            fifo_rreq_r      <= '0';
            eth_valid_r      <= '0';
            eth_sop_r        <= '0';
            eth_eop_r        <= '0';

            case state is

            -- --------------------------------------------------------------
            -- Wait for the first word of a new EEM packet to be at the head
            -- of the FIFO, then issue a read for it (data appears next cycle).
            -- --------------------------------------------------------------
            when S_IDLE =>
                if fifo_empty = '0' then
                    fifo_rreq_r <= '1';
                    state       <= S_HDR;
                end if;

            -- --------------------------------------------------------------
            -- Header word is on fifo_rdata.  Decode and set up emit state
            -- for data packets, or drain-only state for command packets.
            -- --------------------------------------------------------------
            when S_HDR =>
                bmtype    := fifo_rdata(15);
                bmcrc     := fifo_rdata(14);
                eth_len   := unsigned(fifo_rdata(13 downto 0));
                cmd_code  := fifo_rdata(13 downto 11);
                cmd_param := unsigned(fifo_rdata(10 downto 0));

                last_bmtype_r     <= bmtype;
                last_eth_length_r <= fifo_rdata(13 downto 0);

                if bmtype = '0' then
                    -- Data packet.
                    payload_b := eth_len;
                    if eth_len >= to_unsigned(4, eth_len'length) then
                        br_init := eth_len - 4;
                    else
                        br_init := (others => '0');
                    end if;
                    wl_init := payload_words_after_hdr(payload_b);

                    word_buf        <= fifo_rdata;
                    byte_lane       <= "10";       -- eth[0] is in lane 2
                    bytes_remaining <= br_init;
                    words_left      <= wl_init;
                    eth_length_r    <= br_init;    -- length seen by higher layers
                    sop_pending     <= '1';

                    if br_init = 0 then
                        -- No Ethernet bytes to emit (eth_len <= 4).
                        if wl_init = 0 then
                            pkt_count_reg    <= pkt_count_reg + 1;
                            pkt_done_pulse_r <= '1';
                            state            <= S_IDLE;
                        else
                            state <= S_DRAIN;
                        end if;
                    else
                        state <= S_EMIT;
                    end if;

                else
                    -- Command packet (suspend/response/tickle hints, or
                    -- Echo/EchoResponse).  Compute payload length and
                    -- drain that many words.
                    if cmd_code = "000" or cmd_code = "001" then
                        payload_b := resize(cmd_param, 14);
                    else
                        payload_b := (others => '0');
                    end if;
                    wl_init := payload_words_after_hdr(payload_b);
                    words_left <= wl_init;
                    if wl_init = 0 then
                        pkt_count_reg    <= pkt_count_reg + 1;
                        pkt_done_pulse_r <= '1';
                        state            <= S_IDLE;
                    else
                        state <= S_DRAIN;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Emit one Ethernet byte per cycle from word_buf.  When the four
            -- lanes are exhausted, either fetch the next word or finish.
            -- When bytes_remaining = 0 we're past the Ethernet payload (in
            -- the FCS region) -- still iterate lanes to keep word draining
            -- on schedule, but suppress eth_valid.
            -- --------------------------------------------------------------
            when S_EMIT =>
                if bytes_remaining /= 0 then
                    eth_data_r  <= byte_at(word_buf, byte_lane);
                    eth_valid_r <= '1';
                    if sop_pending = '1' then
                        eth_sop_r   <= '1';
                        sop_pending <= '0';
                    end if;
                    if bytes_remaining = 1 then
                        eth_eop_r <= '1';
                    end if;
                    bytes_remaining <= bytes_remaining - 1;
                end if;

                if byte_lane = "11" then
                    -- Done with this word.  Either pull another from the
                    -- FIFO or wrap up this EEM packet.
                    byte_lane <= "00";
                    if words_left = 0 then
                        pkt_count_reg    <= pkt_count_reg + 1;
                        pkt_done_pulse_r <= '1';
                        state            <= S_IDLE;
                    else
                        fifo_rreq_r <= '1';
                        words_left  <= words_left - 1;
                        state       <= S_FETCH;
                    end if;
                else
                    byte_lane <= byte_lane + 1;
                end if;

            -- --------------------------------------------------------------
            -- One-cycle wait for the next word's data on fifo_rdata, then
            -- back to S_EMIT starting at lane 0.
            -- --------------------------------------------------------------
            when S_FETCH =>
                word_buf  <= fifo_rdata;
                byte_lane <= "00";
                state     <= S_EMIT;

            -- --------------------------------------------------------------
            -- Drain remaining FIFO words without emitting (command packets
            -- whose payload we don't expose).  Two-cycle pace per word
            -- (issue + wait).
            -- --------------------------------------------------------------
            when S_DRAIN =>
                if words_left = 0 then
                    pkt_count_reg    <= pkt_count_reg + 1;
                    pkt_done_pulse_r <= '1';
                    state            <= S_IDLE;
                else
                    fifo_rreq_r <= '1';
                    words_left  <= words_left - 1;
                    state       <= S_DRAIN_W;
                end if;

            when S_DRAIN_W =>
                state <= S_DRAIN;

            end case;
        end if;
    end process;

end architecture;
