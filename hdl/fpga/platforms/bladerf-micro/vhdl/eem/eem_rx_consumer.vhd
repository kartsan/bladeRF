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
-- originally written by the host via cdc_eem's bulk OUT endpoint). For each
-- EEM packet it:
--   * reads the 2-byte EEM header from the low half of the head word,
--   * decodes payload length (bmType=0 -> ethernet_length; bmType=1 with
--     Echo/EchoResponse cmd -> bmEEMCmdParam; other commands -> 0),
--   * consumes the remaining payload words from the FIFO and discards them,
--   * pulses pkt_done_pulse and bumps pkt_count.
--
-- The first word of every EEM packet contains [hdr_hi:hdr_lo : pld[1]:pld[0]]
-- so 2 payload bytes are already in hand after the header read; the remaining
-- payload word count is ceil((payload_bytes - 2) / 4).
--
-- Assumes the FX3->FPGA GPIF TX2 burst is terminated cleanly on DMA_RDY_TH2
-- deassertion (see the corresponding fix in fx3_gpif.vhd's SAMPLE_WRITE for
-- TX2). Without that fix, the FIFO would contain trailing garbage past the
-- real URB end and the parser would desync at the first packet boundary.
--
-- For initial TX-path bringup the payload is discarded after counting. The
-- same skeleton later grows three outputs (payload_data / payload_valid /
-- payload_eop) feeding the eventual Ethernet/UDP front-end.
--
-- Throughput: paced at one FIFO read every two pclk cycles (~50 Mword/s on a
-- 100 MHz pclk = 200 MB/s). EEM bandwidth need is < 10 MB/s; ample margin.
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

        -- Observability
        pkt_done_pulse  : out std_logic;                       -- 1-cycle pulse per completed packet
        pkt_count       : out std_logic_vector(15 downto 0);   -- saturating wrap-around packet counter
        last_eth_length : out std_logic_vector(13 downto 0);   -- header bits 13:0 of last packet
        last_bmtype     : out std_logic                        -- header bit 15 of last packet
    );
end entity;

architecture arch of eem_rx_consumer is

    type state_t is (S_IDLE, S_HDR, S_DRAIN);

    -- 14 bits covers EthernetLength up to 16383, well past 1518 B max Ethernet
    -- frame; words_left counts 32-bit reads (max ~380 for a max-length frame).
    constant WORDS_LEFT_W : natural := 11;

    signal state              : state_t                                := S_IDLE;
    signal words_left         : unsigned(WORDS_LEFT_W-1 downto 0)      := (others => '0');
    signal read_in_flight     : std_logic                              := '0';
    signal pkt_count_reg      : unsigned(15 downto 0)                  := (others => '0');
    signal last_eth_length_r  : std_logic_vector(13 downto 0)          := (others => '0');
    signal last_bmtype_r      : std_logic                              := '0';
    signal pkt_done_pulse_r   : std_logic                              := '0';
    signal fifo_rreq_r        : std_logic                              := '0';

    -- ceil((payload_bytes - 2) / 4), with payload_bytes <= 2 collapsing to 0
    function payload_words_after_hdr (payload_bytes : unsigned) return unsigned is
        variable rem_bytes : unsigned(payload_bytes'range);
    begin
        if payload_bytes <= to_unsigned(2, payload_bytes'length) then
            return to_unsigned(0, WORDS_LEFT_W);
        else
            rem_bytes := payload_bytes - 2;
            -- ceil(rem_bytes / 4) = (rem_bytes + 3) / 4
            return resize(shift_right(rem_bytes + 3, 2), WORDS_LEFT_W);
        end if;
    end function;

begin

    fifo_rreq       <= fifo_rreq_r;
    pkt_done_pulse  <= pkt_done_pulse_r;
    pkt_count       <= std_logic_vector(pkt_count_reg);
    last_eth_length <= last_eth_length_r;
    last_bmtype     <= last_bmtype_r;

    fsm : process(clock, reset)
        variable bmtype       : std_logic;
        variable bmcrc        : std_logic;
        variable eth_len      : unsigned(13 downto 0);
        variable cmd_code     : std_logic_vector(2 downto 0);
        variable cmd_param    : unsigned(10 downto 0);
        variable payload_b    : unsigned(13 downto 0);
        variable wl_init      : unsigned(WORDS_LEFT_W-1 downto 0);
    begin
        if reset = '1' then
            state             <= S_IDLE;
            words_left        <= (others => '0');
            read_in_flight    <= '0';
            pkt_count_reg     <= (others => '0');
            last_eth_length_r <= (others => '0');
            last_bmtype_r     <= '0';
            pkt_done_pulse_r  <= '0';
            fifo_rreq_r       <= '0';
        elsif rising_edge(clock) then
            -- Defaults
            pkt_done_pulse_r <= '0';
            fifo_rreq_r      <= '0';

            case state is
            -- --------------------------------------------------------------
            -- Wait for the first word of a new EEM packet to be at the head
            -- of the FIFO, then issue a read for it (data appears next cycle).
            -- --------------------------------------------------------------
            when S_IDLE =>
                if fifo_empty = '0' then
                    fifo_rreq_r    <= '1';
                    read_in_flight <= '1';
                    state          <= S_HDR;
                end if;

            -- --------------------------------------------------------------
            -- The header word is on fifo_rdata. Decode it, compute payload
            -- word count, decide whether the packet is fully consumed by
            -- this single read (payload <= 2 bytes => entirely in header
            -- word's upper half) or whether we need to drain more.
            -- --------------------------------------------------------------
            when S_HDR =>
                if read_in_flight = '1' then
                    read_in_flight <= '0';

                    bmtype    := fifo_rdata(15);
                    bmcrc     := fifo_rdata(14);
                    eth_len   := unsigned(fifo_rdata(13 downto 0));
                    cmd_code  := fifo_rdata(13 downto 11);
                    cmd_param := unsigned(fifo_rdata(10 downto 0));

                    last_bmtype_r     <= bmtype;
                    last_eth_length_r <= fifo_rdata(13 downto 0);

                    if bmtype = '0' then
                        -- Ethernet data packet: payload = EthernetLength bytes
                        -- (includes trailing 4-byte FCS or sentinel 0xDEADBEEF).
                        payload_b := eth_len;
                    else
                        -- EEM command packet: only Echo (000) and EchoResponse
                        -- (001) carry a payload of bmEEMCmdParam bytes; all
                        -- others (suspend/response/tickle hints) carry none.
                        if cmd_code = "000" or cmd_code = "001" then
                            payload_b := resize(cmd_param, 14);
                        else
                            payload_b := (others => '0');
                        end if;
                    end if;

                    wl_init := payload_words_after_hdr(payload_b);

                    if wl_init = 0 then
                        -- Packet fully contained in the header word.
                        pkt_count_reg    <= pkt_count_reg + 1;
                        pkt_done_pulse_r <= '1';
                        state            <= S_IDLE;
                    else
                        words_left <= wl_init;
                        state      <= S_DRAIN;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Pop `words_left` more words from the FIFO and discard them.
            -- Paced at one read every two cycles (issue / wait-for-data).
            -- --------------------------------------------------------------
            when S_DRAIN =>
                if read_in_flight = '1' then
                    -- The read we issued last cycle has landed on fifo_rdata
                    -- this cycle; the data isn't used (bringup-only). Mark
                    -- it consumed and decide whether more reads are needed.
                    read_in_flight <= '0';
                    if words_left = 1 then
                        words_left       <= (others => '0');
                        pkt_count_reg    <= pkt_count_reg + 1;
                        pkt_done_pulse_r <= '1';
                        state            <= S_IDLE;
                    else
                        words_left <= words_left - 1;
                    end if;
                elsif fifo_empty = '0' and words_left /= 0 then
                    fifo_rreq_r    <= '1';
                    read_in_flight <= '1';
                end if;
            end case;
        end if;
    end process;

end architecture;
