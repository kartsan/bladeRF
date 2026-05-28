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
-- hpsdr_rx_chain
--
-- Decimates the bladeRF's RFIC-corrected RX sample stream from the AD9361's
-- native rate (we keep the chip configured for 1.536 MSPS via libbladeRF) down
-- to one of the openHPSDR Protocol 2 audio rates (48/96/192/384/768/1536 kHz)
-- using a 5-stage CIC filter per quadrature leg, then crosses the result
-- from the rx_clock sample-rate domain to the fx3_pclk_pll EEM domain via a
-- dual-clock async FIFO.
--
-- This iteration is a bring-up cut: decimation is HARDCODED to 32 (i.e.
-- 1536 / 32 = 48 kHz output) by tying MIN_DECIMATION=MAX_DECIMATION in the
-- CIC instance.  When hpsdr_ddc_specific_handler (UDP/1025 host->radio
-- decoder) lands, the CIC will be re-parameterised with runtime decimation
-- driven from the host's chosen sample rate.
--
-- Architecture
-- ------------
--   adc_i, adc_q (16-bit, rx_clock domain)
--     |
--     +-- cic_i (STAGES=5, decim=32, 16->24 bit) ----+
--     +-- cic_q                                     |
--                                                   v
--                                  pack {I[23:0], Q[23:0]} -> 48-bit
--                                                   |
--                                                   v
--                          common_dcfifo (48-bit wide, 256 deep)
--                          wrclk=rx_clock, rdclk=iq_clock
--                                                   |
--                                                   v
--                                       unpack -> iq_i, iq_q, iq_valid
--                                       (iq_clock domain)
--
-- Reset note
-- ----------
-- The upstream cic.v / cic_integrator.v / cic_comb.v modules declare a
-- `reset` port but do not use it inside the implementation -- they rely on
-- Cyclone V's config-time initial values.  This is fine on power-up but
-- means a runtime rx_reset will NOT clear integrator state; samples will
-- continue to settle until the IIR memory naturally flushes (~N_STAGES
-- output samples).  For our use case this is acceptable; if it becomes a
-- problem we can fork cic.v to honour reset.
--
-- Timing
-- ------
-- Worst-case write rate is 1.536 MS/s (one CIC output every ~21 us at
-- decim=32).  Read rate is 100 MHz worst-case (continuous drain).  256-deep
-- buffer absorbs even pessimistic dcfifo synchronizer latency (RDSYNC_
-- DELAYPIPE=5, WRSYNC_DELAYPIPE=5 -> ~50 ns each way at 100 MHz).
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_rx_chain is
    generic (
        -- CIC stages.  5 is Orion2's choice and gives good alias rejection
        -- for our 48 kHz audio output from a 3072 kHz native rate.
        CIC_STAGES        : natural := 5;

        -- Fixed decimation factor.  3072 kHz / 64 = 48 kHz.  The AD9361 runs
        -- at 3.072 MSPS (above the 2.083 MHz plain-datapath floor) so the
        -- NIOS autonomous bring-up can set the rate with a single SAMPLERATE
        -- command -- no 4x decimation-FIR dance required.  Will become a
        -- runtime input when the DDC Specific (UDP/1025) decoder lands; for
        -- now MIN_DECIMATION == MAX_DECIMATION makes cic.v drop the runtime
        -- decimation logic entirely.
        DECIMATION        : natural := 64;

        -- I/Q widths.  The AD9361 actually produces 12-bit signed samples
        -- LSB-justified in mux_streams' signed(15:0) container, so we narrow
        -- via `resize(adc_i, 12)` at the input and let the CIC operate on
        -- 12-bit data.  At decim=64 this gives ACC_WIDTH = 12 + 5*6 = 42
        -- bits per stage.  OUT_WIDTH stays at 24 to match HPSDR P2's "bits
        -- per sample = 24" wire format.
        CIC_IN_WIDTH      : natural := 12;
        CIC_OUT_WIDTH     : natural := 24;

        -- Async FIFO depth.  Only needs to absorb a few samples of slack
        -- between the CIC's 48 kHz output (one write every ~2 ms relative
        -- to the fx3_pclk_pll consumer) and the EEM-side drain rate.  64
        -- entries is ample; smaller doesn't help (one M10K is the
        -- granularity anyway).
        FIFO_DEPTH        : natural := 64
    );
    port (
        -- RX (sample-rate) clock domain.  rx_clock is the AD9361-derived
        -- sample clock; adc_valid pulses once per native sample.
        rx_clock          : in  std_logic;
        rx_reset          : in  std_logic;
        adc_i             : in  signed(15 downto 0);
        adc_q             : in  signed(15 downto 0);
        adc_valid         : in  std_logic;

        -- IQ (EEM) clock domain.  iq_clock is fx3_pclk_pll (100 MHz).
        -- iq_valid pulses for one iq_clock cycle per decimated sample
        -- pair.  Consumer must capture (iq_i, iq_q) on that cycle.
        iq_clock          : in  std_logic;
        iq_reset          : in  std_logic;
        iq_i              : out signed(23 downto 0);
        iq_q              : out signed(23 downto 0);
        iq_valid          : out std_logic
    );
end entity;

architecture arch of hpsdr_rx_chain is

    -- cic.v's `decimation` port is declared as
    --   input [$clog2(MAX_DECIMATION):0] decimation;
    -- which for MAX_DECIMATION=64 is a 7-bit bus ([clog2(64):0] = [6:0]).
    -- Match it exactly in the component decl so Quartus's mixed-language
    -- elaboration does not complain about implicit width inference.  Also
    -- note to_unsigned(64, DEC_WIDTH) below needs DEC_WIDTH >= 7.
    constant DEC_WIDTH : natural := 7;

    component cic
        generic (
            STAGES         : natural := 5;
            MIN_DECIMATION : natural := 2;
            MAX_DECIMATION : natural := 40;
            IN_WIDTH       : natural := 18;
            OUT_WIDTH      : natural := 18
        );
        port (
            reset      : in  std_logic;
            decimation : in  std_logic_vector(DEC_WIDTH-1 downto 0);
            clock      : in  std_logic;
            in_strobe  : in  std_logic;
            out_strobe : out std_logic;
            in_data    : in  std_logic_vector(IN_WIDTH-1 downto 0);
            out_data   : out std_logic_vector(OUT_WIDTH-1 downto 0)
        );
    end component;

    signal decim_vec   : std_logic_vector(DEC_WIDTH-1 downto 0);

    signal cic_i_out      : std_logic_vector(CIC_OUT_WIDTH-1 downto 0);
    signal cic_q_out      : std_logic_vector(CIC_OUT_WIDTH-1 downto 0);
    signal cic_i_strobe   : std_logic;
    signal cic_q_strobe   : std_logic;

    -- Both CICs share rx_clock and in_strobe -> their out_strobes are
    -- bit-aligned. We use the I leg's strobe as the FIFO write enable.

    signal fifo_wr_data   : std_logic_vector(2*CIC_OUT_WIDTH-1 downto 0);
    signal fifo_wr_req    : std_logic;
    signal fifo_wr_full   : std_logic;

    signal fifo_rd_data   : std_logic_vector(2*CIC_OUT_WIDTH-1 downto 0);
    signal fifo_rd_req    : std_logic;
    signal fifo_rd_empty  : std_logic;

    -- Numwords needs to be a power of 2 for Altera dcfifo.
    function clog2(n : positive) return natural is
        variable v : natural := 1;
        variable r : natural := 0;
    begin
        while v < n loop
            v := v * 2;
            r := r + 1;
        end loop;
        return r;
    end function;

    constant FIFO_USEDW_W : natural := clog2(FIFO_DEPTH);

begin

    -- Decimation value driven onto the unused (but still present) port.
    decim_vec <= std_logic_vector(to_unsigned(DECIMATION, DEC_WIDTH));

    -- -----------------------------------------------------------------
    -- Two CIC filters (I and Q) in rx_clock domain.
    -- -----------------------------------------------------------------
    -- AD9361 12-bit samples are LSB-justified in the 16-bit container, so
    -- `resize` to the CIC's IN_WIDTH (12) drops only the sign-extension
    -- MSBs.  If the AD9361 IP turns out to be MSB-justified instead, swap
    -- this for `shift_right(adc_i, 4)` -- audio would be present either
    -- way, just at different magnitude.
    U_cic_i : cic
        generic map (
            STAGES         => CIC_STAGES,
            MIN_DECIMATION => DECIMATION,
            MAX_DECIMATION => DECIMATION,
            IN_WIDTH       => CIC_IN_WIDTH,
            OUT_WIDTH      => CIC_OUT_WIDTH
        )
        port map (
            reset      => rx_reset,
            decimation => decim_vec,
            clock      => rx_clock,
            in_strobe  => adc_valid,
            out_strobe => cic_i_strobe,
            in_data    => std_logic_vector(resize(adc_i, CIC_IN_WIDTH)),
            out_data   => cic_i_out
        );

    U_cic_q : cic
        generic map (
            STAGES         => CIC_STAGES,
            MIN_DECIMATION => DECIMATION,
            MAX_DECIMATION => DECIMATION,
            IN_WIDTH       => CIC_IN_WIDTH,
            OUT_WIDTH      => CIC_OUT_WIDTH
        )
        port map (
            reset      => rx_reset,
            decimation => decim_vec,
            clock      => rx_clock,
            in_strobe  => adc_valid,
            out_strobe => cic_q_strobe,
            in_data    => std_logic_vector(resize(adc_q, CIC_IN_WIDTH)),
            out_data   => cic_q_out
        );

    -- -----------------------------------------------------------------
    -- Pack {I, Q} into a single FIFO word and write on strobe.
    -- -----------------------------------------------------------------
    fifo_wr_data <= cic_i_out & cic_q_out;
    fifo_wr_req  <= cic_i_strobe and not fifo_wr_full;

    -- -----------------------------------------------------------------
    -- Async FIFO: rx_clock -> iq_clock, 48-bit wide.
    -- common_dcfifo defaults work fine; only width/depth and clock
    -- topology need overrides.
    -- -----------------------------------------------------------------
    U_iq_fifo : entity work.common_dcfifo
        generic map (
            ADD_RAM_OUTPUT_REGISTER => "OFF",
            ADD_USEDW_MSB_BIT       => "OFF",
            CLOCKS_ARE_SYNCHRONIZED => "FALSE",
            DELAY_RDUSEDW           => 1,
            DELAY_WRUSEDW           => 1,
            INTENDED_DEVICE_FAMILY  => "Cyclone V",
            LPM_NUMWORDS            => FIFO_DEPTH,
            LPM_SHOWAHEAD           => "ON",
            LPM_WIDTH               => 2*CIC_OUT_WIDTH,
            LPM_WIDTH_R             => 2*CIC_OUT_WIDTH,
            OVERFLOW_CHECKING       => "ON",
            RDSYNC_DELAYPIPE        => 5,
            READ_ACLR_SYNCH         => "ON",
            UNDERFLOW_CHECKING      => "ON",
            USE_EAB                 => "ON",
            WRITE_ACLR_SYNCH        => "ON",
            WRSYNC_DELAYPIPE        => 5
        )
        port map (
            aclr      => iq_reset,
            data      => fifo_wr_data,
            wrclk     => rx_clock,
            wrreq     => fifo_wr_req,
            wrfull    => fifo_wr_full,
            wrempty   => open,
            wrusedw   => open,
            rdclk     => iq_clock,
            rdreq     => fifo_rd_req,
            q         => fifo_rd_data,
            rdempty   => fifo_rd_empty,
            rdfull    => open,
            rdusedw   => open
        );

    -- -----------------------------------------------------------------
    -- Continuous drain in iq_clock domain.  Show-ahead FIFO means q is
    -- valid the cycle BEFORE we pulse rdreq.  We assert iq_valid on the
    -- cycle of rdreq so the consumer can latch (iq_i, iq_q) at the same
    -- moment we acknowledge.  iq_valid is at most 1-of-N, so consumer
    -- doesn't need backpressure (well below 48 kHz << 100 MHz).
    -- -----------------------------------------------------------------
    fifo_rd_req <= not fifo_rd_empty;

    drain_proc : process(iq_clock)
    begin
        if rising_edge(iq_clock) then
            if iq_reset = '1' then
                iq_valid <= '0';
                iq_i     <= (others => '0');
                iq_q     <= (others => '0');
            else
                iq_valid <= fifo_rd_req;
                iq_i     <= signed(fifo_rd_data(2*CIC_OUT_WIDTH-1 downto CIC_OUT_WIDTH));
                iq_q     <= signed(fifo_rd_data(CIC_OUT_WIDTH-1 downto 0));
            end if;
        end if;
    end process drain_proc;

end architecture;
