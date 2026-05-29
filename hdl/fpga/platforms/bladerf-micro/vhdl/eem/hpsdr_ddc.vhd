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
-- hpsdr_ddc
--
-- CIC decimator for the HPSDR DDC RX path.  Unlike the openHPSDR/orion2
-- receiver (receiver2.v) there is NO CORDIC here: the AD9361 delivers complex
-- baseband I/Q already tuned by its internal RX LO, so this block only has to
-- decimate the fixed native rate down to the HPSDR DDC rate.
--
-- Reuses orion2's cic.v / cic_integrator.v / cic_comb.v (GPLv2) verbatim, one
-- CIC per I and Q.  Both CICs share clock, in_strobe and decimation, so their
-- out_strobes are bit-aligned and the I channel's serves as the pair's valid.
--
-- Native AD9361 rate is 12.288 MSPS; DECIMATION = 12_288_000 / hpsdr_rate.
-- This first cut is fixed at 48 kHz => DECIMATION = 256, built as a single-rate
-- CIC (MIN_DECIMATION = MAX_DECIMATION), which uses cic.v's fixed-rounding path
-- and ignores the decimation port.  Phase 2 (DDC-Specific decode) re-parameter-
-- ises to MIN/MAX = 2/256 and drives a runtime decimation select.
--
-- Runs entirely in rx_clock (= ad9361.clock).  A downstream async FIFO crosses
-- the decimated stream to the EEM/system clock for hpsdr_ddc_iq_sender.
--
-- NOTE: cic.v has no functional reset (see orion2 CIC reset-port-unused note);
-- the reset port is wired for completeness but integrator/comb state relies on
-- Cyclone V config-time zero init.  Continuous streaming makes any stale state
-- transient, so this is harmless for the free-running RX path.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_ddc is
    generic (
        IN_WIDTH   : natural := 16;    -- adc_streams(0) sample width
        OUT_WIDTH  : natural := 24;    -- HPSDR DDC IQ sample width
        STAGES     : natural := 5;
        DECIMATION : natural := 256    -- 12.288 MHz / 48 kHz
    );
    port (
        clock     : in  std_logic;     -- rx_clock (= ad9361.clock)
        reset     : in  std_logic;

        -- Complex baseband from the AD9361 tap (adc_streams(0)); in_valid
        -- strobes one I/Q sample at the native AD9361 rate.
        in_i      : in  signed(IN_WIDTH-1 downto 0);
        in_q      : in  signed(IN_WIDTH-1 downto 0);
        in_valid  : in  std_logic;

        -- Decimated I/Q; out_valid pulses one sample at the HPSDR DDC rate.
        out_i     : out signed(OUT_WIDTH-1 downto 0);
        out_q     : out signed(OUT_WIDTH-1 downto 0);
        out_valid : out std_logic
    );
end entity;

architecture arch of hpsdr_ddc is

    -- ceil(log2(n)), matching Verilog $clog2 for the cic.v decimation port.
    function clog2(n : natural) return natural is
        variable r : natural := 0;
        variable v : natural := n - 1;
    begin
        while v > 0 loop
            v := v / 2;
            r := r + 1;
        end loop;
        return r;
    end function;

    -- cic.v declares decimation as [$clog2(MAX_DECIMATION):0] => clog2+1 bits.
    -- Unused in single-rate (MIN=MAX) mode, but must still match width & be tied.
    constant DECIM_PORT_W : natural := clog2(DECIMATION) + 1;
    constant DECIM_VEC    : std_logic_vector(DECIM_PORT_W-1 downto 0)
        := std_logic_vector(to_unsigned(DECIMATION, DECIM_PORT_W));

    component cic is
        generic (
            STAGES         : natural;
            MIN_DECIMATION : natural;
            MAX_DECIMATION : natural;
            IN_WIDTH       : natural;
            OUT_WIDTH      : natural
        );
        port (
            reset      : in  std_logic;
            decimation : in  std_logic_vector(DECIM_PORT_W-1 downto 0);
            clock      : in  std_logic;
            in_strobe  : in  std_logic;
            out_strobe : out std_logic;
            in_data    : in  std_logic_vector(IN_WIDTH-1 downto 0);
            out_data   : out std_logic_vector(OUT_WIDTH-1 downto 0)
        );
    end component;

    signal cic_out_i : std_logic_vector(OUT_WIDTH-1 downto 0);
    signal cic_out_q : std_logic_vector(OUT_WIDTH-1 downto 0);
    signal strobe_i  : std_logic;

begin

    U_cic_i : cic
        generic map (
            STAGES         => STAGES,
            MIN_DECIMATION => DECIMATION,
            MAX_DECIMATION => DECIMATION,
            IN_WIDTH       => IN_WIDTH,
            OUT_WIDTH      => OUT_WIDTH
        )
        port map (
            reset      => reset,
            decimation => DECIM_VEC,
            clock      => clock,
            in_strobe  => in_valid,
            out_strobe => strobe_i,
            in_data    => std_logic_vector(in_i),
            out_data   => cic_out_i
        );

    U_cic_q : cic
        generic map (
            STAGES         => STAGES,
            MIN_DECIMATION => DECIMATION,
            MAX_DECIMATION => DECIMATION,
            IN_WIDTH       => IN_WIDTH,
            OUT_WIDTH      => OUT_WIDTH
        )
        port map (
            reset      => reset,
            decimation => DECIM_VEC,
            clock      => clock,
            in_strobe  => in_valid,
            out_strobe => open,
            in_data    => std_logic_vector(in_q),
            out_data   => cic_out_q
        );

    out_i     <= signed(cic_out_i);
    out_q     <= signed(cic_out_q);
    out_valid <= strobe_i;

end architecture;
