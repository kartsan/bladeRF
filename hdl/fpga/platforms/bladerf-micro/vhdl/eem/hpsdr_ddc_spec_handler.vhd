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
-- hpsdr_ddc_spec_handler
--
-- OpenHPSDR DDC Specific (Rx_specific) command receiver (host -> radio,
-- UDP/1025 in Orion2 V4.0 / Hermes legacy port layout - the inverse of the
-- V4.4 spec doc; see feedback_hpsdr_orion2_port_mapping memory note).
-- Consumes udp_rx_handler's hpsdr_ddc_spec_* byte stream and extracts the
-- active DDC sample-rate field that drives the CIC decimation.
--
-- We emulate Hermes (single DDC0), so DDC0's rate is preferred; DDC2's rate
-- is kept as a fallback for deskHPSDR's Orion2 personality (which carries the
-- main RX on DDC2).  This mirrors the DDC0-preferred / DDC2-fallback choice in
-- hpsdr_hp_cmd_handler for the RX frequency.
--
-- The DDC Specific payload (1444 bytes per Hermes/orion2 Rx_specific_C&C.v)
-- carries per-DDC enable bits then per-DDC config in 6-byte entries starting
-- at byte 17 (entry n: ADC# @17+6n, rate[15:8] @18+6n, rate[7:0] @19+6n,
-- CIC1/2, sample size).  Layout this handler cares about:
--
--   byte 7   EnableRx[0..7]   bit n = '1' iff the host enabled Rx_n.  We watch
--                              bit 0 (DDC0) and bit 2 (DDC2).
--   byte 18  DDC0 rate [15:8] kHz, big-endian   (entry 0 = byte 17 + 6*0)
--   byte 19  DDC0 rate [ 7:0]
--   byte 30  DDC2 rate [15:8] kHz, big-endian   (entry 2 = byte 17 + 6*2)
--   byte 31  DDC2 rate [ 7:0]
--
-- All other fields (ADC#, CIC1/2, sample size, sync bitmaps near byte 1363+,
-- Mux byte 1443) are ignored.
--
-- Output: rate_id_gray, a 3-bit reflected-gray-coded selector that the
-- consumer (hpsdr_ddc in rx_clock domain) syncs across the clock-domain
-- boundary with a per-bit 2-FF synchroniser and decodes to a CIC decimation
-- value.  Gray coding ensures any single-bit metastability resolves to an
-- adjacent valid code, not a garbage rate.  Defaults to "000" (48 kHz,
-- decim 256) at reset and on unrecognised rate values - matching Thetis's
-- power-on behaviour and the fall-back design decision (any unknown rate
-- collapses to 48 kHz rather than holding a possibly-stale value).
--
-- Latch policy: rate_id_gray updates at rx_eop of each command.  If
-- EnableRx[0] was set, DDC0's rate wins; else if EnableRx[2] was set, DDC2's
-- rate is used; if neither was enabled the previously selected rate is held
-- (so a "Rx disabled" reconfiguration command does not clobber it).  An
-- enabled-but-unrecognised rate falls back to 48 kHz.  The CIC has no
-- functional reset (see feedback_orion2_cic_reset_port_unused) so it glitches
-- through the transition - acceptable per design.
--
-- PureSignal (PS) detection (Hermes model, ref Y:\Ilkka\ham\bladerf\hpsdr):
-- Hermes turns on the PS feedback interleave from two fields of this very
-- packet (Hermes.v select_input_RX + Rx_fifo_ctrl0 .Sync(Mux)):
--
--   byte 23   RxADC[1] == 0x01 -> DDC1 is fed from the TX DAC (the reference
--                                 stream), not a physical ADC.
--   byte 1363 Mux      bit0='1' -> Rx0 is in multiplexed mode: the RF feedback
--                                 (Rx0) and the DAC reference (Rx1) get
--                                 interleaved onto the DDC0 / port-1035 stream.
--
-- We export ps_active = Mux[0] AND (RxADC[1] = 0x01), latched at rx_eop next to
-- rate_id_gray.  The Hermes FIFO controller actually gates on Mux /= 0; we also
-- require RxADC[1]=1 so a stray Mux bit can't be mistaken for the PS config.
--
-- CAVEAT: the Hermes RTL writes Mux from payload byte 1363, but the openHPSDR
-- protocol doc lists the Mux byte at 1443.  BYTE_MUX below follows the RTL;
-- confirm it against a Thetis tcpdump with PureSignal enabled before trusting
-- it (same "decode by what Thetis sends, not the spec doc" lesson as the
-- 1025/1027 port swap).  ps_active is a single bit -- cross it to the consumer
-- clock with a plain 2-FF synchroniser (inherently metastability-safe; no gray
-- coding needed unlike the multi-bit rate selector).
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_ddc_spec_handler is
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        -- DDC Specific byte stream (UDP/1025 payload, header stripped) from
        -- udp_rx_handler's hpsdr_ddc_spec_* outputs.
        rx_data       : in  std_logic_vector(7 downto 0);
        rx_valid      : in  std_logic;
        rx_sop        : in  std_logic;
        rx_eop        : in  std_logic;

        -- 3-bit reflected-gray-coded active-DDC sample-rate selector (DDC0
        -- preferred, DDC2 fallback).  Stable between commands; updates at
        -- most once per command (at rx_eop).  Cross to rx_clock with a
        -- per-bit 2-FF synchroniser, then decode to a CIC decimation value.
        --
        --   gray "000" -> 48   kHz (decim 256)  (default / fall-back)
        --        "001" -> 96   kHz (decim 128)
        --        "011" -> 192  kHz (decim  64)
        --        "010" -> 384  kHz (decim  32)
        --        "110" -> 768  kHz (decim  16)
        --        "111" -> 1536 kHz (decim   8)
        --        "100"/"101"   -> invalid (consumer falls back to 48 kHz)
        rate_id_gray  : out std_logic_vector(2 downto 0);

        -- PureSignal feedback-interleave request, decoded from the same
        -- command (Mux[0] AND RxADC[1]=0x01).  Held stable between commands,
        -- updated at most once per command (at rx_eop).  Single bit: sync to
        -- the consumer clock with a 2-FF synchroniser.  Defaults '0'.
        ps_active     : out std_logic
    );
end entity;

architecture arch of hpsdr_ddc_spec_handler is

    constant BYTE_ENABLE_RX0_7 : natural := 7;
    constant BYTE_DDC0_RATE_HI : natural := 18;
    constant BYTE_DDC0_RATE_LO : natural := 19;
    constant BYTE_DDC2_RATE_HI : natural := 30;
    constant BYTE_DDC2_RATE_LO : natural := 31;

    -- PureSignal fields: RxADC[1] (entry 1 = byte 17 + 6*1) and the Mux byte.
    -- See header for the BYTE_MUX (1363 RTL vs 1443 doc) caveat.
    constant BYTE_RXADC1       : natural := 23;
    constant BYTE_MUX          : natural := 1363;

    -- 11 bits cover the full 1444-byte payload index range (0..1443).
    signal byte_idx       : unsigned(10 downto 0) := (others => '0');

    -- Per-command captures, reset at SOP so a malformed prior command can't
    -- bleed state into the next one: EnableRx[0]/[2] from byte 7, and the
    -- DDC0 (bytes 18..19) and DDC2 (bytes 30..31) rate words.  The selection
    -- (DDC0 preferred) is resolved at rx_eop.
    signal en_rx0_r       : std_logic                    := '0';
    signal en_rx2_r       : std_logic                    := '0';
    signal ddc0_hi_r      : std_logic_vector(7 downto 0) := (others => '0');
    signal ddc0_lo_r      : std_logic_vector(7 downto 0) := (others => '0');
    signal ddc2_hi_r      : std_logic_vector(7 downto 0) := (others => '0');
    signal ddc2_lo_r      : std_logic_vector(7 downto 0) := (others => '0');

    -- PureSignal per-command captures (RxADC[1] @23, Mux @1363), reset at SOP
    -- with the others; resolved into ps_active at rx_eop.
    signal rxadc1_r       : std_logic_vector(7 downto 0) := (others => '0');
    signal mux_r          : std_logic_vector(7 downto 0) := (others => '0');

    -- Slow-changing output; default to 48 kHz at reset.
    signal rate_id_gray_r : std_logic_vector(2 downto 0) := "000";

    -- Slow-changing PS flag; default off at reset.
    signal ps_active_r    : std_logic := '0';

    -- Map a 16-bit BE rate in kHz to its gray code + a validity flag.
    -- Six recognised values; everything else returns valid='0' and the
    -- caller falls back to 48 kHz per design.
    type rate_lookup_t is record
        gray  : std_logic_vector(2 downto 0);
        valid : std_logic;
    end record;

    function lookup_rate(hi : std_logic_vector(7 downto 0);
                         lo : std_logic_vector(7 downto 0))
        return rate_lookup_t is
        variable r : rate_lookup_t := ("000", '0');
    begin
        if    hi = x"00" and lo = x"30" then  -- 48 kHz
            r := ("000", '1');
        elsif hi = x"00" and lo = x"60" then  -- 96 kHz
            r := ("001", '1');
        elsif hi = x"00" and lo = x"c0" then  -- 192 kHz
            r := ("011", '1');
        elsif hi = x"01" and lo = x"80" then  -- 384 kHz
            r := ("010", '1');
        elsif hi = x"03" and lo = x"00" then  -- 768 kHz
            r := ("110", '1');
        elsif hi = x"06" and lo = x"00" then  -- 1536 kHz
            r := ("111", '1');
        else
            r := ("000", '0');
        end if;
        return r;
    end function;

begin

    rate_id_gray <= rate_id_gray_r;
    ps_active    <= ps_active_r;

    fsm : process(clock, reset)
        variable n_byte_idx : unsigned(10 downto 0);
        variable lk         : rate_lookup_t;
    begin
        if reset = '1' then
            byte_idx       <= (others => '0');
            en_rx0_r       <= '0';
            en_rx2_r       <= '0';
            ddc0_hi_r      <= (others => '0');
            ddc0_lo_r      <= (others => '0');
            ddc2_hi_r      <= (others => '0');
            ddc2_lo_r      <= (others => '0');
            rxadc1_r       <= (others => '0');
            mux_r          <= (others => '0');
            rate_id_gray_r <= "000";
            ps_active_r    <= '0';
        elsif rising_edge(clock) then
            if rx_valid = '1' then
                n_byte_idx := byte_idx;

                if rx_sop = '1' then
                    n_byte_idx := (others => '0');
                    en_rx0_r   <= '0';
                    en_rx2_r   <= '0';
                    ddc0_hi_r  <= (others => '0');
                    ddc0_lo_r  <= (others => '0');
                    ddc2_hi_r  <= (others => '0');
                    ddc2_lo_r  <= (others => '0');
                    rxadc1_r   <= (others => '0');
                    mux_r      <= (others => '0');
                end if;

                case to_integer(n_byte_idx) is
                    when BYTE_ENABLE_RX0_7 =>
                        en_rx0_r <= rx_data(0);
                        en_rx2_r <= rx_data(2);

                    when BYTE_DDC0_RATE_HI =>
                        ddc0_hi_r <= rx_data;
                    when BYTE_DDC0_RATE_LO =>
                        ddc0_lo_r <= rx_data;

                    when BYTE_DDC2_RATE_HI =>
                        ddc2_hi_r <= rx_data;
                    when BYTE_DDC2_RATE_LO =>
                        ddc2_lo_r <= rx_data;

                    when BYTE_RXADC1 =>
                        rxadc1_r <= rx_data;
                    when BYTE_MUX =>
                        mux_r <= rx_data;

                    when others =>
                        null;
                end case;

                -- Resolve the active rate at end-of-command: DDC0 preferred,
                -- DDC2 fallback, hold previous if neither DDC was enabled.
                if rx_eop = '1' then
                    if en_rx0_r = '1' then
                        lk := lookup_rate(ddc0_hi_r, ddc0_lo_r);
                        if lk.valid = '1' then
                            rate_id_gray_r <= lk.gray;
                        else
                            rate_id_gray_r <= "000";  -- unrecognised -> 48 kHz
                        end if;
                    elsif en_rx2_r = '1' then
                        lk := lookup_rate(ddc2_hi_r, ddc2_lo_r);
                        if lk.valid = '1' then
                            rate_id_gray_r <= lk.gray;
                        else
                            rate_id_gray_r <= "000";  -- unrecognised -> 48 kHz
                        end if;
                    end if;

                    -- PureSignal: Rx0 muxed (Mux[0]) AND Rx1 sourced from the
                    -- TX DAC (RxADC[1]=0x01).  Unconditional each command (no
                    -- "hold previous" like rate): a command that clears Mux or
                    -- re-points RxADC[1] turns PS off, matching Hermes where
                    -- the interleave follows Mux live.
                    if mux_r(0) = '1' and rxadc1_r = x"01" then
                        ps_active_r <= '1';
                    else
                        ps_active_r <= '0';
                    end if;

                    n_byte_idx := (others => '0');
                else
                    n_byte_idx := n_byte_idx + 1;
                end if;

                byte_idx <= n_byte_idx;
            end if;
        end if;
    end process;

end architecture;
