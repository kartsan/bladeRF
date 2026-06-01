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
-- DDC2 sample-rate field for the wideband panadapter source.
--
-- The DDC Specific payload (1444 bytes per orion2 Rx_specific_C&C.v) carries
-- per-DDC enable bits and per-DDC config.  Layout this handler cares about:
--
--   byte 7   EnableRx[0..7]   bit n = '1' iff Thetis enabled Rx_n
--                              We only watch bit 2 (= DDC2, the wideband pan
--                              source per project_thetis_ddc_role_mapping).
--   byte 30  DDC2 rate [15:8] kHz, big-endian
--   byte 31  DDC2 rate [ 7:0]
--
-- DDC2 entry begins at payload byte 17 + 6*2 = 29 (6 bytes per DDC entry).
-- We skip every other field (ADC#, CIC1/2, sample size) since the only
-- variable that drives behaviour today is the CIC decimation, derived from
-- the rate.  The remaining bytes (sync bitmaps near byte 1363+, Mux byte
-- 1443) are also ignored.
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
-- Latch policy: rate_id_gray updates at byte 31 of each command, but only
-- if EnableRx[2] was '1' in byte 7 of the SAME command.  This lets Thetis
-- send "Rx2 disabled" commands (e.g. during reconfiguration) without
-- clobbering the previously selected rate; the consumer keeps using the
-- last valid rate.  The CIC has no functional reset (see
-- feedback_orion2_cic_reset_port_unused) so it glitches through the
-- transition - acceptable per design.
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

        -- 3-bit reflected-gray-coded DDC2 sample-rate selector.  Stable
        -- between commands; updates at most once per command (at byte 31
        -- of the payload).  Cross to rx_clock with a per-bit 2-FF
        -- synchroniser, then decode to a CIC decimation value.
        --
        --   gray "000" -> 48   kHz (decim 256)  (default / fall-back)
        --        "001" -> 96   kHz (decim 128)
        --        "011" -> 192  kHz (decim  64)
        --        "010" -> 384  kHz (decim  32)
        --        "110" -> 768  kHz (decim  16)
        --        "111" -> 1536 kHz (decim   8)
        --        "100"/"101"   -> invalid (consumer falls back to 48 kHz)
        rate_id_gray  : out std_logic_vector(2 downto 0)
    );
end entity;

architecture arch of hpsdr_ddc_spec_handler is

    constant BYTE_ENABLE_RX0_7 : natural := 7;
    constant BYTE_DDC2_RATE_HI : natural := 30;
    constant BYTE_DDC2_RATE_LO : natural := 31;

    -- 11 bits cover the full 1444-byte payload index range (0..1443).
    signal byte_idx       : unsigned(10 downto 0) := (others => '0');

    -- Per-command captures: EnableRx[2] from byte 7 and the high byte of
    -- the DDC2 rate from byte 30.  Reset at SOP so a malformed prior
    -- command can't bleed state into the next one.
    signal en_rx2_r       : std_logic                    := '0';
    signal rate_hi_r      : std_logic_vector(7 downto 0) := (others => '0');

    -- Slow-changing output; default to 48 kHz at reset.
    signal rate_id_gray_r : std_logic_vector(2 downto 0) := "000";

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

    fsm : process(clock, reset)
        variable n_byte_idx : unsigned(10 downto 0);
        variable lk         : rate_lookup_t;
    begin
        if reset = '1' then
            byte_idx       <= (others => '0');
            en_rx2_r       <= '0';
            rate_hi_r      <= (others => '0');
            rate_id_gray_r <= "000";
        elsif rising_edge(clock) then
            if rx_valid = '1' then
                n_byte_idx := byte_idx;

                if rx_sop = '1' then
                    n_byte_idx := (others => '0');
                    en_rx2_r   <= '0';
                    rate_hi_r  <= (others => '0');
                end if;

                case to_integer(n_byte_idx) is
                    when BYTE_ENABLE_RX0_7 =>
                        en_rx2_r <= rx_data(2);

                    when BYTE_DDC2_RATE_HI =>
                        rate_hi_r <= rx_data;

                    when BYTE_DDC2_RATE_LO =>
                        if en_rx2_r = '1' then
                            lk := lookup_rate(rate_hi_r, rx_data);
                            if lk.valid = '1' then
                                rate_id_gray_r <= lk.gray;
                            else
                                -- Fall back to 48 kHz on unrecognised rate.
                                rate_id_gray_r <= "000";
                            end if;
                        end if;

                    when others =>
                        null;
                end case;

                if rx_eop = '1' then
                    n_byte_idx := (others => '0');
                else
                    n_byte_idx := n_byte_idx + 1;
                end if;

                byte_idx <= n_byte_idx;
            end if;
        end if;
    end process;

end architecture;
