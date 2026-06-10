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
-- hpsdr_duc_spec_handler
--
-- OpenHPSDR "DUC Specific" command receiver (host -> radio, UDP/1026), the TX
-- counterpart of hpsdr_ddc_spec_handler.  Consumes udp_rx_handler's
-- hpsdr_duc_spec_* byte stream and extracts the DUC0 (Tx0) sample rate.
--
-- DUC Specific payload layout (V4.4 spec p.27-29 / orion2 Tx_specific_C&C.v),
-- byte index = UDP-payload offset (sequence number included):
--
--   byte 0..3   Sequence number
--   byte 4      Number of DACs
--   byte 5      CW/keyer mode bits
--   ...
--   byte 14     DUC0 Sampling Rate [15:8]   kHz, big-endian
--   byte 15     DUC0 Sampling Rate [ 7:0]
--   byte 16     DUC0 Bits (24 for all current hardware)
--   ...
--
-- The spec notes the DUC rate "for current hardware [is] fixed at 192ksps", so
-- 192 kHz is both the boot default and the unrecognised-value fall-back here
-- (vs the DDC handler's 48 kHz default) -- matching what Thetis actually
-- sends for transmit.  Unlike the DDC handler there is no per-DDC enable
-- gate: DUC0 is the single transmit chain, so the rate latches at byte 15 of
-- every DUC Specific command.
--
-- Output: rate_id_gray, a 3-bit reflected-gray-coded selector identical in
-- encoding to hpsdr_ddc_spec_handler (so hpsdr_duc reuses the same decode
-- table -- interpolation factor 12.288e6/rate happens to equal the DDC
-- decimation for each rate).  Cross to tx_clock with a per-bit 2-FF
-- synchroniser inside hpsdr_duc.
--
--   gray "000" -> 48   kHz (interp 256)
--        "001" -> 96   kHz (interp 128)
--        "011" -> 192  kHz (interp  64)  (default / fall-back)
--        "010" -> 384  kHz (interp  32)
--        "110" -> 768  kHz (interp  16)
--        "111" -> 1536 kHz (interp   8)
--        "100"/"101"   -> invalid (consumer falls back to 192 kHz)
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_duc_spec_handler is
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        -- DUC Specific byte stream (UDP/1026 payload, header stripped) from
        -- udp_rx_handler's hpsdr_duc_spec_* outputs.
        rx_data       : in  std_logic_vector(7 downto 0);
        rx_valid      : in  std_logic;
        rx_sop        : in  std_logic;
        rx_eop        : in  std_logic;

        -- 3-bit reflected-gray-coded DUC0 sample-rate selector.  Stable
        -- between commands; updates at most once per command (at byte 15 of
        -- the payload).  Boot default = 192 kHz ("011").
        rate_id_gray  : out std_logic_vector(2 downto 0)
    );
end entity;

architecture arch of hpsdr_duc_spec_handler is

    constant BYTE_DUC0_RATE_HI : natural := 14;
    constant BYTE_DUC0_RATE_LO : natural := 15;

    -- Default / fall-back rate is 192 kHz (gray "011"), per spec.
    constant RATE_DEFAULT_GRAY : std_logic_vector(2 downto 0) := "011";

    -- 11 bits cover the full 1444-byte payload index range (0..1443).
    signal byte_idx       : unsigned(10 downto 0) := (others => '0');

    -- High byte captured at byte 14; combined with the low byte at byte 15.
    signal rate_hi_r      : std_logic_vector(7 downto 0) := (others => '0');

    signal rate_id_gray_r : std_logic_vector(2 downto 0) := RATE_DEFAULT_GRAY;

    type rate_lookup_t is record
        gray  : std_logic_vector(2 downto 0);
        valid : std_logic;
    end record;

    function lookup_rate(hi : std_logic_vector(7 downto 0);
                         lo : std_logic_vector(7 downto 0))
        return rate_lookup_t is
        variable r : rate_lookup_t := (RATE_DEFAULT_GRAY, '0');
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
            r := (RATE_DEFAULT_GRAY, '0');
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
            rate_hi_r      <= (others => '0');
            rate_id_gray_r <= RATE_DEFAULT_GRAY;
        elsif rising_edge(clock) then
            if rx_valid = '1' then
                n_byte_idx := byte_idx;

                if rx_sop = '1' then
                    n_byte_idx := (others => '0');
                    rate_hi_r  <= (others => '0');
                end if;

                case to_integer(n_byte_idx) is
                    when BYTE_DUC0_RATE_HI =>
                        rate_hi_r <= rx_data;

                    when BYTE_DUC0_RATE_LO =>
                        lk := lookup_rate(rate_hi_r, rx_data);
                        if lk.valid = '1' then
                            rate_id_gray_r <= lk.gray;
                        else
                            -- Fall back to 192 kHz on unrecognised rate.
                            rate_id_gray_r <= RATE_DEFAULT_GRAY;
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
