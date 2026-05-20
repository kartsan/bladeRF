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
-- local_mac_gen
--
-- Derives a stable, locally-administered 48-bit Ethernet MAC address by
-- folding an arbitrary byte-stream identifier into 40 bits and prepending
-- the IEEE-802 locally-administered unicast OUI prefix 0x02.
--
-- For bladeRF the intended source is the 32-byte ASCII serial number the
-- FX3 already extracts from its OTP / calibration area (see
-- fx3_firmware/src/bladeRF.c::extractSerialAndCal) and exposes as the USB
-- serial-number string descriptor.  That serial is already guaranteed unique
-- per board and visible to host tools, which makes the MAC ↔ board mapping
-- trivially auditable from outside the FPGA.  The path to get those bytes
-- into the FPGA fabric is "FX3 -> NIOS -> Avalon write conduit -> byte
-- stream into this entity", but the entity itself is source-agnostic:
-- anything that hands it (data, valid, sop, eop) in order works.
--
-- The byte-stream interface is the same shape used by eem_tx_framer and
-- eem_rx_consumer:
--   * serial_sop : asserted with the first byte of an identifier; restarts
--                  the internal fold from zero.
--   * serial_eop : asserted with the last byte; latches the resulting MAC
--                  into local_mac and pulses mac_loaded for one cycle.
--   * serial_valid : '1' on cycles where a byte is being presented; the
--                    entity accepts unconditionally (no backpressure).
--   * length is implicit -- the producer marks both ends, so this works for
--     a 32-byte ASCII serial, a 16-byte hex-decoded serial, or any other
--     length without changing the entity.
--
-- A new serial may be pushed at any time after the first; the MAC simply
-- re-derives.  Until the first successful load, local_mac is the DEFAULT_MAC
-- generic and mac_valid is '0'; downstream Ethernet logic can either gate on
-- mac_valid (preferred, so it doesn't ARP-announce a fallback address) or
-- just trust the fallback to be a well-formed locally-administered unicast.
--
-- Fold function
-- -------------
-- Per-byte step: acc <= rol(acc, 5) xor zero_extend(byte_in, 40).  Chained
-- across all input bytes; final acc is the 40-bit MAC tail.  This is a
-- "Pearson-style" fold -- not cryptographic, but spreads byte entropy
-- across all 40 output bits in one pass and synthesises into a few LEs of
-- combinational shift + XOR plus a 40-bit register.
--
-- A formal CRC-32-IEEE would also work and was considered; it costs ~80 LEs
-- more for no measurable uniformity gain on the bladeRF serial population
-- (32 hex characters, effectively uniform over ~2^128).
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity local_mac_gen is
    generic (
        -- Fallback MAC asserted on reset and until the first identifier has
        -- been loaded via the byte-stream port.  Must be a locally-
        -- administered unicast address (first-octet bit 1 = 1, bit 0 = 0).
        DEFAULT_MAC : std_logic_vector(47 downto 0) := x"02_FF_FF_FF_FF_FF"
    );
    port (
        clock        : in  std_logic;
        reset        : in  std_logic;

        -- Identifier byte-stream input.
        serial_data  : in  std_logic_vector(7 downto 0);
        serial_valid : in  std_logic;
        serial_sop   : in  std_logic;
        serial_eop   : in  std_logic;

        -- 48-bit MAC output and observability.
        local_mac    : out std_logic_vector(47 downto 0);
        mac_valid    : out std_logic;
        mac_loaded   : out std_logic
    );
end entity;

architecture arch of local_mac_gen is

    -- Rotate-left-by-5 of a 40-bit value, XOR-ed with an 8-bit byte
    -- zero-extended to 40 bits.  rol(5) is chosen to (a) be coprime with 8
    -- so each new byte lands on a different bit boundary than the previous,
    -- and (b) leave bit 0 driven by the LSB of every byte so short
    -- identifiers (or runs of identical bytes) still affect the bottom of
    -- the MAC.
    function fold_step (acc      : std_logic_vector(39 downto 0);
                        byte_in  : std_logic_vector(7 downto 0))
        return std_logic_vector is
        variable rolled : std_logic_vector(39 downto 0);
        variable ext    : std_logic_vector(39 downto 0);
    begin
        rolled := acc(34 downto 0) & acc(39 downto 35);
        ext    := (others => '0');
        ext(7 downto 0) := byte_in;
        return rolled xor ext;
    end function;

    signal mac_reg   : std_logic_vector(47 downto 0) := DEFAULT_MAC;
    signal fold_acc  : std_logic_vector(39 downto 0) := (others => '0');
    signal valid_r   : std_logic := '0';
    signal loaded_r  : std_logic := '0';

begin

    local_mac  <= mac_reg;
    mac_valid  <= valid_r;
    mac_loaded <= loaded_r;

    fold_proc : process (clock, reset)
        variable next_fold : std_logic_vector(39 downto 0);
    begin
        if reset = '1' then
            mac_reg  <= DEFAULT_MAC;
            fold_acc <= (others => '0');
            valid_r  <= '0';
            loaded_r <= '0';
        elsif rising_edge(clock) then
            -- Default: loaded is a one-cycle pulse on eop.
            loaded_r <= '0';

            if serial_valid = '1' then
                -- On sop, start a fresh fold from zero; otherwise continue.
                if serial_sop = '1' then
                    next_fold := fold_step((others => '0'), serial_data);
                else
                    next_fold := fold_step(fold_acc, serial_data);
                end if;
                fold_acc <= next_fold;

                -- On eop, latch the MAC and announce.  sop+eop in the same
                -- cycle (a 1-byte identifier) is handled correctly because
                -- next_fold is computed from the sop'd-empty accumulator.
                if serial_eop = '1' then
                    mac_reg  <= x"02" & next_fold;
                    valid_r  <= '1';
                    loaded_r <= '1';
                end if;
            end if;
        end if;
    end process;

end architecture;
