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
-- eem_test_frame_src
--
-- Bringup-only producer for eem_tx_framer.  Emits the same 50-byte 0x88B5
-- Ethernet frame that the (now-removed) ROM-style injector proved end-to-end
-- on 2026-05-19, but now driven through the framer's byte-stream input so
-- the framer's header / FCS / dummy logic is exercised on every transmission.
--
-- Trigger: a free-running tick counter fires once per PERIOD_TICKS clocks
-- (default 100_000_000 = 1 Hz at fx3_pclk_pll = 100 MHz).  On fire the source
-- enters its streaming phase, walks the 50-byte FRAME constant out one byte
-- per cycle, honors frame_out_ready backpressure (the framer holds ready=0
-- for the two header-injection cycles immediately after sop), and returns
-- to idle once eop has been accepted.
--
-- Replace with the real Ethernet/UDP front-end once those layers exist; this
-- file is the smallest possible exerciser for eem_tx_framer.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity eem_test_frame_src is
    generic (
        -- Clock ticks between successive frame emissions.  At fx3_pclk_pll
        -- = 100 MHz, 100_000_000 = 1 Hz.
        PERIOD_TICKS : natural := 100_000_000
    );
    port (
        clock            : in  std_logic;
        reset            : in  std_logic;

        -- Byte-stream output (mates with eem_tx_framer's frame_in_* port)
        frame_out_data   : out std_logic_vector(7 downto 0);
        frame_out_valid  : out std_logic;
        frame_out_sop    : out std_logic;
        frame_out_eop    : out std_logic;
        frame_out_length : out unsigned(13 downto 0);
        frame_out_ready  : in  std_logic
    );
end entity;

architecture arch of eem_test_frame_src is

    constant FRAME_LEN : natural := 50;

    type byte_array_t is array (0 to FRAME_LEN-1) of std_logic_vector(7 downto 0);

    -- Same frame the bringup ROM injector used.  Layout:
    --   [0..5]   Dst MAC 02:00:00:0B:1A:DE  (locally administered, addr 0)
    --   [6..11]  Src MAC 02:00:00:0B:1A:DF  (locally administered, addr 1)
    --   [12..13] EtherType 0x88B5            (Local Experimental Ethertype 1)
    --   [14..49] 36 bytes of 0x55 payload
    -- Total 50 bytes (< Ethernet min 60, but Linux usbnet doesn't enforce
    -- min on receive; the kernel happily delivers it to AF_PACKET listeners).
    constant FRAME : byte_array_t := (
        0  => x"02", 1  => x"00", 2  => x"00", 3  => x"0B", 4  => x"1A", 5  => x"DE",
        6  => x"02", 7  => x"00", 8  => x"00", 9  => x"0B", 10 => x"1A", 11 => x"DF",
        12 => x"88", 13 => x"B5",
        others => x"55"
    );

    -- ceil(log2(PERIOD_TICKS)) for a tight counter.  PERIOD_TICKS up to
    -- 2**32 - 1 fits comfortably in 32 bits; we use 32 bits for simplicity.
    signal tick_cnt   : unsigned(31 downto 0) := (others => '0');
    signal armed      : std_logic := '0';

    -- Streaming state
    signal stream_on  : std_logic := '0';
    signal byte_idx   : unsigned(13 downto 0) := (others => '0');

begin

    frame_out_length <= to_unsigned(FRAME_LEN, 14);

    -- ----------------------------------------------------------------------
    -- One-shot trigger: free-running tick counter raises `armed` for one
    -- cycle every PERIOD_TICKS clocks.  The streaming FSM consumes the
    -- pulse on the next clock by transitioning out of the idle wait.
    -- ----------------------------------------------------------------------
    tick_proc : process (clock, reset)
    begin
        if reset = '1' then
            tick_cnt <= (others => '0');
            armed    <= '0';
        elsif rising_edge(clock) then
            armed <= '0';
            if tick_cnt = to_unsigned(PERIOD_TICKS - 1, tick_cnt'length) then
                tick_cnt <= (others => '0');
                armed    <= '1';
            else
                tick_cnt <= tick_cnt + 1;
            end if;
        end if;
    end process;

    -- ----------------------------------------------------------------------
    -- Streaming FSM: when armed pulses and we're idle, begin streaming.
    -- Present FRAME(byte_idx) every cycle with valid=1; advance byte_idx
    -- only when ready=1 (honors the framer's 2-cycle header-injection
    -- backpressure at sop).  sop is asserted only on the first byte;
    -- eop on the last.
    -- ----------------------------------------------------------------------
    stream_proc : process (clock, reset)
    begin
        if reset = '1' then
            stream_on <= '0';
            byte_idx  <= (others => '0');
        elsif rising_edge(clock) then
            if stream_on = '0' then
                if armed = '1' then
                    stream_on <= '1';
                    byte_idx  <= (others => '0');
                end if;
            else
                if frame_out_ready = '1' then
                    if byte_idx = to_unsigned(FRAME_LEN - 1, byte_idx'length) then
                        stream_on <= '0';
                        byte_idx  <= (others => '0');
                    else
                        byte_idx <= byte_idx + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- Combinatorial byte / valid / sop / eop output.
    frame_out_data  <= FRAME(to_integer(byte_idx));
    frame_out_valid <= stream_on;
    frame_out_sop   <= '1' when (stream_on = '1' and byte_idx = 0) else '0';
    frame_out_eop   <= '1' when (stream_on = '1' and byte_idx = to_unsigned(FRAME_LEN - 1, byte_idx'length)) else '0';

end architecture;
