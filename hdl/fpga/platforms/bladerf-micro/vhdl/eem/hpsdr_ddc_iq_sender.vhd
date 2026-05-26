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
-- hpsdr_ddc_iq_sender
--
-- OpenHPSDR Protocol 2 "DDC IQ" producer (radio -> host, UDP src port 1035).
-- Modelled on Orion / Orion MkII reference firmware's sdr_send.v RX_SEND /
-- RX_SEND_2 path (Y:\Ilkka\ham\bladerf\orion2\Ethernet\sdr_send.v lines
-- 331-391).  Real Orion2 round-robins per-DDC FIFOs and emits one 1444-byte
-- UDP payload per ready FIFO; this iteration is a single-DDC zero-IQ
-- placeholder to satisfy Thetis's post-engagement "I expect IQ data to flow"
-- timer.
--
-- Disconnect hypothesis
-- ---------------------
-- Empirical: bladeRF AND hpsdr_sim -P2 both disconnect from Thetis within
-- ~2 seconds of HP Command run=1.  Both fail to send anything on UDP/1035.
-- Per Orion2 sdr_send.v:201-221, the radio is supposed to continuously emit
-- DDC IQ packets while run=1 (interleaved with CC_SEND / MIC_SEND / WIDEBAND
-- via priority arbitration).  Thetis is presumed to start a "no-IQ" timer
-- when it commands run=1 and disconnect if no packets appear on the
-- configured Rx0_to_PC_port within that window.  Sending even an all-zero
-- IQ stream should satisfy the timer.
--
-- Payload (1444 bytes, from Orion2 sdr_send.v:331-391)
-- ----------------------------------------------------
--   bytes  0..3  : Sequence number (BE)
--   bytes  4..11 : Timestamp (64-bit BE; zero unless GPS PPS plumbed)
--   bytes 12..13 : Bits per sample (16-bit BE; = 24)
--   bytes 14..15 : Samples per frame (16-bit BE; = 238)
--   bytes 16..1443: IQ data (238 samples * 6 bytes [24-bit I + 24-bit Q])
--
-- All IQ samples are zero in this placeholder.  When a real DDC is wired
-- up, bytes 16..1443 get filled from the AD9361 RX stream via a 6-byte-wide
-- FIFO.
--
-- Addressing
-- ----------
-- {Eth dst, IP dst} = the discovered host snapshot from the discovery
-- responder (host_mac, host_ip).
-- UDP dst port    = host_port (currently wired from hpsdr_hp_cmd_handler's
--                   HP Command source ephemeral, same convention as
--                   hpsdr_hp_status_sender).  When a DDC Specific receiver
--                   lands and decodes the host-configured Rx0_to_PC_port,
--                   we'll route that here instead.
-- UDP src port    = 1035 (= P2 default Rx0 source port per V4.4 spec).
--
-- Rate
-- ----
-- 48 kHz sample rate * 238 samples/packet = 4.958 ms inter-packet = 201.7 Hz.
-- TICK_CYCLES default = 500_000 at 100 MHz fx3_pclk_pll = 5.0 ms = 200 Hz,
-- close enough to satisfy Thetis without overrunning USB.  At 1486 bytes
-- on the wire * 200 Hz = ~2.4 Mbps, comfortable for USB 2.0 CDC EEM.
--
-- Gating
-- ------
-- Same (host_valid AND host_run) gate as hpsdr_hp_status_sender.  Sequence
-- number resets to 0 each time host_run drops to 0 (Orion2 sdr_send.v:223-244
-- behaviour: !run clears all Rx_sequence_number[].)
--
-- FSM
-- ---
--   S_IDLE: tx_valid='0'.  Count tick_counter while (host_valid AND host_run).
--           On TICK_CYCLES reached, reset counter, go to S_TX (sequence
--           incremented in S_TX exit, matching Orion2's
--           "Rx_sequence_number <= Rx_sequence_number + 1" at end-of-frame).
--   S_TX:   tx_valid='1'.  Walk tx_byte_idx 0..FRAME_BYTES-1; advance on
--           tx_ready='1'.  On final byte, pulse send_pulse, return to S_IDLE.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.bladerf_p.all;

entity hpsdr_ddc_iq_sender is
    generic (
        -- Inter-frame period in clock cycles.  Default 500_000 at 100 MHz
        -- fx3_pclk_pll = 5 ms = 200 Hz, matching default 48 kHz / 238
        -- samples per Orion2 RX_SEND.  Override upwards (e.g. to slow the
        -- test rate during bring-up) or downwards (192 kHz operation would
        -- want ~250 us) as needed.
        TICK_CYCLES         : natural := 500_000
    );
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        -- Local addresses
        our_mac       : in  std_logic_vector(47 downto 0);
        our_ip        : in  std_logic_vector(31 downto 0);

        -- Discovered host's address triple, same source as
        -- hpsdr_hp_status_sender (host_mac/ip from discovery responder,
        -- host_port from HP Command source ephemeral).
        host_mac      : in  std_logic_vector(47 downto 0);
        host_ip       : in  std_logic_vector(31 downto 0);
        host_port     : in  std_logic_vector(15 downto 0);
        host_valid    : in  std_logic;

        -- Engagement gate.  '0' silences the sender; '1' starts the
        -- TICK_CYCLES-paced output and resets seq.
        host_run      : in  std_logic;

        -- TX byte stream output (to tx_arbiter port F -> eem_tx_framer).
        tx_data       : out std_logic_vector(7 downto 0);
        tx_valid      : out std_logic;
        tx_sop        : out std_logic;
        tx_eop        : out std_logic;
        tx_length     : out unsigned(13 downto 0);
        tx_ready      : in  std_logic;

        -- Observability: one cycle when the final byte of each DDC IQ
        -- packet is emitted (~200 Hz steady-state once engaged).
        send_pulse    : out std_logic
    );
end entity;

architecture arch of hpsdr_ddc_iq_sender is

    -- ------------------------------------------------------------------
    -- Geometry
    -- ------------------------------------------------------------------
    constant ETH_HDR_BYTES : natural := 14;
    constant IP_HDR_BYTES  : natural := 20;
    constant UDP_HDR_BYTES : natural := 8;
    constant P2_PAYLOAD    : natural := 1444;
    constant FRAME_BYTES   : natural := ETH_HDR_BYTES + IP_HDR_BYTES
                                      + UDP_HDR_BYTES + P2_PAYLOAD;        -- 1486
    constant IP_TOTAL_LEN  : natural := IP_HDR_BYTES + UDP_HDR_BYTES + P2_PAYLOAD; -- 1472
    constant UDP_LEN       : natural := UDP_HDR_BYTES + P2_PAYLOAD;        -- 1452

    constant IP_TLEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(IP_TOTAL_LEN, 16));
    constant UDP_LEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(UDP_LEN, 16));

    -- UDP src = 1035 (= P2 Rx0_port default per V4.4 / Orion2 General_CC.v).
    constant HPSDR_DDC_PORT_HI : std_logic_vector(7 downto 0) := x"04";
    constant HPSDR_DDC_PORT_LO : std_logic_vector(7 downto 0) := x"0b";

    -- Per-packet fixed payload bytes
    constant BITS_PER_SAMPLE : std_logic_vector(15 downto 0) := x"0018"; -- 24
    constant SAMPLES_PER_FRAME : std_logic_vector(15 downto 0) := x"00ee"; -- 238

    -- ------------------------------------------------------------------
    -- One's-complement 16-bit addition (matches the other responders).
    -- ------------------------------------------------------------------
    function oc_add(a, b : unsigned(15 downto 0)) return unsigned is
        variable sum : unsigned(16 downto 0);
    begin
        sum := ('0' & a) + ('0' & b);
        if sum(16) = '1' then
            return sum(15 downto 0) + 1;
        else
            return sum(15 downto 0);
        end if;
    end function;

    function calc_ip_const_part return unsigned is
        variable s : unsigned(15 downto 0);
    begin
        s := oc_add(to_unsigned(16#4500#, 16), to_unsigned(IP_TOTAL_LEN, 16));
        s := oc_add(s, to_unsigned(16#4000#, 16));
        s := oc_add(s, to_unsigned(16#4011#, 16));
        return s;
    end function;

    constant IP_CONST_PART : unsigned(15 downto 0) := calc_ip_const_part;

    signal ip_fixed_part_r : unsigned(15 downto 0)         := (others => '0');
    signal ip_chk_r        : std_logic_vector(15 downto 0) := (others => '0');

    -- ------------------------------------------------------------------
    -- State
    -- ------------------------------------------------------------------
    type state_t is (S_IDLE, S_TX);
    signal state : state_t := S_IDLE;

    -- 20-bit counter covers the default 500_000 ticks with headroom.
    signal tick_counter    : unsigned(19 downto 0) := (others => '0');

    -- Sequence number (incremented per packet, reset when !host_run per
    -- Orion2 sdr_send.v:224).
    signal seq_r           : unsigned(31 downto 0) := (others => '0');

    signal tx_byte_idx     : unsigned(13 downto 0) := (others => '0');

    signal send_pulse_r    : std_logic := '0';

    -- ------------------------------------------------------------------
    -- Combinational byte lookup for the 1486-byte DDC IQ frame.  IQ data
    -- bytes (idx 58..1485 = payload bytes 16..1443) are all zero in this
    -- placeholder implementation; only the headers + per-packet
    -- descriptors carry non-zero data.
    -- ------------------------------------------------------------------
    function ddc_byte_at(
        idx          : natural;
        host_mac     : std_logic_vector(47 downto 0);
        our_mac      : std_logic_vector(47 downto 0);
        our_ip       : std_logic_vector(31 downto 0);
        host_ip      : std_logic_vector(31 downto 0);
        host_port    : std_logic_vector(15 downto 0);
        ip_chk       : std_logic_vector(15 downto 0);
        seq          : unsigned(31 downto 0)
    ) return std_logic_vector is
    begin
        case idx is
            -- ---- Ethernet header ----
            when  0 => return host_mac(47 downto 40);
            when  1 => return host_mac(39 downto 32);
            when  2 => return host_mac(31 downto 24);
            when  3 => return host_mac(23 downto 16);
            when  4 => return host_mac(15 downto  8);
            when  5 => return host_mac( 7 downto  0);
            when  6 => return our_mac(47 downto 40);
            when  7 => return our_mac(39 downto 32);
            when  8 => return our_mac(31 downto 24);
            when  9 => return our_mac(23 downto 16);
            when 10 => return our_mac(15 downto  8);
            when 11 => return our_mac( 7 downto  0);
            when 12 => return x"08";
            when 13 => return x"00";

            -- ---- IPv4 header ----
            when 14 => return x"45";
            when 15 => return x"00";
            when 16 => return IP_TLEN_VEC(15 downto 8);
            when 17 => return IP_TLEN_VEC( 7 downto 0);
            when 18 | 19 => return x"00";
            when 20 => return x"40";
            when 21 => return x"00";
            when 22 => return x"40";
            when 23 => return x"11";
            when 24 => return ip_chk(15 downto 8);
            when 25 => return ip_chk( 7 downto 0);
            when 26 => return our_ip(31 downto 24);
            when 27 => return our_ip(23 downto 16);
            when 28 => return our_ip(15 downto  8);
            when 29 => return our_ip( 7 downto  0);
            when 30 => return host_ip(31 downto 24);
            when 31 => return host_ip(23 downto 16);
            when 32 => return host_ip(15 downto  8);
            when 33 => return host_ip( 7 downto  0);

            -- ---- UDP header ----
            when 34 => return HPSDR_DDC_PORT_HI;  -- UDP src = 1035
            when 35 => return HPSDR_DDC_PORT_LO;
            when 36 => return host_port(15 downto 8);  -- UDP dst = host ephemeral
            when 37 => return host_port( 7 downto 0);
            when 38 => return UDP_LEN_VEC(15 downto 8);
            when 39 => return UDP_LEN_VEC( 7 downto 0);
            when 40 | 41 => return x"00";          -- UDP checksum = 0

            -- ---- HPSDR P2 DDC IQ payload (1444 bytes) ----
            -- payload bytes 0..3 (frame idx 42..45) = sequence (BE)
            when 42 => return std_logic_vector(seq(31 downto 24));
            when 43 => return std_logic_vector(seq(23 downto 16));
            when 44 => return std_logic_vector(seq(15 downto  8));
            when 45 => return std_logic_vector(seq( 7 downto  0));
            -- payload bytes 4..11 (frame idx 46..53) = timestamp = 0
            when 46 to 53 => return x"00";
            -- payload bytes 12..13 (frame idx 54..55) = bits per sample = 24
            when 54 => return BITS_PER_SAMPLE(15 downto 8);
            when 55 => return BITS_PER_SAMPLE( 7 downto 0);
            -- payload bytes 14..15 (frame idx 56..57) = samples/frame = 238
            when 56 => return SAMPLES_PER_FRAME(15 downto 8);
            when 57 => return SAMPLES_PER_FRAME( 7 downto 0);
            -- payload bytes 16..1443 (frame idx 58..1485) = IQ samples,
            -- all zero in this placeholder.
            when others => return x"00";
        end case;
    end function;

begin

    -- ----------------------------------------------------------------------
    -- Output drivers
    -- ----------------------------------------------------------------------
    tx_data_mux : process(state, tx_byte_idx, host_mac, our_mac,
                          our_ip, host_ip, host_port, ip_chk_r, seq_r)
    begin
        if state = S_TX then
            tx_data <= ddc_byte_at(to_integer(tx_byte_idx),
                                   host_mac, our_mac,
                                   our_ip, host_ip, host_port,
                                   ip_chk_r, seq_r);
        else
            tx_data <= (others => '0');
        end if;
    end process tx_data_mux;

    tx_valid   <= '1' when state = S_TX else '0';
    tx_sop     <= '1' when (state = S_TX and tx_byte_idx = 0) else '0';
    tx_eop     <= '1' when (state = S_TX and
                            tx_byte_idx = to_unsigned(FRAME_BYTES - 1,
                                                      tx_byte_idx'length))
                      else '0';
    tx_length  <= to_unsigned(FRAME_BYTES, tx_length'length);
    send_pulse <= send_pulse_r;

    -- ----------------------------------------------------------------------
    -- Registered IP checksum -- same two-stage cascade as the other
    -- responders.  Stage 1 folds in our_ip, stage 2 folds in host_ip and
    -- inverts.  IP_CONST_PART differs from hpsdr_hp_status_sender because
    -- IP_TOTAL_LEN is different (1472 vs 88 bytes).
    -- ----------------------------------------------------------------------
    ip_fixed_part_proc : process(clock, reset)
        variable s : unsigned(15 downto 0);
    begin
        if reset = '1' then
            ip_fixed_part_r <= (others => '0');
        elsif rising_edge(clock) then
            s := IP_CONST_PART;
            s := oc_add(s, unsigned(our_ip(31 downto 16)));
            s := oc_add(s, unsigned(our_ip(15 downto  0)));
            ip_fixed_part_r <= s;
        end if;
    end process ip_fixed_part_proc;

    ip_chk_proc : process(clock, reset)
        variable s : unsigned(15 downto 0);
    begin
        if reset = '1' then
            ip_chk_r <= (others => '0');
        elsif rising_edge(clock) then
            s := ip_fixed_part_r;
            s := oc_add(s, unsigned(host_ip(31 downto 16)));
            s := oc_add(s, unsigned(host_ip(15 downto  0)));
            ip_chk_r <= std_logic_vector(not s);
        end if;
    end process ip_chk_proc;

    -- ----------------------------------------------------------------------
    -- Main FSM with rate timer
    -- ----------------------------------------------------------------------
    fsm : process(clock, reset)
    begin
        if reset = '1' then
            state         <= S_IDLE;
            tick_counter  <= (others => '0');
            seq_r         <= (others => '0');
            tx_byte_idx   <= (others => '0');
            send_pulse_r  <= '0';
        elsif rising_edge(clock) then
            send_pulse_r <= '0';

            case state is

            when S_IDLE =>
                -- Same gate as hpsdr_hp_status_sender: only emit when the
                -- host is both discovered AND engaged (HP Command run=1).
                -- Dropping host_run clears the tick counter AND the
                -- sequence number, matching Orion2 sdr_send.v:223-244
                -- "!run resets Rx_sequence_number[]" behaviour.
                if host_valid = '1' and host_run = '1' then
                    if tick_counter = to_unsigned(TICK_CYCLES - 1,
                                                  tick_counter'length) then
                        tick_counter <= (others => '0');
                        tx_byte_idx  <= (others => '0');
                        state        <= S_TX;
                    else
                        tick_counter <= tick_counter + 1;
                    end if;
                else
                    tick_counter <= (others => '0');
                    seq_r        <= (others => '0');
                end if;

            when S_TX =>
                if tx_ready = '1' then
                    if tx_byte_idx = to_unsigned(FRAME_BYTES - 1,
                                                 tx_byte_idx'length) then
                        send_pulse_r <= '1';
                        seq_r        <= seq_r + 1;
                        state        <= S_IDLE;
                        tx_byte_idx  <= (others => '0');
                    else
                        tx_byte_idx <= tx_byte_idx + 1;
                    end if;
                end if;

            end case;
        end if;
    end process fsm;

end architecture;
