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
-- hpsdr_mic_sender
--
-- OpenHPSDR Protocol 2 "Mic samples" producer (radio -> host, UDP src 1026),
-- per Orion2 sdr_send.v MIC_SEND.  Payload is 4-byte seq + 128 bytes of mic
-- data; this implementation streams all-zero mic data, which is enough to
-- satisfy Thetis's "is the audio path alive?" gate so the RX1 demodulator
-- actually runs.  Without this packet stream Thetis stays silent on receive
-- even though DDC IQ on UDP/1035 is flowing -- the panadapter draws but no
-- audio reaches the soundcard.
--
-- Cadence: TICK_CYCLES default 133_333 at 100 MHz = ~750 Hz, matching the
-- 48 kHz / 64-samples-per-packet rate the Orion2 reference hardware emits.
-- Rate doesn't have to be exact -- Thetis just needs the stream to exist --
-- but staying close to Orion2 avoids edge cases in the demodulator timing.
--
-- Addressing: Eth/IP dst = discovered host (host_mac/ip); UDP dst = host_port
-- = HP Command's source ephemeral (the same destination as HP Status and
-- DDC IQ, NOT 1026).  Gated on (host_valid AND host_run): silent until the
-- host sends HP Command run=1, same engagement contract as the other
-- periodic senders.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.udp_tx_pkg.all;

entity hpsdr_mic_sender is
    generic (
        -- Inter-frame period in clock cycles.  133_333 at 100 MHz = ~750 Hz
        -- (48 kHz / 64 samples-per-packet, Orion2 default).
        TICK_CYCLES : natural := 133_333
    );
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        our_mac       : in  std_logic_vector(47 downto 0);
        our_ip        : in  std_logic_vector(31 downto 0);

        -- Discovered host (same source as hpsdr_hp_status_sender / DDC IQ).
        host_mac      : in  std_logic_vector(47 downto 0);
        host_ip       : in  std_logic_vector(31 downto 0);
        host_port     : in  std_logic_vector(15 downto 0);
        host_valid    : in  std_logic;

        -- Engagement gate (HP Command byte 4 bit 0).
        host_run      : in  std_logic;

        tx_data       : out std_logic_vector(7 downto 0);
        tx_valid      : out std_logic;
        tx_sop        : out std_logic;
        tx_eop        : out std_logic;
        tx_length     : out unsigned(13 downto 0);
        tx_ready      : in  std_logic;

        -- One cycle per emitted packet.
        send_pulse    : out std_logic
    );
end entity;

architecture arch of hpsdr_mic_sender is

    constant P2_PAYLOAD   : natural := 132;                                -- 4 seq + 128 mic
    constant FRAME_BYTES  : natural := L2L3L4_BYTES + P2_PAYLOAD;          -- 174
    constant IP_TOTAL_LEN : natural := IP_HDR_BYTES + UDP_HDR_BYTES
                                     + P2_PAYLOAD;                         -- 160
    constant UDP_LEN      : natural := UDP_HDR_BYTES + P2_PAYLOAD;         -- 140

    constant IP_TLEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(IP_TOTAL_LEN, 16));
    constant UDP_LEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(UDP_LEN, 16));

    constant HPSDR_MIC_PORT : std_logic_vector(15 downto 0) := x"0402"; -- 1026

    signal ip_chk : std_logic_vector(15 downto 0);

    type state_t is (S_IDLE, S_TX);
    signal state : state_t := S_IDLE;

    -- 18 bits cover the 133_333 default with headroom; widen for larger TICK.
    signal tick_counter : unsigned(17 downto 0) := (others => '0');
    signal seq_r        : unsigned(31 downto 0) := (others => '0');
    signal tx_byte_idx  : unsigned(13 downto 0) := (others => '0');
    signal send_pulse_r : std_logic := '0';

    -- Byte idx of the 174-byte frame: shared header for 0..41, mic-samples
    -- payload for 42..173.  Sequence at bytes 0..3 of payload; the remaining
    -- 128 bytes are zero (Orion2 emits the FIFO's mic samples here -- we
    -- ship silence, which Thetis demodulates as "no mic energy", indistinct
    -- from an Orion2 hardware mic with PTT not pressed).
    function mic_byte_at(
        idx       : natural;
        host_mac  : std_logic_vector(47 downto 0);
        our_mac   : std_logic_vector(47 downto 0);
        our_ip    : std_logic_vector(31 downto 0);
        host_ip   : std_logic_vector(31 downto 0);
        host_port : std_logic_vector(15 downto 0);
        ip_chk    : std_logic_vector(15 downto 0);
        seq       : unsigned(31 downto 0)
    ) return std_logic_vector is
        variable p : natural;
    begin
        if idx < L2L3L4_BYTES then
            return eth_ip_udp_hdr_byte(idx, host_mac, our_mac, our_ip, host_ip,
                                       HPSDR_MIC_PORT, host_port,
                                       IP_TLEN_VEC, UDP_LEN_VEC, ip_chk);
        end if;
        p := idx - L2L3L4_BYTES;
        case p is
            when 0 => return std_logic_vector(seq(31 downto 24));
            when 1 => return std_logic_vector(seq(23 downto 16));
            when 2 => return std_logic_vector(seq(15 downto  8));
            when 3 => return std_logic_vector(seq( 7 downto  0));
            when others => return x"00";
        end case;
    end function;

begin

    U_ip_chk : entity work.ip_hdr_checksum
        generic map ( IP_TOTAL_LEN => IP_TOTAL_LEN )
        port map (
            clock    => clock,
            reset    => reset,
            our_ip   => our_ip,
            peer_ip  => host_ip,
            checksum => ip_chk
        );

    tx_data_mux : process(state, tx_byte_idx, host_mac, our_mac,
                          our_ip, host_ip, host_port, ip_chk, seq_r)
    begin
        if state = S_TX then
            tx_data <= mic_byte_at(to_integer(tx_byte_idx),
                                   host_mac, our_mac, our_ip, host_ip,
                                   host_port, ip_chk, seq_r);
        else
            tx_data <= (others => '0');
        end if;
    end process;

    tx_valid   <= '1' when state = S_TX else '0';
    tx_sop     <= '1' when (state = S_TX and tx_byte_idx = 0) else '0';
    tx_eop     <= '1' when (state = S_TX and
                            tx_byte_idx = to_unsigned(FRAME_BYTES - 1,
                                                      tx_byte_idx'length))
                      else '0';
    tx_length  <= to_unsigned(FRAME_BYTES, tx_length'length);
    send_pulse <= send_pulse_r;

    -- Same free-running-timer / seq-after-send / reset-on-disengage scheme as
    -- hpsdr_hp_status_sender.
    fsm : process(clock, reset)
    begin
        if reset = '1' then
            state        <= S_IDLE;
            tick_counter <= (others => '0');
            seq_r        <= (others => '0');
            tx_byte_idx  <= (others => '0');
            send_pulse_r <= '0';
        elsif rising_edge(clock) then
            send_pulse_r <= '0';

            if host_valid = '1' and host_run = '1' then
                if tick_counter /= to_unsigned(TICK_CYCLES - 1,
                                               tick_counter'length) then
                    tick_counter <= tick_counter + 1;
                end if;
            else
                tick_counter <= (others => '0');
                seq_r        <= (others => '0');
            end if;

            case state is

            when S_IDLE =>
                if host_valid = '1' and host_run = '1' and
                   tick_counter = to_unsigned(TICK_CYCLES - 1,
                                              tick_counter'length) then
                    tick_counter <= (others => '0');
                    tx_byte_idx  <= (others => '0');
                    state        <= S_TX;
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
    end process;

end architecture;
