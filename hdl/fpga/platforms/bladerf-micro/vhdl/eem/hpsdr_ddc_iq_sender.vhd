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
-- OpenHPSDR Protocol 2 "DDC IQ" producer (radio -> host, UDP src 1035), per
-- Orion2 sdr_send.v RX_SEND.  Single-DDC.
--
-- Two payload-source modes selected by the IQ_FROM_FIFO generic:
--
--   false (default): zero-IQ placeholder, paced by a free-running timer
--                    (TICK_CYCLES).  Keeps Thetis engaged before the RX DSP
--                    path exists.  This is the build the `hosted` revision uses.
--
--   true:            real I/Q drained from a show-ahead async FIFO fed by
--                    hpsdr_ddc.  A packet is emitted whenever the FIFO holds
--                    >= 238 samples, so the packet cadence self-scales with the
--                    DDC output rate (~200 Hz at 48 kHz).  This is the `hpsdr`
--                    revision build.
--
-- Payload (1444 bytes, V4.4 spec p.53): seq[0:3], timestamp[4:11] (zero),
-- bits-per-sample[12:13]=24, samples-per-frame[14:15]=238, then 238 * 6 IQ
-- bytes (I[23:16],I[15:8],I[7:0],Q[23:16],Q[15:8],Q[7:0]).  Addressing/gating
-- identical to hpsdr_hp_status_sender.  Drives tx_arbiter port F.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.udp_tx_pkg.all;

entity hpsdr_ddc_iq_sender is
    generic (
        -- Inter-frame period in clock cycles for the timer-paced (zero-IQ)
        -- mode.  5e5 at 100 MHz = 5 ms (200 Hz), matching 48 kHz / 238 samples.
        TICK_CYCLES  : natural := 500_000;
        -- false: timer-paced zero IQ.  true: drain real IQ from the FIFO.
        IQ_FROM_FIFO : boolean := false
    );
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        our_mac       : in  std_logic_vector(47 downto 0);
        our_ip        : in  std_logic_vector(31 downto 0);

        -- Discovered host (same source as hpsdr_hp_status_sender).
        host_mac      : in  std_logic_vector(47 downto 0);
        host_ip       : in  std_logic_vector(31 downto 0);
        host_port     : in  std_logic_vector(15 downto 0);
        host_valid    : in  std_logic;

        -- Engagement gate; dropping it resets the sequence number.
        host_run      : in  std_logic;

        tx_data       : out std_logic_vector(7 downto 0);
        tx_valid      : out std_logic;
        tx_sop        : out std_logic;
        tx_eop        : out std_logic;
        tx_length     : out unsigned(13 downto 0);
        tx_ready      : in  std_logic;

        -- One cycle per emitted packet.
        send_pulse    : out std_logic;

        -- DDC IQ source FIFO (read side, this clock domain).  Show-ahead: iq_q
        -- presents the head sample; iq_rdreq pops it.  Used only when
        -- IQ_FROM_FIFO=true; defaults leave it idle for the zero-IQ build.
        -- iq_level is the FIFO rdusedw (9 bits => 512-deep FIFO).
        iq_rdreq      : out std_logic;
        iq_q          : in  std_logic_vector(47 downto 0) := (others => '0');
        iq_rdempty    : in  std_logic                     := '1';
        iq_level      : in  std_logic_vector(8 downto 0)  := (others => '0')
    );
end entity;

architecture arch of hpsdr_ddc_iq_sender is

    constant P2_PAYLOAD   : natural := 1444;
    constant FRAME_BYTES  : natural := L2L3L4_BYTES + P2_PAYLOAD;          -- 1486
    constant IP_TOTAL_LEN : natural := IP_HDR_BYTES + UDP_HDR_BYTES
                                     + P2_PAYLOAD;                         -- 1472
    constant UDP_LEN      : natural := UDP_HDR_BYTES + P2_PAYLOAD;         -- 1452

    constant IP_TLEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(IP_TOTAL_LEN, 16));
    constant UDP_LEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(UDP_LEN, 16));

    constant HPSDR_DDC_PORT      : std_logic_vector(15 downto 0) := x"040b"; -- 1035 = DDC0
    constant BITS_PER_SAMPLE     : std_logic_vector(15 downto 0) := x"0018"; -- 24
    constant SAMPLES_PER_FRAME   : std_logic_vector(15 downto 0) := x"00ee"; -- 238

    constant SAMPLES_N : natural := 238;
    -- IQ payload occupies frame bytes [IQ_FIRST, IQ_LAST] = 238 samples * 6 B.
    constant IQ_FIRST  : natural := L2L3L4_BYTES + 16;                 -- 58
    constant IQ_LAST   : natural := L2L3L4_BYTES + 16 + SAMPLES_N*6 - 1; -- 1485

    signal ip_chk : std_logic_vector(15 downto 0);

    type state_t is (S_IDLE, S_TX);
    signal state : state_t := S_IDLE;

    -- 20 bits cover the 5e5 default with headroom; widen for larger TICK.
    signal tick_counter : unsigned(19 downto 0) := (others => '0');
    signal seq_r        : unsigned(31 downto 0) := (others => '0');
    signal tx_byte_idx  : unsigned(13 downto 0) := (others => '0');
    signal send_pulse_r : std_logic := '0';

    -- IQ-from-FIFO serialiser state.
    signal cur_sample   : std_logic_vector(47 downto 0) := (others => '0');
    signal bsel         : unsigned(2 downto 0) := (others => '0');  -- byte 0..5
    signal samp_idx     : unsigned(7 downto 0) := (others => '0');  -- 0..237
    signal iq_rdreq_r   : std_logic := '0';

    -- Byte idx of the 1486-byte frame: shared header for 0..41, then DDC IQ
    -- payload (V4.4 spec p.53).  Returns 0 for the IQ region (bytes 16..1443);
    -- in IQ_FROM_FIFO mode that region is overridden by cur_sample below.
    function ddc_byte_at(
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
                                       HPSDR_DDC_PORT, host_port,
                                       IP_TLEN_VEC, UDP_LEN_VEC, ip_chk);
        end if;
        p := idx - L2L3L4_BYTES;
        case p is
            when 0  => return std_logic_vector(seq(31 downto 24));
            when 1  => return std_logic_vector(seq(23 downto 16));
            when 2  => return std_logic_vector(seq(15 downto  8));
            when 3  => return std_logic_vector(seq( 7 downto  0));
            when 12 => return BITS_PER_SAMPLE(15 downto 8);
            when 13 => return BITS_PER_SAMPLE( 7 downto 0);
            when 14 => return SAMPLES_PER_FRAME(15 downto 8);
            when 15 => return SAMPLES_PER_FRAME( 7 downto 0);
            when others => return x"00";              -- timestamp + IQ = 0
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

    tx_data_mux : process(all)
    begin
        if state = S_TX then
            if IQ_FROM_FIFO and
               tx_byte_idx >= to_unsigned(IQ_FIRST, tx_byte_idx'length) and
               tx_byte_idx <= to_unsigned(IQ_LAST,  tx_byte_idx'length) then
                case to_integer(bsel) is
                    when 0      => tx_data <= cur_sample(47 downto 40); -- I[23:16]
                    when 1      => tx_data <= cur_sample(39 downto 32); -- I[15:8]
                    when 2      => tx_data <= cur_sample(31 downto 24); -- I[7:0]
                    when 3      => tx_data <= cur_sample(23 downto 16); -- Q[23:16]
                    when 4      => tx_data <= cur_sample(15 downto  8); -- Q[15:8]
                    when others => tx_data <= cur_sample( 7 downto  0); -- Q[7:0]
                end case;
            else
                tx_data <= ddc_byte_at(to_integer(tx_byte_idx),
                                       host_mac, our_mac, our_ip, host_ip,
                                       host_port, ip_chk, seq_r);
            end if;
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
    iq_rdreq   <= iq_rdreq_r;

    -- Same free-running-timer / seq-after-send / reset-on-disengage scheme as
    -- hpsdr_hp_status_sender, with an alternate FIFO-fill trigger and IQ-byte
    -- serialiser when IQ_FROM_FIFO is set.
    fsm : process(clock, reset)
    begin
        if reset = '1' then
            state        <= S_IDLE;
            tick_counter <= (others => '0');
            seq_r        <= (others => '0');
            tx_byte_idx  <= (others => '0');
            send_pulse_r <= '0';
            cur_sample   <= (others => '0');
            bsel         <= (others => '0');
            samp_idx     <= (others => '0');
            iq_rdreq_r   <= '0';
        elsif rising_edge(clock) then
            send_pulse_r <= '0';
            iq_rdreq_r   <= '0';

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
                if host_valid = '1' and host_run = '1' then
                    if IQ_FROM_FIFO then
                        -- Start once a full frame of samples is queued.
                        if unsigned(iq_level) >=
                           to_unsigned(SAMPLES_N, iq_level'length) then
                            tx_byte_idx <= (others => '0');
                            samp_idx    <= (others => '0');
                            bsel        <= (others => '0');
                            cur_sample  <= iq_q;     -- prime sample 0
                            iq_rdreq_r  <= '1';      -- pop it; q -> sample 1
                            state       <= S_TX;
                        end if;
                    else
                        if tick_counter = to_unsigned(TICK_CYCLES - 1,
                                                      tick_counter'length) then
                            tick_counter <= (others => '0');
                            tx_byte_idx  <= (others => '0');
                            state        <= S_TX;
                        end if;
                    end if;
                end if;

            when S_TX =>
                if tx_ready = '1' then
                    -- Advance the IQ serialiser one byte; pop the FIFO one
                    -- sample ahead so cur_sample always holds the byte we emit.
                    if IQ_FROM_FIFO and
                       tx_byte_idx >= to_unsigned(IQ_FIRST, tx_byte_idx'length) and
                       tx_byte_idx <= to_unsigned(IQ_LAST,  tx_byte_idx'length) then
                        if bsel = to_unsigned(5, bsel'length) then
                            -- Last byte of this sample; load the next one
                            -- unless we just finished sample 237 (frame end).
                            if samp_idx /= to_unsigned(SAMPLES_N - 1,
                                                       samp_idx'length) then
                                cur_sample <= iq_q;
                                iq_rdreq_r <= '1';
                                samp_idx   <= samp_idx + 1;
                                bsel       <= (others => '0');
                            end if;
                        else
                            bsel <= bsel + 1;
                        end if;
                    end if;

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
