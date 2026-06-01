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
-- hpsdr_hp_status_sender
--
-- OpenHPSDR Protocol 2 "High Priority Status" heartbeat (radio -> host, UDP
-- src 1025), per Orion2 CC_encoder.v / sdr_send.v.  60-byte payload; idle
-- radio sends all zero except the sequence number (V4.4 spec p.46), which is
-- the spec-faithful Orion2 idle output.
--
-- Addressing: Eth/IP dst = discovered host (host_mac/ip); UDP dst = host_port
-- = HP Command's source ephemeral (NOT 1025 - Thetis binds its rx socket
-- there).  Gated on (host_valid AND host_run): the radio stays silent between
-- discovery and the host's first HP Command with run=1.  Drives tx_arbiter
-- port E.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.udp_tx_pkg.all;

entity hpsdr_hp_status_sender is
    generic (
        -- Inter-frame period in clock cycles.  5e6 at 100 MHz = 50 ms (20 Hz).
        TICK_CYCLES : natural := 5_000_000
    );
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        our_mac       : in  std_logic_vector(47 downto 0);
        our_ip        : in  std_logic_vector(31 downto 0);

        -- Discovered host.  host_mac/ip from the discovery responder;
        -- host_port = HP Command's source ephemeral.  Stable while host_valid.
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

        -- One cycle per emitted packet (~20 Hz once engaged).
        send_pulse    : out std_logic
    );
end entity;

architecture arch of hpsdr_hp_status_sender is

    constant P2_PAYLOAD   : natural := 60;
    constant FRAME_BYTES  : natural := L2L3L4_BYTES + P2_PAYLOAD;          -- 102
    constant IP_TOTAL_LEN : natural := IP_HDR_BYTES + UDP_HDR_BYTES
                                     + P2_PAYLOAD;                         -- 88
    constant UDP_LEN      : natural := UDP_HDR_BYTES + P2_PAYLOAD;         -- 68

    constant IP_TLEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(IP_TOTAL_LEN, 16));
    constant UDP_LEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(UDP_LEN, 16));

    constant HPSDR_HP_PORT : std_logic_vector(15 downto 0) := x"0401"; -- 1025

    signal ip_chk : std_logic_vector(15 downto 0);

    type state_t is (S_IDLE, S_TX);
    signal state : state_t := S_IDLE;

    -- 23 bits cover the 5e6 default with headroom; widen for larger TICK.
    signal tick_counter : unsigned(22 downto 0) := (others => '0');
    signal seq_r        : unsigned(31 downto 0) := (others => '0');
    signal tx_byte_idx  : unsigned(13 downto 0) := (others => '0');
    signal send_pulse_r : std_logic := '0';

    -- Byte idx of the 102-byte frame: shared header for 0..41, HP Status
    -- payload (V4.4 spec p.46) for 42..101.  Sequence at bytes 0..3,
    -- PLL-locked (byte 4 bit 4) hardwired set; everything else zero.
    function status_byte_at(
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
                                       HPSDR_HP_PORT, host_port,
                                       IP_TLEN_VEC, UDP_LEN_VEC, ip_chk);
        end if;
        p := idx - L2L3L4_BYTES;
        case p is
            when 0 => return std_logic_vector(seq(31 downto 24));
            when 1 => return std_logic_vector(seq(23 downto 16));
            when 2 => return std_logic_vector(seq(15 downto  8));
            when 3 => return std_logic_vector(seq( 7 downto  0));
            -- byte 4 bit 4 = 10 MHz reference PLL locked (V4.4 spec
            -- p.46/47).  Some clients (Thetis) wait for this bit to
            -- settle before fully engaging; the bladeRF reference is
            -- always stable, so report locked from the first packet.
            when 4 => return x"10";
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
            tx_data <= status_byte_at(to_integer(tx_byte_idx),
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

    -- S_IDLE: free-run the inter-frame timer while engaged (counts in both
    -- states so packet airtime is absorbed into the period); fire on expiry.
    -- S_TX: walk the frame on tx_ready; increment seq after the last byte so
    -- the first packet of a session is seq 0.  Disengaging resets seq.
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
