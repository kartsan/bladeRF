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
-- hpsdr_discovery_responder
--
-- OpenHPSDR Protocol 2 discovery responder, modelled on the Orion MkII
-- reference firmware (Y:\Ilkka\ham\bladerf\orion2).  Listens on udp_rx_handler's
-- hpsdr_* channel (UDP/1024 payload), accepts the discovery probe (payload
-- byte 4 = 0x02), and emits a 102-byte unicast Orion MkII reply.
--
-- Reply addressing: Eth/IP/UDP dst = probe's src MAC/IP/port (snapshotted at
-- rx_sop); IP src = our_ip.  60-byte payload per V4.4 spec p.42 (board type,
-- versions, DDC count, etc. via generics).  Drives tx_arbiter port D.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.udp_tx_pkg.all;

entity hpsdr_discovery_responder is
    generic (
        -- Payload byte 11, board ID (V4.4 spec p.44).  0x01 = HERMES.  We
        -- emulate Hermes (single DDC0 receiver) rather than ORION Mk II
        -- (0x05): Hermes is the simplest P2 personality and clients map the
        -- main receiver to DDC0, matching hpsdr_hp_cmd_handler (RX from DDC0)
        -- and hpsdr_ddc_iq_sender (IQ out on DDC0 src port 1035).
        BOARD_TYPE       : std_logic_vector(7 downto 0) := x"01";
        -- Byte 12: protocol version, decimal (0x2C = 44 -> "v4.4").
        PROTOCOL_VERSION : std_logic_vector(7 downto 0) := x"2C";
        -- Byte 13: firmware version, decimal (0x16 = 22 -> Orion "v2.2").
        CODE_VERSION     : std_logic_vector(7 downto 0) := x"16";
        -- Byte 20: number of DDCs implemented.
        NUMBER_DDCS      : std_logic_vector(7 downto 0) := x"01";
        -- Byte 21: 0 = frequency in Hz, 1 = phase word.  0x01 is Orion2's
        -- value; flip to 0x00 when a Hz-based RX retune consumer lands.
        FREQ_PHASE       : std_logic_vector(7 downto 0) := x"01";
        -- Byte 23: beta tag (0 = release).
        BETA_VERSION     : std_logic_vector(7 downto 0) := x"0A"
    );
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        our_mac       : in  std_logic_vector(47 downto 0);
        our_ip        : in  std_logic_vector(31 downto 0);

        -- Probing host's addresses, held stable across the inbound payload.
        peer_mac      : in  std_logic_vector(47 downto 0);
        peer_ip       : in  std_logic_vector(31 downto 0);
        peer_port     : in  std_logic_vector(15 downto 0);

        -- HPSDR byte stream in (UDP/1024 payload, headers stripped).
        rx_data       : in  std_logic_vector(7 downto 0);
        rx_valid      : in  std_logic;
        rx_sop        : in  std_logic;
        rx_eop        : in  std_logic;
        rx_length     : in  std_logic_vector(13 downto 0);

        -- TX byte stream out (to tx_arbiter -> eem_tx_framer).
        tx_data       : out std_logic_vector(7 downto 0);
        tx_valid      : out std_logic;
        tx_sop        : out std_logic;
        tx_eop        : out std_logic;
        tx_length     : out unsigned(13 downto 0);
        tx_ready      : in  std_logic;

        -- Committed client identity for downstream producers.  host_valid
        -- latches '1' on first accepted probe and stays high; host_mac/ip
        -- refresh on each subsequent probe.
        host_mac      : out std_logic_vector(47 downto 0);
        host_ip       : out std_logic_vector(31 downto 0);
        host_port     : out std_logic_vector(15 downto 0);
        host_valid    : out std_logic;

        -- One cycle when the final reply byte is emitted.
        reply_pulse   : out std_logic
    );
end entity;

architecture arch of hpsdr_discovery_responder is

    constant P2_PAYLOAD   : natural := 60;
    constant FRAME_BYTES  : natural := L2L3L4_BYTES + P2_PAYLOAD;          -- 102
    constant IP_TOTAL_LEN : natural := IP_HDR_BYTES + UDP_HDR_BYTES
                                     + P2_PAYLOAD;                         -- 88
    constant UDP_LEN      : natural := UDP_HDR_BYTES + P2_PAYLOAD;         -- 68

    constant IP_TLEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(IP_TOTAL_LEN, 16));
    constant UDP_LEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(UDP_LEN, 16));

    constant P2_CMD_DISCOVERY : std_logic_vector(7 downto 0)  := x"02";
    constant HPSDR_DISC_PORT  : std_logic_vector(15 downto 0) := x"0400"; -- 1024

    signal ip_chk : std_logic_vector(15 downto 0);

    type state_t is (S_RX, S_TX);
    signal state : state_t := S_RX;

    signal rx_byte_idx     : unsigned(13 downto 0) := (others => '0');
    signal is_discovery_r  : std_logic             := '0';

    signal peer_mac_r      : std_logic_vector(47 downto 0) := (others => '0');
    signal peer_ip_r       : std_logic_vector(31 downto 0) := (others => '0');
    signal peer_port_r     : std_logic_vector(15 downto 0) := (others => '0');

    signal host_mac_r      : std_logic_vector(47 downto 0) := (others => '0');
    signal host_ip_r       : std_logic_vector(31 downto 0) := (others => '0');
    signal host_port_r     : std_logic_vector(15 downto 0) := (others => '0');
    signal host_valid_r    : std_logic                     := '0';

    signal tx_byte_idx     : unsigned(13 downto 0) := (others => '0');
    signal reply_pulse_r   : std_logic := '0';

    -- Byte idx of the 102-byte reply: shared header for idx 0..41, discovery
    -- payload (V4.4 spec p.42) for idx 42..101.
    function disc_byte_at(
        idx       : natural;
        peer_mac  : std_logic_vector(47 downto 0);
        our_mac   : std_logic_vector(47 downto 0);
        our_ip    : std_logic_vector(31 downto 0);
        peer_ip   : std_logic_vector(31 downto 0);
        peer_port : std_logic_vector(15 downto 0);
        ip_chk    : std_logic_vector(15 downto 0)
    ) return std_logic_vector is
        variable p : natural;
    begin
        if idx < L2L3L4_BYTES then
            return eth_ip_udp_hdr_byte(idx, peer_mac, our_mac, our_ip, peer_ip,
                                       HPSDR_DISC_PORT, peer_port,
                                       IP_TLEN_VEC, UDP_LEN_VEC, ip_chk);
        end if;
        p := idx - L2L3L4_BYTES;
        case p is
            when  4 => return x"02";                  -- status: idle (Orion2: 02+run)
            when  5 => return our_mac(47 downto 40);
            when  6 => return our_mac(39 downto 32);
            when  7 => return our_mac(31 downto 24);
            when  8 => return our_mac(23 downto 16);
            when  9 => return our_mac(15 downto  8);
            when 10 => return our_mac( 7 downto  0);
            when 11 => return BOARD_TYPE;
            when 12 => return PROTOCOL_VERSION;
            when 13 => return CODE_VERSION;
            when 20 => return NUMBER_DDCS;
            when 21 => return FREQ_PHASE;
            when 23 => return BETA_VERSION;
            when others => return x"00";              -- seq, sub-board vers, pad
        end case;
    end function;

begin

    U_ip_chk : entity work.ip_hdr_checksum
        generic map ( IP_TOTAL_LEN => IP_TOTAL_LEN )
        port map (
            clock    => clock,
            reset    => reset,
            our_ip   => our_ip,
            peer_ip  => peer_ip_r,
            checksum => ip_chk
        );

    tx_data_mux : process(state, tx_byte_idx, peer_mac_r, our_mac,
                          our_ip, peer_ip_r, peer_port_r, ip_chk)
    begin
        if state = S_TX then
            tx_data <= disc_byte_at(to_integer(tx_byte_idx),
                                    peer_mac_r, our_mac, our_ip,
                                    peer_ip_r, peer_port_r, ip_chk);
        else
            tx_data <= (others => '0');
        end if;
    end process;

    tx_valid    <= '1' when state = S_TX else '0';
    tx_sop      <= '1' when (state = S_TX and tx_byte_idx = 0) else '0';
    tx_eop      <= '1' when (state = S_TX and
                             tx_byte_idx = to_unsigned(FRAME_BYTES - 1,
                                                       tx_byte_idx'length))
                       else '0';
    tx_length   <= to_unsigned(FRAME_BYTES, tx_length'length);
    reply_pulse <= reply_pulse_r;

    host_mac    <= host_mac_r;
    host_ip     <= host_ip_r;
    host_port   <= host_port_r;
    host_valid  <= host_valid_r;

    -- S_RX: walk the probe, latch peer addrs at sop, flag discovery at byte 4.
    -- On rx_eop with a valid probe, commit host_* and emit the reply (S_TX).
    fsm : process(clock, reset)
        variable n_byte_idx : unsigned(13 downto 0);
        variable n_is_disc  : std_logic;
    begin
        if reset = '1' then
            state          <= S_RX;
            rx_byte_idx    <= (others => '0');
            is_discovery_r <= '0';
            peer_mac_r     <= (others => '0');
            peer_ip_r      <= (others => '0');
            peer_port_r    <= (others => '0');
            host_mac_r     <= (others => '0');
            host_ip_r      <= (others => '0');
            host_port_r    <= (others => '0');
            host_valid_r   <= '0';
            tx_byte_idx    <= (others => '0');
            reply_pulse_r  <= '0';
        elsif rising_edge(clock) then
            reply_pulse_r <= '0';

            case state is

            when S_RX =>
                n_byte_idx := rx_byte_idx;
                n_is_disc  := is_discovery_r;

                if rx_valid = '1' then
                    if rx_sop = '1' then
                        n_byte_idx  := (others => '0');
                        n_is_disc   := '0';
                        peer_mac_r  <= peer_mac;
                        peer_ip_r   <= peer_ip;
                        peer_port_r <= peer_port;
                    end if;

                    if n_byte_idx = to_unsigned(4, n_byte_idx'length) and
                       rx_data = P2_CMD_DISCOVERY then
                        n_is_disc := '1';
                    end if;

                    if rx_eop = '1' then
                        if n_is_disc = '1' and
                           n_byte_idx >= to_unsigned(4, n_byte_idx'length) then
                            tx_byte_idx  <= (others => '0');
                            state        <= S_TX;
                            host_mac_r   <= peer_mac_r;
                            host_ip_r    <= peer_ip_r;
                            host_port_r  <= peer_port_r;
                            host_valid_r <= '1';
                        end if;
                        n_byte_idx := (others => '0');
                        n_is_disc  := '0';
                    else
                        n_byte_idx := n_byte_idx + 1;
                    end if;
                end if;

                rx_byte_idx    <= n_byte_idx;
                is_discovery_r <= n_is_disc;

            when S_TX =>
                if tx_ready = '1' then
                    if tx_byte_idx = to_unsigned(FRAME_BYTES - 1,
                                                 tx_byte_idx'length) then
                        reply_pulse_r <= '1';
                        state         <= S_RX;
                        tx_byte_idx   <= (others => '0');
                    else
                        tx_byte_idx <= tx_byte_idx + 1;
                    end if;
                end if;

            end case;
        end if;
    end process;

end architecture;
