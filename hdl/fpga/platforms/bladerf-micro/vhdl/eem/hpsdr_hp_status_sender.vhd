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
-- Periodic transmitter for OpenHPSDR Protocol 2 "High Priority Status from
-- Radio to Host" packets on UDP/1025.  This is the heartbeat Thetis (and
-- piHPSDR) watch for to declare a discovered radio "alive": after the
-- discovery handshake completes, Thetis sends 1444-byte High-Priority
-- Command-to-Radio packets at ~30 Hz on the same port, and times the radio
-- out after ~3 s of silence in the reverse direction.  This entity provides
-- that reverse direction.
--
-- High Priority Status From Radio (radio -> host, UDP/1025, 60-byte payload)
-- -------------------------------------------------------------------------
--   byte 0..3 : sequence number (BE, incremented per packet)
--   byte 4    : PTT / MOX / dot / dash status (bit 0 = PTT; idle = 0x00)
--   byte 5    : ADC1 overflow flags (0x00 = no overflow)
--   byte 6    : ADC2 overflow flags (0x00 = no overflow)
--   byte 7    : Penelope-board firmware version (1-byte identifier;
--               hpsdr_sim hardcodes 0x8b at steady state).  Thetis
--               appears to read this during its "graceful power off"
--               disconnect check -- sending 0x00 here makes Thetis
--               hang when the user presses the power button.
--   byte 8..48: per-DDC frequency echoes, temperature, supply current/voltage,
--               forward/reverse power, ALEX state, exciter power -- all
--               irrelevant for the "radio alive" indicator, kept zero.
--   byte 49   : Mercury-board supported sample-rate bitmask
--               (likely interpretation: bit 0 = 48 kHz, bit 1 = 96 kHz,
--               ..., bit 5 = 1536 kHz; hpsdr_sim hardcodes 0x3f =
--               "all six standard rates supported").  Also part of
--               Thetis's connection-state validation.
--   byte 50..59: reserved (zero)
--
-- A "radio is idle and present" status is the minimum to keep Thetis happy
-- through the full session lifecycle (incl. clean disconnect on power-off);
-- the two non-zero defaults above were copied byte-for-byte from
-- hpsdr_sim's wire output 2026-05-23, since the openHPSDR P2 spec
-- documentation we have doesn't pin down their exact semantics.  Real
-- PTT echo, ADC overflow surfacing, frequency confirmations, etc., are
-- layered on later once the corresponding upstream consumers exist;
-- both byte 7 and byte 49 will become live signals when the real
-- Penelope / Mercury sources are wired up (overridable via generics).
--
-- Addressing
-- ----------
-- The reply triple {Eth dst, IP dst, UDP dst port} is the host snapshot
-- the discovery responder captured at the last successful discovery:
--   host_mac    -> Eth dst
--   host_ip     -> IP dst
--   host_port   -> UDP dst port (NOT 1025 -- same gotcha as the discovery
--                  reply; Thetis's receive socket is bound to its
--                  sendto() ephemeral port).  See
--                  [[feedback_hpsdr_reply_udp_dst_port]].
-- UDP src port is 1025 (P2 High-Priority port; Thetis's send-from port
-- for high-priority commands is also 1025-bound-by-host-socket, but the
-- *radio* always uses 1025 as src in both directions).
--
-- We gate everything on host_valid='1' -- before discovery completes,
-- host_mac / host_ip / host_port are all zero and we hold S_IDLE silently.
--
-- IP checksum
-- -----------
-- Identical structure / IP_CONST_PART value to hpsdr_discovery_responder
-- (same 88-byte total length, same TTL/proto, same flags).  Two-stage
-- register cascade folds in our_ip at stage 1, host_ip at stage 2.
--
-- UDP checksum = 0 (legal "no checksum" on IPv4), same as discovery
-- responder.
--
-- Rate timer
-- ----------
-- A free-running counter in S_IDLE counts up to TICK_CYCLES then triggers
-- S_TX.  At 100 MHz with the default TICK_CYCLES = 5_000_000 this gives
-- 20 Hz, matching hpsdr_sim's observed steady-state rate (50 ms intervals
-- exactly).  Thetis tolerates a wide range -- it was also fine with our
-- earlier 30 Hz default -- but matching hpsdr_sim removes a degree of
-- freedom from any future "why does Thetis behave differently" debugging.
-- The counter is held at 0 in S_TX, so the per-frame rate is "TICK_CYCLES
-- idle + 102 byte emission + arbiter / framer drain" -- the drain is
-- microseconds and well-bounded under the 50 ms idle period; no race
-- with the next trigger.
--
-- The counter also holds at 0 while host_valid='0', so the first packet
-- doesn't go out until TICK_CYCLES after the first successful discovery.
--
-- FSM
-- ---
--   S_IDLE: tx_valid='0'.  Count tick_counter while host_valid='1'; on
--           TICK_CYCLES reached, increment seq, reset counter, go to S_TX.
--   S_TX  : tx_valid='1'.  Walk tx_byte_idx 0..FRAME_BYTES-1 via the
--           combinational byte mux; advance only when tx_ready='1' (same
--           sop-hold-until-ready contract as the other producers).  On
--           the final byte, pulse send_pulse and return to S_IDLE.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.bladerf_p.all;

entity hpsdr_hp_status_sender is
    generic (
        -- Inter-frame idle period in clock cycles.  Default 5_000_000
        -- at 100 MHz = 20 Hz, matching hpsdr_sim's observed steady-
        -- state rate (50 ms intervals exactly).  Bump down for faster
        -- heartbeat (piHPSDR is fine with 200 Hz, Thetis tolerates
        -- anything from ~10 Hz upward); bump up to reduce arbiter /
        -- framer load if other producers compete for tx_arbiter port
        -- time.
        TICK_CYCLES         : natural := 5_000_000;

        -- Payload byte 7 (Penelope-board firmware version, best guess).
        -- Default 0x8b mirrors hpsdr_sim's hardcoded steady-state value;
        -- Thetis reads this during its disconnect handshake.  Override
        -- with the real Penelope-version register output when it exists.
        PENELOPE_VER_BYTE7  : std_logic_vector(7 downto 0) := x"8b";

        -- Payload byte 49 (Mercury-board supported sample-rate bitmask,
        -- best guess).  Default 0x3f = 0b00111111 = "first six
        -- sample rates supported" (48/96/192/384/768/1536 kHz), again
        -- mirroring hpsdr_sim.  Override with the real Mercury
        -- capability bitmask when the DDC packetizer lands and we know
        -- which rates we can actually source IQ at.
        MERCURY_CAPS_BYTE49 : std_logic_vector(7 downto 0) := x"3f"
    );
    port (
        clock         : in  std_logic;
        reset         : in  std_logic;

        -- Local addresses
        our_mac       : in  std_logic_vector(47 downto 0);
        our_ip        : in  std_logic_vector(31 downto 0);

        -- Discovered host's address triple, snapshotted by
        -- hpsdr_discovery_responder.  host_valid='1' gates the whole
        -- sender; the three address fields are stable while it's high.
        host_mac      : in  std_logic_vector(47 downto 0);
        host_ip       : in  std_logic_vector(31 downto 0);
        host_port     : in  std_logic_vector(15 downto 0);
        host_valid    : in  std_logic;

        -- PTT0 echo: the host's last-seen PTT-channel-0 intent, from
        -- hpsdr_hp_command_receiver (= bit 1 of HP Command payload
        -- byte 4).  Mirrored into HP Status payload byte 4 bit 0 so
        -- Thetis sees its PTT request confirmed.  Tied to '0' if no
        -- HP-command receiver is wired up (the default), which produces
        -- the same all-zero byte 4 as before this hook existed.
        host_ptt0     : in  std_logic := '0';

        -- TX byte stream output (to tx_arbiter -> eem_tx_framer).
        tx_data       : out std_logic_vector(7 downto 0);
        tx_valid      : out std_logic;
        tx_sop        : out std_logic;
        tx_eop        : out std_logic;
        tx_length     : out unsigned(13 downto 0);
        tx_ready      : in  std_logic;

        -- Observability: one cycle when the final byte of each status
        -- packet is emitted.  ~30 Hz steady-state once host_valid='1'.
        send_pulse    : out std_logic
    );
end entity;

architecture arch of hpsdr_hp_status_sender is

    -- ------------------------------------------------------------------
    -- Geometry
    -- ------------------------------------------------------------------
    constant ETH_HDR_BYTES : natural := 14;
    constant IP_HDR_BYTES  : natural := 20;
    constant UDP_HDR_BYTES : natural := 8;
    constant P2_PAYLOAD    : natural := 60;
    constant FRAME_BYTES   : natural := ETH_HDR_BYTES + IP_HDR_BYTES
                                      + UDP_HDR_BYTES + P2_PAYLOAD;        -- 102
    constant IP_TOTAL_LEN  : natural := IP_HDR_BYTES + UDP_HDR_BYTES + P2_PAYLOAD; -- 88
    constant UDP_LEN       : natural := UDP_HDR_BYTES + P2_PAYLOAD;        -- 68

    constant IP_TLEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(IP_TOTAL_LEN, 16));
    constant UDP_LEN_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(UDP_LEN, 16));

    -- UDP/1025 = HPSDR P2 high-priority port (both directions).  This
    -- is the radio's *src* port; the host's port comes from host_port
    -- which is the host's ephemeral source from the discovery probe.
    constant HPSDR_HP_PORT_HI : std_logic_vector(7 downto 0) := x"04";
    constant HPSDR_HP_PORT_LO : std_logic_vector(7 downto 0) := x"01";

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

    -- Precomputed contribution to the IPv4-header checksum from all
    -- compile-time-constant fields of the reply.  Same value as the
    -- discovery responder uses (same total length, same proto/TTL,
    -- same DF flag); src and dst halves are added at register stages.
    function calc_ip_const_part return unsigned is
        variable s : unsigned(15 downto 0);
    begin
        s := oc_add(to_unsigned(16#4500#, 16), to_unsigned(IP_TOTAL_LEN, 16));
        s := oc_add(s, to_unsigned(16#4000#, 16));
        s := oc_add(s, to_unsigned(16#4011#, 16));
        return s;
    end function;

    constant IP_CONST_PART : unsigned(15 downto 0) := calc_ip_const_part;

    -- Stage-1 register: IP_CONST_PART + our_ip halves (varies only with
    -- our_ip, typically once at DHCP-ACK).
    signal ip_fixed_part_r : unsigned(15 downto 0)         := (others => '0');

    -- Stage-2 register: ~(ip_fixed_part_r + host_ip halves).  Updated
    -- one clock after host_ip changes (typically once, at host_valid
    -- rising edge); the per-byte TX mux reads it at idx 24..25.
    signal ip_chk_r        : std_logic_vector(15 downto 0) := (others => '0');

    -- ------------------------------------------------------------------
    -- State
    -- ------------------------------------------------------------------
    type state_t is (S_IDLE, S_TX);
    signal state : state_t := S_IDLE;

    -- Rate timer.  TICK_CYCLES defaults to 5_000_000 (23 bits) so use
    -- a 24-bit counter for a bit of headroom if the generic is bumped.
    signal tick_counter    : unsigned(23 downto 0) := (others => '0');

    -- Sequence number, incremented per outgoing packet.  Wraps to 0
    -- after 2^32-1; not meaningful semantically -- Thetis just uses it
    -- to detect dropped frames, which is purely informational here.
    signal seq_r           : unsigned(31 downto 0) := (others => '0');

    -- TX-side counter (walks 0..FRAME_BYTES-1)
    signal tx_byte_idx     : unsigned(13 downto 0) := (others => '0');

    signal send_pulse_r    : std_logic := '0';

    -- ------------------------------------------------------------------
    -- Combinational byte lookup for the 102-byte reply frame.
    -- ------------------------------------------------------------------
    function status_byte_at(
        idx       : natural;
        host_mac  : std_logic_vector(47 downto 0);
        our_mac   : std_logic_vector(47 downto 0);
        our_ip    : std_logic_vector(31 downto 0);
        host_ip   : std_logic_vector(31 downto 0);
        host_port : std_logic_vector(15 downto 0);
        ip_chk    : std_logic_vector(15 downto 0);
        seq       : unsigned(31 downto 0);
        host_ptt0 : std_logic
    ) return std_logic_vector is
    begin
        case idx is
            -- ---- Ethernet header ----
            -- Eth dst MAC = the discovered host
            when  0 => return host_mac(47 downto 40);
            when  1 => return host_mac(39 downto 32);
            when  2 => return host_mac(31 downto 24);
            when  3 => return host_mac(23 downto 16);
            when  4 => return host_mac(15 downto  8);
            when  5 => return host_mac( 7 downto  0);
            -- Eth src MAC = us
            when  6 => return our_mac(47 downto 40);
            when  7 => return our_mac(39 downto 32);
            when  8 => return our_mac(31 downto 24);
            when  9 => return our_mac(23 downto 16);
            when 10 => return our_mac(15 downto  8);
            when 11 => return our_mac( 7 downto  0);
            -- Ethertype = IPv4
            when 12 => return x"08";
            when 13 => return x"00";

            -- ---- IPv4 header ----
            -- V/IHL=0x45, DSCP/ECN=0x00
            when 14 => return x"45";
            when 15 => return x"00";
            -- IP total length = 88
            when 16 => return IP_TLEN_VEC(15 downto 8);
            when 17 => return IP_TLEN_VEC( 7 downto 0);
            -- IP identification = 0
            when 18 | 19 => return x"00";
            -- IP flags + frag offset = 0x4000 (DF)
            when 20 => return x"40";
            when 21 => return x"00";
            -- IP TTL=64, protocol=UDP (0x11)
            when 22 => return x"40";
            when 23 => return x"11";
            -- IP header checksum (two-stage register cascade)
            when 24 => return ip_chk(15 downto 8);
            when 25 => return ip_chk( 7 downto 0);
            -- IP src = our_ip
            when 26 => return our_ip(31 downto 24);
            when 27 => return our_ip(23 downto 16);
            when 28 => return our_ip(15 downto  8);
            when 29 => return our_ip( 7 downto  0);
            -- IP dst = the discovered host
            when 30 => return host_ip(31 downto 24);
            when 31 => return host_ip(23 downto 16);
            when 32 => return host_ip(15 downto  8);
            when 33 => return host_ip( 7 downto  0);

            -- ---- UDP header ----
            -- UDP src port = 1025 (radio's HPSDR high-priority port)
            when 34 => return HPSDR_HP_PORT_HI;
            when 35 => return HPSDR_HP_PORT_LO;
            -- UDP dst port = host's ephemeral source port (snapshot
            -- from discovery; NOT 1025).
            when 36 => return host_port(15 downto 8);
            when 37 => return host_port( 7 downto 0);
            -- UDP length = 68
            when 38 => return UDP_LEN_VEC(15 downto 8);
            when 39 => return UDP_LEN_VEC( 7 downto 0);
            -- UDP checksum = 0 (no checksum)
            when 40 | 41 => return x"00";

            -- ---- HPSDR P2 High Priority Status payload (60 bytes) ----
            -- bytes 0..3 of payload (idx 42..45): sequence number, BE
            when 42 => return std_logic_vector(seq(31 downto 24));
            when 43 => return std_logic_vector(seq(23 downto 16));
            when 44 => return std_logic_vector(seq(15 downto  8));
            when 45 => return std_logic_vector(seq( 7 downto  0));
            -- byte 4 of payload (idx 46): PTT/MOX/dot/dash status.
            -- Bit 0 echoes host_ptt0 so Thetis sees its PTT request
            -- confirmed.  Other bits (dot/dash, additional PTT channels)
            -- stay zero until we have real sources for them.
            when 46 => return "0000000" & host_ptt0;
            -- byte 5 of payload (idx 47): ADC1 overflow flags (none)
            when 47 => return x"00";
            -- byte 6 of payload (idx 48): ADC2 overflow flags (none)
            when 48 => return x"00";
            -- byte 7 of payload (idx 49): Penelope firmware version
            -- (hpsdr_sim mirror; Thetis disconnect-handshake input).
            when 49 => return PENELOPE_VER_BYTE7;
            -- byte 49 of payload (idx 91): Mercury supported-sample-
            -- rate bitmask (hpsdr_sim mirror; Thetis disconnect-
            -- handshake input).
            when 91 => return MERCURY_CAPS_BYTE49;
            -- bytes 8..48 (idx 50..90) and 50..59 (idx 92..101):
            -- per-DDC freq echoes, temperature, supply current/voltage,
            -- forward/reverse power, ALEX state -- all zero for the
            -- "alive, idle" placeholder.  Real values arrive when the
            -- upstream consumers (DDC packetizer, ADC overflow monitor,
            -- ...) land.
            when others => return x"00";
        end case;
    end function;

begin

    -- ----------------------------------------------------------------------
    -- Output drivers
    -- ----------------------------------------------------------------------
    tx_data_mux : process(state, tx_byte_idx, host_mac, our_mac,
                          our_ip, host_ip, host_port, ip_chk_r, seq_r,
                          host_ptt0)
    begin
        if state = S_TX then
            tx_data <= status_byte_at(to_integer(tx_byte_idx),
                                      host_mac, our_mac,
                                      our_ip, host_ip, host_port,
                                      ip_chk_r, seq_r,
                                      host_ptt0);
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
    -- Registered IP checksum -- two-stage cascade.  Identical structure
    -- to hpsdr_discovery_responder's: stage 1 folds our_ip, stage 2
    -- folds host_ip and inverts.  host_ip only changes at the
    -- host_valid rising edge (typically once per session), so ip_chk_r
    -- has many cycles to settle before the first S_TX.
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
    -- Main FSM + rate timer
    -- ----------------------------------------------------------------------
    fsm : process(clock, reset)
    begin
        if reset = '1' then
            state          <= S_IDLE;
            tick_counter   <= (others => '0');
            seq_r          <= (others => '0');
            tx_byte_idx    <= (others => '0');
            send_pulse_r   <= '0';
        elsif rising_edge(clock) then
            send_pulse_r <= '0';

            case state is

            -- ----------------------------------------------------------------
            -- Idle: count up to TICK_CYCLES whenever host_valid='1'.
            -- When the threshold is reached, increment seq, reset the
            -- counter, and arm S_TX.  If host_valid drops (no host
            -- selected yet, or post-disconnect), hold the counter at 0
            -- and stay quiet.
            -- ----------------------------------------------------------------
            when S_IDLE =>
                if host_valid = '0' then
                    tick_counter <= (others => '0');
                elsif tick_counter = to_unsigned(TICK_CYCLES - 1,
                                                 tick_counter'length) then
                    tick_counter <= (others => '0');
                    seq_r        <= seq_r + 1;
                    tx_byte_idx  <= (others => '0');
                    state        <= S_TX;
                else
                    tick_counter <= tick_counter + 1;
                end if;

            -- ----------------------------------------------------------------
            -- Transmit: walk tx_byte_idx 0..FRAME_BYTES-1 emitting reply
            -- bytes via the combinational tx_data mux.  Advance only on
            -- tx_ready='1' (sop-hold-until-tx_ready contract).  On the
            -- final byte, pulse send_pulse and return to S_IDLE; the
            -- arbiter holds its port-E selection until eem_tx_framer
            -- fires pkt_done_pulse, which gives plenty of cooldown
            -- before the next S_IDLE -> S_TX cycle (TICK_CYCLES at
            -- 100 MHz with default 20 Hz = 50 ms >> framer drain).
            -- ----------------------------------------------------------------
            when S_TX =>
                if tx_ready = '1' then
                    if tx_byte_idx = to_unsigned(FRAME_BYTES - 1,
                                                 tx_byte_idx'length) then
                        send_pulse_r <= '1';
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
