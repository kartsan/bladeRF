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
-- udp_rx_handler
--
-- Walks the 8-byte UDP header on every packet coming out of ip_rx_handler's
-- udp_* channel and routes the payload to a per-application output channel.
-- Currently recognised applications:
--
--   dst_port = HPSDR_PORT         (default 1024) -> hpsdr_*        channel
--                                                   (Discovery + General
--                                                    Packet from host)
--   dst_port = HPSDR_HP_CMD_PORT  (default 1027) -> hpsdr_hp_cmd_* channel
--                                                   (High Priority Command
--                                                    from host: run/PTT/
--                                                    freq/drive)
--   dst_port = HPSDR_DDC_SPEC_PORT (default 1025) -> hpsdr_ddc_spec_* channel
--                                                   (DDC Specific / Rx_specific
--                                                    Command: per-DDC enable
--                                                    bitmap + sample rates)
--   dst_port = DHCP_PORT          (default 68)   -> dhcp_*         channel
--   dst_port in {HPSDR_DUC_SPEC_PORT (1026),
--                HPSDR_DUC_IQ_PORT   (1029)}
--                                                -> payload dropped, but
--                                                   hpsdr_cc_pulse fires
--                                                   (feeds hp_cmd_handler's
--                                                   watchdog refresh; spec
--                                                   p.7-8 "any C&C packet")
--   anything else                                -> silently dropped
--
-- Input byte stream layout (output of ip_rx_handler, IP header stripped):
--
--   byte 0..1  : Source port      (big-endian)
--   byte 2..3  : Destination port (big-endian)
--   byte 4..5  : UDP length       (big-endian; includes the 8-byte header)
--   byte 6..7  : UDP checksum     (ignored; not validated)
--   byte 8..N-1: UDP payload      (forwarded to the matching app channel)
--
-- A udp_length sideband from ip_rx_handler (= IP payload length, which
-- equals the UDP packet length) is reused to derive the forwarded
-- payload length = udp_length - 8.
--
-- Detection pulses
-- ----------------
-- hpsdr_pulse / hpsdr_hp_cmd_pulse / hpsdr_ddc_spec_pulse / dhcp_pulse fire
-- for one cycle at the moment we transition from header-walk into the
-- matching forward state (after dst_port matches at byte 3 AND we reach
-- byte 7 without rx_eop closing the packet prematurely).  This signals
-- "saw a syntactically valid header for the given application" even if
-- the payload itself is zero bytes long.
--
-- hpsdr_cc_pulse fires on the same boundary for any HPSDR C&C dst port -
-- 1024/1025/1026/1027/1029 - so the HP Command handler's watchdog can
-- accept any C&C packet as a keepalive (V4.4 spec p.7-8 "any C&C packet
-- must be sent ... at least every second").  Note that 1025 fires BOTH
-- hpsdr_ddc_spec_pulse (dedicated channel) AND hpsdr_cc_pulse (watchdog).
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity udp_rx_handler is
    generic (
        -- UDP destination port that selects the HPSDR general channel.
        -- Default 1024 matches OpenHPSDR Protocol 2 (V4.4 spec): host
        -- sends Discovery probes and General Packets here.  Also UDP src
        -- port of the radio's discovery reply.
        HPSDR_PORT        : natural := 1024;

        -- UDP destination port that selects the HPSDR High-Priority
        -- Command channel.  Default 1027 per V4.4 spec: host->radio
        -- 1444-byte HP Command (run, PTT, freq, drive, Alex, attenuators).
        -- Configurable via General Packet but Thetis sends the default.
        HPSDR_HP_CMD_PORT : natural := 1027;

        -- UDP destination port that selects the DDC Specific channel.
        -- Default 1025 per Orion2 V4.0 (Hermes legacy) layout - what
        -- Thetis-with-Orion2 actually sends.  This is the inverse of
        -- the V4.4 spec ports table; see memory note
        -- feedback_hpsdr_orion2_port_mapping.  1444-byte payload with
        -- per-DDC enable bitmap + sample rate fields.
        HPSDR_DDC_SPEC_PORT : natural := 1025;

        -- Additional HPSDR C&C ports recognised for watchdog-refresh only
        -- (no payload routed yet): DUC Specific (1026, TX/CW config),
        -- DUC0 I&Q (1029, TX samples).  Each emits hpsdr_cc_pulse and
        -- is then discarded.
        HPSDR_DUC_SPEC_PORT : natural := 1026;
        HPSDR_DUC_IQ_PORT   : natural := 1029;

        -- UDP destination port that selects the DHCP-client channel.
        -- DHCP servers respond to clients on port 68 (BOOTP-client).
        DHCP_PORT         : natural := 68
    );
    port (
        clock             : in  std_logic;
        reset             : in  std_logic;

        -- UDP byte stream input (from ip_rx_handler udp_* channel).
        rx_data           : in  std_logic_vector(7 downto 0);
        rx_valid          : in  std_logic;
        rx_sop            : in  std_logic;
        rx_eop            : in  std_logic;
        rx_length         : in  std_logic_vector(13 downto 0);   -- IP payload length

        -- HPSDR general byte stream output (UDP/1024 payload only).
        hpsdr_data        : out std_logic_vector(7 downto 0);
        hpsdr_valid       : out std_logic;
        hpsdr_sop         : out std_logic;
        hpsdr_eop         : out std_logic;
        hpsdr_length      : out std_logic_vector(13 downto 0);   -- payload bytes

        -- HPSDR High-Priority Command byte stream output (UDP/1027
        -- payload only).  Consumed by hpsdr_hp_cmd_handler.
        hpsdr_hp_cmd_data    : out std_logic_vector(7 downto 0);
        hpsdr_hp_cmd_valid   : out std_logic;
        hpsdr_hp_cmd_sop     : out std_logic;
        hpsdr_hp_cmd_eop     : out std_logic;
        hpsdr_hp_cmd_length  : out std_logic_vector(13 downto 0); -- payload bytes

        -- HPSDR DDC Specific byte stream output (UDP/1025 payload only).
        -- Consumed by hpsdr_ddc_spec_handler walker to extract DDC2's
        -- sample-rate selection.
        hpsdr_ddc_spec_data   : out std_logic_vector(7 downto 0);
        hpsdr_ddc_spec_valid  : out std_logic;
        hpsdr_ddc_spec_sop    : out std_logic;
        hpsdr_ddc_spec_eop    : out std_logic;
        hpsdr_ddc_spec_length : out std_logic_vector(13 downto 0); -- payload bytes

        -- HPSDR DUC Specific byte stream output (UDP/1026 payload only).
        -- Consumed by hpsdr_duc_spec_handler to extract DUC0's TX sample
        -- rate (bytes 14..15).
        hpsdr_duc_spec_data   : out std_logic_vector(7 downto 0);
        hpsdr_duc_spec_valid  : out std_logic;
        hpsdr_duc_spec_sop    : out std_logic;
        hpsdr_duc_spec_eop    : out std_logic;
        hpsdr_duc_spec_length : out std_logic_vector(13 downto 0); -- payload bytes

        -- HPSDR DUC0 I&Q byte stream output (UDP/1029 payload only).
        -- Consumed by hpsdr_tx_iq_handler, which unpacks the host's 24-bit
        -- I&Q samples into the DUC IQ FIFO.
        hpsdr_duc_iq_data     : out std_logic_vector(7 downto 0);
        hpsdr_duc_iq_valid    : out std_logic;
        hpsdr_duc_iq_sop      : out std_logic;
        hpsdr_duc_iq_eop      : out std_logic;
        hpsdr_duc_iq_length   : out std_logic_vector(13 downto 0); -- payload bytes

        -- DHCP byte stream output (UDP payload only).
        dhcp_data         : out std_logic_vector(7 downto 0);
        dhcp_valid        : out std_logic;
        dhcp_sop          : out std_logic;
        dhcp_eop          : out std_logic;
        dhcp_length       : out std_logic_vector(13 downto 0);   -- payload bytes

        -- Sidebands: latched at classify (byte 3), held until the next
        -- classified header.
        src_port          : out std_logic_vector(15 downto 0);
        dst_port          : out std_logic_vector(15 downto 0);

        -- Observability: one-cycle pulse on each accepted header.
        hpsdr_pulse         : out std_logic;
        hpsdr_hp_cmd_pulse  : out std_logic;
        hpsdr_ddc_spec_pulse: out std_logic;
        hpsdr_duc_spec_pulse: out std_logic;
        hpsdr_duc_iq_pulse  : out std_logic;
        dhcp_pulse          : out std_logic;

        -- One-cycle pulse on any recognised HPSDR C&C dst port
        -- (1024/1025/1026/1027/1029).  Feeds the HP Command handler's
        -- watchdog so non-1027 C&C keeps run alive (spec p.7-8).
        hpsdr_cc_pulse    : out std_logic
    );
end entity;

architecture arch of udp_rx_handler is

    constant UDP_HDR_BYTES   : natural := 8;

    -- Pre-resolve the dst-port comparison constants once at elaboration so
    -- the byte-3 classify is a plain vector compare.
    constant HPSDR_PORT_VEC          : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(HPSDR_PORT,          16));
    constant HPSDR_HP_CMD_PORT_VEC   : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(HPSDR_HP_CMD_PORT,   16));
    constant HPSDR_DDC_SPEC_PORT_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(HPSDR_DDC_SPEC_PORT, 16));
    constant HPSDR_DUC_SPEC_PORT_VEC : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(HPSDR_DUC_SPEC_PORT, 16));
    constant HPSDR_DUC_IQ_PORT_VEC   : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(HPSDR_DUC_IQ_PORT,   16));
    constant DHCP_PORT_VEC           : std_logic_vector(15 downto 0)
        := std_logic_vector(to_unsigned(DHCP_PORT,           16));

    -- Classified destination of the in-flight packet.  K_HPSDR_CC is the
    -- catch-all for HPSDR C&C dst ports without a dedicated forward channel;
    -- the DUC TX ports now have their own pairs (K_HPSDR_DUC_SPEC for 1026,
    -- K_HPSDR_DUC_IQ for 1029), as 1025 does for the DDC.
    type kind_t is (K_NONE, K_HPSDR, K_HPSDR_HP_CMD, K_HPSDR_DDC_SPEC,
                    K_HPSDR_DUC_SPEC, K_HPSDR_DUC_IQ, K_HPSDR_CC, K_DHCP);
    signal kind        : kind_t := K_NONE;

    type state_t is (S_HDR, S_FWD_HPSDR, S_FWD_HPSDR_HP_CMD,
                     S_FWD_HPSDR_DDC_SPEC, S_FWD_HPSDR_DUC_SPEC,
                     S_FWD_HPSDR_DUC_IQ, S_FWD_DHCP, S_DISCARD);
    signal state : state_t := S_HDR;

    -- 4 bits cover hdr_byte_idx 0..7 plus margin.
    signal hdr_byte_idx : unsigned(3 downto 0)         := (others => '0');

    -- Captured header fields
    signal src_port_r   : std_logic_vector(15 downto 0) := (others => '0');
    signal dst_port_r   : std_logic_vector(15 downto 0) := (others => '0');

    -- Derived from rx_length at classify time.  One register, used for
    -- whichever channel is active -- only one classify per packet so
    -- there's no contention.
    signal pay_len_r    : std_logic_vector(13 downto 0) := (others => '0');

    signal sop_pending  : std_logic := '0';

    -- Registered outputs (HPSDR general / port 1024)
    signal hpsdr_data_r  : std_logic_vector(7 downto 0)  := (others => '0');
    signal hpsdr_valid_r : std_logic                     := '0';
    signal hpsdr_sop_r   : std_logic                     := '0';
    signal hpsdr_eop_r   : std_logic                     := '0';
    signal hpsdr_pulse_r : std_logic                     := '0';

    -- Registered outputs (HPSDR HP Command / port 1027)
    signal hp_cmd_data_r  : std_logic_vector(7 downto 0) := (others => '0');
    signal hp_cmd_valid_r : std_logic                    := '0';
    signal hp_cmd_sop_r   : std_logic                    := '0';
    signal hp_cmd_eop_r   : std_logic                    := '0';
    signal hp_cmd_pulse_r : std_logic                    := '0';

    -- Registered outputs (HPSDR DDC Specific / port 1025)
    signal ddc_spec_data_r  : std_logic_vector(7 downto 0) := (others => '0');
    signal ddc_spec_valid_r : std_logic                    := '0';
    signal ddc_spec_sop_r   : std_logic                    := '0';
    signal ddc_spec_eop_r   : std_logic                    := '0';
    signal ddc_spec_pulse_r : std_logic                    := '0';

    -- Registered outputs (HPSDR DUC Specific / port 1026)
    signal duc_spec_data_r  : std_logic_vector(7 downto 0) := (others => '0');
    signal duc_spec_valid_r : std_logic                    := '0';
    signal duc_spec_sop_r   : std_logic                    := '0';
    signal duc_spec_eop_r   : std_logic                    := '0';
    signal duc_spec_pulse_r : std_logic                    := '0';

    -- Registered outputs (HPSDR DUC0 I&Q / port 1029)
    signal duc_iq_data_r    : std_logic_vector(7 downto 0) := (others => '0');
    signal duc_iq_valid_r   : std_logic                    := '0';
    signal duc_iq_sop_r     : std_logic                    := '0';
    signal duc_iq_eop_r     : std_logic                    := '0';
    signal duc_iq_pulse_r   : std_logic                    := '0';

    -- Registered outputs (DHCP)
    signal dhcp_data_r   : std_logic_vector(7 downto 0)  := (others => '0');
    signal dhcp_valid_r  : std_logic                     := '0';
    signal dhcp_sop_r    : std_logic                     := '0';
    signal dhcp_eop_r    : std_logic                     := '0';
    signal dhcp_pulse_r  : std_logic                     := '0';

    -- Generic HPSDR C&C activity pulse (watchdog feed); fires for any of
    -- 1024/1025/1026/1027/1029 on the same boundary as the per-channel
    -- pulses above.
    signal hpsdr_cc_pulse_r : std_logic := '0';

begin

    hpsdr_data         <= hpsdr_data_r;
    hpsdr_valid        <= hpsdr_valid_r;
    hpsdr_sop          <= hpsdr_sop_r;
    hpsdr_eop          <= hpsdr_eop_r;
    hpsdr_length       <= pay_len_r;
    hpsdr_pulse        <= hpsdr_pulse_r;

    hpsdr_hp_cmd_data  <= hp_cmd_data_r;
    hpsdr_hp_cmd_valid <= hp_cmd_valid_r;
    hpsdr_hp_cmd_sop   <= hp_cmd_sop_r;
    hpsdr_hp_cmd_eop   <= hp_cmd_eop_r;
    hpsdr_hp_cmd_length<= pay_len_r;
    hpsdr_hp_cmd_pulse <= hp_cmd_pulse_r;

    hpsdr_ddc_spec_data   <= ddc_spec_data_r;
    hpsdr_ddc_spec_valid  <= ddc_spec_valid_r;
    hpsdr_ddc_spec_sop    <= ddc_spec_sop_r;
    hpsdr_ddc_spec_eop    <= ddc_spec_eop_r;
    hpsdr_ddc_spec_length <= pay_len_r;
    hpsdr_ddc_spec_pulse  <= ddc_spec_pulse_r;

    hpsdr_duc_spec_data   <= duc_spec_data_r;
    hpsdr_duc_spec_valid  <= duc_spec_valid_r;
    hpsdr_duc_spec_sop    <= duc_spec_sop_r;
    hpsdr_duc_spec_eop    <= duc_spec_eop_r;
    hpsdr_duc_spec_length <= pay_len_r;
    hpsdr_duc_spec_pulse  <= duc_spec_pulse_r;

    hpsdr_duc_iq_data     <= duc_iq_data_r;
    hpsdr_duc_iq_valid    <= duc_iq_valid_r;
    hpsdr_duc_iq_sop      <= duc_iq_sop_r;
    hpsdr_duc_iq_eop      <= duc_iq_eop_r;
    hpsdr_duc_iq_length   <= pay_len_r;
    hpsdr_duc_iq_pulse    <= duc_iq_pulse_r;

    dhcp_data          <= dhcp_data_r;
    dhcp_valid         <= dhcp_valid_r;
    dhcp_sop           <= dhcp_sop_r;
    dhcp_eop           <= dhcp_eop_r;
    dhcp_length        <= pay_len_r;
    dhcp_pulse         <= dhcp_pulse_r;

    hpsdr_cc_pulse     <= hpsdr_cc_pulse_r;

    src_port           <= src_port_r;
    dst_port           <= dst_port_r;

    fsm : process(clock, reset)
        variable n_byte_idx : unsigned(3 downto 0);
        variable n_src_port : std_logic_vector(15 downto 0);
        variable n_dst_port : std_logic_vector(15 downto 0);
        variable n_kind     : kind_t;
        variable payload_len : unsigned(13 downto 0);
    begin
        if reset = '1' then
            state          <= S_HDR;
            hdr_byte_idx   <= (others => '0');
            src_port_r     <= (others => '0');
            dst_port_r     <= (others => '0');
            kind           <= K_NONE;
            pay_len_r      <= (others => '0');
            sop_pending    <= '0';
            hpsdr_data_r   <= (others => '0');
            hpsdr_valid_r  <= '0';
            hpsdr_sop_r    <= '0';
            hpsdr_eop_r    <= '0';
            hpsdr_pulse_r  <= '0';
            hp_cmd_data_r  <= (others => '0');
            hp_cmd_valid_r <= '0';
            hp_cmd_sop_r   <= '0';
            hp_cmd_eop_r   <= '0';
            hp_cmd_pulse_r <= '0';
            ddc_spec_data_r  <= (others => '0');
            ddc_spec_valid_r <= '0';
            ddc_spec_sop_r   <= '0';
            ddc_spec_eop_r   <= '0';
            ddc_spec_pulse_r <= '0';
            duc_spec_data_r  <= (others => '0');
            duc_spec_valid_r <= '0';
            duc_spec_sop_r   <= '0';
            duc_spec_eop_r   <= '0';
            duc_spec_pulse_r <= '0';
            duc_iq_data_r    <= (others => '0');
            duc_iq_valid_r   <= '0';
            duc_iq_sop_r     <= '0';
            duc_iq_eop_r     <= '0';
            duc_iq_pulse_r   <= '0';
            dhcp_data_r    <= (others => '0');
            dhcp_valid_r   <= '0';
            dhcp_sop_r     <= '0';
            dhcp_eop_r     <= '0';
            dhcp_pulse_r   <= '0';
            hpsdr_cc_pulse_r <= '0';
        elsif rising_edge(clock) then
            -- One-cycle defaults
            hpsdr_valid_r  <= '0';
            hpsdr_sop_r    <= '0';
            hpsdr_eop_r    <= '0';
            hpsdr_pulse_r  <= '0';
            hp_cmd_valid_r <= '0';
            hp_cmd_sop_r   <= '0';
            hp_cmd_eop_r   <= '0';
            hp_cmd_pulse_r <= '0';
            ddc_spec_valid_r <= '0';
            ddc_spec_sop_r   <= '0';
            ddc_spec_eop_r   <= '0';
            ddc_spec_pulse_r <= '0';
            duc_spec_valid_r <= '0';
            duc_spec_sop_r   <= '0';
            duc_spec_eop_r   <= '0';
            duc_spec_pulse_r <= '0';
            duc_iq_valid_r   <= '0';
            duc_iq_sop_r     <= '0';
            duc_iq_eop_r     <= '0';
            duc_iq_pulse_r   <= '0';
            dhcp_valid_r   <= '0';
            dhcp_sop_r     <= '0';
            dhcp_eop_r     <= '0';
            dhcp_pulse_r   <= '0';
            hpsdr_cc_pulse_r <= '0';

            case state is

            -- --------------------------------------------------------------
            -- Walk the 8-byte UDP header, capturing src/dst port.  At
            -- byte 3, classify (kind).  At byte 7 (last header byte),
            -- transition: HPSDR / DHCP if matched and rx_eop didn't close
            -- the packet early; otherwise DISCARD.
            -- --------------------------------------------------------------
            when S_HDR =>
                n_byte_idx := hdr_byte_idx;
                n_src_port := src_port_r;
                n_dst_port := dst_port_r;
                n_kind     := kind;

                if rx_valid = '1' then
                    if rx_sop = '1' then
                        -- New frame: discard any half-parsed prior state.
                        n_byte_idx := (others => '0');
                        n_src_port := (others => '0');
                        n_dst_port := (others => '0');
                        n_kind     := K_NONE;
                    end if;

                    case to_integer(n_byte_idx) is
                        -- bytes 0..1: Source port (big-endian)
                        when 0 =>
                            n_src_port := rx_data & n_src_port(7 downto 0);
                        when 1 =>
                            n_src_port := n_src_port(15 downto 8) & rx_data;

                        -- bytes 2..3: Destination port (big-endian)
                        when 2 =>
                            n_dst_port := rx_data & n_dst_port(7 downto 0);
                        when 3 =>
                            n_dst_port := n_dst_port(15 downto 8) & rx_data;
                            if n_dst_port = HPSDR_PORT_VEC then
                                n_kind := K_HPSDR;
                            elsif n_dst_port = HPSDR_HP_CMD_PORT_VEC then
                                n_kind := K_HPSDR_HP_CMD;
                            elsif n_dst_port = HPSDR_DDC_SPEC_PORT_VEC then
                                n_kind := K_HPSDR_DDC_SPEC;
                            elsif n_dst_port = HPSDR_DUC_SPEC_PORT_VEC then
                                n_kind := K_HPSDR_DUC_SPEC;
                            elsif n_dst_port = HPSDR_DUC_IQ_PORT_VEC then
                                n_kind := K_HPSDR_DUC_IQ;
                            elsif n_dst_port = DHCP_PORT_VEC then
                                n_kind := K_DHCP;
                            else
                                n_kind := K_NONE;
                            end if;

                        -- bytes 4..7: length + checksum, not used here.
                        when others =>
                            null;
                    end case;

                    if n_byte_idx = to_unsigned(UDP_HDR_BYTES - 1, n_byte_idx'length) then
                        -- End of UDP header.  Classify.
                        if rx_eop = '1' then
                            -- Header was the whole packet; no payload to forward.
                            -- Still pulse on detection so the LED catches a
                            -- zero-payload probe.
                            case n_kind is
                                when K_HPSDR          =>
                                    hpsdr_pulse_r    <= '1';
                                    hpsdr_cc_pulse_r <= '1';
                                when K_HPSDR_HP_CMD   =>
                                    hp_cmd_pulse_r   <= '1';
                                    hpsdr_cc_pulse_r <= '1';
                                when K_HPSDR_DDC_SPEC =>
                                    ddc_spec_pulse_r <= '1';
                                    hpsdr_cc_pulse_r <= '1';
                                when K_HPSDR_DUC_SPEC =>
                                    duc_spec_pulse_r <= '1';
                                    hpsdr_cc_pulse_r <= '1';
                                when K_HPSDR_DUC_IQ   =>
                                    duc_iq_pulse_r   <= '1';
                                    hpsdr_cc_pulse_r <= '1';
                                when K_HPSDR_CC       =>
                                    hpsdr_cc_pulse_r <= '1';
                                when K_DHCP           =>
                                    dhcp_pulse_r     <= '1';
                                when others           => null;
                            end case;
                            state      <= S_HDR;
                            n_byte_idx := (others => '0');
                        elsif n_kind = K_HPSDR or n_kind = K_HPSDR_HP_CMD or
                              n_kind = K_HPSDR_DDC_SPEC or
                              n_kind = K_HPSDR_DUC_SPEC or
                              n_kind = K_HPSDR_DUC_IQ or
                              n_kind = K_HPSDR_CC or n_kind = K_DHCP then
                            -- Compute payload length = IP payload bytes - 8.
                            if unsigned(rx_length) >= to_unsigned(UDP_HDR_BYTES, rx_length'length) then
                                payload_len := unsigned(rx_length)
                                               - to_unsigned(UDP_HDR_BYTES, rx_length'length);
                            else
                                payload_len := (others => '0');
                            end if;
                            pay_len_r   <= std_logic_vector(payload_len);
                            sop_pending <= '1';
                            case n_kind is
                            when K_HPSDR =>
                                hpsdr_pulse_r    <= '1';
                                hpsdr_cc_pulse_r <= '1';
                                state            <= S_FWD_HPSDR;
                            when K_HPSDR_HP_CMD =>
                                hp_cmd_pulse_r   <= '1';
                                hpsdr_cc_pulse_r <= '1';
                                state            <= S_FWD_HPSDR_HP_CMD;
                            when K_HPSDR_DDC_SPEC =>
                                -- Dedicated forward channel for the DDC
                                -- Specific command; cc_pulse still fires
                                -- so the HP Command handler's watchdog
                                -- refreshes on 1025 traffic too.
                                ddc_spec_pulse_r <= '1';
                                hpsdr_cc_pulse_r <= '1';
                                state            <= S_FWD_HPSDR_DDC_SPEC;
                            when K_HPSDR_DUC_SPEC =>
                                -- DUC Specific (1026): TX sample rate etc.
                                duc_spec_pulse_r <= '1';
                                hpsdr_cc_pulse_r <= '1';
                                state            <= S_FWD_HPSDR_DUC_SPEC;
                            when K_HPSDR_DUC_IQ =>
                                -- DUC0 I&Q (1029): host transmit samples.
                                duc_iq_pulse_r   <= '1';
                                hpsdr_cc_pulse_r <= '1';
                                state            <= S_FWD_HPSDR_DUC_IQ;
                            when K_HPSDR_CC =>
                                -- Catch-all for any recognised HPSDR C&C dst
                                -- port without a dedicated forward channel;
                                -- pulse for the watchdog and drop.
                                hpsdr_cc_pulse_r <= '1';
                                state            <= S_DISCARD;
                            when others =>  -- K_DHCP
                                dhcp_pulse_r     <= '1';
                                state            <= S_FWD_DHCP;
                            end case;
                            n_byte_idx := (others => '0');
                        else
                            state      <= S_DISCARD;
                            n_byte_idx := (others => '0');
                        end if;
                    elsif rx_eop = '1' then
                        -- Frame ended before header completed; bail out.
                        state      <= S_HDR;
                        n_byte_idx := (others => '0');
                    else
                        n_byte_idx := n_byte_idx + 1;
                    end if;
                end if;

                hdr_byte_idx <= n_byte_idx;
                src_port_r   <= n_src_port;
                dst_port_r   <= n_dst_port;
                kind         <= n_kind;

            -- --------------------------------------------------------------
            -- Forward bytes to HPSDR channel.  First emitted byte gets sop;
            -- rx_eop becomes hpsdr_eop.  Return to S_HDR on eop so the
            -- next packet's sop hits the header parser cleanly.
            -- --------------------------------------------------------------
            when S_FWD_HPSDR =>
                if rx_valid = '1' then
                    hpsdr_data_r  <= rx_data;
                    hpsdr_valid_r <= '1';
                    if sop_pending = '1' then
                        hpsdr_sop_r <= '1';
                        sop_pending <= '0';
                    end if;
                    if rx_eop = '1' then
                        hpsdr_eop_r <= '1';
                        state       <= S_HDR;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Forward bytes to HPSDR HP Command channel (UDP/1027).
            -- Mirror of S_FWD_HPSDR for the hpsdr_hp_cmd_* port set;
            -- payload is the 1444-byte HP Command (run/PTT/freq/...).
            -- --------------------------------------------------------------
            when S_FWD_HPSDR_HP_CMD =>
                if rx_valid = '1' then
                    hp_cmd_data_r  <= rx_data;
                    hp_cmd_valid_r <= '1';
                    if sop_pending = '1' then
                        hp_cmd_sop_r <= '1';
                        sop_pending  <= '0';
                    end if;
                    if rx_eop = '1' then
                        hp_cmd_eop_r <= '1';
                        state        <= S_HDR;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Forward bytes to HPSDR DDC Specific channel (UDP/1025).
            -- Mirror of S_FWD_HPSDR_HP_CMD for the hpsdr_ddc_spec_* port
            -- set; payload is the 1444-byte Rx_specific command per the
            -- orion2 V4.0 layout (per-DDC enable bitmap + sample rates).
            -- --------------------------------------------------------------
            when S_FWD_HPSDR_DDC_SPEC =>
                if rx_valid = '1' then
                    ddc_spec_data_r  <= rx_data;
                    ddc_spec_valid_r <= '1';
                    if sop_pending = '1' then
                        ddc_spec_sop_r <= '1';
                        sop_pending    <= '0';
                    end if;
                    if rx_eop = '1' then
                        ddc_spec_eop_r <= '1';
                        state          <= S_HDR;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Forward bytes to HPSDR DUC Specific channel (UDP/1026).
            -- Mirror of S_FWD_HPSDR_DDC_SPEC; payload is the DUC Specific
            -- command (TX sample rate, CW/keyer config, ...).
            -- --------------------------------------------------------------
            when S_FWD_HPSDR_DUC_SPEC =>
                if rx_valid = '1' then
                    duc_spec_data_r  <= rx_data;
                    duc_spec_valid_r <= '1';
                    if sop_pending = '1' then
                        duc_spec_sop_r <= '1';
                        sop_pending    <= '0';
                    end if;
                    if rx_eop = '1' then
                        duc_spec_eop_r <= '1';
                        state          <= S_HDR;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Forward bytes to HPSDR DUC0 I&Q channel (UDP/1029).  Payload
            -- is the host's transmit samples (4-byte seq + 240 x 6-byte
            -- 24-bit I&Q), unpacked downstream by hpsdr_tx_iq_handler.
            -- --------------------------------------------------------------
            when S_FWD_HPSDR_DUC_IQ =>
                if rx_valid = '1' then
                    duc_iq_data_r  <= rx_data;
                    duc_iq_valid_r <= '1';
                    if sop_pending = '1' then
                        duc_iq_sop_r <= '1';
                        sop_pending  <= '0';
                    end if;
                    if rx_eop = '1' then
                        duc_iq_eop_r <= '1';
                        state        <= S_HDR;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Forward bytes to DHCP channel.  Mirror of S_FWD_HPSDR for
            -- the dhcp_* port set.
            -- --------------------------------------------------------------
            when S_FWD_DHCP =>
                if rx_valid = '1' then
                    dhcp_data_r  <= rx_data;
                    dhcp_valid_r <= '1';
                    if sop_pending = '1' then
                        dhcp_sop_r  <= '1';
                        sop_pending <= '0';
                    end if;
                    if rx_eop = '1' then
                        dhcp_eop_r <= '1';
                        state      <= S_HDR;
                    end if;
                end if;

            -- --------------------------------------------------------------
            -- Drop bytes until eop (rejected at classify: dst_port not
            -- in our recognised set).
            -- --------------------------------------------------------------
            when S_DISCARD =>
                if rx_valid = '1' and rx_eop = '1' then
                    state <= S_HDR;
                end if;

            end case;
        end if;
    end process;

end architecture;
