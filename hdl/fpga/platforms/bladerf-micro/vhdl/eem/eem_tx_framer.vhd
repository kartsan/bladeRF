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
-- eem_tx_framer
--
-- Wraps an outbound Ethernet frame (raw byte stream) in a CDC EEM data packet
-- on the FPGA -> host direction and presents it to fx3_gpif's RX1 SAMPLE_READ
-- via a show-ahead 32-bit FIFO interface.
--
-- For every frame in:
--   * Prepends a 2-byte EEM data-packet header (bmType=0, bmCRC=0,
--     EthernetLength = N + 4 bytes including the 4-byte FCS sentinel).
--   * Appends a 4-byte FCS sentinel 0xDEADBEEF (URB byte order DE AD BE EF;
--     bmCRC=0 lets the host's cdc_eem driver match the value directly).
--   * Appends one extra 32-bit dummy word at the end of the burst.  The FX3
--     GPIF II FSM always drops the last beat of every IF_RX_1 burst (the
--     "last-word-dropped" artifact captured in
--     project_eem_gpif_wrapup_rx1_artifacts.md); the dummy is the bringup
--     workaround until that's fixed in GPIF II Designer.
--
-- URB byte / 32-bit-word packing convention (matches eem_rx_consumer's
-- inverse and the working bringup test injector):
--   URB byte k  ->  fifo_rdata((k mod 4)*8 + 7 downto (k mod 4)*8)
-- i.e. byte 0 of every word goes in bits 7..0, byte 3 in bits 31..24.  So
-- the first word's low half holds the EEM header and its high half holds
-- the first two Ethernet bytes.
--
-- Internal storage is a sync-write / async-read array read combinatorially
-- via `fifo_rdata <= buf(to_integer(word_rp))`.  Quartus auto-infers this
-- as Cyclone V MLAB (LAB-resident LUT-RAM) rather than M10K -- MLAB is the
-- only block on Cyclone V that supports async read, and its address
-- decode is internal so there's no wide fabric mux on the read path.
-- This matches the FWFT semantics fx3_gpif's RX1 SAMPLE_READ requires
-- (the two-cycle pipeline through nuand sync_fifo + gpif_mux caused the
-- first-word-duplication trap observed during bringup).
--
-- A BRAM-backed FWFT variant with a 1-deep prefetch register was tried
-- 2026-05-22 (ramstyle="M10K" with explicit head_r + S_PRIME bridge) and
-- WORKED for DHCP DISCOVER as far as Linux's promisc-mode capture was
-- concerned, but the resulting URB stream had the second 32-bit word
-- silently replaced by the third (= every frame's bytes 2..5 carried
-- bytes 6..9's content, clobbering the requester MAC in ARP replies and
-- the broadcast bytes in DHCP DISCOVER).  Paper traces of the prefetch
-- formula bram_raddr <= word_rp+3 give the correct sequence buf[0],
-- buf[1], buf[2]... so the bug is somewhere in Quartus's M10K inference
-- behaviour (likely an extra register stage we didn't model), not in
-- the abstract pipeline math.  M10K isn't actually needed for HPSDR-1/2
-- frames either: at BUF_DEPTH=512 (= 2 KB) the MLAB inference burns
-- ~32 MLABs out of the part's ~1000+, no Fmax cost.  Captured in
-- [[feedback_bram_prefetch_offbyone]].
--
-- BUF_DEPTH default 512 (= 2 KB) sized for HPSDR Protocol 2 DDC IQ frames
-- (Eth+IP+UDP+1444 = 1486 B per packet = 372 32-bit words) with headroom.
-- MLAB storage cost: ~32 MLABs (BUF_DEPTH * 32 bits / 640 bits per
-- MLAB = 25.6 -> 32 rounding up for width).  Forcing `ramstyle="logic"`
-- would push storage into raw FFs (BUF_DEPTH * 32 = 16 k FFs at depth
-- 512) AND introduce a BUF_DEPTH-way combinational read mux -- that's
-- where the Fmax cliff lives, *not* in the MLAB-backed default.
--
-- Byte-stream input interface:
--   frame_in_sop      : first byte of frame; frame_in_length is sampled in
--                       the same cycle and is the Ethernet length in bytes
--                       (no FCS, no EEM header).
--   frame_in_eop      : last byte of frame.
--   frame_in_valid    : a byte is being presented this cycle.
--   frame_in_ready    : framer can consume the byte this cycle.  Deasserted
--                       for two cycles immediately after sop (while the
--                       framer injects the two header bytes ahead of the
--                       Ethernet payload), then held high for the duration
--                       of the frame.  Producer must hold {data, sop, eop,
--                       length, valid} stable until ready='1'.
--
-- Show-ahead output interface:
--   fifo_empty='0'    : fifo_rdata holds a valid head word this cycle.
--   fifo_rreq         : pulses one cycle to consume the head word.  The
--                       next word (if any) is presented combinatorially on
--                       the same edge -- no extra latency.
--   fifo_rdata        : 32-bit head word.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity eem_tx_framer is
    generic (
        -- Word depth of the internal frame buffer (each word is 32 bits).
        -- Default 512 (= 2 KB) fits HPSDR Protocol 2 DDC IQ frames with
        -- headroom and Quartus auto-infers as MLAB (~32 MLABs) so there's
        -- no Fmax cost on the combinational read side.  Bumping further
        -- is cheap until MLAB count starts mattering to the rest of the
        -- design.
        BUF_DEPTH : natural := 512
    );
    port (
        clock           : in  std_logic;
        reset           : in  std_logic;

        -- Byte-stream input
        frame_in_data   : in  std_logic_vector(7 downto 0);
        frame_in_valid  : in  std_logic;
        frame_in_sop    : in  std_logic;
        frame_in_eop    : in  std_logic;
        frame_in_length : in  unsigned(13 downto 0);
        frame_in_ready  : out std_logic;

        -- Show-ahead 32-bit FIFO interface to fx3_gpif RX1 SAMPLE_READ
        fifo_empty      : out std_logic;
        fifo_rdata      : out std_logic_vector(31 downto 0);
        fifo_rreq       : in  std_logic;

        -- Observability
        pkt_done_pulse  : out std_logic;
        pkt_count       : out std_logic_vector(15 downto 0)
    );
end entity;

architecture arch of eem_tx_framer is

    -- ceil(log2(N)) for N >= 2
    function clog2(n : natural) return natural is
        variable r : natural := 0;
        variable v : natural := n - 1;
    begin
        while v > 0 loop
            r := r + 1;
            v := v / 2;
        end loop;
        if r = 0 then
            return 1;
        else
            return r;
        end if;
    end function;

    constant BUF_ADDR_W : natural := clog2(BUF_DEPTH);

    type word_array_t is array (0 to BUF_DEPTH-1) of std_logic_vector(31 downto 0);
    signal buf : word_array_t := (others => (others => '0'));

    -- One byte per cycle flows through a single packing accumulator from
    -- four sources in turn: HDR (2 bytes), ETH (N bytes), FCS (4 bytes),
    -- DUMMY (4 bytes).  When the accumulator fills (byte_bp wraps 3->0)
    -- the current word commits to buf[word_wp] and word_wp advances.
    -- After the dummy bytes the FSM enters S_DRIVE which exposes the
    -- buffer combinatorially to fx3_gpif.
    type state_t is (S_IDLE, S_HDR0, S_HDR1, S_ETH, S_FCS, S_DUMMY, S_DRIVE);
    signal state : state_t := S_IDLE;

    signal word_wp    : unsigned(BUF_ADDR_W-1 downto 0) := (others => '0');
    signal word_rp    : unsigned(BUF_ADDR_W-1 downto 0) := (others => '0');
    signal word_total : unsigned(BUF_ADDR_W-1 downto 0) := (others => '0');
    signal byte_bp    : unsigned(1 downto 0)            := (others => '0');
    signal word_acc   : std_logic_vector(31 downto 0)   := (others => '0');
    signal stage_idx  : unsigned(1 downto 0)            := (others => '0');

    signal hdr_len_r  : unsigned(13 downto 0)           := (others => '0');

    signal pkt_count_r : unsigned(15 downto 0)          := (others => '0');
    signal pdone_r     : std_logic                      := '0';

    -- FCS sentinel: URB byte order DE AD BE EF (matches Linux cdc_eem
    -- rx_fixup's literal 0xDEADBEEF compare when bmCRC=0).
    function fcs_byte_of (idx : unsigned(1 downto 0))
        return std_logic_vector is
    begin
        case idx is
            when "00"   => return x"DE";
            when "01"   => return x"AD";
            when "10"   => return x"BE";
            when others => return x"EF";
        end case;
    end function;

    -- Dummy padding bytes.  Value is don't-care (FX3 drops the last word
    -- of every burst); 0xFECAADDE / "DE AD CA FE" is easy to spot in
    -- usbmon dumps if it ever leaks through.
    function dummy_byte_of (idx : unsigned(1 downto 0))
        return std_logic_vector is
    begin
        case idx is
            when "00"   => return x"DE";
            when "01"   => return x"AD";
            when "10"   => return x"CA";
            when others => return x"FE";
        end case;
    end function;

    -- Pack one byte into a 32-bit accumulator at the given byte lane.
    function pack_byte (acc : std_logic_vector(31 downto 0);
                        lane : unsigned(1 downto 0);
                        b   : std_logic_vector(7 downto 0))
        return std_logic_vector is
        variable r : std_logic_vector(31 downto 0);
    begin
        r := acc;
        case lane is
            when "00" => r(7  downto  0) := b;
            when "01" => r(15 downto  8) := b;
            when "10" => r(23 downto 16) := b;
            when others => r(31 downto 24) := b;
        end case;
        return r;
    end function;

    -- Compute EEM header byte 0 = EthernetLength[7:0]
    function eem_hdr_b0 (eth_len : unsigned(13 downto 0))
        return std_logic_vector is
    begin
        return std_logic_vector(eth_len(7 downto 0));
    end function;

    -- Compute EEM header byte 1 = (bmType<<7)|(bmCRC<<6)|EthernetLength[13:8]
    -- bmType=0 (data packet), bmCRC=0 (deadbeef sentinel)
    function eem_hdr_b1 (eth_len : unsigned(13 downto 0))
        return std_logic_vector is
    begin
        return "00" & std_logic_vector(eth_len(13 downto 8));
    end function;

begin

    fifo_rdata     <= buf(to_integer(word_rp));
    pkt_done_pulse <= pdone_r;
    pkt_count      <= std_logic_vector(pkt_count_r);

    -- fifo_empty is asserted whenever the FSM is not actively presenting a
    -- buffered frame.  word_rp == word_total marks the end (we never let it
    -- wrap mid-frame because the FX3 drains << 1518 B/burst and we issue
    -- one frame at a time).
    fifo_empty <= '0' when (state = S_DRIVE and word_rp < word_total) else '1';

    -- Byte-stream backpressure: accept bytes only in S_ETH.  Producer must
    -- hold the sop beat stable until we leave S_HDR1 and assert ready.
    frame_in_ready <= '1' when state = S_ETH else '0';

    fsm : process(clock, reset)
        variable new_acc   : std_logic_vector(31 downto 0);
        variable byte_val  : std_logic_vector(7 downto 0);
        variable do_commit : std_logic;
        variable bp_next   : unsigned(1 downto 0);
    begin
        if reset = '1' then
            state         <= S_IDLE;
            word_wp       <= (others => '0');
            word_rp       <= (others => '0');
            word_total    <= (others => '0');
            byte_bp       <= (others => '0');
            word_acc      <= (others => '0');
            stage_idx     <= (others => '0');
            hdr_len_r     <= (others => '0');
            pkt_count_r   <= (others => '0');
            pdone_r       <= '0';
        elsif rising_edge(clock) then
            pdone_r <= '0';
            do_commit := '0';
            new_acc   := word_acc;
            byte_val  := (others => '0');
            bp_next   := byte_bp;

            case state is

            -- ----------------------------------------------------------------
            -- Wait for the start of a new frame.  Capture length, reset the
            -- byte-pack accumulator, and begin injecting the EEM header.
            -- frame_in_ready stays '0' here -- the producer is expected to
            -- hold the sop beat stable.
            -- ----------------------------------------------------------------
            when S_IDLE =>
                if frame_in_valid = '1' and frame_in_sop = '1' then
                    hdr_len_r <= frame_in_length + 4;  -- +4 for FCS sentinel
                    word_wp   <= (others => '0');
                    word_acc  <= (others => '0');
                    byte_bp   <= (others => '0');
                    stage_idx <= (others => '0');
                    state     <= S_HDR0;
                end if;

            -- ----------------------------------------------------------------
            -- Inject EEM header byte 0 (length low) into lane 0 of the
            -- accumulator.  No commit possible yet (only 1 byte in).
            -- ----------------------------------------------------------------
            when S_HDR0 =>
                byte_val := eem_hdr_b0(hdr_len_r);
                new_acc  := pack_byte(word_acc, "00", byte_val);
                bp_next  := "01";
                word_acc <= new_acc;
                byte_bp  <= bp_next;
                state    <= S_HDR1;

            -- ----------------------------------------------------------------
            -- Inject EEM header byte 1 (bmType/bmCRC/length high) into lane
            -- 1.  Still no commit.  Next cycle we open ready=1 and start
            -- pulling Ethernet bytes into lanes 2,3,0,1,2,3,...
            -- ----------------------------------------------------------------
            when S_HDR1 =>
                byte_val := eem_hdr_b1(hdr_len_r);
                new_acc  := pack_byte(word_acc, "01", byte_val);
                bp_next  := "10";
                word_acc <= new_acc;
                byte_bp  <= bp_next;
                state    <= S_ETH;

            -- ----------------------------------------------------------------
            -- Stream Ethernet bytes from the producer.  One byte per cycle
            -- when frame_in_valid='1'; pack into the accumulator; commit
            -- on every fourth byte (byte_bp wraps 3->0).  On eop, transition
            -- to FCS injection -- the producer is expected to drop valid
            -- the cycle after eop.
            -- ----------------------------------------------------------------
            when S_ETH =>
                if frame_in_valid = '1' then
                    byte_val := frame_in_data;
                    new_acc  := pack_byte(word_acc, byte_bp, byte_val);
                    if byte_bp = "11" then
                        do_commit := '1';
                        bp_next   := "00";
                    else
                        bp_next   := byte_bp + 1;
                    end if;

                    if do_commit = '1' then
                        buf(to_integer(word_wp)) <= new_acc;
                        word_wp <= word_wp + 1;
                        word_acc <= (others => '0');
                    else
                        word_acc <= new_acc;
                    end if;
                    byte_bp <= bp_next;

                    if frame_in_eop = '1' then
                        stage_idx <= (others => '0');
                        state     <= S_FCS;
                    end if;
                end if;

            -- ----------------------------------------------------------------
            -- Inject the 4-byte FCS sentinel one byte per cycle, continuing
            -- the byte_bp lane sequence from where ETH left off.  Commits
            -- happen whenever the accumulator fills.
            -- ----------------------------------------------------------------
            when S_FCS =>
                byte_val := fcs_byte_of(stage_idx);
                new_acc  := pack_byte(word_acc, byte_bp, byte_val);
                if byte_bp = "11" then
                    do_commit := '1';
                    bp_next   := "00";
                else
                    bp_next   := byte_bp + 1;
                end if;

                if do_commit = '1' then
                    buf(to_integer(word_wp)) <= new_acc;
                    word_wp <= word_wp + 1;
                    word_acc <= (others => '0');
                else
                    word_acc <= new_acc;
                end if;
                byte_bp <= bp_next;

                if stage_idx = "11" then
                    stage_idx <= (others => '0');
                    state     <= S_DUMMY;
                else
                    stage_idx <= stage_idx + 1;
                end if;

            -- ----------------------------------------------------------------
            -- Inject the dummy trailing word, one byte per cycle, continuing
            -- byte_bp.  When stage_idx wraps 3->0, the last dummy byte has
            -- landed; commit any partial accumulator (S_FCS may have left
            -- byte_bp at a non-zero lane), finalize word_total, and enter
            -- the drive phase.
            --
            -- N=50 byte Ethernet:    eth bytes 2..51, fcs bytes 52..55, dummy
            --                        bytes 56..59 -> all four boundaries are
            --                        word-aligned, no partial flush needed.
            -- N=14 byte ARP:         eth 2..15, fcs 16..19, dummy 20..23 ->
            --                        same; word-aligned by construction.
            -- N=15 byte odd:         eth 2..16, fcs 17..20, dummy 21..24 ->
            --                        stage ends at byte 24 with byte_bp=01;
            --                        partial accumulator has byte 24 packed
            --                        into lane 0 and must be flushed.
            -- ----------------------------------------------------------------
            when S_DUMMY =>
                byte_val := dummy_byte_of(stage_idx);
                new_acc  := pack_byte(word_acc, byte_bp, byte_val);
                if byte_bp = "11" then
                    do_commit := '1';
                    bp_next   := "00";
                else
                    bp_next   := byte_bp + 1;
                end if;

                if stage_idx = "11" then
                    -- Last dummy byte: commit the accumulator regardless of
                    -- whether the byte landed in lane 3 (full word) or
                    -- earlier (partial word -- any not-yet-written lanes
                    -- carry zeros from the prior reset).  The whole dummy
                    -- region gets dropped by the FX3 last-word workaround
                    -- so the exact bit pattern is don't-care -- *as long as
                    -- (frame_in_length+6) mod 4 = 0*.  For other lengths
                    -- (e.g. 60-byte minimum Ethernet) the URB ends up with
                    -- 1..3 trailing zero bytes between the FCS and the
                    -- dummy word boundary; the host's cdc_eem parser will
                    -- interpret those bytes as a malformed second EEM
                    -- packet header.  Cosmetic for AF_PACKET listeners
                    -- (the first frame still arrives correctly), but worth
                    -- fixing alongside real arbitrary-length traffic --
                    -- emit a 2-byte ResponseHint EEM command packet to
                    -- consume any odd pad slack.
                    buf(to_integer(word_wp)) <= new_acc;
                    word_total <= word_wp + 1;
                    word_acc <= (others => '0');
                    byte_bp  <= (others => '0');
                    word_rp  <= (others => '0');
                    state    <= S_DRIVE;
                else
                    if do_commit = '1' then
                        buf(to_integer(word_wp)) <= new_acc;
                        word_wp  <= word_wp + 1;
                        word_acc <= (others => '0');
                    else
                        word_acc <= new_acc;
                    end if;
                    byte_bp <= bp_next;
                    stage_idx <= stage_idx + 1;
                end if;

            -- ----------------------------------------------------------------
            -- Present buffered words to fx3_gpif via the show-ahead output.
            -- One word advance per fifo_rreq pulse; on the final advance
            -- pulse pkt_done and return to idle.
            -- ----------------------------------------------------------------
            when S_DRIVE =>
                if fifo_rreq = '1' and word_rp < word_total then
                    if (word_rp + 1) = word_total then
                        pkt_count_r <= pkt_count_r + 1;
                        pdone_r     <= '1';
                        state       <= S_IDLE;
                    end if;
                    word_rp <= word_rp + 1;
                end if;

            end case;
        end if;
    end process;

end architecture;
