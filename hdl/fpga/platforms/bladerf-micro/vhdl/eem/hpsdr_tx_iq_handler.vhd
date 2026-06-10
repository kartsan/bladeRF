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
-- hpsdr_tx_iq_handler
--
-- OpenHPSDR Protocol 2 "DUC0 I&Q" receiver (host -> radio, UDP/1029).  The TX
-- counterpart of the DDC IQ sender; a VHDL re-implementation of orion2's
-- byte_to_48bits.v (Phil Harman VK6PH).  Sits on udp_rx_handler's
-- hpsdr_duc_iq_* byte stream, unpacks the host's 24-bit I & 24-bit Q samples,
-- and writes each as a 48-bit word into the DUC IQ FIFO that feeds hpsdr_duc.
--
-- UDP/1029 payload layout (V4.4 spec p.31-32 / orion2 byte_to_48bits.v):
--
--   byte 0..3     Sequence number (32-bit, big-endian)
--   byte 4..1443  240 samples x 6 bytes:
--                   I[23:16] I[15:8] I[7:0]  Q[23:16] Q[15:8] Q[7:0]
--
-- Each 6-byte group is shifted MSB-first into a 48-bit register, so the
-- completed word is { I[23:0], Q[23:0] } -- data_out[47:24]=I, [23:0]=Q,
-- matching what hpsdr_duc / cic_interp expect (x_real=I, x_imag=Q).
--
-- Writes are gated by `enable` (= host engaged AND PTT): the FIFO is only fed
-- while transmitting, and the top level holds the FIFO in aclr otherwise so
-- each transmission starts empty.  If the FIFO reports full the sample is
-- dropped (orion2 behaviour) rather than stalling the byte stream.
--
-- sequence_errors counts non-consecutive sequence numbers for SignalTap /
-- bring-up observability; it has no functional consumer.  Runs in the EEM
-- clock domain (fx3_pclk_pll), same as udp_rx_handler; the FIFO performs the
-- crossing to tx_clock for the DUC.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_tx_iq_handler is
    port (
        clock           : in  std_logic;
        reset           : in  std_logic;

        -- Gate: write to the FIFO only while engaged + PTT (host transmit).
        enable          : in  std_logic;

        -- DUC0 I&Q byte stream (UDP/1029 payload, header stripped) from
        -- udp_rx_handler's hpsdr_duc_iq_* outputs.
        rx_data         : in  std_logic_vector(7 downto 0);
        rx_valid        : in  std_logic;
        rx_sop          : in  std_logic;
        rx_eop          : in  std_logic;

        -- DUC IQ FIFO write side ({I[23:0], Q[23:0]}).  fifo_full back-pressure
        -- causes the sample to be dropped, never a byte-stream stall.
        fifo_wrdata     : out std_logic_vector(47 downto 0);
        fifo_wrreq      : out std_logic;
        fifo_full       : in  std_logic;

        -- SignalTap observability only (no functional consumer).
        sequence_errors : out std_logic_vector(31 downto 0)
    );
end entity;

architecture arch of hpsdr_tx_iq_handler is

    -- 11 bits cover byte 0..1443.
    signal byte_idx        : unsigned(10 downto 0)        := (others => '0');

    -- 0..5 within each 6-byte I/Q sample group.
    signal sample_byte     : unsigned(2 downto 0)         := (others => '0');

    -- MSB-first assembly register for the current 48-bit sample.
    signal word_sr         : std_logic_vector(47 downto 0) := (others => '0');

    signal seq_sr          : std_logic_vector(31 downto 0) := (others => '0');
    signal last_seq        : std_logic_vector(31 downto 0) := (others => '0');
    signal seq_err_r       : unsigned(31 downto 0)         := (others => '0');

    signal wrdata_r        : std_logic_vector(47 downto 0) := (others => '0');
    signal wrreq_r         : std_logic                     := '0';

begin

    fifo_wrdata     <= wrdata_r;
    fifo_wrreq      <= wrreq_r;
    sequence_errors <= std_logic_vector(seq_err_r);

    fsm : process(clock, reset)
        variable n_byte_idx : unsigned(10 downto 0);
        variable n_word     : std_logic_vector(47 downto 0);
    begin
        if reset = '1' then
            byte_idx    <= (others => '0');
            sample_byte <= (others => '0');
            word_sr     <= (others => '0');
            seq_sr      <= (others => '0');
            last_seq    <= (others => '0');
            seq_err_r   <= (others => '0');
            wrdata_r    <= (others => '0');
            wrreq_r     <= '0';
        elsif rising_edge(clock) then
            wrreq_r <= '0';

            if rx_valid = '1' then
                n_byte_idx := byte_idx;
                if rx_sop = '1' then
                    n_byte_idx  := (others => '0');
                    sample_byte <= (others => '0');
                end if;

                -- Shift the in-flight byte into the 48-bit assembly register;
                -- n_word is the post-shift value used for the write below.
                n_word  := word_sr(39 downto 0) & rx_data;
                word_sr <= n_word;

                if n_byte_idx <= to_unsigned(3, n_byte_idx'length) then
                    -- Sequence number bytes (big-endian).
                    seq_sr      <= seq_sr(23 downto 0) & rx_data;
                    sample_byte <= (others => '0');
                else
                    -- Sample payload: emit a word every 6th byte.
                    if sample_byte = to_unsigned(5, sample_byte'length) then
                        sample_byte <= (others => '0');
                        if enable = '1' and fifo_full = '0' then
                            wrdata_r <= n_word;
                            wrreq_r  <= '1';
                        end if;
                    else
                        sample_byte <= sample_byte + 1;
                    end if;
                end if;

                if rx_eop = '1' then
                    -- Commit sequence-error check at the packet boundary.
                    if seq_sr /= std_logic_vector(unsigned(last_seq) + 1) then
                        seq_err_r <= seq_err_r + 1;
                    end if;
                    last_seq    <= seq_sr;
                    n_byte_idx  := (others => '0');
                    sample_byte <= (others => '0');
                else
                    n_byte_idx := n_byte_idx + 1;
                end if;

                byte_idx <= n_byte_idx;
            end if;
        end if;
    end process;

end architecture;
