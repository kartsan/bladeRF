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
-- hpsdr_duc
--
-- CIC interpolator for the HPSDR DUC TX path -- the exact inverse of
-- hpsdr_ddc.  Unlike the openHPSDR/orion2 transmitter (Orion.v, which follows
-- the CIC with a CORDIC NCO to mix baseband up to the real-IF DAC) there is NO
-- CORDIC here: the AD9361 takes complex baseband I/Q and performs the RF
-- upconversion internally with its own TX LO, so this block only has to
-- interpolate the host DUC0 sample rate up to the fixed native rate.
--
-- Native AD9361 rate is 12.288 MSPS; INTERPOLATION = 12_288_000 / duc0_rate.
-- Runtime-selectable across the rates Thetis can request for DUC0:
--   48/96/192/384/768/1536 kHz  <->  interp 256/128/64/32/16/8
-- (the same numeric set as the DDC decimation).  Drive `rate_id_gray` from
-- hpsdr_duc_spec_handler (EEM clock domain); this block syncs and decodes it.
-- Boot default and any invalid gray code fall back to 192 kHz / interp 64 --
-- the spec's fixed transmit rate.
--
-- Pacing: the interpolator advances one output per `strobe`, which the top
-- level drives from the AD9361 DAC sample request (native 12.288 MSPS, gated
-- by PTT).  Every INTERPOLATION outputs the embedded cic_interp pulses `req`
-- to pull the next 48-bit { I, Q } word from the show-ahead DUC IQ FIFO;
-- fifo_rdreq is masked by fifo_rdempty so an underrun re-uses the last sample
-- rather than reading past the FIFO (benign for bring-up -- the FIFO is held
-- in aclr unless PTT, so it is primed before transmit).
--
-- See feedback_orion2_cic_reset_port_unused: cic_interp has no functional
-- reset on its accumulators; correctness relies on Cyclone V config-time zero
-- init plus the FIFO aclr between transmissions.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_duc is
    generic (
        IN_WIDTH       : natural := 24;   -- HPSDR DUC IQ sample width
        OUT_WIDTH      : natural := 16;   -- AD9361 DAC data field width
        STAGES         : natural := 5;
        MIN_INTERP     : natural := 8;    -- 12.288 MHz / 1536 kHz
        MAX_INTERP     : natural := 256   -- 12.288 MHz / 48 kHz
    );
    port (
        clock        : in  std_logic;     -- tx_clock (= ad9361.clock)
        reset        : in  std_logic;

        -- Gray-coded DUC0 rate selector from hpsdr_duc_spec_handler (EEM clock
        -- domain).  Synchronised internally (per-bit 2-FF) and decoded to an
        -- interpolation factor.  "000"->256 "001"->128 "011"->64 "010"->32
        -- "110"->16 "111"->8; "100"/"101"/boot -> 64 (192 kHz default).
        rate_id_gray : in  std_logic_vector(2 downto 0);

        -- One pulse per AD9361 DAC sample request (native rate, PTT-gated).
        strobe       : in  std_logic;

        -- Show-ahead DUC IQ FIFO read side ({ I[23:0], Q[23:0] }).
        fifo_q       : in  std_logic_vector(47 downto 0);
        fifo_rdempty : in  std_logic;
        fifo_rdreq   : out std_logic;

        -- Interpolated complex baseband to the DAC mux; out_valid pulses one
        -- sample per accepted `strobe`.
        out_i        : out signed(OUT_WIDTH-1 downto 0);
        out_q        : out signed(OUT_WIDTH-1 downto 0);
        out_valid    : out std_logic
    );
end entity;

architecture arch of hpsdr_duc is

    function clog2(n : natural) return natural is
        variable r : natural := 0;
        variable v : natural := n - 1;
    begin
        while v > 0 loop
            v := v / 2;
            r := r + 1;
        end loop;
        return r;
    end function;

    -- cic_interp's interpolation port is [clog2(MAX_INTERP):0] = clog2+1 bits.
    constant INTERP_PORT_W : natural := clog2(MAX_INTERP) + 1;

    component cic_interp is
        generic (
            IBITS      : natural;
            OBITS      : natural;
            STAGES     : natural;
            MAX_INTERP : natural;
            MIN_INTERP : natural
        );
        port (
            clock         : in  std_logic;
            clock_en      : in  std_logic;
            interpolation : in  std_logic_vector(INTERP_PORT_W-1 downto 0);
            req           : out std_logic;
            x_real        : in  std_logic_vector(IBITS-1 downto 0);
            x_imag        : in  std_logic_vector(IBITS-1 downto 0);
            y_real        : out std_logic_vector(OBITS-1 downto 0);
            y_imag        : out std_logic_vector(OBITS-1 downto 0)
        );
    end component;

    -- Per-bit 2-FF synchroniser for rate_id_gray (gray coding => any single
    -- metastable bit resolves to an adjacent valid rate, not garbage).
    signal rate_id_meta : std_logic_vector(2 downto 0) := "011";
    signal rate_id_sync : std_logic_vector(2 downto 0) := "011";

    -- Decoded interpolation factor; boot default 64 (192 kHz).
    signal interp_r : std_logic_vector(INTERP_PORT_W-1 downto 0)
        := std_logic_vector(to_unsigned(64, INTERP_PORT_W));

    signal cic_req   : std_logic;
    signal cic_y_i   : std_logic_vector(OUT_WIDTH-1 downto 0);
    signal cic_y_q   : std_logic_vector(OUT_WIDTH-1 downto 0);
    signal out_valid_r : std_logic := '0';

begin

    -- --------------------------------------------------------------------
    -- Sync + decode rate_id_gray -> interpolation factor.  Invalid gray
    -- codes (4/5) and boot fall back to 192 kHz / interp 64 per spec.
    -- --------------------------------------------------------------------
    cdc : process(clock)
    begin
        if rising_edge(clock) then
            rate_id_meta <= rate_id_gray;
            rate_id_sync <= rate_id_meta;

            case to_integer(unsigned(rate_id_sync)) is
                when 0 =>      -- gray "000" -> 48   kHz
                    interp_r <= std_logic_vector(to_unsigned(256, INTERP_PORT_W));
                when 1 =>      -- gray "001" -> 96   kHz
                    interp_r <= std_logic_vector(to_unsigned(128, INTERP_PORT_W));
                when 3 =>      -- gray "011" -> 192  kHz (default)
                    interp_r <= std_logic_vector(to_unsigned(64,  INTERP_PORT_W));
                when 2 =>      -- gray "010" -> 384  kHz
                    interp_r <= std_logic_vector(to_unsigned(32,  INTERP_PORT_W));
                when 6 =>      -- gray "110" -> 768  kHz
                    interp_r <= std_logic_vector(to_unsigned(16,  INTERP_PORT_W));
                when 7 =>      -- gray "111" -> 1536 kHz
                    interp_r <= std_logic_vector(to_unsigned(8,   INTERP_PORT_W));
                when others => -- 4 / 5 invalid -> 192 kHz
                    interp_r <= std_logic_vector(to_unsigned(64,  INTERP_PORT_W));
            end case;
        end if;
    end process;

    -- FIFO read request: cic_interp pulls one input every INTERPOLATION
    -- outputs.  Mask against an empty FIFO so an underrun re-uses the head
    -- rather than asserting rdreq on no data.
    fifo_rdreq <= cic_req and (not fifo_rdempty);

    U_cic_interp : cic_interp
        generic map (
            IBITS      => IN_WIDTH,
            OBITS      => OUT_WIDTH,
            STAGES     => STAGES,
            MAX_INTERP => MAX_INTERP,
            MIN_INTERP => MIN_INTERP
        )
        port map (
            clock         => clock,
            clock_en      => strobe,
            interpolation => interp_r,
            req           => cic_req,
            x_real        => fifo_q(47 downto 24),   -- I[23:0]
            x_imag        => fifo_q(23 downto  0),   -- Q[23:0]
            y_real        => cic_y_i,
            y_imag        => cic_y_q
        );

    -- cic_interp registers its output on clock_en, so out_valid trails strobe
    -- by one cycle and marks a freshly-produced sample.
    out_valid_proc : process(clock, reset)
    begin
        if reset = '1' then
            out_valid_r <= '0';
        elsif rising_edge(clock) then
            out_valid_r <= strobe;
        end if;
    end process;

    out_i     <= signed(cic_y_i);
    out_q     <= signed(cic_y_q);
    out_valid <= out_valid_r;

end architecture;
