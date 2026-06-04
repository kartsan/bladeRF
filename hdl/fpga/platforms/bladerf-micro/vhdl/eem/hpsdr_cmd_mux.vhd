-- Copyright (c) 2026 Nuand LLC
--
-- HPSDR command-mailbox multiplexer.  Funnels per-knob request streams
-- from the HPSDR fabric (RX0 frequency, future GAIN/BW/RSSI/...) into the
-- single (cmd_op, cmd_data_in) NIOS-bound PIO pair.  Maintains a 4-bit
-- seq counter so duplicate opcodes (e.g. the same frequency re-applied)
-- still trigger a new NIOS dispatch.
--
-- Output encoding mirrors the layout documented in nios_system.tcl's hpsdr
-- block and consumed by the NIOS dispatcher in bladeRF_nios.c:
--
--   cmd_op bits [7:0]   opcode  (BLADERF_RFIC_COMMAND_* from bladerf2_common.h)
--          bits [10:8]  channel (0=RX0, 1=RX1, 2=TX0, 3=TX1, 7=SYSTEM)
--          bit  [11]    rw      (0=write, 1=read)
--          bits [15:12] seq     (strobe; incremented on every new request)
--
-- Initial source set: just RX0 FREQUENCY, driven from
-- hpsdr_hp_cmd_handler.host_rx0_freq.  Mux detects edge-change of the
-- 32-bit phase word and pulses a new request.  Read-back ports
-- (cmd_data_lo/_hi, cmd_status) come in synchronised to this clock but
-- aren't yet consumed -- they're exposed so the wiring stays uniform when
-- the first read source (RSSI -> hp_status_sender) lands.

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_cmd_mux is
    port (
        clock           : in  std_logic;
        reset           : in  std_logic;

        -- Source A: RX0 frequency (NCO phase word from hp_cmd_handler).
        -- Mux issues a write on any change away from the previously-seen
        -- non-zero value, matching the firmware's prior 2-poll heuristic.
        rx0_freq        : in  std_logic_vector(31 downto 0);

        -- Outputs to NIOS PIOs (driven into per-bit CDC synchronisers
        -- in bladerf-hpsdr.vhd; this module stays in fx3_pclk_pll domain).
        cmd_op          : out std_logic_vector(15 downto 0);
        cmd_data_in     : out std_logic_vector(31 downto 0);

        -- Inputs from NIOS PIOs (already synced to this clock domain).
        -- Not consumed yet; provided for the first read source to plug in.
        cmd_status      : in  std_logic_vector(7 downto 0);
        cmd_data_lo     : in  std_logic_vector(31 downto 0);
        cmd_data_hi     : in  std_logic_vector(31 downto 0)
    );
end entity;

architecture rtl of hpsdr_cmd_mux is

    -- BLADERF_RFIC_COMMAND_* opcode table (subset; see bladerf2_common.h).
    constant OP_FREQUENCY : std_logic_vector(7 downto 0) := x"04";

    -- Channel codes for the 3-bit channel field.  Matches the decoder in
    -- bladeRF_nios.c's HPSDR dispatcher (hpsdr_bch).
    constant CH_RX0       : std_logic_vector(2 downto 0) := "000";

    constant RW_WRITE     : std_logic := '0';

    signal rx0_freq_prev  : std_logic_vector(31 downto 0) := (others => '0');
    signal seq_r          : unsigned(3 downto 0) := (others => '0');
    signal cmd_op_r       : std_logic_vector(15 downto 0) := (others => '0');
    signal cmd_data_in_r  : std_logic_vector(31 downto 0) := (others => '0');

    -- cmd_status / cmd_data_lo / cmd_data_hi are unused for now but the
    -- response-handling code coming next iteration needs the signals visible
    -- in the architecture.  Mark them keep so synthesis doesn't optimise the
    -- pre-mux synchroniser chains away before they have a consumer.
    attribute keep : boolean;
    attribute keep of cmd_status  : signal is true;
    attribute keep of cmd_data_lo : signal is true;
    attribute keep of cmd_data_hi : signal is true;

begin

    cmd_op      <= cmd_op_r;
    cmd_data_in <= cmd_data_in_r;

    process(clock, reset)
    begin
        if (reset = '1') then
            rx0_freq_prev <= (others => '0');
            seq_r         <= (others => '0');
            cmd_op_r      <= (others => '0');
            cmd_data_in_r <= (others => '0');
        elsif rising_edge(clock) then
            -- RX0 FREQUENCY source: fire on any change to a non-zero value.
            -- Zero is the boot/idle phase word; ignore so spurious 0 reads
            -- between HP Commands don't issue bogus retunes.
            if (rx0_freq /= rx0_freq_prev) and (rx0_freq /= x"00000000") then
                seq_r         <= seq_r + 1;
                cmd_data_in_r <= rx0_freq;
                cmd_op_r      <= std_logic_vector(seq_r + 1) &
                                 RW_WRITE                    &
                                 CH_RX0                      &
                                 OP_FREQUENCY;
            end if;

            rx0_freq_prev <= rx0_freq;
        end if;
    end process;

end architecture;
