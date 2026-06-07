-- Copyright (c) 2026 Nuand LLC
--
-- HPSDR command-mailbox multiplexer.  Funnels per-knob request streams
-- from the HPSDR fabric (RX0 frequency, RX0 gain, future BW/RSSI/...) into
-- the single (cmd_op, cmd_data_in) NIOS-bound PIO pair.  Maintains a 4-bit
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
-- Issue policy: the mux is HP-Command-pulse driven.  Every full HP Command
-- (hp_cmd_pulse from hpsdr_hp_cmd_handler, ~10 Hz once Thetis engages) emits
-- at most one request, selecting the highest-priority source whose latest
-- HP Cmd value differs from the value we last issued.  Priority is freq
-- before gain; if both change in one packet, gain catches up on the next
-- pulse (~100 ms later).  Pulse-driven serialisation gives NIOS the entire
-- inter-packet gap to read cmd_op + cmd_data_in stable across its 2-poll
-- consistency check, so we don't need a separate hold timer in fabric.
--
-- Sources today:
--   * rx0_freq  (32b NCO phase word, from hp_cmd_handler bytes 9..12) ->
--               (FREQUENCY, RX0, W, phase).  Skipped while 0 so the
--               pre-engagement default doesn't issue a bogus retune.
--   * rx0_atten (5b step attenuator, from hp_cmd_handler byte 1443) ->
--               (GAIN, RX0, W, 60 - atten).  Issued whenever the latest
--               HP Cmd value differs from the value we last issued, so the
--               first packet after engagement that carries atten=0 won't
--               fire (matches the bring-up GAIN=60 default).
--
-- Read-back ports (cmd_data_lo/_hi, cmd_status) come in synchronised to
-- this clock but aren't yet consumed -- they're exposed so the wiring
-- stays uniform when the first read source (RSSI -> hp_status_sender)
-- lands.

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity hpsdr_cmd_mux is
    port (
        clock           : in  std_logic;
        reset           : in  std_logic;

        -- Source A: RX0 frequency (NCO phase word from hp_cmd_handler).
        rx0_freq        : in  std_logic_vector(31 downto 0);

        -- Source B: RX0 step attenuator in dB, 0..31 (hp_cmd_handler
        -- byte 1443).  Mux issues GAIN = 60 - atten.
        rx0_atten       : in  std_logic_vector(4 downto 0);

        -- One-cycle pulse at the end of every fully-decoded HP Command
        -- (hpsdr_hp_cmd_handler.hp_cmd_pulse).  Drives the issue cadence.
        hp_cmd_pulse    : in  std_logic;

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
    constant OP_GAIN      : std_logic_vector(7 downto 0) := x"07";

    -- Channel codes for the 3-bit channel field.  Matches the decoder in
    -- bladeRF_nios.c's HPSDR dispatcher (hpsdr_decode_channel).
    constant CH_RX0       : std_logic_vector(2 downto 0) := "000";

    constant RW_WRITE     : std_logic := '0';

    -- AD9361 gain anchor: byte_1443 = 0 -> GAIN = 32 dB, byte_1443 = 31 ->
    -- GAIN = +1 dB.  +1 dB is the AD9361 floor in its <=1.3 GHz band
    -- (ad9361.c TBL_200_1300_MHZ starting_gain_db = 1); anything below
    -- returns -EINVAL.  ANCHOR=32 keeps the full Thetis attenuator slider
    -- inside the valid window across every band the bladeRF micro RX can
    -- tune.  NIOS bring-up GAIN must match this anchor (atten=0 default).
    constant GAIN_ANCHOR_DB : integer := 32;

    -- "Last issued" snapshots; we only fire if the latest HP Cmd value
    -- differs from what NIOS already knows.  Initialised to 0 so the first
    -- HP Cmd carrying atten=0 / freq=0 is a no-op.
    signal rx0_freq_issued  : std_logic_vector(31 downto 0) := (others => '0');
    signal rx0_atten_issued : std_logic_vector(4 downto 0)  := (others => '0');

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
        -- Gain can go negative once GAIN_ANCHOR_DB < 31, so the variable and
        -- the cmd_data_in encoding are signed (two's-complement int32).  The
        -- NIOS dispatcher sign-extends din to int64 before handing it to
        -- rfic_command_write_immed.
        variable gain_dB_v   : integer range -31 to GAIN_ANCHOR_DB;
        variable next_seq_v  : unsigned(3 downto 0);
    begin
        if (reset = '1') then
            rx0_freq_issued  <= (others => '0');
            rx0_atten_issued <= (others => '0');
            seq_r            <= (others => '0');
            cmd_op_r         <= (others => '0');
            cmd_data_in_r    <= (others => '0');
        elsif rising_edge(clock) then
            -- One issue per HP Cmd, freq before gain.  Source B's deferred
            -- write catches up on the very next pulse.
            if hp_cmd_pulse = '1' then
                next_seq_v := seq_r + 1;

                if (rx0_freq /= rx0_freq_issued) and
                   (rx0_freq /= x"00000000") then
                    seq_r           <= next_seq_v;
                    cmd_data_in_r   <= rx0_freq;
                    cmd_op_r        <= std_logic_vector(next_seq_v) &
                                       RW_WRITE                     &
                                       CH_RX0                       &
                                       OP_FREQUENCY;
                    rx0_freq_issued <= rx0_freq;
                elsif rx0_atten /= rx0_atten_issued then
                    gain_dB_v        := GAIN_ANCHOR_DB -
                                        to_integer(unsigned(rx0_atten));
                    seq_r            <= next_seq_v;
                    cmd_data_in_r    <= std_logic_vector(
                                            to_signed(gain_dB_v, 32));
                    cmd_op_r         <= std_logic_vector(next_seq_v) &
                                        RW_WRITE                     &
                                        CH_RX0                       &
                                        OP_GAIN;
                    rx0_atten_issued <= rx0_atten;
                end if;
            end if;
        end if;
    end process;

end architecture;
