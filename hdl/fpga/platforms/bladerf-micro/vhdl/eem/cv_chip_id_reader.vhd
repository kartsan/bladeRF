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
-- cv_chip_id_reader
--
-- Reads the 64-bit per-device unique chip ID out of the Cyclone V
-- `cyclonev_chipidblock` hard primitive without going through Intel's
-- altchip_id IP wrapper.  The wrapper IP is ~250 lines of Verilog over the
-- same primitive plus a Gray counter, a generate block for cross-family
-- portability, and an LPM shift register; this file is the bladeRF Micro
-- (Cyclone V only) trimmed-down equivalent.
--
-- Wire-level protocol of cyclonev_chipidblock:
--
--   * clk      : sample clock; we drive it from `clock`.
--   * shiftnld : 0 = load fuse value into the block's internal shift
--                register (must hold for >= 1 clk).
--                1 = shift the loaded value out, one bit per clk, LSB-first
--                on `regout`.
--   * regout   : serial output, 1 bit per clk during the shift phase.
--
-- The minimum legal handshake is therefore 1 load cycle + 64 shift cycles;
-- after that the chip ID is in our internal shift register and data_valid
-- stays asserted.  We park in S_DONE and drive shiftnld='0' there so the
-- primitive idles cleanly.
--
-- shiftnld is driven combinationally from `state` rather than from a
-- separate register.  That keeps the FSM <-> primitive handshake aligned:
-- on the clock edge that transitions state from S_LOAD to S_SHIFT, the
-- primitive samples the *old* shiftnld value (=0, performing one final
-- load) and our state register simultaneously moves to S_SHIFT; the next
-- edge sees shiftnld=1 and the first bit appears on regout, which we
-- capture in S_SHIFT cycle 0.
--
-- Bit ordering: the primitive emits LSB-first.  Our shift register feeds
-- regout into bit 63 and shifts right, so after 64 captures bit 0 holds
-- the first regout (= chip_id bit 0) and bit 63 holds the last regout
-- (= chip_id bit 63).  The output therefore matches what altchip_id
-- produces.  For the chip_id_mac fold this ordering is immaterial -- any
-- deterministic mapping yields a stable per-board MAC -- but matching the
-- IP's convention means swap-in is byte-for-byte compatible.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity cv_chip_id_reader is
    port (
        clock      : in  std_logic;
        reset      : in  std_logic;
        chip_id    : out std_logic_vector(63 downto 0);
        data_valid : out std_logic
    );
end entity;

architecture arch of cv_chip_id_reader is

    -- Vendor primitive.  Black-box component declaration: Quartus resolves
    -- it from the Cyclone V atoms library at synthesis time.  ModelSim
    -- needs cyclonev_atoms.v from $QUARTUS_ROOTDIR/eda/sim_lib/ compiled
    -- into the cyclonev (or work) library before sourcing compile.do.
    component cyclonev_chipidblock
        port (
            clk      : in  std_logic;
            regout   : out std_logic;
            shiftnld : in  std_logic
        );
    end component;

    type state_t is (S_LOAD, S_SHIFT, S_DONE);
    signal state      : state_t                       := S_LOAD;

    -- 64 shift cycles fit in 6 bits (0..63).
    signal cycle_cnt  : unsigned(5 downto 0)          := (others => '0');
    signal shift_reg  : std_logic_vector(63 downto 0) := (others => '0');
    signal id_reg     : std_logic_vector(63 downto 0) := (others => '0');
    signal valid_r    : std_logic                     := '0';

    signal shiftnld_w : std_logic;
    signal regout_w   : std_logic;

begin

    -- Combinational shiftnld from state -- see header comment for why.
    shiftnld_w <= '1' when state = S_SHIFT else '0';

    U_chipid : cyclonev_chipidblock
        port map (
            clk      => clock,
            regout   => regout_w,
            shiftnld => shiftnld_w
        );

    chip_id    <= id_reg;
    data_valid <= valid_r;

    fsm : process (clock, reset)
    begin
        if reset = '1' then
            state     <= S_LOAD;
            cycle_cnt <= (others => '0');
            shift_reg <= (others => '0');
            id_reg    <= (others => '0');
            valid_r   <= '0';
        elsif rising_edge(clock) then
            case state is

            -- ----------------------------------------------------------------
            -- One-cycle load: shiftnld_w='0' (combinationally) tells the
            -- primitive to latch its fuse value into the internal shift
            -- register.  No capture happens here; on the next edge we
            -- enter S_SHIFT and the first bit appears on regout.
            -- ----------------------------------------------------------------
            when S_LOAD =>
                cycle_cnt <= (others => '0');
                state     <= S_SHIFT;

            -- ----------------------------------------------------------------
            -- 64 shift cycles: capture one bit of regout per cycle into the
            -- top of shift_reg (which shifts right), so after 64 cycles
            -- shift_reg holds the chip ID LSB-aligned.  On the last cycle
            -- snapshot it into id_reg and announce data_valid.
            -- ----------------------------------------------------------------
            when S_SHIFT =>
                shift_reg <= regout_w & shift_reg(63 downto 1);
                if cycle_cnt = to_unsigned(63, cycle_cnt'length) then
                    id_reg  <= regout_w & shift_reg(63 downto 1);
                    valid_r <= '1';
                    state   <= S_DONE;
                else
                    cycle_cnt <= cycle_cnt + 1;
                end if;

            -- ----------------------------------------------------------------
            -- Idle forever holding the captured chip ID.  shiftnld_w drops
            -- back to '0' (state /= S_SHIFT) so the primitive harmlessly
            -- re-loads its internal register every clock; we ignore regout.
            -- ----------------------------------------------------------------
            when S_DONE =>
                null;

            end case;
        end if;
    end process;

end architecture;
