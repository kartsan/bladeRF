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
-- tx_arbiter
--
-- Four-input mux + arbiter sitting in front of eem_tx_framer's byte-stream
-- input.  Producers compete for the framer; once a producer wins, it owns
-- the framer until the framer's pkt_done_pulse fires (= the host has
-- drained the last word over USB), then the arbiter returns to IDLE and
-- the next producer can be selected.
--
-- Priority: A > B > C > D.  In the typical wiring:
--   A = arp_responder              (NUD-timed; strictest deadline)
--   B = icmp_responder             (ping reply; loose deadline but user-visible)
--   C = dhcp_client                (4 s retransmit; very loose)
--   D = hpsdr_discovery_responder  (piHPSDR retries ~2-3 s; very loose)
--
-- HPSDR IQ streaming, when it lands, will run at higher rates than any of
-- the bring-up protocols here so it'll likely demand its own arbiter
-- behaviour (round-robin against control plane / dedicated framer), at
-- which point this priority arbiter becomes the "control plane" mux
-- feeding a higher-level scheduler.  Discovery stays here -- it's bursty
-- (one 102-byte frame per host probe) and well-served by strict priority.
--
-- The unchosen producer sees its ready signal held low, so it just keeps
-- holding its sop beat stable -- the same sop-hold contract that
-- eem_tx_framer already requires.  When the arbiter releases, the held
-- producer is picked up on the next cycle and begins streaming.
-- =============================================================================

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity tx_arbiter is
    port (
        clock      : in  std_logic;
        reset      : in  std_logic;

        -- Producer A (highest priority).  Typical wiring: arp_responder.
        a_data     : in  std_logic_vector(7 downto 0);
        a_valid    : in  std_logic;
        a_sop      : in  std_logic;
        a_eop      : in  std_logic;
        a_length   : in  unsigned(13 downto 0);
        a_ready    : out std_logic;

        -- Producer B (middle priority).  Typical wiring: icmp_responder.
        b_data     : in  std_logic_vector(7 downto 0);
        b_valid    : in  std_logic;
        b_sop      : in  std_logic;
        b_eop      : in  std_logic;
        b_length   : in  unsigned(13 downto 0);
        b_ready    : out std_logic;

        -- Producer C (third priority).  Typical wiring: dhcp_client.
        c_data     : in  std_logic_vector(7 downto 0);
        c_valid    : in  std_logic;
        c_sop      : in  std_logic;
        c_eop      : in  std_logic;
        c_length   : in  unsigned(13 downto 0);
        c_ready    : out std_logic;

        -- Producer D (lowest priority).  Typical wiring:
        -- hpsdr_discovery_responder.
        d_data     : in  std_logic_vector(7 downto 0);
        d_valid    : in  std_logic;
        d_sop      : in  std_logic;
        d_eop      : in  std_logic;
        d_length   : in  unsigned(13 downto 0);
        d_ready    : out std_logic;

        -- To framer's frame_in_* port
        tx_data    : out std_logic_vector(7 downto 0);
        tx_valid   : out std_logic;
        tx_sop     : out std_logic;
        tx_eop     : out std_logic;
        tx_length  : out unsigned(13 downto 0);
        tx_ready   : in  std_logic;

        -- Framer feedback: 1-cycle pulse when the framer's S_DRIVE
        -- finishes streaming the previous frame over the FX3 RX1 path.
        -- The arbiter releases the active producer here so the next
        -- can be selected.
        pkt_done   : in  std_logic
    );
end entity;

architecture arch of tx_arbiter is

    type state_t is (S_IDLE, S_A_ACTIVE, S_B_ACTIVE, S_C_ACTIVE, S_D_ACTIVE);
    signal state : state_t := S_IDLE;

begin

    -- Combinational mux: tx_* follows the active producer; producers' ready
    -- signals gated by selection.
    tx_data   <= a_data   when state = S_A_ACTIVE else
                 b_data   when state = S_B_ACTIVE else
                 c_data   when state = S_C_ACTIVE else
                 d_data   when state = S_D_ACTIVE else
                 (others => '0');
    tx_valid  <= a_valid  when state = S_A_ACTIVE else
                 b_valid  when state = S_B_ACTIVE else
                 c_valid  when state = S_C_ACTIVE else
                 d_valid  when state = S_D_ACTIVE else
                 '0';
    tx_sop    <= a_sop    when state = S_A_ACTIVE else
                 b_sop    when state = S_B_ACTIVE else
                 c_sop    when state = S_C_ACTIVE else
                 d_sop    when state = S_D_ACTIVE else
                 '0';
    tx_eop    <= a_eop    when state = S_A_ACTIVE else
                 b_eop    when state = S_B_ACTIVE else
                 c_eop    when state = S_C_ACTIVE else
                 d_eop    when state = S_D_ACTIVE else
                 '0';
    tx_length <= a_length when state = S_A_ACTIVE else
                 b_length when state = S_B_ACTIVE else
                 c_length when state = S_C_ACTIVE else
                 d_length when state = S_D_ACTIVE else
                 (others => '0');

    a_ready <= tx_ready when state = S_A_ACTIVE else '0';
    b_ready <= tx_ready when state = S_B_ACTIVE else '0';
    c_ready <= tx_ready when state = S_C_ACTIVE else '0';
    d_ready <= tx_ready when state = S_D_ACTIVE else '0';

    fsm : process(clock, reset)
    begin
        if reset = '1' then
            state <= S_IDLE;
        elsif rising_edge(clock) then
            case state is
            when S_IDLE =>
                -- Priority A > B > C > D; pick whoever is presenting a
                -- fresh frame.
                if a_valid = '1' and a_sop = '1' then
                    state <= S_A_ACTIVE;
                elsif b_valid = '1' and b_sop = '1' then
                    state <= S_B_ACTIVE;
                elsif c_valid = '1' and c_sop = '1' then
                    state <= S_C_ACTIVE;
                elsif d_valid = '1' and d_sop = '1' then
                    state <= S_D_ACTIVE;
                end if;
            when S_A_ACTIVE =>
                -- Hold the producer through framer's full processing
                -- pipeline (S_HDR, S_ETH, S_FCS, S_DUMMY, S_DRIVE);
                -- release on pkt_done so the framer is back in S_IDLE
                -- and the next frame can begin cleanly.
                if pkt_done = '1' then
                    state <= S_IDLE;
                end if;
            when S_B_ACTIVE =>
                if pkt_done = '1' then
                    state <= S_IDLE;
                end if;
            when S_C_ACTIVE =>
                if pkt_done = '1' then
                    state <= S_IDLE;
                end if;
            when S_D_ACTIVE =>
                if pkt_done = '1' then
                    state <= S_IDLE;
                end if;
            end case;
        end if;
    end process;

end architecture;
