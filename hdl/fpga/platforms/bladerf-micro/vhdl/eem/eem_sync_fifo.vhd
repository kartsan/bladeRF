-- Copyright (c) 2013 Nuand LLC
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
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
-- THE SOFTWARE.

-- =============================================================================
-- eem_sync_fifo
--
-- EEM-local fork of nuand.sync_fifo that fixes the simultaneous read+write
-- silent-corruption trap (the original advanced neither pointer when both
-- write_en and read_en were '1' at the same edge; that no-op silently lost
-- the write and stalled the read).  See feedback_sync_fifo_rw_collision.md.
--
-- The EEM TX FIFO is the first place in the bladeRF design where a same-
-- clock high-rate writer (fx3_gpif TX2 burst) meets a same-clock reader
-- (eem_rx_consumer) -- a collision pattern the original sync_fifo's
-- if/elsif structure never had to handle correctly.  Kept here as an
-- EEM-only entity so the upstream shared IP isn't touched and future
-- bladeRF source updates don't conflict.
--
-- Interface is identical to nuand.sync_fifo.  READ_AHEAD generic is
-- still declared but unused (matches upstream behavior); data_out has
-- a 1-cycle registered latency regardless.
-- =============================================================================

library ieee ;
    use ieee.std_logic_1164.all ;
    use ieee.numeric_std.all ;

entity eem_sync_fifo is
  generic (
    DEPTH       :       positive    := 32 ;
    WIDTH       :       positive    := 16 ;
    READ_AHEAD  :       boolean     := true
  ) ;
  port (
    areset      :   in  std_logic ;
    clock       :   in  std_logic ;

    -- FIFO Status information
    full        :   out std_logic ;
    empty       :   out std_logic ;
    used_words  :   out natural range 0 to DEPTH ;

    -- FIFO Input Side
    data_in     :   in  std_logic_vector(WIDTH-1 downto 0) ;
    write_en    :   in  std_logic ;

    -- FIFO Output Side
    data_out    :   out std_logic_vector(WIDTH-1 downto 0) ;
    read_en     :   in  std_logic
  ) ;
end entity ; -- eem_sync_fifo

architecture arch of eem_sync_fifo is

    -- House keeping
    signal write_address    : natural range 0 to DEPTH-1 ;
    signal read_address     : natural range 0 to DEPTH-1 ;
    signal used             : natural range 0 to DEPTH ;

    -- Dual port RAM
    type ram_t is array(0 to DEPTH-1) of std_logic_vector(WIDTH-1 downto 0) ;
    signal ram              : ram_t ;

begin

    empty <= '1' when used = 0 else '0' ;
    full <= '1' when used = DEPTH else '0' ;
    used_words <= used ;

    follow_used_words : process( clock, areset )
    begin
        if( areset = '1' ) then
            used <= 0 ;
            write_address <= 0 ;
            read_address <= 0 ;
        elsif( rising_edge(clock) ) then
            -- Simultaneous read+write must advance BOTH pointers independently.
            -- The original sync_fifo skipped both pointer updates on
            -- write_en='1' AND read_en='1', which silently lost the write
            -- (overwritten next time) and stalled the read (returned stale
            -- data the next cycle).  See feedback_sync_fifo_rw_collision.md.
            if( write_en = '1' and used < DEPTH ) then
                write_address <= (write_address + 1) mod DEPTH ;
            elsif( write_en = '1' ) then
                report "Trying to write a full FIFO!" severity error ;
            end if ;

            if( read_en = '1' and used > 0 ) then
                read_address <= (read_address + 1) mod DEPTH ;
            elsif( read_en = '1' ) then
                report "Trying to read an empty FIFO!" severity error ;
            end if ;

            -- used count: +1 on write-only, -1 on read-only, unchanged on R+W
            -- (the new word goes straight through), unchanged on idle.
            if( write_en = '1' and read_en = '0' and used < DEPTH ) then
                used <= used + 1 ;
            elsif( read_en = '1' and write_en = '0' and used > 0 ) then
                used <= used - 1 ;
            end if ;
        end if ;
    end process ;

    access_ram : process( clock )
    begin
        if( rising_edge( clock ) ) then
            data_out <= ram(read_address) ;
            if( write_en = '1' ) then
                ram(write_address) <= data_in ;
            end if ;
        end if ;
    end process ;

end architecture ; -- arch
