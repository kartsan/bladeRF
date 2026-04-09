-- eem_rx_adapter.vhd
--
-- Converts the 32-bit word stream produced by eem_deframer into a byte-serial
-- stream compatible with the HPSDR mac_recv / ip_recv / ... chain.
--
-- Interface contract (from eem_deframer):
--   eth_data_out     : valid Ethernet frame bytes packed little-endian
--                      (byte 0 = bits 7:0, byte 1 = bits 15:8, ...)
--   eth_data_valid   : word is valid this clock
--   eth_packet_start : set on the first word of a new frame (coincides with valid)
--   eth_packet_end   : set on the last word of a frame (coincides with valid)
--   eth_packet_empty : number of UNUSED bytes in the last word (0..3)
--                      e.g. empty=1 means bytes 0,1,2 are valid; byte 3 is pad
--
-- Output interface (HPSDR byte-serial):
--   rx_data          : one valid Ethernet byte per clock
--   rx_enable        : high for every clock that rx_data is valid, spanning the
--                      whole frame; goes low one cycle after the last byte
--
-- Timing: eem_deframer produces one word per clock (when the queue is
-- non-empty).  This adapter always takes exactly 4 clocks per word (one per
-- byte).  If the deframer produces words consecutively this adapter will have
-- a 4-cycle latency from word to first byte, which is fine — the deframer
-- queue absorbs bursts from the GPIF 4-bytes-per-clock delivery.

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity eem_rx_adapter is
    port (
        clock            : in  std_logic;
        reset            : in  std_logic;

        -- From eem_deframer
        eth_data_in      : in  std_logic_vector(31 downto 0);
        eth_data_valid   : in  std_logic;
        eth_packet_start : in  std_logic;
        eth_packet_end   : in  std_logic;
        eth_packet_empty : in  std_logic_vector(1 downto 0);

        -- To HPSDR mac_recv et al.
        rx_data          : out std_logic_vector(7 downto 0);
        rx_enable        : out std_logic
    );
end entity;

architecture arch of eem_rx_adapter is

    -- Small FIFO to hold incoming 32-bit words while we serialise them.
    -- The EEM deframer's 4-word output queue means we can see at most 4
    -- consecutive words before it stalls; a depth of 8 is more than enough.
    constant FIFO_DEPTH : natural := 8;

    type data_fifo_t  is array (0 to FIFO_DEPTH-1) of std_logic_vector(31 downto 0);
    type empty_fifo_t is array (0 to FIFO_DEPTH-1) of std_logic_vector(1 downto 0);
    type flag_fifo_t  is array (0 to FIFO_DEPTH-1) of std_logic;

    signal fifo_data  : data_fifo_t  := (others => (others => '0'));
    signal fifo_start : flag_fifo_t  := (others => '0');
    signal fifo_end   : flag_fifo_t  := (others => '0');
    signal fifo_empty : empty_fifo_t := (others => (others => '0'));
    signal fifo_head  : integer range 0 to FIFO_DEPTH-1 := 0;
    signal fifo_tail  : integer range 0 to FIFO_DEPTH-1 := 0;
    signal fifo_count : integer range 0 to FIFO_DEPTH   := 0;

    -- Serialiser state
    signal ser_data   : std_logic_vector(31 downto 0) := (others => '0');
    signal ser_end    : std_logic := '0';
    signal ser_empty  : std_logic_vector(1 downto 0) := (others => '0');
    signal ser_byte   : integer range 0 to 3 := 0;   -- next byte to emit
    signal ser_valid_bytes : integer range 1 to 4 := 4; -- valid bytes in cur word
    signal ser_active : std_logic := '0';             -- currently serialising
    signal in_packet  : std_logic := '0';             -- rx_enable frame open

    signal rx_enable_i : std_logic := '0';

begin

    rx_enable <= rx_enable_i;

    process(clock, reset)
        variable head_v  : integer range 0 to FIFO_DEPTH-1;
        variable tail_v  : integer range 0 to FIFO_DEPTH-1;
        variable count_v : integer range 0 to FIFO_DEPTH;
    begin
        if reset = '1' then
            fifo_head  <= 0;
            fifo_tail  <= 0;
            fifo_count <= 0;
            ser_active <= '0';
            ser_byte   <= 0;
            ser_end    <= '0';
            ser_empty  <= (others => '0');
            ser_valid_bytes <= 4;
            in_packet  <= '0';
            rx_data    <= (others => '0');
            rx_enable_i <= '0';

        elsif rising_edge(clock) then
            head_v  := fifo_head;
            tail_v  := fifo_tail;
            count_v := fifo_count;

            -- ----------------------------------------------------------------
            -- Enqueue incoming words
            -- ----------------------------------------------------------------
            if eth_data_valid = '1' then
                if count_v < FIFO_DEPTH then
                    fifo_data(tail_v)  <= eth_data_in;
                    fifo_start(tail_v) <= eth_packet_start;
                    fifo_end(tail_v)   <= eth_packet_end;
                    fifo_empty(tail_v) <= eth_packet_empty;
                    tail_v  := (tail_v + 1) mod FIFO_DEPTH;
                    count_v := count_v + 1;
                end if;
                -- Words silently dropped when FIFO is full; this should not
                -- happen in practice because the deframer also stalls.
            end if;

            -- ----------------------------------------------------------------
            -- Dequeue and serialise one byte per clock
            -- ----------------------------------------------------------------
            rx_data     <= (others => '0');
            rx_enable_i <= '0';

            if ser_active = '1' then
                -- Emit current byte
                case ser_byte is
                    when 0 => rx_data <= ser_data(7  downto 0);
                    when 1 => rx_data <= ser_data(15 downto 8);
                    when 2 => rx_data <= ser_data(23 downto 16);
                    when 3 => rx_data <= ser_data(31 downto 24);
                end case;

                rx_enable_i <= '1';

                if ser_byte = ser_valid_bytes - 1 then
                    -- Last byte of this word
                    ser_byte   <= 0;
                    ser_active <= '0';

                    if ser_end = '1' then
                        -- Last word of frame: close rx_enable next cycle
                        in_packet <= '0';
                        -- rx_enable goes low on the NEXT clock automatically
                        -- because ser_active will be '0' and in_packet '0'.
                        -- But we need one more cycle of '1' for this byte —
                        -- already done above.  The fall happens next cycle.
                    end if;
                else
                    ser_byte <= ser_byte + 1;
                end if;

            elsif count_v > 0 then
                -- Load next word from FIFO
                ser_data  <= fifo_data(head_v);
                ser_end   <= fifo_end(head_v);
                ser_empty <= fifo_empty(head_v);

                -- Valid bytes = 4 - empty count
                case fifo_empty(head_v) is
                    when "01"   => ser_valid_bytes <= 3;
                    when "10"   => ser_valid_bytes <= 2;
                    when "11"   => ser_valid_bytes <= 1;
                    when others => ser_valid_bytes <= 4;
                end case;

                if fifo_start(head_v) = '1' then
                    in_packet <= '1';
                end if;

                ser_active <= '1';
                ser_byte   <= 0;

                head_v  := (head_v + 1) mod FIFO_DEPTH;
                count_v := count_v - 1;
            end if;

            fifo_head  <= head_v;
            fifo_tail  <= tail_v;
            fifo_count <= count_v;
        end if;
    end process;

end architecture;
