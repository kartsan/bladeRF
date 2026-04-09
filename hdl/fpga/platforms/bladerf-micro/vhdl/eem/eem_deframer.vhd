library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity eem_deframer is
    port (
        clock            : in  std_logic;
        reset            : in  std_logic;
        eem_data_in      : in  std_logic_vector(31 downto 0);
        eem_data_valid   : in  std_logic;
        eth_data_out     : out std_logic_vector(31 downto 0);
        eth_data_valid   : out std_logic;
        eth_packet_start : out std_logic;
        eth_packet_end   : out std_logic;
        eth_packet_empty : out std_logic_vector(1 downto 0)
    );
end entity;

architecture arch of eem_deframer is

    constant MAX_EEM_ITEM_BYTES : natural := 2047;
    constant OUTPUT_QUEUE_DEPTH : natural := 4;

    type parser_state_t is (READ_HEADER_LO, READ_HEADER_HI, SKIP_ITEM, FORWARD_DATA);
    type word_queue_t is array (0 to OUTPUT_QUEUE_DEPTH - 1) of std_logic_vector(31 downto 0);
    type flag_queue_t is array (0 to OUTPUT_QUEUE_DEPTH - 1) of std_logic;
    type empty_queue_t is array (0 to OUTPUT_QUEUE_DEPTH - 1) of std_logic_vector(1 downto 0);

    function get_byte(word : std_logic_vector(31 downto 0);
                      index : natural) return std_logic_vector is
    begin
        case index is
            when 0 => return word(7 downto 0);
            when 1 => return word(15 downto 8);
            when 2 => return word(23 downto 16);
            when others => return word(31 downto 24);
        end case;
    end function;

    function set_byte(word  : std_logic_vector(31 downto 0);
                      index : natural;
                      byte  : std_logic_vector(7 downto 0)) return std_logic_vector is
        variable next_word : std_logic_vector(31 downto 0) := word;
    begin
        case index is
            when 0 => next_word(7 downto 0)   := byte;
            when 1 => next_word(15 downto 8)  := byte;
            when 2 => next_word(23 downto 16) := byte;
            when others => next_word(31 downto 24) := byte;
        end case;

        return next_word;
    end function;

    signal parser_state          : parser_state_t := READ_HEADER_LO;
    signal header_lo             : std_logic_vector(7 downto 0) := (others => '0');
    signal item_bytes_remaining  : integer range 0 to MAX_EEM_ITEM_BYTES := 0;
    signal frame_bytes_remaining : integer range 0 to MAX_EEM_ITEM_BYTES := 0;
    signal out_word_accum        : std_logic_vector(31 downto 0) := (others => '0');
    signal out_word_fill         : integer range 0 to 3 := 0;
    signal next_word_is_start    : std_logic := '0';
    signal last_word_slot_valid  : std_logic := '0';
    signal last_word_slot        : integer range 0 to OUTPUT_QUEUE_DEPTH - 1 := 0;

    signal queue_words           : word_queue_t := (others => (others => '0'));
    signal queue_start           : flag_queue_t := (others => '0');
    signal queue_end             : flag_queue_t := (others => '0');
    signal queue_empty           : empty_queue_t := (others => (others => '0'));
    signal queue_head            : integer range 0 to OUTPUT_QUEUE_DEPTH - 1 := 0;
    signal queue_tail            : integer range 0 to OUTPUT_QUEUE_DEPTH - 1 := 0;
    signal queue_count           : integer range 0 to OUTPUT_QUEUE_DEPTH := 0;

begin

    process(clock, reset)
        variable header_value      : std_logic_vector(15 downto 0);
        variable item_length       : integer range 0 to MAX_EEM_ITEM_BYTES;
        variable byte_in           : std_logic_vector(7 downto 0);
        variable queue_words_v     : word_queue_t;
        variable queue_start_v     : flag_queue_t;
        variable queue_end_v       : flag_queue_t;
        variable queue_empty_v     : empty_queue_t;
        variable queue_head_v      : integer range 0 to OUTPUT_QUEUE_DEPTH - 1;
        variable queue_tail_v      : integer range 0 to OUTPUT_QUEUE_DEPTH - 1;
        variable queue_count_v     : integer range 0 to OUTPUT_QUEUE_DEPTH;
        variable parser_state_v    : parser_state_t;
        variable header_lo_v       : std_logic_vector(7 downto 0);
        variable item_bytes_v      : integer range 0 to MAX_EEM_ITEM_BYTES;
        variable frame_bytes_v     : integer range 0 to MAX_EEM_ITEM_BYTES;
        variable out_word_v        : std_logic_vector(31 downto 0);
        variable out_fill_v        : integer range 0 to 3;
        variable next_start_v      : std_logic;
        variable last_slot_valid_v : std_logic;
        variable last_slot_v       : integer range 0 to OUTPUT_QUEUE_DEPTH - 1;
        variable push_slot         : integer range 0 to OUTPUT_QUEUE_DEPTH - 1;

        procedure enqueue_word(
            word_value   : in std_logic_vector(31 downto 0);
            start_value  : in std_logic;
            end_value    : in std_logic;
            empty_value  : in std_logic_vector(1 downto 0)
        ) is
        begin
            if queue_count_v < OUTPUT_QUEUE_DEPTH then
                push_slot := queue_tail_v;
                queue_words_v(push_slot) := word_value;
                queue_start_v(push_slot) := start_value;
                queue_end_v(push_slot)   := end_value;
                queue_empty_v(push_slot) := empty_value;
                queue_tail_v             := (queue_tail_v + 1) mod OUTPUT_QUEUE_DEPTH;
                queue_count_v            := queue_count_v + 1;
                last_slot_v              := push_slot;
                last_slot_valid_v        := '1';
            end if;
        end procedure;

        procedure finalize_frame is
        begin
            if out_fill_v > 0 then
                enqueue_word(
                    word_value  => out_word_v,
                    start_value => next_start_v,
                    end_value   => '1',
                    empty_value => std_logic_vector(to_unsigned(4 - out_fill_v, 2))
                );
                out_word_v        := (others => '0');
                out_fill_v        := 0;
                next_start_v      := '0';
                last_slot_valid_v := '0';
            elsif last_slot_valid_v = '1' then
                queue_end_v(last_slot_v)   := '1';
                queue_empty_v(last_slot_v) := "00";
                last_slot_valid_v          := '0';
            end if;
        end procedure;
    begin
        if reset = '1' then
            eth_data_out       <= (others => '0');
            eth_data_valid     <= '0';
            eth_packet_start   <= '0';
            eth_packet_end     <= '0';
            eth_packet_empty   <= (others => '0');

            parser_state       <= READ_HEADER_LO;
            header_lo          <= (others => '0');
            item_bytes_remaining <= 0;
            frame_bytes_remaining <= 0;
            out_word_accum     <= (others => '0');
            out_word_fill      <= 0;
            next_word_is_start <= '0';
            last_word_slot_valid <= '0';
            last_word_slot     <= 0;

            queue_words        <= (others => (others => '0'));
            queue_start        <= (others => '0');
            queue_end          <= (others => '0');
            queue_empty        <= (others => (others => '0'));
            queue_head         <= 0;
            queue_tail         <= 0;
            queue_count        <= 0;
        elsif rising_edge(clock) then
            eth_data_out       <= (others => '0');
            eth_data_valid     <= '0';
            eth_packet_start   <= '0';
            eth_packet_end     <= '0';
            eth_packet_empty   <= (others => '0');

            queue_words_v      := queue_words;
            queue_start_v      := queue_start;
            queue_end_v        := queue_end;
            queue_empty_v      := queue_empty;
            queue_head_v       := queue_head;
            queue_tail_v       := queue_tail;
            queue_count_v      := queue_count;
            parser_state_v     := parser_state;
            header_lo_v        := header_lo;
            item_bytes_v       := item_bytes_remaining;
            frame_bytes_v      := frame_bytes_remaining;
            out_word_v         := out_word_accum;
            out_fill_v         := out_word_fill;
            next_start_v       := next_word_is_start;
            last_slot_valid_v  := last_word_slot_valid;
            last_slot_v        := last_word_slot;

            if queue_count_v > 0 then
                eth_data_out     <= queue_words_v(queue_head_v);
                eth_data_valid   <= '1';
                eth_packet_start <= queue_start_v(queue_head_v);
                eth_packet_end   <= queue_end_v(queue_head_v);
                eth_packet_empty <= queue_empty_v(queue_head_v);

                queue_head_v     := (queue_head_v + 1) mod OUTPUT_QUEUE_DEPTH;
                queue_count_v    := queue_count_v - 1;
            end if;

            if eem_data_valid = '1' then
                for i in 0 to 3 loop
                    byte_in := get_byte(eem_data_in, i);

                    case parser_state_v is
                        when READ_HEADER_LO =>
                            header_lo_v    := byte_in;
                            parser_state_v := READ_HEADER_HI;

                        when READ_HEADER_HI =>
                            header_value := byte_in & header_lo_v;

                            if header_value = x"0000" then
                                parser_state_v := READ_HEADER_LO;
                            elsif header_value(15) = '1' then
                                if header_value(14) = '0' then
                                    item_length := to_integer(unsigned(header_value(10 downto 0)));
                                    item_bytes_v := item_length;
                                    if item_length = 0 then
                                        parser_state_v := READ_HEADER_LO;
                                    else
                                        parser_state_v := SKIP_ITEM;
                                    end if;
                                else
                                    parser_state_v := READ_HEADER_LO;
                                end if;
                            else
                                item_length := to_integer(unsigned(header_value(13 downto 0)));

                                if item_length = 0 then
                                    parser_state_v := READ_HEADER_LO;
                                else
                                    item_bytes_v := item_length;
                                    if item_length > 4 then
                                        frame_bytes_v  := item_length - 4;
                                        next_start_v   := '1';
                                        parser_state_v := FORWARD_DATA;
                                        last_slot_valid_v := '0';
                                    else
                                        frame_bytes_v  := 0;
                                        next_start_v   := '0';
                                        parser_state_v := SKIP_ITEM;
                                    end if;
                                end if;
                            end if;

                        when SKIP_ITEM =>
                            if item_bytes_v > 0 then
                                item_bytes_v := item_bytes_v - 1;
                            end if;

                            if item_bytes_v = 0 then
                                parser_state_v := READ_HEADER_LO;
                            end if;

                        when FORWARD_DATA =>
                            if frame_bytes_v > 0 then
                                out_word_v := set_byte(out_word_v, out_fill_v, byte_in);

                                if out_fill_v = 3 then
                                    enqueue_word(
                                        word_value  => out_word_v,
                                        start_value => next_start_v,
                                        end_value   => '0',
                                        empty_value => "00"
                                    );

                                    out_word_v   := (others => '0');
                                    out_fill_v   := 0;
                                    next_start_v := '0';
                                else
                                    out_fill_v := out_fill_v + 1;
                                end if;

                                frame_bytes_v := frame_bytes_v - 1;
                            end if;

                            if item_bytes_v > 0 then
                                item_bytes_v := item_bytes_v - 1;
                            end if;

                            if item_bytes_v = 0 then
                                parser_state_v := READ_HEADER_LO;
                                finalize_frame;
                            end if;
                    end case;
                end loop;
            end if;

            parser_state          <= parser_state_v;
            header_lo             <= header_lo_v;
            item_bytes_remaining  <= item_bytes_v;
            frame_bytes_remaining <= frame_bytes_v;
            out_word_accum        <= out_word_v;
            out_word_fill         <= out_fill_v;
            next_word_is_start    <= next_start_v;
            last_word_slot_valid  <= last_slot_valid_v;
            last_word_slot        <= last_slot_v;

            queue_words           <= queue_words_v;
            queue_start           <= queue_start_v;
            queue_end             <= queue_end_v;
            queue_empty           <= queue_empty_v;
            queue_head            <= queue_head_v;
            queue_tail            <= queue_tail_v;
            queue_count           <= queue_count_v;
        end if;
    end process;

end architecture;