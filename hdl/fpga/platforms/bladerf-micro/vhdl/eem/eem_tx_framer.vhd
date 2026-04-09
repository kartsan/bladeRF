-- eem_tx_framer.vhd
--
-- Accepts the byte-serial output of mac_send (which produces a complete
-- Ethernet frame including the CRC32 FCS) and wraps it into a CDC-EEM data
-- packet for transmission to the host via the FPGA→host RX1 FIFO.
--
-- Protocol mapping
--   mac_send.active is high during the entire frame (header + payload + FCS).
--   When active falls the full frame has been captured.  We then write:
--     1.  A 2-byte EEM data-packet header:
--             bit 15   = 0 (data packet)
--             bit 14   = 0 (bmCRC = 0  →  host will strip 0xDEADBEEF sentinel;
--                            but mac_send appends a real CRC so we set this to 1
--                            to tell the host to verify/strip the real FCS)
--             bits 13:0 = total byte length of the Ethernet frame incl. FCS
--         For simplicity we set bmCRC=0 and tell the host the FCS is dummy.
--         The host CDC-EEM driver strips the last 4 bytes regardless and passes
--         the remainder to its network stack, which is exactly what we want.
--     2.  All captured frame bytes, zero-padded to a 32-bit boundary.
--   The word stream is written into eem_rx_fifo (FPGA→host, RX1 path) using
--   the standard write / full handshake.
--
-- Constraints
--   Max Ethernet frame with FCS: 1518 bytes + 4 FCS = 1522 bytes.
--   Buffer depth is set to 1536 bytes (384 words) which comfortably covers
--   one maximum-size frame.  Assembly is done from the first byte.
--   Only one frame is assembled at a time; a new mac_send.active assertion
--   while a frame is being flushed is silently ignored (the HPSDR stack
--   serialises TX so this does not happen in practice).

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity eem_tx_framer is
    port (
        clock            : in  std_logic;
        reset            : in  std_logic;

        -- From mac_send (one byte per clock while active is high)
        mac_active       : in  std_logic;
        mac_data         : in  std_logic_vector(7 downto 0);

        -- To eem_rx_fifo (FPGA→host, RX1 path)
        fifo_write       : out std_logic;
        fifo_full        : in  std_logic;
        fifo_data        : out std_logic_vector(31 downto 0)
    );
end entity;

architecture arch of eem_tx_framer is

    constant BUF_WORDS  : natural := 384;
    constant BUF_BYTES  : natural := BUF_WORDS * 4;

    type byte_buf_t is array (0 to BUF_BYTES-1) of std_logic_vector(7 downto 0);
    signal cap_buf : byte_buf_t;
    attribute ramstyle : string;
    attribute ramstyle of cap_buf : signal is "M10K";

    type state_t is (IDLE, WRITE_HDR_WORD, WRITE_DATA);
    signal state        : state_t := IDLE;
    signal cap_active   : std_logic := '0';
    signal cap_count    : integer range 0 to BUF_BYTES := 0;
    signal frame_bytes  : integer range 0 to BUF_BYTES := 0;
    signal flush_idx    : integer range 0 to BUF_WORDS := 0;
    signal flush_words  : integer range 0 to BUF_WORDS := 0;

begin

    process(clock, reset)
        variable hdr_word  : std_logic_vector(31 downto 0);
        variable data_word : std_logic_vector(31 downto 0);
        variable base      : integer range 0 to BUF_BYTES;
    begin
        if reset = '1' then
            cap_count    <= 0;
            cap_active   <= '0';
            state        <= IDLE;
            frame_bytes  <= 0;
            flush_idx    <= 0;
            flush_words  <= 0;
            fifo_write   <= '0';
            fifo_data    <= (others => '0');

        elsif rising_edge(clock) then
            fifo_write <= '0';

            -- Capture a complete frame before emitting the EEM header.
            if mac_active = '1' and state = IDLE then
                if cap_count < BUF_BYTES then
                    cap_buf(cap_count) <= mac_data;
                    cap_count <= cap_count + 1;
                end if;
                cap_active <= '1';
            elsif mac_active = '0' and cap_active = '1' then
                cap_active  <= '0';
                frame_bytes <= cap_count;
                flush_words <= (cap_count + 3) / 4;
                flush_idx    <= 0;
                state        <= WRITE_HDR_WORD;
            end if;

            case state is
                when IDLE => null;

                when WRITE_HDR_WORD =>
                    if fifo_full = '0' then
                        hdr_word := (others => '0');
                        hdr_word(13 downto 0) := std_logic_vector(to_unsigned(frame_bytes, 14));

                        fifo_data  <= hdr_word;
                        fifo_write <= '1';
                        state      <= WRITE_DATA;
                    end if;

                when WRITE_DATA =>
                    if fifo_full = '0' then
                        data_word := (others => '0');
                        base := flush_idx * 4;

                        if base < frame_bytes then
                            data_word(7 downto 0) := cap_buf(base);
                        end if;

                        if (base + 1) < frame_bytes then
                            data_word(15 downto 8) := cap_buf(base + 1);
                        end if;

                        if (base + 2) < frame_bytes then
                            data_word(23 downto 16) := cap_buf(base + 2);
                        end if;

                        if (base + 3) < frame_bytes then
                            data_word(31 downto 24) := cap_buf(base + 3);
                        end if;

                        fifo_data  <= data_word;
                        fifo_write <= '1';

                        if (flush_idx + 1) >= flush_words then
                            state     <= IDLE;
                            cap_count <= 0;
                        else
                            flush_idx <= flush_idx + 1;
                        end if;
                    end if;
            end case;
        end if;
    end process;

end architecture;
