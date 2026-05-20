library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

entity chip_id_mac is
    port (
        clock     : in  std_logic;
        reset     : in  std_logic;
        local_mac : out std_logic_vector(47 downto 0)
    );
end entity;

architecture arch of chip_id_mac is

    -- Direct Cyclone V chip ID primitive wrapper (replaces Intel's
    -- altchip_id IP wrapper; same 64-bit output, no qip/sip catalog files).
    component cv_chip_id_reader is
        port (
            clock      : in  std_logic;
            reset      : in  std_logic;
            chip_id    : out std_logic_vector(63 downto 0);
            data_valid : out std_logic
        );
    end component;

    signal chip_id_data  : std_logic_vector(63 downto 0) := (others => '0');
    signal chip_id_valid : std_logic := '0';
    signal mac_reg       : std_logic_vector(47 downto 0) := x"12_22_33_44_55_66";
begin

    U_chip_id_reader : cv_chip_id_reader
        port map (
            clock      => clock,
            reset      => reset,
            chip_id    => chip_id_data,
            data_valid => chip_id_valid
        );

    process(clock, reset)
        variable fold_a : std_logic_vector(39 downto 0);
        variable fold_b : std_logic_vector(39 downto 0);
        variable tail   : std_logic_vector(39 downto 0);
    begin
        if reset = '1' then
            mac_reg <= x"12_22_33_44_55_66";
        elsif rising_edge(clock) then
            if chip_id_valid = '1' then
                fold_a := chip_id_data(39 downto 0);
                fold_b := chip_id_data(63 downto 24);
                tail   := fold_a xor fold_b;

                -- First octet 0x02 marks a locally administered unicast MAC.
                mac_reg <= x"02" & tail;
            end if;
        end if;
    end process;

    local_mac <= mac_reg;

end architecture;