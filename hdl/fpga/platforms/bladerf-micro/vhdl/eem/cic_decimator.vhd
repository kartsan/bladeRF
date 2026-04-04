library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity cic_decimator is
    generic (
        STAGES       : integer := 3;
        IN_WIDTH     : integer := 16;
        -- For M=640 (30.72MHz to 48kHz), growth is ~30 bits. 
        -- 16 + 30 = 46 bits internal.
        ACCUM_WIDTH  : integer := 48 
    );
    port (
        clk          : in  std_logic;
        reset        : in  std_logic;
        
        -- Configuration
        decimation_m : in  unsigned(15 downto 0); -- The M value
        
        -- Input (High Speed)
        data_in      : in  std_logic_vector(IN_WIDTH-1 downto 0);
        data_valid_in: in  std_logic;
        
        -- Output (Low Speed)
        data_out     : out std_logic_vector(IN_WIDTH-1 downto 0);
        data_valid_out: out std_logic
    );
end entity;

architecture rtl of cic_decimator is

    -- Integrator signals
    type accum_array is array (0 to STAGES) of signed(ACCUM_WIDTH-1 downto 0);
    signal integrators : accum_array := (others => (others => '0'));
    
    -- Decimation signals
    signal m_counter   : unsigned(15 downto 0) := (others => '0');
    signal decimate_en : std_logic := '0';
    
    -- Comb signals
    signal comb_input  : signed(ACCUM_WIDTH-1 downto 0) := (others => '0');
    type comb_array is array (0 to STAGES) of signed(ACCUM_WIDTH-1 downto 0);
    signal combs       : comb_array := (others => (others => '0'));
    signal combs_prev  : comb_array := (others => (others => '0'));

    -- Leaky Integrator DC Removal signals
    signal dc_removed_signed : signed(IN_WIDTH-1 downto 0);
    signal cic_input_signed  : signed(IN_WIDTH-1 downto 0);
    signal dc_accum          : signed(IN_WIDTH+12-1 downto 0) := (others => '0'); -- 12-bit shift for Alpha
    constant DC_SHIFT        : integer := 12;

    -- Defined her for now
    constant ctrl_dc_rm_bypass : bit := '1';
begin

    -- 0. Process for DC Offset Estimation and Removal
    process(clk)
        variable error : signed(IN_WIDTH-1 downto 0);
    begin
        if rising_edge(clk) then
            if reset = '1' then
                dc_accum <= (others => '0');
                dc_removed_signed <= (others => '0');
            elsif data_valid_in = '1' then
                -- 1. Calculate the 'Average' (The DC Component)
                -- We shift the accumulator to get the current DC estimate
                error := signed(data_in) - dc_accum(dc_accum'high downto DC_SHIFT);
                
                -- 2. Update the accumulator (Leaky integration)
                dc_accum <= dc_accum + error;
                
                -- 3. Subtract DC from the input
                dc_removed_signed <= signed(data_in) - dc_accum(dc_accum'high downto DC_SHIFT);
            end if;
        end if;
    end process;

    -- The Bypass Multiplexer
    -- ctrl_dc_rm_bypass: '1' = Raw Data, '0' = Filtered Data
    cic_input_signed <= signed(data_in) when ctrl_dc_rm_bypass = '1' else dc_removed_signed;

    -- 1. Integrator Section (High Frequency)
    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                integrators <= (others => (others => '0'));
            elsif data_valid_in = '1' then
                integrators(0) <= resize(signed(cic_input_signed), ACCUM_WIDTH);
                for i in 1 to STAGES loop
                    integrators(i) <= integrators(i) + integrators(i-1);
                end loop;
            end if;
        end if;
    end process;

    -- 2. Decimator (Downsampling Switch)
    process(clk)
    begin
        if rising_edge(clk) then
            decimate_en <= '0';
            if reset = '1' then
                m_counter <= (others => '0');
            elsif data_valid_in = '1' then
                if m_counter >= decimation_m - 1 then
                    m_counter <= (others => '0');
                    decimate_en <= '1';
                    comb_input <= integrators(STAGES);
                else
                    m_counter <= m_counter + 1;
                end if;
            end if;
        end if;
    end process;

    -- 3. Comb Section (Low Frequency)
    process(clk)
    begin
        if rising_edge(clk) then
            data_valid_out <= '0';
            if reset = '1' then
                combs <= (others => (others => '0'));
                combs_prev <= (others => (others => '0'));
            elsif decimate_en = '1' then
                combs(0) <= comb_input;
                for i in 1 to STAGES loop
                    combs(i)      <= combs(i-1) - combs_prev(i-1);
                    combs_prev(i-1) <= combs(i-1);
                end loop;
                data_valid_out <= '1';
            end if;
        end if;
    end process;

    -- 4. Output Scaling (Truncate or Rounding)
    -- Note: Real designs often use a barrel shifter here to compensate for 
    -- gain changes when M varies.
    -- data_out <= std_logic_vector(rescale_or_truncate(combs(STAGES))); -- Simplified logic
    data_out <= std_logic_vector(combs(STAGES));

end architecture;
