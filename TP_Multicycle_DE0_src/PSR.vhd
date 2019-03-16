library IEEE;
use IEEE.Std_logic_1164.all;
use IEEE.numeric_std.all;

entity PSR is
	port (DataIn: in std_logic_vector(31 downto 0);
         CLK, RST, WE: in std_logic;
			DataOut: out std_logic_vector(31 downto 0));
end entity PSR;

architecture RTL of PSR is
signal reg: std_logic_vector(31 downto 0) := (others => '0');
begin

  DataOut <= reg;

process(CLK,RST)
begin
   if RST = '1' then 
      reg <= (others => '0');
   elsif CLK = '1' and CLK'event then 
		if WE = '1' then
			reg <= DataIn;
		end if;
	end if;

end process;

end architecture;
