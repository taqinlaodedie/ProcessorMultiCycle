library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity Reg32 is port(
	DATAIN   : in std_logic_vector(31 downto 0);
	CLK, RST : in std_logic;
	DATAOUT  : out std_logic_vector(31 downto 0));
end Reg32;

architecture RTL of Reg32 is
	signal DATA : std_logic_vector(31 downto 0);
begin
	DATAOUT <= DATA;
	
	process(RST, CLK)
	begin
		if RST = '1' then
			DATA <= X"00000000";
		elsif rising_edge(CLK) then
			DATA <= DATAIN;
		end if;
	end process;
end architecture RTL;