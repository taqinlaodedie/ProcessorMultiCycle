library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity MUX4 is port(
	A, B, C, D : in std_logic_vector(31 downto 0);
	COM : in std_logic_vector(1 downto 0);
	S : out std_logic_vector(31 downto 0));
end entity;

architecture RTL of MUX4 is
begin
	process(A, B, C, D, COM)
	begin
		if COM = "00" then
			S <= A;
		elsif COM = "01" then
			S <= B;
		elsif COM = "10" then
			S <= C;
		else 
			S <= D;
		end if;
	end process;
end architecture RTL;