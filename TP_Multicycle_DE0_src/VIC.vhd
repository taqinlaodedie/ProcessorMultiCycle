library IEEE;
use IEEE.Std_logic_1164.all;
use IEEE.numeric_std.all;

entity VIC is port (
	clk, rst, IRQ_SERV, IRQ0, IRQ1: in std_logic;
   IRQ									: out std_logic;
   VICPC									: out std_logic_vector(31 downto 0));
end entity VIC;

architecture RTL of VIC is
	signal irq0_memo, irq1_memo, irq0_nv, irq1_nv: std_logic;
	signal irq0_ech, irq1_ech:  std_logic_vector(1 downto 0);
begin
irq <= irq0_memo or irq1_memo;

	process(clk,rst)
	begin
		if rst = '1' then
			irq0_ech <= "00";
			irq1_ech <= "00";
		elsif rising_edge(clk) then
			irq0_ech(0) <= irq0;
			irq1_ech(0) <= irq1;
			irq0_ech(1) <= irq0_ech(0);
			irq1_ech(1) <= irq1_ech(0);
		end if;
	end process;

	process(irq0_ech,irq1_ech,irq_serv,rst)
	begin
		if rst = '1' then 
			irq0_memo <= '0';
			irq1_memo <= '0';
		else
			if (irq0_ech = "01") then
				irq0_memo <= '1';
			elsif (irq1_ech = "01") then
				irq1_memo <= '1';
			elsif irq_serv = '1' then
				irq0_memo <= '0';
				irq1_memo <= '0';
			end if;
		end if;
	end process;

	process(irq0_memo,irq1_memo)
	begin
		if irq0_memo = '1' then
			VICPC <= x"00000009";
		elsif irq1_memo = '1' then
			VICPC <= x"00000015";
		else
			VICPC <= (others => '0');
		end if;  
	end process;
end architecture;
