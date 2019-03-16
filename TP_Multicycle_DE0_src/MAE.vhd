library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity MAE is port (
	clk, rst, irq, N  									: in  std_logic;
	inst_mem,inst_reg 									: in std_logic_vector(31 downto 0);
   irq_serv,AdrSel,memRdEn,memWrEn,irWrEn,WSel,RegWrEn,AluSelA,
	CpsrSel,CpsrWrEn,SpsrWrEn,PCWrEn,LRWrEn,ResWrEn : out std_logic;
	AluSelB,AluOP,PCSel									: out std_logic_vector(1 downto 0));
end entity MAE;

architecture behav of MAE is
	type enum_instruction is (MOV, ADDi, ADDr, CMP, LDR, STR, BAL, BLT, BX, NOP);
	signal instr_courante: enum_instruction; 
	signal state: integer;
	signal isr: std_logic; 
begin
	process(clk,rst)
	begin
		if rst = '1' then --reset asynchrone
			isr <= '0';
			state <= 0;
		elsif rising_edge(clk) then
			case state is
			when 0 =>Irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '0';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '1';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '0';
						ALUSelB <= "00";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '0';
						state <= 1;
			when 1 =>irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '0';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '1';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '0';
						ALUSelB <= "00";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '0';
						if (irq='1' and isr='0') then
							state <= 15;
							isr <= '1'; 
						elsif (isr='1' and (inst_mem(31 downto 24) = "11101011")) then	
							instr_courante <= BX;
							state <= 17;
							isr <= '0'; 
						else                     
							if inst_mem(31 downto 24) = "11101010" then
								instr_courante <= BAL;  
								state <= 2;
							elsif inst_mem(31 downto 24) = "10111010" then
								instr_courante <= BLT;  
								if N = '1' then 
									state <= 2; 
								else 
									state <= 3;
								end if;             
							else 
								case inst_mem(31 downto 20) is
									when "111000101000" => instr_courante <= ADDi;  --E28
									when "111000001000" => instr_courante <= ADDr;  --E08
									when "111000111010" => instr_courante <= MOV;   --E3A
									when "111000110101" => instr_courante <= CMP;   --E35
									when "111001100001" => instr_courante <= LDR;   --E61
									when "111001100000" => instr_courante <= STR;   --E60
									when others => instr_courante <= NOP;
								end case;
								state <= 4;
							end if;
						end if;        
			when 2 =>irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '1';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '0';
						ALUSelB <= "10";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '0';
						state <= 0;
			when 3 =>  irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '1';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '0';
						ALUSelB <= "11";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '0';
						state <= 0;
			when 4 =>irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '1';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '0';
						ALUSelB <= "11";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '0';
						state <= 5;
			when 5 =>PCWrEn <= '0';
						case instr_courante is
						when ADDr => state <= 7;
						when MOV => state <= 8;
						when CMP => state <= 9;
						when others => state <= 6; --LDR,STR,ADDi
						end case;
			when 6 =>irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '0';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '1';
						ALUSelB <= "01";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '1';
						case instr_courante is
						when LDR => state <= 10;
						when STR => state <= 11;
						when others => state <= 12;   
						end case;
			when 7 =>irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '0';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '1';
						ALUSelB <= "00";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '1';
						state <= 12;
			when 8 =>irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '0';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '0';
						ALUSelB <= "01";
						ALUOP <= "01";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '1'; 
						state <= 12;
			when 9 =>irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '0';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '1';
						ALUSelB <= "01";
						ALUOP <= "10";
						CPSRSel <= '0';
						CPSRWrEn <= '1';
						SPSRWrEn <= '0';
						ResWrEn <= '0';
						state <= 0;
			when 10 =>irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '0';
						LRWrEn <= '0';
						AdrSel <= '1';
						MemRdEn <= '1';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '0';
						ALUSelB <= "00";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '1'; 
						state <= 13;
			when 11 =>irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '0';
						LRWrEn <= '0';
						AdrSel <= '1';
						MemRdEn <= '0';
						MemWrEn <= '1';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '0';
						ALUSelB <= "00";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '1'; 
						state <= 0;
			when 12 =>irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '0';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '1';
						RegWrEn <= '1';
						ALUSelA <= '0';
						ALUSelB <= "00";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '1';
						state <= 0;
			when 13 =>state <= 14;
			when 14 =>irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '0';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '1';
						ALUSelA <= '0';
						ALUSelB <= "00";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '0';
						state <= 0;		
			--interruptions
			when 15 =>irq_serv <= '0';
						PCSel <= "00";
						PCWrEn <= '0';
						LRWrEn <= '1';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '0';
						ALUSelB <= "00";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '1';
						ResWrEn <= '0';
						state <= 16;
			when 16 =>irq_serv <= '0';
						PCSel <= "11";
						PCWrEn <= '1';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '0';
						ALUSelB <= "00";
						ALUOP <= "00";
						CPSRSel <= '0';
						CPSRWrEn <= '0';
						SPSRWrEn <= '0';
						ResWrEn <= '0';
						isr <= '1';
						state <= 0;
			when others =>	irq_serv <= '1';
						PCSel <= "10";
						PCWrEn <= '1';
						LRWrEn <= '0';
						AdrSel <= '0';
						MemRdEn <= '0';
						MemWrEn <= '0';
						IRWrEn <= '0';
						WSel <= '0';
						RegWrEn <= '0';
						ALUSelA <= '0';
						ALUSelB <= "00";
						ALUOP <= "00";
						CPSRSel <= '1';
						CPSRWrEn <= '1';
						SPSRWrEn <= '0';
						ResWrEn <= '0';
						isr <= '0';
						state <= 0;
			end case;
		end if;
	end process;
end architecture;