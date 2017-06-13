--
-- Z80 compatible microprocessor core, preudo-asynchronous top level
--
-- Version : 0247pa
--
-- Copyright (c) 2001-2002 Daniel Wallner (jesus@opencores.org)
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- Redistributions in synthesized form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
--
-- Neither the name of the author nor the names of other contributors may
-- be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- Please report bugs to the author, but before you do so, please
-- make sure that this is not a derivative work and that
-- you have the latest version of this file.
--
-- The latest version of this file can be found at:
--	http://www.opencores.org/cvsweb.shtml/t80/
--
-- Limitations :
--
-- File history :
--
--	0208 : First complete release
--
--	0211 : Fixed interrupt cycle
--
--	0235 : Updated for T80 interface change
--
--	0238 : Updated for T80 interface change
--
--	0240 : Updated for T80 interface change
--
--	0242 : Updated for T80 interface change
--
--	0247 : Fixed bus req/ack cycle
--
-- 0247pa: convert to preudo-asynchronous model with original Z80 timings (by Sorgelig).
--

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.T80_Pack.all;

entity T80pa is
	generic(
		Mode : integer := 0	-- 0 => Z80, 1 => Fast Z80, 2 => 8080, 3 => GB
	);
	port(
		RESET_n		: in  std_logic;
		CLK			: in  std_logic;
		CEN_p       : in  std_logic;
		CEN_n       : in  std_logic;
		WAIT_n		: in  std_logic;
		INT_n			: in  std_logic;
		NMI_n			: in  std_logic;
		BUSRQ_n		: in  std_logic;
		M1_n			: out std_logic;
		MREQ_n		: out std_logic;
		IORQ_n		: out std_logic;
		RD_n			: out std_logic;
		WR_n			: out std_logic;
		RFSH_n		: out std_logic;
		HALT_n		: out std_logic;
		BUSAK_n		: out std_logic;
		A				: out std_logic_vector(15 downto 0);
		DI				: in  std_logic_vector(7 downto 0);
		DO				: out std_logic_vector(7 downto 0);
		REG			: out std_logic_vector(207 downto 0) -- IY, HL', DE', BC', IX, HL, DE, BC, PC, SP, R, I, F', A', F, A
	);
end T80pa;

architecture rtl of T80pa is

	signal IntCycle_n		: std_logic;
	signal IORQ				: std_logic;
	signal NoRead			: std_logic;
	signal Write			: std_logic;
	signal MREQ				: std_logic;
	signal MReq_Inhibit	: std_logic;
	signal Req_Inhibit	: std_logic;
	signal RD				: std_logic;
	signal BUSAK			: std_logic;
	signal DI_Reg			: std_logic_vector (7 downto 0);	-- Input synchroniser
	signal Wait_s			: std_logic;
	signal MCycle			: std_logic_vector(2 downto 0);
	signal TState			: std_logic_vector(2 downto 0);
	signal CEN_ne			: std_logic;

begin

	MREQ_n <= not MREQ or (Req_Inhibit and MReq_Inhibit);
	RD_n <= not RD or Req_Inhibit;
	BUSAK_n <= BUSAK;

	u0 : T80
		generic map(
			Mode => Mode,
			IOWait => 1)
		port map(
			CEN => CEN_p,
			M1_n => M1_n,
			IORQ => IORQ,
			NoRead => NoRead,
			Write => Write,
			RFSH_n => RFSH_n,
			HALT_n => HALT_n,
			WAIT_n => Wait_s,
			INT_n => INT_n,
			NMI_n => NMI_n,
			RESET_n => RESET_n,
			BUSRQ_n => BUSRQ_n,
			BUSAK_n => BUSAK,
			CLK_n => CLK,
			A => A,
			DInst => DI,
			DI => DI_Reg,
			DO => DO,
			REG => REG,
			MC => MCycle,
			TS => TState,
			IntCycle_n => IntCycle_n);

	process(CLK)
	begin
		if CLK'event and CLK = '1' then
			if RESET_n = '0' then
				WR_n <= '1';
				Req_Inhibit <= '0';
				MReq_Inhibit <= '0';
				RD <= '0';
				IORQ_n <= '1';
				MREQ <= '0';
				DI_Reg <= "00000000";
				Wait_s <= '1';
				CEN_ne <= '0';
			elsif CEN_p = '1' then
				WR_n <= '1';
				if TState = "001" then
					WR_n <= not Write;
				end if;
				if MCycle = "001" and TState = "010" then
					Req_Inhibit <= '1';
				else
					Req_Inhibit <= '0';
				end if;
				CEN_ne <= '1';
			elsif CEN_n = '1' and CEN_ne = '1' then
				Wait_s <= WAIT_n;
				if TState = "011" and BUSAK = '1' then
					DI_Reg <= DI;
				end if;
				if MCycle = "001" and TState = "010" then
					MReq_Inhibit <= '1';
				else
					MReq_Inhibit <= '0';
				end if;
				if MCycle = "001" then
					if TState = "001" then
						RD <= IntCycle_n;
						MREQ <= IntCycle_n;
						IORQ_n <= IntCycle_n;
					end if;
					if TState = "011" then
						RD <= '0';
						IORQ_n <= '1';
						MREQ <= '1';
					end if;
					if TState = "100" then
						MREQ <= '0';
					end if;
				else
					if TState = "001" and NoRead = '0' then
						RD <= not Write;
						IORQ_n <= not IORQ;
						MREQ <= not IORQ;
					end if;
					if TState = "011" then
						RD <= '0';
						IORQ_n <= '1';
						MREQ <= '0';
					end if;
				end if;
			end if;
		end if;
	end process;

end;
