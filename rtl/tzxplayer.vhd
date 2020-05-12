---------------------------------------------------------------------------------
-- TZX player
-- by György Szombathelyi
-- basic idea for the structure based on c1530 tap player by darfpga
--
---------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use IEEE.STD_LOGIC_ARITH.ALL;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

entity tzxplayer is
generic (
	TZX_MS : integer := 64000       -- CE periods for one milliseconds
);
port(
	clk             : in std_logic;
	ce              : in std_logic;
	restart_tape    : in std_logic; -- keep to 1 to long enough to clear fifo
	                                -- reset tap header bytes skip counter

	host_tap_in     : in std_logic_vector(7 downto 0);  -- 8bits fifo input
	tzx_req         : buffer std_logic;                 -- request for new byte (edge trigger)
	tzx_ack         : in std_logic;                     -- new data available
	loop_start      : out std_logic;                    -- active for one clock if a loop starts
	loop_next       : out std_logic;                    -- active for one clock at the next iteration
	stop            : out std_logic;                    -- tape should be stopped
	stop48k         : out std_logic;                    -- tape should be stopped in 48k mode

	cass_read  : buffer std_logic;   -- tape read signal
	cass_motor : in  std_logic    -- 1 = tape motor is powered
);
end tzxplayer;

architecture struct of tzxplayer is

-- ZX Spectrum
constant NORMAL_PILOT_LEN    : integer := 2168;
constant NORMAL_SYNC1_LEN    : integer := 667;
constant NORMAL_SYNC2_LEN    : integer := 735;
constant NORMAL_ZERO_LEN     : integer := 855;
constant NORMAL_ONE_LEN      : integer := 1710;
constant NORMAL_PILOT_PULSES : integer := 4031;

-- Amstrad CPC
--constant NORMAL_PILOT_LEN    : integer := 2000;
--constant NORMAL_SYNC1_LEN    : integer := 855;
--constant NORMAL_SYNC2_LEN    : integer := 855;
--constant NORMAL_ZERO_LEN     : integer := 855;
--constant NORMAL_ONE_LEN      : integer := 1710;
--constant NORMAL_PILOT_PULSES : integer := 4096;

signal tap_fifo_do    : std_logic_vector(7 downto 0);
signal tick_cnt       : std_logic_vector(16 downto 0);
signal wave_cnt       : std_logic_vector(15 downto 0);
signal wave_period    : std_logic;
signal skip_bytes     : std_logic;
signal playing        : std_logic;  -- 1 = tap or wav file is playing
signal bit_cnt        : std_logic_vector(2 downto 0);

type tzx_state_t is (
	TZX_HEADER,
	TZX_NEWBLOCK,
	TZX_LOOP_START,
	TZX_LOOP_END,
	TZX_PAUSE,
	TZX_PAUSE2,
	TZX_STOP48K,
	TZX_HWTYPE,
	TZX_TEXT,
	TZX_MESSAGE,
	TZX_ARCHIVE_INFO,
	TZX_CUSTOM_INFO,
	TZX_GLUE,
	TZX_TONE,
	TZX_PULSES,
	TZX_DATA,
	TZX_NORMAL,
	TZX_TURBO,
	TZX_PLAY_TONE,
	TZX_PLAY_SYNC1,
	TZX_PLAY_SYNC2,
	TZX_PLAY_TAPBLOCK,
	TZX_PLAY_TAPBLOCK2,
	TZX_PLAY_TAPBLOCK3,
	TZX_PLAY_TAPBLOCK4);

signal tzx_state: tzx_state_t;

signal tzx_offset     : std_logic_vector( 7 downto 0);
signal pause_len      : std_logic_vector(15 downto 0);
signal ms_counter     : std_logic_vector(15 downto 0);
signal pilot_l        : std_logic_vector(15 downto 0);
signal sync1_l        : std_logic_vector(15 downto 0);
signal sync2_l        : std_logic_vector(15 downto 0);
signal zero_l         : std_logic_vector(15 downto 0);
signal one_l          : std_logic_vector(15 downto 0);
signal pilot_pulses   : std_logic_vector(15 downto 0);
signal last_byte_bits : std_logic_vector( 3 downto 0);
signal data_len       : std_logic_vector(23 downto 0);
signal pulse_len      : std_logic_vector(15 downto 0);
signal end_period     : std_logic;
signal cass_motor_D   : std_logic;
signal motor_counter  : std_logic_vector(21 downto 0);
signal loop_iter      : std_logic_vector(15 downto 0);
signal data_len_dword : std_logic_vector(31 downto 0);

begin

tap_fifo_do <= host_tap_in;
process(clk)
begin
  if rising_edge(clk) then
	if restart_tape = '1' then

		tzx_offset <= (others => '0');
		tzx_state <= TZX_HEADER;
		pulse_len <= (others => '0');
		motor_counter <= (others => '0');
		wave_period <= '0';
		playing <= '0';
		tzx_req <= tzx_ack;
		loop_start <= '0';
		loop_next <= '0';
		loop_iter <= (others => '0');

	else

		-- simulate tape motor momentum
		-- don't change the playing state if the motor is switched in 50 ms
		-- Opera Soft K17 protection needs this!
		cass_motor_D <= cass_motor;
		if cass_motor_D /= cass_motor then
			motor_counter <= CONV_STD_LOGIC_VECTOR(50*TZX_MS, motor_counter'length);
		elsif motor_counter /= 0 then
			if ce = '1' then motor_counter <= motor_counter - 1; end if;
		else
			playing <= cass_motor;
		end if;

		if playing = '0' then
			--cass_read <= '1';
		end if;	

		if pulse_len /= 0 then
			if ce = '1' then
				tick_cnt <= tick_cnt + 3500;
				if tick_cnt >= (TZX_MS - 3500) then
					tick_cnt <= tick_cnt - (TZX_MS - 3500);
					wave_cnt <= wave_cnt + 1;
					if wave_cnt = pulse_len then
						wave_cnt <= (others => '0');
						cass_read <= wave_period;
						wave_period <= not wave_period;
						if wave_period = end_period then
							pulse_len <= (others => '0');
						end if;
					end if;
				end if;
			end if;
		else
			tick_cnt <= (others => '0');
			wave_cnt <= (others => '0');
		end if;

		loop_start <= '0';
		loop_next  <= '0';
		stop       <= '0';
		stop48k    <= '0';

		if playing = '1' and pulse_len = 0 and tzx_req = tzx_ack then

			tzx_req <= not tzx_ack; -- default request for new data

			case tzx_state is
			when TZX_HEADER =>
				cass_read <= '1';
				tzx_offset <= tzx_offset + 1;
				if tzx_offset = x"0A" then -- skip 9 bytes, offset lags 1
					tzx_state <= TZX_NEWBLOCK;
				end if;

			when TZX_NEWBLOCK =>
				tzx_offset <= (others=>'0');
				ms_counter <= (others=>'0');
				case tap_fifo_do is
					when x"10" => tzx_state <= TZX_NORMAL;
					when x"11" => tzx_state <= TZX_TURBO;
					when x"12" => tzx_state <= TZX_TONE;
					when x"13" => tzx_state <= TZX_PULSES;
					when x"14" => tzx_state <= TZX_DATA;
					when x"15" => null; -- Direct Recoding Block (not implemented)
					when x"18" => null; -- CSW recording (not implemented)
					when x"19" => null; -- Generalized data block (not implemented)
					when x"20" => tzx_state <= TZX_PAUSE;
					when x"21" => tzx_state <= TZX_TEXT; -- Group start
					when x"22" => null; -- Group end
					when x"23" => null; -- Jump to block (not implemented)
					when x"24" => tzx_state <= TZX_LOOP_START;
					when x"25" => tzx_state <= TZX_LOOP_END;
					when x"26" => null; -- Call sequence (not implemented)
					when x"27" => null; -- Return from sequence (not implemented)
					when x"28" => null; -- Select block (not implemented)
					when x"2A" => tzx_state <= TZX_STOP48K;
					when x"2B" => null; -- Set signal level (not implemented)
					when x"30" => tzx_state <= TZX_TEXT;
					when x"31" => tzx_state <= TZX_MESSAGE;
					when x"32" => tzx_state <= TZX_ARCHIVE_INFO;
					when x"33" => tzx_state <= TZX_HWTYPE;
					when x"35" => tzx_state <= TZX_CUSTOM_INFO;
					when x"5A" => tzx_state <= TZX_GLUE;
					when others => null;
				end case;

			when TZX_LOOP_START =>
				tzx_offset <= tzx_offset + 1;
				if    tzx_offset = x"00" then loop_iter( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"01" then
					loop_iter(15 downto  8) <= tap_fifo_do;
					tzx_state <= TZX_NEWBLOCK;
					loop_start <= '1';
				end if;

			when TZX_LOOP_END =>
				if loop_iter > 1 then
					loop_iter <= loop_iter - 1;
					loop_next <= '1';
				else
					tzx_req <= tzx_ack; -- don't request new byte
				end if;
				tzx_state <= TZX_NEWBLOCK;

			when TZX_PAUSE =>
				tzx_offset <= tzx_offset + 1;
				if tzx_offset = x"00" then 
					pause_len(7 downto 0) <= tap_fifo_do;
				elsif tzx_offset = x"01" then
					pause_len(15 downto 8) <= tap_fifo_do;
					tzx_state <= TZX_PAUSE2;
					if pause_len(7 downto 0) = 0 and tap_fifo_do = 0 then
						stop <= '1';
					end if;
				end if;

			when TZX_PAUSE2 =>
				tzx_req <= tzx_ack; -- don't request new byte
				if ms_counter /= 0 then if ce = '1' then ms_counter <= ms_counter - 1; end if;
				elsif pause_len /= 0 then
					pause_len <= pause_len - 1;
					ms_counter <= conv_std_logic_vector(TZX_MS, 16);
				else
					tzx_state <= TZX_NEWBLOCK;
				end if;

			when TZX_STOP48K =>
				tzx_offset <= tzx_offset + 1;
				if tzx_offset = x"03" then
					stop48k <= '1';
					tzx_state <= TZX_NEWBLOCK;
				end if;

			when TZX_HWTYPE =>
				tzx_offset <= tzx_offset + 1;
				-- 0, 1-3, 1-3, ...
				if    tzx_offset = x"00" then data_len( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"03" then
					if data_len(7 downto 0) = x"01" then
						tzx_state <= TZX_NEWBLOCK;
					else
						data_len(7 downto 0) <= data_len(7 downto 0) - 1;
						tzx_offset <= x"01";
					end if;
				end if;

			when TZX_MESSAGE =>
				-- skip display time, then then same as TEXT DESRCRIPTION
				tzx_state <= TZX_TEXT;

			when TZX_TEXT =>
				tzx_offset <= tzx_offset + 1;
				if    tzx_offset = x"00" then data_len( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = data_len(7 downto 0) then
						tzx_state <= TZX_NEWBLOCK;
				end if;

			when TZX_ARCHIVE_INFO =>
				tzx_offset <= tzx_offset + 1;
				if    tzx_offset = x"00" then data_len( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"01" then data_len(15 downto  8) <= tap_fifo_do;
				else
					tzx_offset <= x"02";
					data_len <= data_len - 1;
					if data_len = 1 then
						tzx_state <= TZX_NEWBLOCK;
					end if;
				end if;

			when TZX_CUSTOM_INFO =>
				tzx_offset <= tzx_offset + 1;
				if    tzx_offset = x"10" then data_len_dword( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"11" then data_len_dword(15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"12" then data_len_dword(23 downto 16) <= tap_fifo_do;
				elsif tzx_offset = x"13" then data_len_dword(31 downto 24) <= tap_fifo_do;
				elsif tzx_offset = x"14" then
					tzx_offset <= x"14";
					if data_len_dword = 1 then
						tzx_state <= TZX_NEWBLOCK;
					else
						data_len_dword <= data_len_dword - 1;
					end if;
				end if;

			when TZX_GLUE =>
				tzx_offset <= tzx_offset + 1;
				if tzx_offset = x"08" then
					tzx_state <= TZX_NEWBLOCK;
				end if;

			when TZX_TONE =>
				tzx_offset <= tzx_offset + 1;
				-- 0, 1, 2, 3, 4, 4, 4, ...
				if    tzx_offset = x"00" then pilot_l( 7 downto 0) <= tap_fifo_do;
				elsif tzx_offset = x"01" then pilot_l(15 downto 8) <= tap_fifo_do;
				elsif tzx_offset = x"02" then pilot_pulses( 7 downto 0) <= tap_fifo_do;
				elsif tzx_offset = x"03" then
					tzx_req <= tzx_ack; -- don't request new byte
					pilot_pulses(15 downto 8) <= tap_fifo_do;
				else
					tzx_offset <= x"04";
					tzx_req <= tzx_ack; -- don't request new byte
					if pilot_pulses = 0 then
						tzx_req <= not tzx_ack; -- default request for new data
						tzx_state <= TZX_NEWBLOCK;
					else
						pilot_pulses <= pilot_pulses - 1;
						end_period <= wave_period;
						pulse_len <= pilot_l;
					end if;
				end if;

			when TZX_PULSES =>
				tzx_offset <= tzx_offset + 1;
				-- 0, 1-2+3, 1-2+3, ...
				if    tzx_offset = x"00" then data_len( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"01" then one_l( 7 downto 0) <= tap_fifo_do;
				elsif tzx_offset = x"02" then
					tzx_req <= tzx_ack; -- don't request new byte
					end_period <= wave_period;
					pulse_len <= tap_fifo_do & one_l( 7 downto 0);
				elsif tzx_offset = x"03" then
					if data_len(7 downto 0) = x"01" then
						tzx_state <= TZX_NEWBLOCK;
					else
						data_len(7 downto 0) <= data_len(7 downto 0) - 1;
						tzx_offset <= x"01";
					end if;
				end if;

			when TZX_DATA =>
				tzx_offset <= tzx_offset + 1;
				if    tzx_offset = x"00" then zero_l ( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"01" then zero_l (15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"02" then one_l  ( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"03" then one_l  (15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"04" then last_byte_bits <= tap_fifo_do(3 downto 0);
				elsif tzx_offset = x"05" then pause_len( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"06" then pause_len(15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"07" then data_len ( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"08" then data_len (15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"09" then
					tzx_req <= tzx_ack; -- don't request new byte
					data_len (23 downto 16) <= tap_fifo_do;
					tzx_state <= TZX_PLAY_TAPBLOCK;
				end if;

			when TZX_NORMAL =>
				tzx_offset <= tzx_offset + 1;
				if    tzx_offset = x"00" then pause_len( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"01" then pause_len(15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"02" then data_len ( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"03" then
					tzx_req <= tzx_ack; -- don't request new byte
					data_len(15 downto  8) <= tap_fifo_do;
					data_len(23 downto 16) <= (others => '0');
					pilot_l <= conv_std_logic_vector(NORMAL_PILOT_LEN, 16);
					sync1_l <= conv_std_logic_vector(NORMAL_SYNC1_LEN, 16);
					sync2_l <= conv_std_logic_vector(NORMAL_SYNC2_LEN, 16);
					zero_l  <= conv_std_logic_vector(NORMAL_ZERO_LEN,  16);
					one_l   <= conv_std_logic_vector(NORMAL_ONE_LEN,   16);
					pilot_pulses <= conv_std_logic_vector(NORMAL_PILOT_PULSES, 16);
					last_byte_bits <= "1000";
					tzx_state <= TZX_PLAY_TONE;
				end if;

			when TZX_TURBO =>
				tzx_offset <= tzx_offset + 1;
				if    tzx_offset = x"00" then pilot_l( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"01" then pilot_l(15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"02" then sync1_l( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"03" then sync1_l(15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"04" then sync2_l( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"05" then sync2_l(15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"06" then zero_l ( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"07" then zero_l (15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"08" then one_l  ( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"09" then one_l  (15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"0A" then pilot_pulses( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"0B" then pilot_pulses(15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"0C" then last_byte_bits <= tap_fifo_do(3 downto 0);
				elsif tzx_offset = x"0D" then pause_len( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"0E" then pause_len(15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"0F" then data_len ( 7 downto  0) <= tap_fifo_do;
				elsif tzx_offset = x"10" then data_len (15 downto  8) <= tap_fifo_do;
				elsif tzx_offset = x"11" then
					tzx_req <= tzx_ack; -- don't request new byte
					data_len (23 downto 16) <= tap_fifo_do;
					tzx_state <= TZX_PLAY_TONE;
				end if;

			when TZX_PLAY_TONE =>
				tzx_req <= tzx_ack; -- don't request new byte
				end_period <= not wave_period;
				pulse_len <= pilot_l;
				if pilot_pulses /= 0 then
					pilot_pulses <= pilot_pulses - 1;
				else
					tzx_state <= TZX_PLAY_SYNC1;
				end if;

			when TZX_PLAY_SYNC1 =>
				tzx_req <= tzx_ack; -- don't request new byte
				end_period <= wave_period;
				pulse_len <= sync1_l;
				tzx_state <= TZX_PLAY_SYNC2;

			when TZX_PLAY_SYNC2 =>
				tzx_req <= tzx_ack; -- don't request new byte
				end_period <= wave_period;
				pulse_len <= sync2_l;
				tzx_state <= TZX_PLAY_TAPBLOCK;

			when TZX_PLAY_TAPBLOCK =>
				bit_cnt <= "111";
				tzx_state <= TZX_PLAY_TAPBLOCK2;

			when TZX_PLAY_TAPBLOCK2 =>
				tzx_req <= tzx_ack; -- don't request new byte
				bit_cnt <= bit_cnt - 1;
				if bit_cnt = "000" or (data_len = 1 and ((bit_cnt = (8 - last_byte_bits)) or (last_byte_bits = 0))) then
					data_len <= data_len - 1;
					tzx_state <= TZX_PLAY_TAPBLOCK3;
				end if;
				end_period <= not wave_period;
				if tap_fifo_do(CONV_INTEGER(bit_cnt)) = '0' then
					pulse_len <= zero_l;
				else
					pulse_len <= one_l;
				end if;

			when TZX_PLAY_TAPBLOCK3 =>
				if data_len = 0 then
					tzx_state <= TZX_PAUSE2;
				else
					tzx_state <= TZX_PLAY_TAPBLOCK4;
				end if;

			when TZX_PLAY_TAPBLOCK4 =>
				tzx_req <= tzx_ack; -- don't request new byte
				tzx_state <= TZX_PLAY_TAPBLOCK2;

			when others => null;
			end case;

		end if; -- play tzx

	end if;
  end if; -- clk
end process;

end struct;
