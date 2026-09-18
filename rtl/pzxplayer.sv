module pzxplayer
#(
	parameter PZX_MS = 3500
)
(
	input         clk,
	input         ce,

	input         restart_tape,
	input         restart_block,
	input         skip_block,
	output reg    new_block,

	input   [7:0] host_tap_in,
	output reg    pzx_req,
	input         pzx_ack,

	output reg    stop,
	output reg    stop48k,

	output reg    cass_read,
	input         cass_motor
);

localparam ST_START   = 0;
localparam ST_HDR     = 1;
localparam ST_PULS    = 2;
localparam ST_PULSE   = 3;
localparam ST_DATA    = 4;
localparam ST_SEQ     = 5;
localparam ST_LOAD    = 6;
localparam ST_BIT     = 7;
localparam ST_SEQOUT  = 8;
localparam ST_NEXTBIT = 9;
localparam ST_TAIL    = 10;
localparam ST_PAUS    = 11;
localparam ST_STOP    = 12;
localparam ST_SKIP    = 13;

reg  [3:0] state;
reg [16:0] tick;
reg [30:0] pulse_len;
reg [30:0] dur;
reg        emit;
reg        level;
reg [31:0] tag;
reg [31:0] blk_size;
reg  [2:0] offs;
reg [15:0] wrd;
reg        hi;
reg [14:0] rep;
reg  [1:0] pphase;
reg [30:0] bits_left;
reg [15:0] tail;
reg  [7:0] p0;
reg  [7:0] p1;
reg  [7:0] pulse_cnt;
reg  [8:0] sptr;
reg [15:0] seq[32];
reg  [7:0] data_byte;
reg  [2:0] bit_idx;
reg        brws_pend;
reg        puls_pend;
reg        playing;
reg        motor_d;
reg [21:0] motor_cnt;

wire [31:0] size_now = {host_tap_in, blk_size[31:8]};
wire [15:0] wnow     = {host_tap_in, wrd[15:8]};
wire  [8:0] pcnt     = {1'b0, p0} + {1'b0, p1};
wire        nofetch  = (state == ST_PULSE) || (state == ST_BIT) || (state == ST_SEQOUT) || (state == ST_NEXTBIT) || (state == ST_TAIL);

always @(posedge clk) begin
	if(restart_tape) begin
		state     <= ST_START;
		pulse_len <= 0;
		emit      <= 0;
		pzx_req   <= pzx_ack;
		cass_read <= 1;
		playing   <= 0;
		motor_cnt <= 0;
		new_block <= 0;
		stop      <= 0;
		stop48k   <= 0;
		level     <= 0;
		hi        <= 0;
		offs      <= 0;
		pphase    <= 0;
		brws_pend <= 0;
		puls_pend <= 0;
	end
	else begin
		motor_d <= cass_motor;
		if(motor_d != cass_motor) motor_cnt <= 50 * PZX_MS;
		else if(motor_cnt) begin
			if(ce) motor_cnt <= motor_cnt - 1'd1;
		end
		else playing <= cass_motor;

		if(pulse_len) begin
			if(ce) begin
				tick <= tick + 17'd3500;
				if(tick >= (PZX_MS - 3500)) begin
					tick <= tick - (PZX_MS - 3500);
					pulse_len <= pulse_len - 1'd1;
				end
			end
		end
		else tick <= 0;

		new_block <= 0;
		stop      <= 0;
		stop48k   <= 0;

		if(emit) begin
			emit  <= 0;
			level <= ~level;
			if(dur) begin
				cass_read <= level;
				pulse_len <= dur;
			end
		end
		else if(playing && !pulse_len && (nofetch || (pzx_req == pzx_ack)) && !restart_block) begin

			case(state)
			ST_START: begin
					pzx_req <= ~pzx_ack;
					offs  <= 0;
					state <= ST_HDR;
				end

			ST_HDR: begin
					pzx_req <= ~pzx_ack;
					offs <= offs + 1'd1;
					if(!offs[2]) tag <= {tag[23:0], host_tap_in};
					else blk_size <= size_now;
					if(offs == 7) begin
						offs  <= 0;
						hi    <= 0;
						state <= ST_SKIP;
						if(size_now)
							case(tag)
							"PULS": begin
									level  <= 0;
									pphase <= 0;
									state  <= ST_PULS;
									if(!brws_pend) new_block <= 1;
									brws_pend <= 0;
									puls_pend <= 1;
								end
							"DATA": begin
									state <= ST_DATA;
									if(!brws_pend && !puls_pend) new_block <= 1;
									brws_pend <= 0;
									puls_pend <= 0;
								end
							"PAUS": state <= ST_PAUS;
							"STOP": state <= ST_STOP;
							"BRWS": begin
									if(!brws_pend) new_block <= 1;
									brws_pend <= 1;
								end
							default: ;
							endcase
					end
				end

			ST_PULS: begin
					pzx_req <= ~pzx_ack;
					blk_size <= blk_size - 1'd1;
					wrd <= wnow;
					hi  <= ~hi;
					if(hi) begin
						case(pphase)
						0: if(wnow > 16'h8000) begin
								rep    <= wnow[14:0];
								pphase <= 1;
							end
							else if(wnow[15]) begin
								dur[30:16] <= wnow[14:0];
								rep    <= 1;
								pphase <= 2;
							end
							else begin
								rep   <= 1;
								dur   <= {15'd0, wnow};
								state <= ST_PULSE;
							end
						1: if(wnow[15]) begin
								dur[30:16] <= wnow[14:0];
								pphase <= 2;
							end
							else begin
								dur   <= {15'd0, wnow};
								state <= ST_PULSE;
							end
						default: begin
								dur[15:0] <= wnow;
								state     <= ST_PULSE;
							end
						endcase
					end
				end

			ST_PULSE: begin
					if(!dur) begin
						level  <= level ^ rep[0];
						pphase <= 0;
						state  <= blk_size ? ST_PULS : ST_SKIP;
					end
					else if(rep) begin
						rep  <= rep - 1'd1;
						emit <= 1;
					end
					else begin
						pphase <= 0;
						state  <= blk_size ? ST_PULS : ST_SKIP;
					end
				end

			ST_DATA: begin
					pzx_req <= ~pzx_ack;
					blk_size <= blk_size - 1'd1;
					offs <= offs + 1'd1;
					case(offs)
					0: bits_left[ 7: 0] <= host_tap_in;
					1: bits_left[15: 8] <= host_tap_in;
					2: bits_left[23:16] <= host_tap_in;
					3: {level, bits_left[30:24]} <= host_tap_in;
					4: tail[ 7:0] <= host_tap_in;
					5: tail[15:8] <= host_tap_in;
					6: p0 <= host_tap_in;
					7: begin
							p1    <= host_tap_in;
							sptr  <= 0;
							offs  <= 0;
							hi    <= 0;
							state <= (p0 | host_tap_in) ? ST_SEQ : (bits_left ? ST_LOAD : ST_TAIL);
						end
					endcase
				end

			ST_SEQ: begin
					pzx_req <= ~pzx_ack;
					blk_size <= blk_size - 1'd1;
					wrd <= wnow;
					hi  <= ~hi;
					if(hi) begin
						if(!sptr[8:5]) seq[sptr[4:0]] <= wnow;
						sptr <= sptr + 1'd1;
						if((sptr + 1'd1) == pcnt) begin
							sptr  <= 0;
							state <= bits_left ? ST_LOAD : ST_TAIL;
						end
					end
				end

			ST_LOAD: begin
					pzx_req   <= ~pzx_ack;
					blk_size  <= blk_size - 1'd1;
					data_byte <= host_tap_in;
					bit_idx   <= 7;
					state     <= ST_BIT;
				end

			ST_BIT: begin
					sptr      <= data_byte[bit_idx] ? {1'b0, p0} : 9'd0;
					pulse_cnt <= data_byte[bit_idx] ? p1 : p0;
					state     <= ST_SEQOUT;
				end

			ST_SEQOUT: begin
					if(pulse_cnt) begin
						dur       <= {15'd0, seq[sptr[4:0]]};
						sptr      <= sptr + 1'd1;
						pulse_cnt <= pulse_cnt - 1'd1;
						emit      <= 1;
					end
					else state <= ST_NEXTBIT;
				end

			ST_NEXTBIT: begin
					bits_left <= bits_left - 1'd1;
					if(bits_left == 1) state <= ST_TAIL;
					else if(!bit_idx) state <= ST_LOAD;
					else begin
						bit_idx <= bit_idx - 1'd1;
						state   <= ST_BIT;
					end
				end

			ST_TAIL: begin
					dur     <= {15'd0, tail};
					emit    <= 1;
					state   <= ST_SKIP;
				end

			ST_PAUS: begin
					pzx_req <= ~pzx_ack;
					blk_size <= blk_size - 1'd1;
					offs <= offs + 1'd1;
					case(offs)
					0: dur[ 7: 0] <= host_tap_in;
					1: dur[15: 8] <= host_tap_in;
					2: dur[23:16] <= host_tap_in;
					default: begin
							{level, dur[30:24]} <= host_tap_in;
							emit  <= 1;
							offs  <= 0;
							state <= ST_SKIP;
						end
					endcase
				end

			ST_STOP: begin
					pzx_req <= ~pzx_ack;
					blk_size <= blk_size - 1'd1;
					wrd <= wnow;
					hi  <= ~hi;
					if(hi) begin
						if(wnow == 16'd1) stop48k <= 1;
						else stop <= 1;
						state <= ST_SKIP;
					end
				end

			ST_SKIP: if(blk_size) begin
					pzx_req  <= ~pzx_ack;
					blk_size <= blk_size - 1'd1;
				end
				else begin
					offs    <= 0;
					hi      <= 0;
					state   <= ST_HDR;
				end

			default: ;
			endcase
		end

		if(skip_block && (state != ST_START) && (state != ST_HDR) && (state != ST_SKIP)) begin
			pulse_len <= 0;
			emit      <= 0;
			state     <= ST_SKIP;
		end

		if(restart_block) begin
			state     <= ST_HDR;
			offs      <= 0;
			hi        <= 0;
			pulse_len <= 0;
			emit      <= 0;
			pzx_req   <= ~pzx_ack;
			brws_pend <= 0;
			puls_pend <= 0;
		end
	end
end

endmodule
