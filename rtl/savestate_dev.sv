module savestate_dev #(parameter ROM_FILE = "rtl/savestate_rom.mif", parameter [27:0] HALT_WAIT = 28'd5_600_000)
(
	input             clk_sys,
	input             reset,
	input             new_game,
	input             game_load,
	input             ioctl_wr,
	input       [7:0] ioctl_dout,
	input             other_image,

	input      [10:0] ps2_key,
	output reg [10:0] ps2_key_kbd = 0,
	input      [15:0] joy0,
	input      [15:0] joy1,
	output            joy_block,

	input             m1,
	input       [7:0] cpu_din,
	input             ce_cpu_p,
	input             cpu_n,
	input             cpu_en,
	input             cpu_halted,
	input             no_stop,
	output            stop_req,
	output reg        stopped = 0,
	output            freezing,

	input     [211:0] cpu_reg,
	output reg        cpu_reset = 0,
	output reg[211:0] dir,
	output reg        dir_set = 0,

	input       [2:0] border,
	input             ear,
	input             mic,
	input       [7:0] port_7ffd,
	input       [7:0] port_1ffd,
	input       [4:0] arch,
	input             issue2,
	output reg        set_ports = 0,
	output      [2:0] new_border,
	output            new_ear,
	output            new_mic,
	output      [7:0] new_7ffd,
	output      [7:0] new_1ffd,
	input      [63:0] rtc,
	input      [31:0] timestamp,
	input             nINT,

	input             aud_reset,
	input             psg_bdir,
	input             psg_bc,
	input       [7:0] psg_di,

	input      [15:0] addr,
	input       [7:0] cpu_dout,
	input             nMREQ,
	input             nIORQ,
	input             nRD,
	input             nWR,
	input             nM1,
	output reg        active = 0,
	output reg  [3:0] bank = 0,
	output            dout_en,
	output reg  [7:0] dout,

	input      [12:0] vram_addr,
	input       [7:0] vram_q,
	output      [7:0] ula_q,
	input       [2:0] border_in,
	output      [2:0] ula_border,

	input             img_mounted,
	input             img_readonly,
	input      [63:0] img_size,
	output reg [31:0] sd_lba,
	output reg  [5:0] sd_blk_cnt,
	output reg        sd_rd = 0,
	output reg        sd_wr = 0,
	input             sd_ack,
	input      [13:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din,
	input             sd_buff_wr
);

localparam [27:0] TIMEOUT = 28'd224_000_000;

wire [15:0] joy = joy0 | joy1;
localparam BTN = 6;

reg insert = 0;
reg open_req = 0;

always @(posedge clk_sys) begin
	reg [10:0] old_key;
	reg        old_btn;

	open_req <= 0;
	old_key  <= ps2_key;
	old_btn  <= joy[BTN];
	if(old_key[10] != ps2_key[10] && ps2_key[8:0] == 9'h170) begin
		if(ps2_key[9] & ~insert) open_req <= 1;
		insert <= ps2_key[9];
	end
	if(joy[BTN] & ~old_btn) open_req <= 1;
end

assign joy_block = active | joy[BTN];

reg  freeze_req = 0;
reg  leave_stop = 0;
reg [27:0] hwait = 0;
wire last_ei;
wire boundary_stop;

savestate_stop stop
(
	.clk(clk_sys),
	.m1(m1),
	.cpu_din(cpu_din),
	.ce_cpu_p(ce_cpu_p),
	.cpu_halted(cpu_halted),
	.freeze_req(freeze_req),
	.stopped(stopped),
	.no_stop(no_stop),
	.halt_only(|hwait),
	.stop_req(boundary_stop),
	.last_ei(last_ei)
);

assign stop_req = boundary_stop | (leave_stop & ~stopped);
assign freezing = freeze_req & ~stopped;

always @(posedge clk_sys) begin
	reg [10:0] old_key;

	old_key <= ps2_key;
	if(old_key[10] != ps2_key[10] &&
	   !((active | freeze_req) && ps2_key[9] && (ps2_key[7:0] == 8'h01 || ps2_key[7:0] == 8'h78)))
		ps2_key_kbd <= {~ps2_key_kbd[10], ps2_key[9:0]};
end

reg  [7:0] ay_reg[16];
reg  [7:0] ay_sel;
reg        ay_chip0;

always @(posedge clk_sys) begin
	reg old_bdir;
	integer i;

	old_bdir <= psg_bdir;
	if(aud_reset) begin
		for(i = 0; i < 16; i = i + 1) ay_reg[i] <= 0;
		ay_sel   <= 0;
		ay_chip0 <= 1;
	end
	else if(~old_bdir & psg_bdir) begin
		if(psg_bc & &psg_di[7:3]) ay_chip0 <= psg_di[0];
		else if(psg_bc) begin
			if(ay_chip0) ay_sel <= psg_di;
		end
		else if(ay_chip0 & ~|ay_sel[7:4]) ay_reg[ay_sel[3:0]] <= psg_di;
	end
end

reg [7:0] sb[64];

assign new_border = sb[27][2:0];
assign new_mic    = sb[27][3];
assign new_ear    = sb[27][4];
assign new_7ffd   = sb[28];
assign new_1ffd   = sb[29];

wire [211:0] sb_dir = {sb[26][3], sb[26][2], sb[26][1:0],
                       sb[19], sb[18], sb[15], sb[14], sb[13], sb[12], sb[11], sb[10],
                       sb[17], sb[16], sb[7], sb[6], sb[5], sb[4], sb[3], sb[2],
                       sb[23], sb[22], sb[21], sb[20], sb[25], sb[24], sb[9], sb[8], sb[1], sb[0]};

localparam [211:0] DIR_ENTRY = {2'b00, 2'b01, 128'd0, 16'h0100, 16'h4000, 8'h00, 8'h00, 32'd0};

wire io_rd = ~nIORQ & ~nRD & nM1;
wire io_wr = ~nIORQ & ~nWR & nM1;
wire mem_rom  = ~nMREQ & (addr[15:13] == 3'b000);
wire mem_pram = ~nMREQ & (addr[15:13] == 3'b001);
wire port_ctl = (addr[7:0] == 8'h0B);
wire port_buf = (addr[7:0] == 8'h0F);

reg        rom_page = 0;
wire [7:0] rom_q;
dpram #(.DATAWIDTH(8), .ADDRWIDTH(14), .MEM_INIT_FILE(ROM_FILE)) rom
(
	.clock(clk_sys),
	.address_a({rom_page, addr[12:0]}),
	.data_a(8'd0),
	.wren_a(1'b0),
	.q_a(rom_q),
	.address_b(14'd0),
	.data_b(8'd0),
	.wren_b(1'b0),
	.q_b()
);

wire [7:0] pram_q, pram_ula;
dpram #(.DATAWIDTH(8), .ADDRWIDTH(13)) pram
(
	.clock(clk_sys),
	.address_a(addr[12:0]),
	.data_a(cpu_dout),
	.wren_a(active & mem_pram & ~nWR),
	.q_a(pram_q),
	.address_b(vram_addr),
	.data_b(8'd0),
	.wren_b(1'b0),
	.q_b(pram_ula)
);

reg  [13:0] ptr = 0;
reg   [7:0] buf_wdata;
reg         buf_we = 0;
wire  [7:0] buf_q;
dpram #(.DATAWIDTH(8), .ADDRWIDTH(14)) buffer
(
	.clock(clk_sys),
	.address_a(ptr),
	.data_a(buf_wdata),
	.wren_a(buf_we),
	.q_a(buf_q),
	.address_b(sd_buff_addr),
	.data_b(sd_buff_dout),
	.wren_b(sd_buff_wr & sd_ack),
	.q_b(sd_buff_din)
);

reg [2:0] ui_border = 0;
assign ula_q      = active ? pram_ula  : vram_q;
assign ula_border = active ? ui_border : border_in;

reg  [5:0] sb_idx = 0;
reg  [7:0] frames = 0;
reg  [7:0] loads = 0;
reg [31:0] crc = 0;
reg [31:0] game_crc = 0;
reg        game_known = 0;
reg [31:0] img_secs = 0;
reg        img_ro = 0;
reg        busy = 0, err = 0;
reg  [1:0] cmd = 0;

wire disk = |img_secs;

assign dout_en = active & ((io_rd & (port_ctl | port_buf)) | mem_rom | mem_pram);

always @(*) begin
	dout = 8'hFF;
	if(mem_rom)        dout = rom_q;
	else if(mem_pram)  dout = pram_q;
	else if(port_buf)  dout = buf_q;
	else case(addr[15:8])
		8'h00: dout = {4'd0, img_ro, disk, err, busy};
		8'h01: dout = img_secs[7:0];
		8'h02: dout = img_secs[15:8];
		8'h03: dout = img_secs[23:16];
		8'h04: dout = img_secs[31:24];
		8'h09: dout = sb[sb_idx];
		8'h0B: dout = {insert, joy[6:0]};
		8'h0C: dout = frames;
		8'h0E: dout = loads;
		8'h0F: dout = {7'd0, game_known};
		8'h10: dout = game_crc[7:0];
		8'h11: dout = game_crc[15:8];
		8'h12: dout = game_crc[23:16];
		8'h13: dout = game_crc[31:24];
		default:;
	endcase
end

localparam [3:0] D_IDLE = 0, D_FREEZE = 1, D_IN_RST = 2, D_IN_DIR = 3, D_RUN = 4,
                 D_OUT_STOP = 5, D_OUT_RST = 6, D_OUT_DIR = 7;

localparam [2:0] S_IDLE = 0, S_REQ = 1, S_ACK = 2, S_DONE = 3;

reg  [3:0] state = D_IDLE;
reg  [2:0] sd_state = S_IDLE;
reg  [3:0] cnt;
reg [27:0] tmo;
reg [27:0] sd_tmo;
reg        sd_op_wr;

function [31:0] crc_byte(input [31:0] c, input [7:0] d);
	integer i;
	begin
		crc_byte = c;
		for(i = 0; i < 8; i = i + 1) crc_byte = {1'b0, crc_byte[31:1]} ^ ((crc_byte[0] ^ d[i]) ? 32'hEDB88320 : 32'd0);
	end
endfunction

always @(posedge clk_sys) begin
	reg old_io, old_int, old_mounted, old_reset, old_load, old_image, crc_we;
	reg [7:0] crc_d;
	reg acc_buf, acc_sb;
	integer i;

	buf_we    <= 0;
	set_ports <= 0;

	if(stop_req & cpu_n & cpu_en) stopped <= 1;
	if(tmo) tmo <= tmo - 1'd1;
	if(hwait) hwait <= hwait - 1'd1;

	old_int <= nINT;
	if(old_int & ~nINT) frames <= frames + 1'd1;

	old_reset <= reset;
	if((reset & ~old_reset) | new_game) loads <= loads + 1'd1;

	old_load <= game_load;
	old_image <= other_image;
	crc_d <= ioctl_dout;
	crc_we <= game_load & ioctl_wr;
	if(~old_load & game_load) crc <= 32'hFFFFFFFF;
	else if(crc_we) crc <= crc_byte(crc, crc_d);
	if(old_load & ~game_load) begin
		game_crc   <= ~crc;
		game_known <= 1;
	end
	if(~old_image & other_image) game_known <= 0;

	old_mounted <= img_mounted;
	if(~old_mounted & img_mounted) begin
		img_secs <= img_size[40:9];
		img_ro   <= img_readonly;
	end

	old_io <= io_rd | io_wr;
	if(~old_io & (io_rd | io_wr)) begin
		acc_buf <= port_buf;
		acc_sb  <= port_ctl & (addr[15:8] == 8'h09);
	end
	if(active & old_io & ~(io_rd | io_wr)) begin
		if(acc_buf) ptr <= ptr + 1'd1;
		if(acc_sb) sb_idx <= sb_idx + 1'd1;
	end
	if(active & io_wr & ~old_io) begin
		if(port_buf) begin
			buf_wdata <= cpu_dout;
			buf_we    <= 1;
		end
		if(port_ctl) case(addr[15:8])
			8'h00: if(~busy) begin cmd <= cpu_dout[1:0]; busy <= |cpu_dout[1:0]; err <= 0; end
			8'h01: sd_lba[7:0]   <= cpu_dout;
			8'h02: sd_lba[15:8]  <= cpu_dout;
			8'h03: sd_lba[23:16] <= cpu_dout;
			8'h04: sd_lba[31:24] <= cpu_dout;
			8'h05: sd_blk_cnt    <= cpu_dout[5:0];
			8'h06: ptr           <= 0;
			8'h07: bank          <= cpu_dout[3:0];
			8'h08: sb_idx        <= cpu_dout[5:0];
			8'h09: if(sb_idx < 30) sb[sb_idx] <= cpu_dout;
			8'h0A: leave_stop    <= 1;
			8'h0D: ui_border     <= cpu_dout[2:0];
			8'h0E: rom_page      <= cpu_dout[0];
			default:;
		endcase
	end

	case(state)
		D_IDLE: begin
			stopped    <= 0;
			leave_stop <= 0;
			if(open_req & ~reset) begin
				freeze_req <= 1;
				hwait      <= HALT_WAIT;
				tmo        <= TIMEOUT;
				state      <= D_FREEZE;
			end
		end

		D_FREEZE:
			if(stopped) begin
				sb[0]  <= cpu_reg[7:0];      sb[1]  <= cpu_reg[15:8];
				sb[2]  <= cpu_reg[87:80];    sb[3]  <= cpu_reg[95:88];
				sb[4]  <= cpu_reg[103:96];   sb[5]  <= cpu_reg[111:104];
				sb[6]  <= cpu_reg[119:112];  sb[7]  <= cpu_reg[127:120];
				sb[8]  <= cpu_reg[23:16];    sb[9]  <= cpu_reg[31:24];
				sb[10] <= cpu_reg[151:144];  sb[11] <= cpu_reg[159:152];
				sb[12] <= cpu_reg[167:160];  sb[13] <= cpu_reg[175:168];
				sb[14] <= cpu_reg[183:176];  sb[15] <= cpu_reg[191:184];
				sb[16] <= cpu_reg[135:128];  sb[17] <= cpu_reg[143:136];
				sb[18] <= cpu_reg[199:192];  sb[19] <= cpu_reg[207:200];
				sb[20] <= cpu_reg[55:48];    sb[21] <= cpu_reg[63:56];
				{sb[23], sb[22]} <= cpu_reg[79:64] - cpu_halted;
				sb[24] <= cpu_reg[39:32];    sb[25] <= cpu_reg[47:40];
				sb[26] <= {4'd0, cpu_reg[211] | last_ei, cpu_reg[210] | last_ei, cpu_reg[209:208]};
				sb[27] <= {3'd0, ear, mic, border};
				sb[28] <= port_7ffd;
				sb[29] <= port_1ffd;
				sb[30] <= {3'd0, arch};
				sb[31] <= ay_sel;
				for(i = 0; i < 16; i = i + 1) sb[32 + i] <= ay_reg[i];
				sb[48] <= {7'd0, issue2};
				for(i = 0; i < 8; i = i + 1) sb[49 + i] <= rtc[i*8 +: 8];
				for(i = 0; i < 4; i = i + 1) sb[57 + i] <= timestamp[i*8 +: 8];
				sb[61] <= 0; sb[62] <= 0; sb[63] <= 0;
				cpu_reset  <= 1;
				cnt        <= 15;
				state      <= D_IN_RST;
			end
			else if(!tmo || reset) begin
				freeze_req <= 0;
				state      <= D_IDLE;
			end

		D_IN_RST: begin
			active <= 1;
			bank   <= 0;
			rom_page <= 0;
			ptr    <= 0;
			sb_idx <= 0;
			ui_border <= 0;
			if(cnt) cnt <= cnt - 1'd1;
			else begin
				cpu_reset <= 0;
				dir       <= DIR_ENTRY;
				dir_set   <= 1;
				cnt       <= 3;
				state     <= D_IN_DIR;
			end
		end

		D_IN_DIR:
			if(cnt) cnt <= cnt - 1'd1;
			else begin
				dir_set    <= 0;
				freeze_req <= 0;
				stopped    <= 0;
				state      <= D_RUN;
			end

		D_RUN:
			if(reset) begin
				active     <= 0;
				leave_stop <= 0;
				state      <= D_IDLE;
			end
			else if(leave_stop & stopped) begin
				cpu_reset <= 1;
				cnt       <= 15;
				state     <= D_OUT_RST;
			end

		D_OUT_RST: begin
			active <= 0;
			if(cnt) cnt <= cnt - 1'd1;
			else begin
				cpu_reset <= 0;
				set_ports <= 1;
				dir       <= sb_dir;
				dir_set   <= 1;
				cnt       <= 3;
				state     <= D_OUT_DIR;
			end
		end

		D_OUT_DIR:
			if(cnt) cnt <= cnt - 1'd1;
			else begin
				dir_set    <= 0;
				leave_stop <= 0;
				stopped    <= 0;
				state      <= D_IDLE;
			end

		default: state <= D_IDLE;
	endcase

	case(sd_state)
		S_IDLE:
			if(busy) begin
				sd_op_wr <= cmd[1];
				sd_tmo   <= TIMEOUT;
				sd_state <= (disk && !(cmd[1] && img_ro)) ? S_REQ : S_DONE;
				if(!disk || (cmd[1] && img_ro)) err <= 1;
			end

		S_REQ: begin
			sd_wr    <= sd_op_wr;
			sd_rd    <= ~sd_op_wr;
			sd_state <= S_ACK;
		end

		S_ACK:
			if(sd_ack) begin
				sd_rd    <= 0;
				sd_wr    <= 0;
				sd_state <= S_DONE;
			end
			else if(!sd_tmo) begin
				sd_rd    <= 0;
				sd_wr    <= 0;
				err      <= 1;
				sd_state <= S_DONE;
			end
			else sd_tmo <= sd_tmo - 1'd1;

		S_DONE:
			if(~sd_ack) begin
				busy     <= 0;
				sd_state <= S_IDLE;
			end
	endcase
end

endmodule
