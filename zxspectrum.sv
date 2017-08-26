//============================================================================
// Sinclair ZX Spectrum host board
// 
//  Port to DE10-nano board. 
//  Copyright (C) 2017 Sorgelig
//
//  Based on sample ZX Spectrum code by Goran Devic
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [37:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	output  [7:0] VIDEO_ARX,
	output  [7:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status ORed with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S, // 1 - signed audio samples, 0 - unsigned
	input         TAPE_IN,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE
);

assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0;

assign AUDIO_S   = 0;

assign LED_USER  = ioctl_download | tape_led;
assign LED_DISK  = 0;
assign LED_POWER = 0;

assign VIDEO_ARX = status[1] ? 8'd16 : 8'd4;
assign VIDEO_ARY = status[1] ? 8'd9  : 8'd3;

localparam CONF_BDI   = "(BDI)";
localparam CONF_PLUSD = "(+D) ";

`include "build_id.v"
localparam CONF_STR = {
	"SPECTRUM;;",
	"-;",
	"S,TRDIMGDSKMGT,Load Disk;",
	"-;",
	"F,TAPCSW,Load Tape;",
	"O6,Fast tape load,On,Off;",
	"-;",
	"O89,Video timings,ULA-48,ULA-128,Pentagon;",
	"O1,Aspect ratio,4:3,16:9;",
	"OFG,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%;",
	"-;",
	"OAC,Memory,Spectrum 128K/+2,Pentagon 512K,Profi 1024K,Spectrum 48K,Spectrum +2A/+3;",
	"ODE,Features,ULA+ & Timex,ULA+,Timex,None;",
	"J,Fire 1,Fire 2;",
	"V,v3.70.",`BUILD_DATE
};


////////////////////   CLOCKS   ///////////////////

assign CLK_VIDEO = clk_sys;

wire locked;
wire clk_sys;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(SDRAM_CLK),
	.locked(locked)
);


reg  ce_psg;  //1.75MHz
reg  ce_7mp;
reg  ce_7mn;
reg  ce_28m;

reg  pause;
reg  cpu_en = 1;
reg  ce_cpu_tp;
reg  ce_cpu_tn;

wire ce_cpu_p = cpu_en & cpu_p;
wire ce_cpu_n = cpu_en & cpu_n;
wire ce_cpu   = cpu_en & ce_cpu_tp;

wire cpu_p = ~&turbo ? ce_cpu_tp : ce_cpu_sp;
wire cpu_n = ~&turbo ? ce_cpu_tn : ce_cpu_sn;

always @(negedge clk_sys) begin
	reg [5:0] counter = 0;

	counter <=  counter + 1'd1;

	ce_28m  <= !counter[1:0];
	ce_7mp  <= !counter[3] & !counter[2:0];
	ce_7mn  <=  counter[3] & !counter[2:0];
	ce_psg  <= !counter[5:0] & ~pause;

	ce_cpu_tp <= !(counter & turbo);
	ce_cpu_tn <= !((counter & turbo) ^ turbo ^ turbo[4:1]);
end

reg [4:0] turbo = 5'b11111, turbo_key = 5'b11111;
always @(posedge clk_sys) begin
	reg [9:4] old_Fn;
	old_Fn <= Fn[9:4];

	if(reset) pause <= 0;

	if(!mod) begin
		if(~old_Fn[4] & Fn[4]) turbo_key <= 5'b11111;
		if(~old_Fn[5] & Fn[5]) turbo_key <= 5'b01111;
		if(~old_Fn[6] & Fn[6]) turbo_key <= 5'b00111;
		if(~old_Fn[7] & Fn[7]) turbo_key <= 5'b00011;
		if(~old_Fn[8] & Fn[8]) turbo_key <= 5'b00001;
		if(~old_Fn[9] & Fn[9]) pause <= ~pause;
	end
end

wire [4:0] turbo_req = (tape_active & ~status[6]) ? 5'b00001 : turbo_key;
always @(posedge clk_sys) begin
	reg [1:0] timeout;

	if(cpu_n) begin
		if(timeout) timeout <= timeout + 1'd1;
		if(turbo != turbo_req) begin
			cpu_en  <= 0;
			timeout <= 1;
			turbo   <= turbo_req;
		end else if(!cpu_en & !timeout & ram_ready) begin
			cpu_en  <= ~pause;
		end else if(!turbo[4:2] & !ram_ready) begin // SDRAM wait for 28MHz/56MHz turbo
			cpu_en  <= 0;
		end else if(!turbo[4:3] & !ram_ready & tape_active) begin // SDRAM wait for TAPE load on 14MHz
			cpu_en  <= 0;
		end else if(cpu_en & pause) begin
			cpu_en  <= 0;
		end
	end
end


//////////////////   HPS I/O   ///////////////////
wire        ps2_kbd_clk;
wire        ps2_kbd_data;
wire        ps2_mouse_clk;
wire        ps2_mouse_data;

wire [15:0] joystick_0;
wire [15:0] joystick_1;
wire  [1:0] buttons;
wire        forced_scandoubler;
wire [31:0] status;

wire 			sd_rd_plus3;
wire 			sd_wr_plus3;
wire [31:0] sd_lba_plus3;
wire [7:0]  sd_buff_din_plus3;

wire 			sd_rd_wd;
wire 			sd_wr_wd;
wire [31:0] sd_lba_wd;
wire [7:0]  sd_buff_din_wd;

wire [31:0] sd_lba = plus3_fdd_ready ? sd_lba_plus3 : sd_lba_wd;
wire        sd_rd = plus3_fdd_ready ? sd_rd_plus3 : sd_rd_wd;
wire        sd_wr = plus3_fdd_ready ? sd_wr_plus3 : sd_wr_wd;
wire        sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din = plus3_fdd_ready ? sd_buff_din_plus3 : sd_buff_din_wd;
wire        sd_buff_wr;
wire        img_mounted;
wire [63:0] img_size;
wire        img_readonly;

wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_download;
wire  [7:0] ioctl_index;
hps_io #(.STRLEN(($size(CONF_STR)>>3)+5)) hps_io
(
	.*,
	.conf_str({CONF_STR, plusd_en ? CONF_PLUSD : CONF_BDI}),
	.sd_conf(0),
	.ioctl_wait(0),
	.sd_ack_conf(),
	.ps2_kbd_led_use(0),
	.ps2_kbd_led_status(0),

	// unused
	.joystick_analog_0(),
	.joystick_analog_1()
);


///////////////////   CPU   ///////////////////
wire [15:0] addr;
wire  [7:0] cpu_din;
wire  [7:0] cpu_dout;
wire        nM1;
wire        nMREQ;
wire        nIORQ;
wire        nRD;
wire        nWR;
wire        nRFSH;
wire        nBUSACK;
wire        nINT;
wire        nBUSRQ = ~ioctl_download;
wire        reset  = buttons[1] | status[0] | cold_reset | warm_reset | shdw_reset | Fn[10];

wire        cold_reset =((mod[2:1] == 1) & Fn[11]) | init_reset;
wire        warm_reset = (mod[2:1] == 2) & Fn[11];
wire        shdw_reset = (mod[2:1] == 3) & Fn[11] & ~plus3;

wire        io_wr = ~nIORQ & ~nWR & nM1;
wire        io_rd = ~nIORQ & ~nRD & nM1;
wire        m1    = ~nM1 & ~nMREQ;

wire[207:0]	cpu_reg;  // IY, HL', DE', BC', IX, HL, DE, BC, PC, SP, R, I, F', A', F, A
wire [15:0] reg_DE  = cpu_reg[111:96];
wire  [7:0] reg_A   = cpu_reg[7:0];

T80pa cpu
(
	.RESET_n(~reset),
	.CLK(clk_sys),
	.CEN_p(ce_cpu_p),
	.CEN_n(ce_cpu_n),
	.WAIT_n(1),
	.INT_n(nINT),
	.NMI_n(~NMI),
	.BUSRQ_n(nBUSRQ),
	.M1_n(nM1),
	.MREQ_n(nMREQ),
	.IORQ_n(nIORQ),
	.RD_n(nRD),
	.WR_n(nWR),
	.RFSH_n(nRFSH),
	.HALT_n(1),
	.BUSAK_n(nBUSACK),
	.A(addr),
	.DO(cpu_dout),
	.DI(cpu_din),
	.REG(cpu_reg)
);

always_comb begin
	casex({nMREQ, tape_dout_en, rom_sel, ~nM1 | nIORQ | nRD, fdd_sel | fdd_sel2 | plus3_fdd, mf3_port, addr[5:0]==8'h1F, portBF, addr[0], psg_enable, ulap_sel})
		'b01XXXXXXXXX: cpu_din = tape_dout;
		'b000XXXXXXXX: cpu_din = ram_dout;
		'b001XXXXXXXX: cpu_din = rom_dout;
		'b1XX01XXXXXX: cpu_din = fdd_dout;
		'b1XX001XXXXX: cpu_din = (addr[14:13] == 2'b11 ? page_reg : page_reg_plus3);
		'b1XX0001XXXX: cpu_din = mouse_sel ? mouse_data : {2'b00, joystick_0[5:0]};
		'b1XX00001XXX: cpu_din = {page_scr_copy, 7'b1111111};
		'b1XX0000011X: cpu_din = (addr[14] ? sound_data : 8'hFF);
		'b1XX00000101: cpu_din = ulap_dout;
		'b1XX00000100: cpu_din = port_ff;
		'b1XX000000XX: cpu_din = {1'b1, ~tape_in, 1'b1, key_data[4:0] & ({5{addr[12]}} | ~{joystick_1[1:0], joystick_1[2], joystick_1[3], joystick_1[4]})};
		'b1XX1XXXXXXX: cpu_din = 8'hFF;
	endcase
end

reg init_reset = 1;
always @(posedge clk_sys) begin
	reg old_rst = 0;
	old_rst <= status[0];
	if(old_rst & ~status[0]) init_reset <= 0;
end

reg NMI;
always @(posedge clk_sys) begin
	reg old_F11;

	old_F11 <= Fn[11];

	if(reset | ~Fn[11] | (m1 & (addr == 'h66))) NMI <= 0;
	else if(~old_F11 & Fn[11] & (mod[2:1] == 0)) NMI <= 1;
end


//////////////////   MEMORY   //////////////////
wire        dma = (reset | ~nBUSACK) & ~nBUSRQ;
reg  [24:0] ram_addr;
reg   [7:0] ram_din;
reg         ram_we;
reg         ram_rd;
wire  [7:0] ram_dout;
wire        ram_ready;

always_comb begin
	casex({dma, tape_req, page_special, addr[15:14]})
		'b1X_X_XX: ram_addr = ioctl_addr;
		'b01_X_XX: ram_addr = tape_addr;
		'b00_0_00: ram_addr = { 4'b1000, page_rom,   addr[13:0]}; //ROM
		'b00_0_01: ram_addr = {        3'd5,         addr[13:0]}; //Non-special page modes
		'b00_0_10: ram_addr = {        3'd2,         addr[13:0]};
		'b00_0_11: ram_addr = {    page_ram,         addr[13:0]};
		'b00_1_00: ram_addr = { |page_reg_plus3[2:1],                      2'b00, addr[13:0]}; //Special page modes
		'b00_1_01: ram_addr = { |page_reg_plus3[2:1], &page_reg_plus3[2:1], 1'b1, addr[13:0]};
		'b00_1_10: ram_addr = { |page_reg_plus3[2:1],                      2'b10, addr[13:0]};
		'b00_1_11: ram_addr = { ~page_reg_plus3[2] & page_reg_plus3[1],    2'b11, addr[13:0]};
	endcase

	casex({dma, tape_req})
		'b1X: ram_din = ioctl_dout;
		'b01: ram_din = 0;
		'b00: ram_din = cpu_dout;
	endcase

	casex({dma, tape_req})
		'b1X: ram_rd = 0;
		'b01: ram_rd = ~nMREQ;
		'b00: ram_rd = ~nMREQ & ~nRD;
	endcase

	casex({dma, tape_req})
		'b1X: ram_we = ioctl_wr;
		'b01: ram_we = 0;
		'b00: ram_we = (page_special | addr[15] | addr[14] | ((plusd_mem | mf128_mem) & addr[13])) & ~nMREQ & ~nWR;
	endcase
end

sdram ram
(
	.*,
	.init(~locked),
	.clk(clk_sys),
	.dout(ram_dout),
	.din (ram_din),
	.addr(ram_addr),
	.wtbt(0),
	.we(ram_we),
	.rd(ram_rd),
	.ready(ram_ready)
);

wire vram_we = (ram_addr[24:16] == 1) & ram_addr[14];
dpram #(.ADDRWIDTH(15)) vram
(
    .clock(clk_sys),

    .address_a({ram_addr[15], ram_addr[13:0]}),
    .data_a(ram_din),
    .wren_a(ram_we & vram_we),

    .address_b(vram_addr),
    .data_b(0),
    .wren_b(0),
    .q_b(vram_dout)
);

wire  [7:0] rom_dout;
wire        rom_sel = (ram_addr[24:18] == 'b1000);
dpram #(.ADDRWIDTH(18), .NUMWORDS(180224), .MEM_INIT_FILE("bios.mif")) rom
(
    .clock(clk_sys),
	 .address_a(ram_addr[17:0]),
	 .data_a(ram_din),
	 .wren_a(rom_sel && ram_we),
	 .q_a(rom_dout),
	 
	 .address_b(0),
	 .data_b(0),
	 .wren_b(0)
);

reg        zx48;
reg        p512;
reg        pf1024;
reg        plus3;
reg        page_scr_copy;
reg        shadow_rom;
reg  [7:0] page_reg;
reg  [7:0] page_reg_plus3;
wire       page_disable = zx48 | page_reg[5];
wire       page_scr     = page_reg[3];
wire [5:0] page_ram     = {page_128k, page_reg[2:0]};
wire       page_write   = ~addr[15] & ~addr[1] & (addr[14] | ~plus3) & ~page_disable;
wire       page_write_plus3 = ~addr[1] & addr[12] & ~addr[13] & ~addr[14] & ~addr[15] & plus3 & ~page_disable;
wire       page_special = page_reg_plus3[0];
reg  [2:0] page_128k;

reg  [3:0] page_rom;
wire       active_48_rom = zx48 | (page_reg[4] & ~plus3) | (plus3 & page_reg[4] & page_reg_plus3[2] & ~page_special);

always_comb begin
	casex({shadow_rom, trdos_en, plusd_mem, mf128_mem, plus3})
		'b1XXXX: page_rom <=   4'b0000; //shadow
		'b01XXX: page_rom <=   4'b0001; //trdos
		'b001XX: page_rom <=   4'b1000; //plusd
		'b0001X: page_rom <= { 2'b10, plus3, ~plus3 }; //MF128/+3
		'b00001: page_rom <= { 2'b01, page_reg_plus3[2], page_reg[4] }; //+3
		'b00000: page_rom <= { 3'b001, zx48 | page_reg[4] }; //up to +2
	endcase
end

always @(posedge clk_sys) begin
	reg old_wr, old_m1, old_reset;
	reg [2:0] rmod;

	old_wr <= io_wr;
	old_m1 <= m1;
	
	old_reset <= reset;
	if(~old_reset & reset) rmod <= mod;

	if(reset) begin
		page_scr_copy <= 0;
		page_reg    <= 0;
		page_reg_plus3 <= 0; 
		page_128k   <= 0;
		page_reg[4] <= Fn[10];
		page_reg_plus3[2] <= Fn[10];
		shadow_rom <= shdw_reset & ~plusd_en;
		if(Fn[10] && (rmod == 1)) begin
			p512   <= 0;
			pf1024 <= 0;
			zx48   <= ~plus3;
		end else begin
			p512  <= (status[12:10] == 1);
			pf1024<= (status[12:10] == 2);
			zx48  <= (status[12:10] == 3);
			plus3 <= (status[12:10] == 4);
		end
	end else begin
		if(m1 && ~old_m1 && addr[15:14]) shadow_rom <= 0;
		if(m1 && ~old_m1 && ~plusd_en && ~mod[0] && (addr == 'h66) && ~plus3) shadow_rom <= 1; 

		if(io_wr & ~old_wr) begin
			if(page_write) begin
				page_reg  <= cpu_dout;
				if(p512)  page_128k[1:0] <= cpu_dout[7:6];
				if(~plusd_mem) page_scr_copy <= page_reg[3];
			end else if (page_write_plus3) begin
				page_reg_plus3 <= cpu_dout; 
			end
			if(pf1024 & (addr == 'hDFFD)) page_128k <= cpu_dout[2:0];
		end
	end
end


////////////////////  ULA PORT  ///////////////////
reg [2:0] border_color;
reg       ear_out;
reg       mic_out;

wire ula_we = ~addr[0] & ~nIORQ & ~nWR & nM1;
always @(posedge clk_sys) begin
	reg old_we;
	old_we <= ula_we;

	if(reset) {ear_out, mic_out} <= 2'b00;
	else if(ula_we & ~old_we) begin
		border_color <= cpu_dout[2:0];
		ear_out <= cpu_dout[4]; 
		mic_out <= cpu_dout[3];
	end
end


////////////////////   AUDIO   ///////////////////
wire [7:0] sound_data;
wire [7:0] psg_ch_a;
wire [7:0] psg_ch_b;
wire [7:0] psg_ch_c;
wire       psg_enable = addr[0] & addr[15] & ~addr[1];
wire       psg_we     = psg_enable & ~nIORQ & ~nWR & nM1;
reg        psg_reset;

ym2149 ym2149
(
	.CLK(clk_sys),
	.CE(ce_psg),
	.RESET(reset | psg_reset),
	.BDIR(psg_we),
	.BC(addr[14]),
	.DI(cpu_dout),
	.DO(sound_data),
	.CHANNEL_A(psg_ch_a),
	.CHANNEL_B(psg_ch_b),
	.CHANNEL_C(psg_ch_c),
	.SEL(0),
	.MODE(0),

	.IOA_in(0),
	.IOB_in(0)
);

assign AUDIO_L = {{1'b0, psg_ch_a, 1'b0} + {2'b00, psg_ch_b} + {2'b00, ear_out, mic_out, tape_in, 5'b00000}, 6'd0};
assign AUDIO_R = {{1'b0, psg_ch_c, 1'b0} + {2'b00, psg_ch_b} + {2'b00, ear_out, mic_out, tape_in, 5'b00000}, 6'd0};


////////////////////   VIDEO   ///////////////////
wire        ce_cpu_sn;
wire        ce_cpu_sp;
wire [14:0] vram_addr;
wire  [7:0] vram_dout;
wire  [7:0] port_ff;
wire        ulap_sel;
wire  [7:0] ulap_dout;
wire  [1:0] ulap_tmx_ena = {~status[13], ~status[14]} & {~trdos_en, ~trdos_en};

reg mZX, m128;
always_comb begin
	case(status[9:8])
		      0: {mZX, m128} <= 2'b10;
		      1: {mZX, m128} <= 2'b11;
		default: {mZX, m128} <= 2'b00;
	endcase
end

video video(.*, .ce_pix(CE_PIXEL), .din(cpu_dout), .page_ram(page_ram[2:0]), .scale(status[16:15]));


////////////////////   HID   ////////////////////
wire [11:1] Fn;
wire  [2:0] mod;
wire  [4:0] key_data;
keyboard kbd( .* );

reg         mouse_sel;
wire  [7:0] mouse_data;
mouse mouse( .*, .reset(cold_reset), .addr(addr[10:8]), .sel(), .dout(mouse_data));

always @(posedge clk_sys) begin
	if(joystick_0[5:0]) mouse_sel <= 0;
	if(~ps2_mouse_clk) mouse_sel <= 1;
end


//////////////////   MF128   ///////////////////
reg         mf128_mem;
reg         mf128_en; // enable MF128 page-in from NMI till reset (or soft off)
wire        mf128_port = ~addr[6] & addr[5] & addr[4] & addr[1];
// read paging registers saved in MF3 (7f3f, 1f3f)
wire        mf3_port = mf128_port & ~addr[7] & (addr[12:8] == 'h1f) & plus3 & mf128_en;

always @(posedge clk_sys) begin
	reg old_m1, old_rd, old_wr;

	old_rd <= io_rd;
	old_wr <= io_wr;
	if(reset) {mf128_mem, mf128_en} <= 0;
	else if(~old_rd & io_rd) begin
		//page in/out for port IN
		if(mf128_port) mf128_mem <= (addr[7] ^ plus3) & mf128_en;
	end else if(~old_wr & io_wr) begin
		//Soft hide
		if(mf128_port) mf128_en <= addr[7] & mf128_en;
	end

	old_m1 <= m1;
	if(~old_m1 & m1 & mod[0] & (addr == 'h66)) {mf128_mem, mf128_en} <= 2'b11;
end

///////////////////   FDC   ///////////////////
reg         plusd_en;
reg         plusd_mem;
wire        plusd_ena = plusd_stealth ? plusd_mem : plusd_en;
wire        fdd_sel2 = plusd_ena & &addr[7:5] & ~addr[2] & &addr[1:0];

reg         trdos_en;
wire  [7:0] wd_dout;
wire        fdd_rd;
reg         fdd_ready;
reg         fdd_drive1;
reg         fdd_side;
reg         fdd_reset;
wire        fdd_intrq;
wire        fdd_drq;
wire        fdd_sel  = trdos_en & addr[2] & addr[1];
reg         fdd_ro;
wire  [7:0] wdc_dout = (addr[7] & ~plusd_en) ? {fdd_intrq, fdd_drq, 6'h3F} : wd_dout;

reg         plus3_fdd_ready;
wire        plus3_fdd = ~addr[1] & addr[13] & ~addr[14] & ~addr[15] & plus3 & ~page_disable;
wire [7:0]  u765_dout;

wire  [7:0] fdd_dout = plus3_fdd ? u765_dout : wdc_dout; 

//
// current +D implementation notes:
// 1) all +D ports (except page out port) are disabled if +D memory isn't paged in.
// 2) only possible way to page in is through hooks at h08, h3A, h66 addresses.
//
// This may break compatibility with some apps written specifically for +D using 
// direct port access (badly written apps), but won't introduce
// incompatibilities with +D unaware apps.
//
wire        plusd_stealth = 1;

// read video page.
// available for MF128 and PlusD(patched).
wire        portBF = mf128_port & addr[7] & (mf128_mem | plusd_mem);

always @(posedge clk_sys) begin
	reg old_wr, old_rd;
	reg old_mounted;
	reg old_m1;

	if(cold_reset) {plus3_fdd_ready, fdd_ready, plusd_en} <= 0;
	if(reset)      {plusd_mem, trdos_en} <= 0;

	old_mounted <= img_mounted;
	if(~old_mounted & img_mounted) begin
		fdd_ro    <= img_readonly;

	   //Only TRDs on +3
		fdd_ready <= (!ioctl_index[7:6] & plus3) | ~plus3;
		plusd_en  <= |ioctl_index[7:6] & ~plus3;
		//DSK only for +3
		plus3_fdd_ready <= plus3 & (ioctl_index[7:6] == 2); 
	end

	old_rd <= io_rd;
	old_wr <= io_wr;
	old_m1 <= m1;
	psg_reset <= 0;

	if(plusd_en) begin
		trdos_en <= 0;
		if(~old_wr & io_wr  & (addr[7:0] == 'hEF) & plusd_ena) {fdd_side, fdd_drive1} <= {cpu_dout[7], cpu_dout[1:0] != 2};
		if(~old_wr & io_wr  & (addr[7:0] == 'hE7)) plusd_mem <= 0;
		if(~old_rd & io_rd  & (addr[7:0] == 'hE7) & ~plusd_stealth) plusd_mem <= 1;
		if(~old_m1 & m1 & ((addr == 'h08) | (addr == 'h3A) | (~mod[0] & (addr == 'h66)))) {psg_reset,plusd_mem} <= {(addr == 'h66), 1'b1};
	end else begin
		plusd_mem <= 0;
		if(~old_wr & io_wr & fdd_sel & addr[7]) {fdd_side, fdd_reset, fdd_drive1} <= {~cpu_dout[4], ~cpu_dout[2], !cpu_dout[1:0]};
		if(m1 && ~old_m1) begin
			if(addr[15:14]) trdos_en <= 0;
				else if((addr[13:8] == 'h3D) & active_48_rom) trdos_en <= 1;
				//else if(~mod[0] & (addr == 'h66)) trdos_en <= 1;
		end
	end
end

wd1793 #(1) fdd
(
	.clk_sys(clk_sys),
	.ce(ce_cpu),
	.reset((fdd_reset & ~plusd_en) | reset),
	.io_en((fdd_sel2 | (fdd_sel & ~addr[7])) & ~nIORQ & nM1),
	.rd(~nRD),
	.wr(~nWR),
	.addr(plusd_en ? addr[4:3] : addr[6:5]),
	.din(cpu_dout),
	.dout(wd_dout),
	.drq(fdd_drq),
	.intrq(fdd_intrq),

	.img_mounted(img_mounted),
	.img_size(img_size[19:0]),
	.sd_lba(sd_lba_wd),
	.sd_rd(sd_rd_wd),
	.sd_wr(sd_wr_wd), 
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din_wd), 
	.sd_buff_wr(sd_buff_wr),

	.wp(fdd_ro),

	.size_code(plusd_en ? 3'd4 : 3'd1),
	.layout(ioctl_index[7:6] == 1),
	.side(fdd_side),
	.ready(fdd_drive1 & fdd_ready),

	.input_active(0),
	.input_addr(0),
	.input_data(0),
	.input_wr(0),
	.buff_din(0)
);


u765 u765
(
	.clk_sys(clk_sys),
	.reset(reset),
	.a0(addr[12]),
	.nRD(plus3_fdd & ~nIORQ & nM1 & nRD),
	.nWR(plus3_fdd & ~nIORQ & nM1 & nWR),
	.din(cpu_dout),
	.dout(u765_dout),

	.img_mounted(img_mounted),
	.img_size(img_size[19:0]),
	.sd_lba(sd_lba_plus3),
	.sd_rd(sd_rd_plus3),
	.sd_wr(sd_wr_plus3),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din_plus3),
	.sd_buff_wr(sd_buff_wr)
);

///////////////////   TAPE   ///////////////////
wire [24:0] tape_addr = 25'h400000 + tape_addr_raw;
wire [24:0] tape_addr_raw;
wire        tape_req;
wire        tape_dout_en;
wire        tape_turbo;
wire  [7:0] tape_dout;
wire        tape_led;
wire        tape_active;
wire        tape_loaded;
wire        tape_in;
wire        tape_vin;

smart_tape tape
(
	.*,
	.reset(reset & ~Fn[10]),
	.ce(ce_cpu),

	.turbo(tape_turbo),
	.pause(Fn[1]),
	.prev(Fn[2]),
	.next(Fn[3]),
	.audio_out(tape_vin),
	.led(tape_led),
	.active(tape_active),
	.available(tape_loaded),
	.req_hdr((reg_DE == 'h11) & !reg_A),

	.buff_rd_en(~nRFSH),
	.buff_rd(tape_req),
	.buff_addr(tape_addr_raw),
	.buff_din(ram_dout),

	.ioctl_download(ioctl_download & (ioctl_index[4:0] == 2)),
	.tape_size(ioctl_addr - 25'h400000 + 1'b1),
	.tape_mode(!ioctl_index[7:6]),

	.m1(~nM1 & ~nMREQ),
	.rom_en(active_48_rom),
	.dout_en(tape_dout_en),
	.dout(tape_dout)
);

reg tape_loaded_reg = 0;
always @(posedge clk_sys) begin
	int timeout = 0;
	
	if(tape_loaded) begin
		tape_loaded_reg <= 1;
		timeout <= 100000000;
	end else begin
		if(timeout) begin
			timeout <= timeout - 1;
		end else begin
			tape_loaded_reg <= 0;
		end
	end
end

assign tape_in = tape_loaded_reg ? tape_vin : TAPE_IN;

endmodule

module dpram #(parameter DATAWIDTH=8, ADDRWIDTH=8, NUMWORDS=1<<ADDRWIDTH, MEM_INIT_FILE="")
(
	input	                 clock,

	input	 [ADDRWIDTH-1:0] address_a,
	input	 [DATAWIDTH-1:0] data_a,
	input	                 wren_a,
	output [DATAWIDTH-1:0] q_a,

	input	 [ADDRWIDTH-1:0] address_b,
	input	 [DATAWIDTH-1:0] data_b,
	input	                 wren_b,
	output [DATAWIDTH-1:0] q_b
);

altsyncram	altsyncram_component (
			.address_a (address_a),
			.address_b (address_b),
			.clock0 (clock),
			.data_a (data_a),
			.data_b (data_b),
			.wren_a (wren_a),
			.wren_b (wren_b),
			.q_a (q_a),
			.q_b (q_b),
			.aclr0 (1'b0),
			.aclr1 (1'b0),
			.addressstall_a (1'b0),
			.addressstall_b (1'b0),
			.byteena_a (1'b1),
			.byteena_b (1'b1),
			.clock1 (1'b1),
			.clocken0 (1'b1),
			.clocken1 (1'b1),
			.clocken2 (1'b1),
			.clocken3 (1'b1),
			.eccstatus (),
			.rden_a (1'b1),
			.rden_b (1'b1));
defparam
	altsyncram_component.wrcontrol_wraddress_reg_b = "CLOCK0",
	altsyncram_component.address_reg_b = "CLOCK0",
	altsyncram_component.indata_reg_b = "CLOCK0",
	altsyncram_component.numwords_a = NUMWORDS,
	altsyncram_component.numwords_b = NUMWORDS,
	altsyncram_component.widthad_a = ADDRWIDTH,
	altsyncram_component.widthad_b = ADDRWIDTH,
	altsyncram_component.width_a = DATAWIDTH,
	altsyncram_component.width_b = DATAWIDTH,
	altsyncram_component.width_byteena_a = 1,
	altsyncram_component.width_byteena_b = 1,

	altsyncram_component.init_file = MEM_INIT_FILE, 
	altsyncram_component.clock_enable_input_a = "BYPASS",
	altsyncram_component.clock_enable_input_b = "BYPASS",
	altsyncram_component.clock_enable_output_a = "BYPASS",
	altsyncram_component.clock_enable_output_b = "BYPASS",
	altsyncram_component.intended_device_family = "Cyclone V",
	altsyncram_component.lpm_type = "altsyncram",
	altsyncram_component.operation_mode = "BIDIR_DUAL_PORT",
	altsyncram_component.outdata_aclr_a = "NONE",
	altsyncram_component.outdata_aclr_b = "NONE",
	altsyncram_component.outdata_reg_a = "UNREGISTERED",
	altsyncram_component.outdata_reg_b = "UNREGISTERED",
	altsyncram_component.power_up_uninitialized = "FALSE",
	altsyncram_component.read_during_write_mode_mixed_ports = "DONT_CARE",
	altsyncram_component.read_during_write_mode_port_a = "NEW_DATA_NO_NBE_READ",
	altsyncram_component.read_during_write_mode_port_b = "NEW_DATA_NO_NBE_READ";


endmodule
