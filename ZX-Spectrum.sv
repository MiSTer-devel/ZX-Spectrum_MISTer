//============================================================================
//  Sinclair ZX Spectrum for MiSTer
//  Copyright (C) 2017-2019 Sorgelig
//
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
	inout  [45:0] HPS_BUS,

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
	output        VGA_F1,
	output  [1:0] VGA_SL,

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

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
	output        SDRAM_nWE,

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

assign USER_OUT = '1;
assign VGA_F1 = 0;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign AUDIO_S   = 1;
assign AUDIO_MIX = status[3:2];

assign LED_USER  = ioctl_download | tape_led | tape_adc_act;
assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;

assign VIDEO_ARX = status[5:4] ? 8'd16 : 8'd4;
assign VIDEO_ARY = status[5:4] ? 8'd9  : 8'd3;

localparam ARCH_ZX48  = 5'b011_00; // ZX 48
localparam ARCH_ZX128 = 5'b000_01; // ZX 128/+2
localparam ARCH_ZX3   = 5'b100_01; // ZX 128 +3
localparam ARCH_P48   = 5'b011_10; // Pentagon 48
localparam ARCH_P128  = 5'b000_10; // Pentagon 128
localparam ARCH_P1024 = 5'b001_10; // Pentagon 1024

localparam CONF_BDI   = "(BDI)";
localparam CONF_PLUSD = "(+D) ";
localparam CONF_PLUS3 = "(+3) ";

`include "build_id.v"
localparam CONF_STR = {
	"Spectrum;;",
	"S,TRDIMGDSKMGT,Load Disk;",
	"-;",
	"F,TAPCSWTZX,Load Tape;",
	"O6,Fast tape load,On,Off;",
	"-;",
	"F,Z80,Load Snapshot;",
	"-;",
	"O89,Video timings,ULA-48,ULA-128,Pentagon;",
	"O45,Aspect ratio,Original,Wide,Zoom;",
	"OFG,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%;",
	"-;",
	"OKL,General Sound,512KB,1MB,2MB,Disabled;",
	"O23,Stereo mix,none,25%,50%,100%;",
	"-;",
	"OHJ,Joystick,Kempston,Sinclair I,Sinclair II,Sinclair I+II,Cursor;",
	"OMO,CPU Speed,Original,7MHz,14MHz,28MHz,56MHz;",
	"OD,Port #FF,Timex,SAA1099;",
	"OE,ULA+,Enabled,Disabled;",
	"OAC,Memory,Spectrum 128K/+2,Pentagon 1024K,Profi 1024K,Spectrum 48K,Spectrum +2A/+3;",
	"D0R0,Reset & apply;",
	"J,Fire 1,Fire 2;",
	"V,v",`BUILD_DATE
};


////////////////////   CLOCKS   ///////////////////

assign CLK_VIDEO = clk_56;

wire locked;
wire clk_sys, clk_56;
wire clk_aud = clk_56;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(SDRAM_CLK),
	.outclk_2(clk_56),
	.locked(locked)
);


(* direct_enable *) reg  ce_7mp;
(* direct_enable *) reg  ce_7mn;

reg  pause;
reg  cpu_en = 1;
reg  ce_cpu_tp;
reg  ce_cpu_tn;
(* direct_enable *) reg ce_tape;
(* direct_enable *) reg ce_wd1793;
(* direct_enable *) reg ce_u765;

(* direct_enable *) wire ce_cpu_p = cpu_en & cpu_p;
(* direct_enable *) wire ce_cpu_n = cpu_en & cpu_n;

wire cpu_p = ~&turbo ? ce_cpu_tp : ce_cpu_sp;
wire cpu_n = ~&turbo ? ce_cpu_tn : ce_cpu_sn;

always @(posedge clk_sys) begin
	reg [5:0] counter = 0;

	counter <=  counter + 1'd1;

	ce_7mp  <= !counter[3] & !counter[2:0];
	ce_7mn  <=  counter[3] & !counter[2:0];

	// split ce for relaxed fitting
	ce_cpu_tp <= !(counter & turbo);
	ce_tape   <= !(counter & turbo) & cpu_en;
	ce_wd1793 <= !(counter & turbo) & cpu_en;
	ce_u765   <= !(counter & turbo) & cpu_en;

	ce_cpu_tn <= !((counter & turbo) ^ turbo ^ turbo[4:1]);
end


wire [4:0] turbo_req;
always_comb begin
	casex({tape_active & ~status[6], status[24:22]})
		 'b1XXX: turbo_req = 5'b00001;
		 'b0001: turbo_req = 5'b01111;
		 'b0010: turbo_req = 5'b00111;
		 'b0011: turbo_req = 5'b00011;
		 'b0100: turbo_req = 5'b00001;
		default: turbo_req = 5'b11111;
	endcase
end

reg [2:0] speed_req;
reg       speed_set;
always @(posedge clk_sys) begin
	reg [9:4] old_Fn;
	old_Fn <= Fn[9:4];

	if(reset) pause <= 0;

	speed_set <= 0;
	if(!mod) begin
		if(~old_Fn[4] & Fn[4]) {speed_set,speed_req} <= 4'b1_000;
		if(~old_Fn[5] & Fn[5]) {speed_set,speed_req} <= 4'b1_001;
		if(~old_Fn[6] & Fn[6]) {speed_set,speed_req} <= 4'b1_010;
		if(~old_Fn[7] & Fn[7]) {speed_set,speed_req} <= 4'b1_011;
		if(~old_Fn[8] & Fn[8]) {speed_set,speed_req} <= 4'b1_100;
		if(~old_Fn[9] & Fn[9]) pause <= ~pause;
	end
end

reg [4:0] turbo = 5'b11111;
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
wire [10:0] ps2_key;
wire [24:0] ps2_mouse;

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
wire        ioctl_wait;

reg         status_set;
reg  [31:0] status_out;

wire [21:0] gamma_bus;

hps_io #(.STRLEN(($size(CONF_STR)>>3)+5)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

	.conf_str({CONF_STR, plus3_fdd_ready ? CONF_PLUS3 : plusd_en ? CONF_PLUSD : CONF_BDI}),

	.ps2_key(ps2_key),
	.ps2_mouse(ps2_mouse),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.buttons(buttons),
	.forced_scandoubler(forced_scandoubler),
	.status(status),
	.status_menumask({~need_apply}),
	.status_set(speed_set|arch_set|snap_hwset),
	.status_in({status[31:25], speed_set ? speed_req : 3'b000, status[21:13], arch_set ? arch : snap_hwset ? snap_hw : status[12:8], status[7:0]}),

	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),
	.img_mounted(img_mounted),
	.img_size(img_size),
	.img_readonly(img_readonly),

	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),
	
	.gamma_bus(gamma_bus)
);

reg  [2:0] cur_mode = 0;
wire       need_apply = status[12:10] != cur_mode;

always @(posedge clk_sys) begin
	if(reset) cur_mode <= status[12:10];
end


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

wire        cold_reset =((mod[2:1] == 1) & Fn[11]) | init_reset | arch_reset | snap_reset;
wire        warm_reset = (mod[2:1] == 2) & Fn[11];
wire        shdw_reset = (mod[2:1] == 3) & Fn[11] & ~plus3;

wire        io_wr = ~nIORQ & ~nWR & nM1;
wire        io_rd = ~nIORQ & ~nRD & nM1;
wire        m1    = ~nM1 & ~nMREQ;

wire[211:0]	cpu_reg;  // IFF2, IFF1, IM, IY, HL', DE', BC', IX, HL, DE, BC, PC, SP, R, I, F', A', F, A
wire [15:0] reg_DE  = cpu_reg[111:96];
wire  [7:0] reg_A   = cpu_reg[7:0];

T80pa cpu
(
	.RESET_n(~reset),
	.CLK(clk_sys),
	.CEN_p(ce_cpu_p),
	.CEN_n(ce_cpu_n),
	.INT_n(nINT),
	.NMI_n(~NMI),
	.BUSRQ_n(nBUSRQ),
	.M1_n(nM1),
	.MREQ_n(nMREQ),
	.IORQ_n(nIORQ),
	.RD_n(nRD),
	.WR_n(nWR),
	.RFSH_n(nRFSH),
	.BUSAK_n(nBUSACK),
	.A(addr),
	.DO(cpu_dout),
	.DI(cpu_din),
	.REG(cpu_reg),
	.DIR(snap_REG),
	.DIRSet(snap_REGSet)
);

always_comb begin
	casex({nMREQ, tape_dout_en, ~nM1 | nIORQ | nRD, fdd_sel | fdd_sel2 | plus3_fdd, mf3_port, addr[5:0]==8'h1F, portBF, gs_sel, psg_enable, ulap_sel, addr[0]})
		'b01XXXXXXXXX: cpu_din = tape_dout;
		'b00XXXXXXXXX: cpu_din = ram_dout;
		'b1X01XXXXXXX: cpu_din = fdd_dout;
		'b1X001XXXXXX: cpu_din = (addr[14:13] == 2'b11 ? page_reg : page_reg_plus3);
		'b1X0001XXXXX: cpu_din = mouse_sel ? mouse_data : {2'b00, joyk};
		'b1X00001XXXX: cpu_din = {page_scr_copy, 7'b1111111};
		'b1X000001XXX: cpu_din = gs_dout;
		'b1X0000001XX: cpu_din = (addr[14] ? sound_data : 8'hFF);
		'b1X00000001X: cpu_din = ulap_dout;
		'b1X000000001: cpu_din = port_ff;
		'b1X000000000: cpu_din = {1'b1, ~tape_in, 1'b1, key_data[4:0] & joy_kbd};
		'b1X1XXXXXXXX: cpu_din = 8'hFF;
	endcase
end

(* maxfan = 5 *) reg init_reset = 1;
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
reg  [24:0] ram_addr;
reg   [7:0] ram_din;
reg         ram_we;
reg         ram_rd;
wire  [7:0] ram_dout;
wire        ram_ready;

reg [24:0] load_addr;
always @(posedge clk_sys) load_addr <= ioctl_addr + (ioctl_index[4:0] ? 25'h400000 : 25'h150000);

reg load;
always @(posedge clk_sys) load <= (reset | ~nBUSACK) & ~nBUSRQ;

always_comb begin
	casex({snap_reset, load, tape_req, page_special, addr[15:14]})
		'b1XX_X_XX: ram_addr = snap_addr;
		'b01X_X_XX: ram_addr = load_addr;
		'b001_X_XX: ram_addr = tape_addr;
		'b000_0_00: ram_addr = { 3'b101, page_rom,    addr[13:0]}; //ROM
		'b000_0_01: ram_addr = {        3'd5,         addr[13:0]}; //Non-special page modes
		'b000_0_10: ram_addr = {        3'd2,         addr[13:0]};
		'b000_0_11: ram_addr = {    page_ram,         addr[13:0]};
		'b000_1_00: ram_addr = { |page_reg_plus3[2:1],                      2'b00, addr[13:0]}; //Special page modes
		'b000_1_01: ram_addr = { |page_reg_plus3[2:1], &page_reg_plus3[2:1], 1'b1, addr[13:0]};
		'b000_1_10: ram_addr = { |page_reg_plus3[2:1],                      2'b10, addr[13:0]};
		'b000_1_11: ram_addr = { ~page_reg_plus3[2] & page_reg_plus3[1],    2'b11, addr[13:0]};
	endcase

	casex({snap_reset, load, tape_req})
		'b1XX: ram_din = snap_data;
		'b01X: ram_din = ioctl_dout;
		'b001: ram_din = 0;
		'b000: ram_din = cpu_dout;
	endcase

	casex({load, tape_req})
		'b1X: ram_rd = 0;
		'b01: ram_rd = ~nMREQ;
		'b00: ram_rd = ~nMREQ & ~nRD;
	endcase

	casex({snap_reset, load, tape_req})
		'b1XX: ram_we = snap_wr;
		'b01X: ram_we = ioctl_wr;
		'b001: ram_we = 0;
		'b000: ram_we = (page_special | addr[15] | addr[14] | ((plusd_mem | mf128_mem) & addr[13])) & ~nMREQ & ~nWR;
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
	.word(0),
	.wr(ram_we),
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
    .q_b(vram_dout)
);

reg        zx48;
reg        p1024;
reg        pf1024;
reg        plus3;
reg        page_scr_copy;
reg        shadow_rom;
reg  [7:0] page_reg;
reg  [7:0] page_reg_plus3;
reg  [7:0] page_reg_p1024;
wire       page_disable = zx48 | (~p1024 & page_reg[5]) | (p1024 & page_reg_p1024[2] & page_reg[5]);
wire       page_scr     = page_reg[3];
wire [5:0] page_ram     = {page_128k, page_reg[2:0]};
wire       page_write   = ~addr[15] & ~addr[1] & (addr[14] | ~plus3) & ~page_disable; //7ffd
wire       page_write_plus3 = ~addr[1] & addr[12] & ~addr[13] & ~addr[14] & ~addr[15] & plus3 & ~page_disable; //1ffd
wire       page_special = page_reg_plus3[0];
wire       motor_plus3 = page_reg_plus3[3];
wire       page_p1024 = addr[15] & addr[14] & addr[13] & ~addr[12] & ~addr[3]; //eff7
reg  [2:0] page_128k;

reg  [3:0] page_rom;
wire       active_48_rom = zx48 | (page_reg[4] & ~plus3) | (plus3 & page_reg[4] & page_reg_plus3[2] & ~page_special);

always_comb begin
	casex({shadow_rom, trdos_en, plusd_mem, mf128_mem, plus3})
		'b1XXXX: page_rom <=   4'b0100; //shadow
		'b01XXX: page_rom <=   4'b0101; //trdos
		'b001XX: page_rom <=   4'b1100; //plusd
		'b0001X: page_rom <= { 2'b11, plus3, ~plus3 }; //MF128/+3
		'b00001: page_rom <= { 2'b10, page_reg_plus3[2], page_reg[4] }; //+3
		'b00000: page_rom <= { zx48, 2'b11, zx48 | page_reg[4] }; //up to +2
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
		page_reg_p1024 <= 0;
		page_128k   <= 0;
		page_reg[4] <= Fn[10];
		page_reg_plus3[2] <= Fn[10];
		shadow_rom <= shdw_reset & ~plusd_en;
		if(Fn[10] && (rmod == 1)) begin
			p1024  <= 0;
			pf1024 <= 0;
			zx48   <= ~plus3;
		end else begin
			p1024 <= (status[12:10] == 1);
			pf1024<= (status[12:10] == 2);
			zx48  <= (status[12:10] == 3);
			plus3 <= (status[12:10] == 4);
		end
	end else begin
		if(snap_REGSet) begin
			if((snap_hw == ARCH_ZX128) || (snap_hw == ARCH_P128) || (snap_hw == ARCH_ZX3)) page_reg <= snap_7ffd;
			if(snap_hw == ARCH_ZX3) page_reg_plus3 <= snap_1ffd;
		end
		else begin
			if(m1 && ~old_m1 && addr[15:14]) shadow_rom <= 0;
			if(m1 && ~old_m1 && ~plusd_en && ~mod[0] && (addr == 'h66) && ~plus3) shadow_rom <= 1; 

			if(io_wr & ~old_wr) begin
				if(page_write) begin
					page_reg  <= cpu_dout;
					if(p1024 & ~page_reg_p1024[2]) page_128k[2:0] <= { cpu_dout[5], cpu_dout[7:6] };
					if(~plusd_mem) page_scr_copy <= cpu_dout[3];
				end else if (page_write_plus3) begin
					page_reg_plus3 <= cpu_dout; 
				end
				if(pf1024 & (addr == 'hDFFD)) page_128k <= cpu_dout[2:0];
				if(p1024 & page_p1024) page_reg_p1024 <= cpu_dout;
			end
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
	
	if(snap_REGSet) border_color <= snap_border;
end


////////////////////   AUDIO   ///////////////////
wire  [7:0] sound_data;
wire [11:0] ts_l, ts_r;
wire        psg_enable = addr[0] & addr[15] & ~addr[1];
wire        psg_we     = psg_enable & ~nIORQ & ~nWR & nM1;
reg         psg_reset;
reg         psg_active;

wire        aud_reset = reset | psg_reset;

(* direct_enable *) reg  ce_ym;  //3.5MHz
always @(posedge clk_aud) begin
	reg [3:0] counter = 0;
	reg p1,p2,p3;
	
	p1 <= pause;
	p2 <= p1;
	p3 <= p2;

	counter <=  counter + 1'd1;
	ce_ym   <= !counter & ~p3;
end

// Turbosound FM: dual YM2203 chips
turbosound turbosound
(
	.RESET(aud_reset),
	.CLK(clk_aud),
	.CE(ce_ym),
	.BDIR(psg_we),
	.BC(addr[14]),
	.DI(cpu_dout),
	.DO(sound_data),
	.CHANNEL_L(ts_l),
	.CHANNEL_R(ts_r)
);

reg  ce_saa;  //8MHz
always @(posedge clk_aud) begin
	reg [3:0] counter = 0;

	counter <=  counter + 1'd1;
	if(counter == 6) counter <= 0;
	
	ce_saa <= !counter;
end

wire [7:0] saa_l;
wire [7:0] saa_r;

saa1099 saa1099
(
	.clk_sys(clk_aud),
	.ce(ce_saa),
	.rst_n(~aud_reset),
	.cs_n((addr[7:0] != 255) | nIORQ | ulap_tmx_ena[1]),
	.a0(addr[8]),
	.wr_n(nWR),
	.din(cpu_dout),
	.out_l(saa_l),
	.out_r(saa_r)
);

wire [7:0] gs_dout;
wire [14:0] gs_l, gs_r;

reg ce_gs = 0;
always @(posedge clk_aud) ce_gs <= ~ce_gs;

gs gs
(
	.RESET(aud_reset),
	.CLK(clk_aud),
	.CE(ce_gs),

	.A(addr[3]),
	.DI(cpu_dout),
	.DO(gs_dout),
	.CS_n(nIORQ | ~gs_sel),
	.WR_n(nWR),
	.RD_n(nRD),

	.MEM_ADDR(gs_mem_addr),
	.MEM_DI(gs_mem_din),
	.MEM_DO(gs_mem_dout | gs_mem_mask),
	.MEM_RD(gs_mem_rd),
	.MEM_WR(gs_mem_wr),
	.MEM_WAIT(~gs_mem_ready),

	.OUTL(gs_l),
	.OUTR(gs_r)
);

assign DDRAM_CLK = clk_sys;

wire [20:0] gs_mem_addr;
wire  [7:0] gs_mem_dout;
wire  [7:0] gs_mem_din;
wire        gs_mem_rd;
wire        gs_mem_wr;
wire        gs_mem_ready;
reg   [7:0] gs_mem_mask;

always_comb begin
	gs_mem_mask = 0;
	case(status[21:20])
		0: if(gs_mem_addr[20:19]) gs_mem_mask = 8'hFF;
		1: if(gs_mem_addr[20])    gs_mem_mask = 8'hFF;
	 2,3:                        gs_mem_mask = 0;
	endcase
end

ddram ddram
(
	.*,

	.addr(gs_mem_addr),
	.dout(gs_mem_dout),
	.din(gs_mem_din),
	.we(gs_mem_wr),
	.rd(gs_mem_rd),
	.ready(gs_mem_ready)
);

wire gs_sel = (addr[7:0] ==? 'b1011?011) & ~&status[21:20];

reg [11:0] audio_l, audio_r;
always @(posedge clk_aud) begin
	audio_l <= ts_l + {{3{gs_l[14]}}, gs_l[13:5]} + {2'b00, saa_l, 2'b00} + {3'b000, ear_out, mic_out, tape_in, 6'b000000};
	audio_r <= ts_r + {{3{gs_r[14]}}, gs_r[13:5]} + {2'b00, saa_r, 2'b00} + {3'b000, ear_out, mic_out, tape_in, 6'b000000};
end

compressor compressor
(
	clk_sys,
	audio_l, audio_r,
	AUDIO_L, AUDIO_R
);

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

wire [1:0] scale = status[16:15];
assign VGA_SL = {scale==3, scale==2};

video video(.*, .clk_vid(CLK_VIDEO), .ce_pix(CE_PIXEL), .din(cpu_dout), .page_ram(page_ram[2:0]), .wide(status[5]));

reg new_vmode = 0;
always @(posedge clk_sys) begin
	reg [1:0] vmode;
	
	vmode<=status[9:8];
	if(vmode != status[9:8]) new_vmode <= ~new_vmode;
end

////////////////////   HID   ////////////////////
wire [11:1] Fn;
wire  [2:0] mod;
wire  [4:0] key_data;
keyboard kbd( .* );

reg         mouse_sel;
wire  [7:0] mouse_data;
mouse mouse( .*, .reset(cold_reset), .addr(addr[10:8]), .sel(), .dout(mouse_data));

always @(posedge clk_sys) begin
	reg old_status = 0;
	old_status <= ps2_mouse[24];

	if(joyk) mouse_sel <= 0;
	if(old_status != ps2_mouse[24]) mouse_sel <= 1;
end

wire [2:0] jsel = status[19:17];

//kempston port 1F
wire [5:0] joyk   = !jsel ? (joystick_0[5:0] | joystick_1[5:0]) : 6'd0;

//sinclair 1 67890
wire [4:0] joys1 = ({5{jsel[0]}} & {joystick_0[1:0], joystick_0[2], joystick_0[3], joystick_0[4]}) | ({5{jsel[1:0]==1}} & {joystick_1[1:0], joystick_1[2], joystick_1[3], joystick_1[4]});

//sinclair 2 12345
wire [4:0] joys2 = ({5{jsel[1]}} & {joystick_1[4:2], joystick_1[0], joystick_1[1]})                | ({5{jsel[1:0]==2}} & {joystick_0[4:2],joystick_0[0],joystick_0[1]});

//cursor 56780
wire [4:0] joyc1 = {5{jsel[2]}} & ({joystick_0[2], joystick_0[3], joystick_0[0],1'b0, joystick_0[4]} | {joystick_1[2], joystick_1[3], joystick_1[0], 1'b0, joystick_1[4]});
wire [4:0] joyc2 = {5{jsel[2]}} & {joystick_0[1] | joystick_1[1], 4'b0000};

//map to keyboard
wire [4:0] joy_kbd = ({5{addr[12]}} | ~(joys1 | joyc1)) & ({5{addr[11]}} | ~(joys2 | joyc2));

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
		fdd_ready <= ((!ioctl_index[7:6] & plus3) | ~plus3) && (img_size != 0);
		plusd_en  <= (|ioctl_index[7:6] & ~plus3) && (img_size != 0);
		//DSK only for +3
		plus3_fdd_ready <= (plus3 & (ioctl_index[7:6] == 2)) && (img_size != 0);
	end

	old_rd <= io_rd;
	old_wr <= io_wr;
	old_m1 <= m1;

	psg_reset <= m1 && (addr == 'h66); // reset AY upon NMI.

	if(plusd_en) begin
		trdos_en <= 0;
		if(~old_wr & io_wr  & (addr[7:0] == 'hEF) & plusd_ena) {fdd_side, fdd_drive1} <= {cpu_dout[7], cpu_dout[1:0] != 2};
		if(~old_wr & io_wr  & (addr[7:0] == 'hE7)) plusd_mem <= 0;
		if(~old_rd & io_rd  & (addr[7:0] == 'hE7) & ~plusd_stealth) plusd_mem <= 1;
		if(~old_m1 & m1 & ((addr == 'h08) | (addr == 'h3A) | (~mod[0] & (addr == 'h66)))) plusd_mem <= 1;
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
	.ce(ce_wd1793),
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


u765 #(20'd1800,1) u765
(
	.clk_sys(clk_sys),
	.ce(ce_u765),
	.reset(reset),
	.a0(addr[12]),
	.ready(plus3_fdd_ready),
	.motor(motor_plus3),
	.available(2'b01),
	.nRD(~plus3_fdd | nIORQ | ~nM1 | nRD),
	.nWR(~plus3_fdd | nIORQ | ~nM1 | nWR),
	.din(cpu_dout),
	.dout(u765_dout),

	.img_mounted(img_mounted),
	.img_size(img_size[19:0]),
	.img_wp(fdd_ro),
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
	.ce(ce_tape),

	.turbo(tape_turbo),
	.mode48k(page_disable),
	.pause(Fn[1] & !mod),
	.prev(Fn[2] & !mod),
	.next(Fn[3] & !mod),
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
	.tape_size(ioctl_addr),
	.tape_mode(ioctl_index[7:6]),

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

assign tape_in = tape_loaded_reg ? tape_vin : tape_adc_act & tape_adc;

wire tape_adc, tape_adc_act;
ltc2308_tape ltc2308_tape
(
	.clk(CLK_50M),
	.ADC_BUS(ADC_BUS),
	.dout(tape_adc),
	.active(tape_adc_act)
);

//////////////////  ARCH SET  //////////////////

reg       arch_set = 0;
reg [4:0] arch;
reg       arch_reset = 0;
always @(posedge clk_sys) begin
	reg [7:0] timeout = 0;
	reg [6:1] old_Fn;
	reg       setwait = 0;

	arch_set <= 0;
	old_Fn <= Fn[6:1];
	if(mod == 2 && (old_Fn != Fn[6:1])) begin
		if(~old_Fn[1] & Fn[1]) {setwait,arch_set,arch} <= {2'b11,ARCH_ZX48 }; // Alt+F1 - ZX 48
		if(~old_Fn[2] & Fn[2]) {setwait,arch_set,arch} <= {2'b11,ARCH_ZX128}; // Alt+F2 - ZX 128/+2
		if(~old_Fn[3] & Fn[3]) {setwait,arch_set,arch} <= {2'b11,ARCH_ZX3  }; // Alt+F3 - ZX 128 +3
		if(~old_Fn[4] & Fn[4]) {setwait,arch_set,arch} <= {2'b11,ARCH_P48  }; // Alt+F4 - Pentagon 48
		if(~old_Fn[5] & Fn[5]) {setwait,arch_set,arch} <= {2'b11,ARCH_P128 }; // Alt+F5 - Pentagon 128
		if(~old_Fn[6] & Fn[6]) {setwait,arch_set,arch} <= {2'b11,ARCH_P1024}; // Alt+F6 - Pentagon 1024
	end

	if(timeout) timeout <= timeout - 1'd1;
	else arch_reset <= 0;

	if(setwait && (status[12:8] == arch)) begin
		timeout <= '1;
		arch_reset <= 1;
		setwait <= 0;
	end
end


//////////////////  SNAPSHOT  //////////////////

wire [211:0] snap_REG;
wire         snap_REGSet;
wire  [24:0] snap_addr;
wire   [7:0] snap_data;
wire         snap_wr;
wire         snap_reset;
wire         snap_hwset;
wire   [4:0] snap_hw;
wire  [31:0] snap_status;
wire   [2:0] snap_border;
wire   [7:0] snap_1ffd;
wire   [7:0] snap_7ffd;

snap_loader #(ARCH_ZX48, ARCH_ZX128, ARCH_ZX3, ARCH_P128) snap_loader
(
	.clk_sys(clk_sys),

	.ioctl_download(ioctl_download && ioctl_index == 4),
	.ioctl_addr(ioctl_addr),
	.ioctl_data(ioctl_dout),
	.ioctl_wr(ioctl_wr),
	.ioctl_wait(ioctl_wait),

	.ram_ready(ram_ready),

	.REG(snap_REG),
	.REGSet(snap_REGSet),

	.addr(snap_addr),
	.dout(snap_data),
	.wr(snap_wr),

   .reset(snap_reset),
   .hwset(snap_hwset),
   .hw(snap_hw),
   .hw_ack(status[12:8]),

   .border(snap_border),
   .reg_1ffd(snap_1ffd),
   .reg_7ffd(snap_7ffd)
);

endmodule
