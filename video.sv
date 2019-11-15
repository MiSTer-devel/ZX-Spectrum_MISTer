//
//
// Spectrum Video Controller implementation
//   - ZX48, ZX128, Pentagon 128 timings
//   - ULA+ v1.1 programmable palette with extended Timex control.
//   - Timex video modes
// 
// Copyright (c) 2016 Sorgelig
//
// 
// This source file is free software: you can redistribute it and/or modify 
// it under the terms of the GNU General Public License as published 
// by the Free Software Foundation, either version 3 of the License, or 
// (at your option) any later version. 
// 
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License 
// along with this program.  If not, see <http://www.gnu.org/licenses/>. 
//

`timescale 1ns / 1ps

module video
(
	input         reset,

	input         clk_sys,
	input         clk_vid,

	input         ce_7mp,
	input         ce_7mn,
	output reg    ce_cpu_sp,
	output reg    ce_cpu_sn,
	
	output        ce_pix,

	// CPU interfacing
	input  [15:0] addr,
	input   [7:0] din,
	input         nMREQ,
	input         nIORQ,
	input         nRFSH,
	input         nWR,
	output reg    nINT,

	// VRAM interfacing
	output [14:0] vram_addr,
	input   [7:0] vram_dout,
	output  [7:0] port_ff,
	
	// ULA+
	input   [1:0] ulap_tmx_ena,
	output        ulap_sel,
	output  [7:0] ulap_dout,

	// Misc. signals
	input         mZX,
	input         m128,
	input         page_scr,
	input   [2:0] page_ram,
	input   [2:0] border_color,
	input   [1:0] scale,
	input         forced_scandoubler,
	input         wide,
	inout  [21:0] gamma_bus,

	// Video outputs
	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_VS,
	output        VGA_HS,
	output        VGA_DE
);

assign vram_addr = vaddr;
assign nINT      = ~INT;
assign port_ff   = tmx_using_ff ? tmx_cfg : mZX ? ff_data : 8'hFF;

// Pixel clock
reg [8:0] hc = 0;
reg [8:0] vc = 0;
reg       mode512;
always @(posedge clk_sys) begin
	reg m512;
	reg sync;

	if(ce_7mp) begin
		if((vc<192) || (hc<256)) m512 <= (m512 | tmx_hi);
		if (hc==((mZX && m128) ? 455 : 447)) begin
			hc <= 0;
			if (vc == (!mZX ? 319 : m128 ? 310 : 311)) begin 
				vc <= 0;
				FlashCnt <= FlashCnt + 1'd1;
			end else begin
				vc <= vc + 1'd1;
			end
			if(mZX ? (vc == 240) : (vc == 248)) begin
				mode512 <= m512;
				m512 <= 0;
			end
		end else begin
			hc <= hc + 1'd1;
		end
		hiSRegister <= {hiSRegister[14:0],1'b0};
	end
	if(ce_7mn) begin
		sync <= 0;
		if(!mZX) begin
			if(hc == 312) HBlank <= 1;
			if(hc == 338) sync   <= 1;
			if(hc == 370) HSync  <= 0;
			if(hc == 420) HBlank <= 0;
		end else if(m128) begin
			if(hc == 312) HBlank <= 1;
			if(hc == 340) sync   <= 1; //ULA 6C
			if(hc == 372) HSync  <= 0; //ULA 6C
			if(hc == 424) HBlank <= 0;
		end else begin
			if(hc == 300) HBlank <= 1;
			if(hc == 336) sync   <= 1; //ULA 5C
			if(hc == 368) HSync  <= 0; //ULA 5C
			if(hc == 428) HBlank <= 0;
		end

		if(wide) HBlank <= !(hc < 312 || hc >= ((mZX && m128) ? 455-33 : 447-33));

		if(sync) begin
			HSync <= 1;
			if(mZX) begin
				if(vc == 236) VBlank <= 1;
				if(vc == 240) VSync  <= 1;
				if(vc == 244) VSync  <= 0;
				if(vc == 264) VBlank <= 0;
			end else begin
				if(vc == 236) VBlank <= 1;
				if(vc == 248) VSync  <= 1;
				if(vc == 256) VSync  <= 0;
				if(vc == 272) VBlank <= 0;
			end
			if(wide) VBlank <= !(vc < (193) || vc >= (!mZX ? 319-4 : m128 ? 310-4 : 311-4));
		end

		if( mZX && (vc == 248) && (hc == (m128 ? 8 : 4))) INT <= 1;
		if(!mZX && (vc == 239) && (hc == 326)) INT <= 1;

		if(INT)  INTCnt <= INTCnt + 1'd1;
		if(!INTCnt) INT <= 0;

		if ((hc[3:0] == 4) || (hc[3:0] == 12)) begin
			SRegister <= VidEN ? bits : 8'd0;
			hiSRegister <= VidEN ? {bits, attr} : 16'd0;
			AttrOut <= tmx_hi ? hiattr : VidEN ? attr : {2'b00,border_color,border_color};
		end else begin
			SRegister   <= {SRegister[6:0],   1'b0};
			hiSRegister <= {hiSRegister[14:0],1'b0};
		end

		//1T update for border in Pentagon mode
		if(!mZX & ((hc<12) | (hc>267) | (vc>=192))) AttrOut <= tmx_hi ? hiattr : {2'b00,border_color,border_color};

		if(hc[3]) VidEN <= ~Border;
	
		if(!Border) begin
			casex({tmx_cfg[1],hc[3:0]})
				5'b01000,
				5'b01100: vaddr <= {stdpage ? page_scr : tmx_cfg[2],tmx_cfg[0],vc[7:6],vc[2:0],vc[5:3],hc[7:4], hc[2]};
				5'b11000,
				5'b11100: vaddr <= {stdpage ? page_scr : tmx_cfg[0],1'b0,vc[7:6],vc[2:0],vc[5:3],hc[7:4], hc[2]};
				5'bX1001,
				5'bX1101: begin bits <= vram_dout; ff_data <= vram_dout; end
				5'b01010,
				5'b01110: vaddr <= {stdpage ? page_scr : tmx_cfg[2],tmx_cfg[0],3'b110,vc[7:3],hc[7:4],hc[2]};
				5'b11010,
				5'b11110: vaddr <= {stdpage ? page_scr : tmx_cfg[0],1'b1,vc[7:6],vc[2:0],vc[5:3],hc[7:4], hc[2]};
				5'bX1011,
				5'bX1111: begin attr <= vram_dout; ff_data <= vram_dout; end
			endcase
		end

		if (hc[3:0] == 1) ff_data <= 255;
	end
end

wire [7:0] hipalette[8] = '{8'b01111000, 8'b01110001, 8'b01101010, 8'b01100011, 
								    8'b01011100, 8'b01010101, 8'b01001110, 8'b01000111};

reg        INT    = 0;
reg  [5:0] INTCnt = 1;
reg  [7:0] ff_data;
reg        HBlank = 1;
reg        VBlank = 1;
reg        HSync;
reg        VSync;

reg  [7:0] SRegister;
reg [15:0] hiSRegister;
reg [14:0] vaddr;

reg  [7:0] AttrOut;
reg  [4:0] FlashCnt;

wire       Border = ((vc[7] & vc[6]) | vc[8] | hc[8]);
reg        VidEN = 0;

reg  [7:0] bits;
reg [15:0] hibits;
reg  [7:0] attr;
wire [7:0] hiattr  = hipalette[tmx_cfg[5:3]];
wire       stdpage = tmx_using_ff | ~tmx_ena;


wire       I,G,R,B;
wire       Pixel = tmx_hi ? hiSRegister[15] : SRegister[7] ^ (AttrOut[7] & FlashCnt[4]);
assign     {I,G,R,B} = Pixel ? {AttrOut[6],AttrOut[2:0]} : {AttrOut[6],AttrOut[5:3]};
wire [7:0] color = palette[(tmx_hi ? hiSRegister[15] : SRegister[7]) ? {AttrOut[7:6],1'b0,AttrOut[2:0]} : {AttrOut[7:6],1'b1,AttrOut[5:3]}];

reg  [3:0] Rx, Gx, Bx;
always_comb casex({HBlank | VSync, ulap_ena, ulap_mono})
	'b1XX: {Gx,Rx,Bx} <= 0;
	'b00X: {Gx,Rx,Bx} <= {G, {3{I & G}}, R, {3{I & R}}, B, {3{I & B}}};
	'b010: {Gx,Rx,Bx} <= {color[7:5], color[7], color[4:2], color[4], color[1:0], color[1:0]};
	'b011: {Gx,Rx,Bx} <= {color, 4'b0000};
endcase

wire ce_sys = ce_7mp | (mode512 & ce_7mn);
reg [1:0] ce_sys2;
always @(posedge clk_sys) ce_sys2 <= {ce_sys2[0],ce_sys};

reg ce_vid;
always @(posedge clk_vid) begin
	reg ce1, ce2, ce3;
	
	ce1 <= |ce_sys2;
	ce2 <= ce1;
	ce3 <= ce2;
	ce_vid <= ce3;
end

video_mixer #(.LINE_LENGTH(896), .HALF_DEPTH(1), .GAMMA(1)) video_mixer
(
	.*,

	.ce_pix(ce_vid),
	.ce_pix_out(ce_pix),

	.hq2x(scale == 1),
	.scanlines(0),
	.scandoubler(scale || forced_scandoubler),

	.R(Rx),
	.G(Gx),
	.B(Bx),
	.mono(ulap_ena & ulap_mono)
);

///////////////////////////////////////////////////////////////////////////////

reg  CPUClk;
reg  ioreqtw3;
reg  mreqt23;

wire ioreq_n    = (addr[0] & ~ulap_acc) | nIORQ;
wire ulaContend = (hc[2] | hc[3]) & ~Border & CPUClk & ioreqtw3;
wire memContend = nRFSH & ioreq_n & mreqt23 & ((addr[15:14] == 2'b01) | (m128 & (addr[15:14] == 2'b11) & page_ram[0]));
wire ioContend  = ~ioreq_n;
wire next_clk   = hc[0] | (mZX & ulaContend & (memContend | ioContend));

always @(posedge clk_sys) begin
	ce_cpu_sp <= 0;
	ce_cpu_sn <= 0;
	if(ce_7mp) begin
		CPUClk <= next_clk;

		if(~CPUClk &  next_clk) ce_cpu_sp <= 1;
		if( CPUClk & ~next_clk) ce_cpu_sn <= 1;

		if(~CPUClk &  next_clk) begin
			ioreqtw3 <= ioreq_n;
			mreqt23  <= nMREQ;
		end
	end
end

/////////////////  ULA+, Timex  ///////////////////

assign     ulap_dout = ulap_group ? {ulap_mono, ulap_ena} : palette[pal_addr];
assign     ulap_sel  = ulap_acc & addr[14] & ulap_tmx_ena[0];

wire       ulap_acc = ({addr[15], 1'b0, addr[13:0]} == 'hBF3B);
wire       io_wr = ~nIORQ & ~nWR;
reg  [5:0] pal_addr;
reg        ulap_group;
reg        ulap_ena;
reg        ulap_mono;
reg  [7:0] palette[64];

reg  [5:0] tmx_cfg;
reg        tmx_ena;
reg        tmx_using_ff;
wire       tmx_hi = &{tmx_ena, tmx_cfg[2:1]};

always @(posedge clk_sys) begin
	reg old_wr;
	old_wr <= io_wr;

	if(reset) begin
		{ulap_ena, tmx_ena, tmx_using_ff, tmx_cfg} <= 0;
		palette <= '{default:0};
	end else if(~old_wr & io_wr) begin
		if(ulap_acc & ulap_tmx_ena[0]) begin
			if(addr[14]) begin
				if(ulap_group) {ulap_mono,ulap_ena} <= din[1:0];
					else palette[pal_addr] <= din;
			end else begin
				case(din[7:6])
					0: {ulap_group, pal_addr} <= {1'b0, din[5:0]};
					1: begin
							ulap_group <= 1;
							if(!tmx_using_ff) {tmx_ena, tmx_cfg}  <= {|din[2:0], din[5:0]};
						end
					default: ;
				endcase
			end
		end
		if((addr[7:0] == 'hFF) & ulap_tmx_ena[1]) {tmx_using_ff, tmx_ena, tmx_cfg} <= {1'b1, |din[2:0], din[5:0]};
	end
end

endmodule
