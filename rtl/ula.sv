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

module ULA
(
	input         reset,

	input         clk_sys,	// master clock
	input         ce_7mp,
	input         ce_7mn,
	output        ce_cpu_sp,
	output        ce_cpu_sn,

	// CPU interfacing
	input  [15:0] addr,
	input   [7:0] din,
	input         nMREQ,
	input         nIORQ,
	input         nRFSH,
	input         nRD,
	input         nWR,
	output        nINT,

	// VRAM interfacing
	output [14:0] vram_addr,
	input   [7:0] vram_dout,
	output  [7:0] port_ff,
	
	// ULA+
	input         ulap_avail,
	output        ulap_sel,
	output  [7:0] ulap_dout,
	output reg    ulap_ena,
	output reg    ulap_mono,
	output  [7:0] ulap_color,

	// Timex mode
	input         tmx_avail,
	output reg    mode512,

	// Misc. signals
	input         snow_ena,
	input         mZX,
	input         m128,
	input         page_scr,
	input   [2:0] page_ram,
	input   [2:0] border_color,
	input         wide,
        input         hide_border,

	// Video outputs
	output reg    HSync,
	output reg    VSync,
	output reg    HBlank,
	output reg    VBlank,
	output        I,R,G,B
);

assign vram_addr = vaddr;
assign nINT      = ~INT;
assign port_ff   = tmx_using_ff ? {2'b00, tmx_cfg} : mZX ? ff_data : 8'hFF;

// Pixel clock
reg  [8:0] hc = 0;
reg  [8:0] vc_next;
reg  [8:0] vc = 0;
reg  [8:0] hc_next;

wire       Border_next = ((vc_next[7] & vc_next[6]) | vc_next[8] | hc_next[8]);
reg        Border;
reg  [4:0] FlashCnt_next;
reg  [4:0] FlashCnt;

always @(*) begin
	vc_next = vc;
	FlashCnt_next = FlashCnt;

	if (hc==((mZX && m128) ? 455 : 447)) begin
		hc_next = 0;
		if (vc == (!mZX ? 319 : m128 ? 310 : 311)) begin
			vc_next = 0;
			FlashCnt_next = FlashCnt + 1'd1;
		end else begin
			vc_next = vc + 1'd1;
		end
	end else begin
		hc_next = hc + 1'd1;
	end
end

always @(posedge clk_sys) begin
	reg m512;

	if(ce_7mp) hiSRegister <= {hiSRegister[14:0],1'b0};

	if(ce_7mn) begin
		vc <= vc_next;
		hc <= hc_next;
		Border <= Border_next;
		FlashCnt <= FlashCnt_next;

		if((vc_next<192) || (hc_next<256)) m512 <= (m512 | tmx_hi);
		if (hc_next == 0) begin
			if(mZX ? (vc_next == 240) : (vc_next == 248)) begin
				mode512 <= m512;
				m512 <= 0;
			end
		end

		if(!mZX) begin       //pentagon
			if ((hc_next == 312) || (hc_next == 268 && hide_border)) HBlank <= 1;
				else if ((hc_next == 420 && !hide_border) || (hc_next == 12)) HBlank <= 0;
			if (hc_next == 338) HSync <= 1;
				else if (hc_next == 370) HSync <= 0;
		end else if(m128) begin      //128
			if ((hc_next == 312) || (hc_next == 268 && hide_border)) HBlank <= 1;
				else if ((hc_next == 424 && !hide_border) || (hc_next == 12)) HBlank <= 0;
			if (hc_next == 340) HSync <= 1;         //ULA 6C
				else if (hc_next == 372) HSync <= 0; //ULA 6C
		end else begin               //48
			if ((hc_next == 312) || (hc_next == 268 && hide_border)) HBlank <= 1;
				else if ((hc_next == 416 && !hide_border) || (hc_next == 12)) HBlank <= 0;
			if (hc_next == 336) HSync <= 1;         //ULA 5C
				else if (hc_next == 368) HSync <= 0; //ULA 5C
		end

		if(mZX) begin               //48 or 128
			if(vc_next == 240) VSync <= 1;
				else if (vc_next == 244) VSync <= 0;
			if((vc_next == 236) || (vc_next == 192 && hide_border)) VBlank <= 1;
				else if((vc == 264 && !hide_border) || (vc_next == 0)) VBlank <= 0;
		end else begin             //pentagon
			if(vc_next == 248) VSync <= 1;
				else if (vc_next == 256) VSync <= 0;
			if((vc_next == 236) || (vc_next == 192 && hide_border)) VBlank <= 1;
				else if((vc_next == 272 && !hide_border) || (vc_next == 0)) VBlank <= 0;
		end

		if(wide && !hide_border) begin
			HBlank <= !(hc_next < 290 || hc_next >= ((mZX && m128) ? 455-11 : 447-11));
		end

		if( mZX && (vc_next == 248) && (hc_next == (m128 ? 8 : 4))) INT <= 1;
		if(!mZX && (vc_next == 239) && (hc_next == 326)) INT <= 1;

		if(INT)  INTCnt <= ((m128 && INTCnt == 71) || (~m128 && INTCnt == 63)) ? 7'd0 : (INTCnt + 1'd1);
		if(INTCnt == 0) INT <= 0;

		if(hc_next[2:0] == 4) begin
			SRegister <= VidEN ? bits : 8'd0;
			hiSRegister <= VidEN ? {bits, attr} : 16'd0;
			AttrOut <= tmx_hi ? hiattr : VidEN ? attr : {2'b00,border_color,border_color};
		end else begin
			SRegister   <= {SRegister[6:0],   1'b0};
			hiSRegister <= {hiSRegister[14:0],1'b0};
		end

		//1T update for border in Pentagon mode
		if(!mZX & ((hc_next<12) | (hc_next>267) | (vc>=192))) AttrOut <= tmx_hi ? hiattr : {2'b00,border_color,border_color};

		if(hc_next[3]) VidEN <= ~Border;
	
		if(~Border_next) begin
			if(~tmx_cfg[1]) case(hc_next[3:0])
				'h8,'hC: vaddr       <= {stdpage ? page_scr : tmx_cfg[2],tmx_cfg[0],vc[7:6],vc[2:0],vc[5:3],hc_next[7:4],hc_next[2]};
				'hA,'hE: vaddr[14:7] <= {stdpage ? page_scr : tmx_cfg[2],tmx_cfg[0],3'b110,vc[7:5]};  // CAS and page(?) only
			endcase

			if(tmx_cfg[1]) case(hc_next[3:0])
				'h8,'hC: vaddr       <= {stdpage ? page_scr : tmx_cfg[0],1'b0,vc[7:6],vc[2:0],vc[5:3],hc_next[7:4], hc_next[2]};
				'hA,'hE: vaddr[14:7] <= {stdpage ? page_scr : tmx_cfg[0],1'b1,vc[7:6],vc[2:0],vc[5]}; // CAS and page(?) only
			endcase

			// Snow effect for ULA-48 only. ULA-128 has no snow bug.
			if (mZX & ~m128 & ~nMREQ & ~nRFSH & contendAddr & snow_ena) case(hc_next[3:0])
				'h8,'hC: vaddr[6:0] <= addr[6:0]; // only RAS got replaced
			endcase

			case(hc_next[3:0])
				'h9,'hD: bits <= vram_dout;
				'hB,'hF: attr <= vram_dout;
			endcase

			if(hc_next[3] & hc_next[0]) ff_data <= vram_dout;
		end

		if (hc_next[3:0] == 1) ff_data <= 255;
	end
end

wire [7:0] hipalette[8];
assign hipalette = '{8'b01111000, 8'b01110001, 8'b01101010, 8'b01100011, 
                     8'b01011100, 8'b01010101, 8'b01001110, 8'b01000111};

reg        INT    = 0;
reg  [6:0] INTCnt = 1;
reg  [7:0] ff_data;

reg  [7:0] SRegister;
reg [15:0] hiSRegister;
reg [14:0] vaddr;

reg  [7:0] AttrOut;

reg        VidEN = 0;

reg  [7:0] bits;
reg [15:0] hibits;
reg  [7:0] attr;
wire [7:0] hiattr  = hipalette[tmx_cfg[5:3]];
wire       stdpage = tmx_using_ff | ~tmx_ena;
wire       Pixel = tmx_hi ? hiSRegister[15] : SRegister[7] ^ (AttrOut[7] & FlashCnt[4]);

assign     {I,G,R,B} = Pixel ? {AttrOut[6],AttrOut[2:0]} : {AttrOut[6],AttrOut[5:3]};
assign     ulap_color = palette[(tmx_hi ? hiSRegister[15] : SRegister[7]) ? {AttrOut[7:6],1'b0,AttrOut[2:0]} : {AttrOut[7:6],1'b1,AttrOut[5:3]}];

///////////////////////////////////////////////////////////////////////////////

reg  CPUClk;
reg  ioreqtw3;
reg  mreqt23;

wire ioreq_n      = (addr[0] & ~(ulap_acc & ulap_avail)) | nIORQ;
wire clkwait_next = hc_next[2] | hc_next[3];
wire ulaContend   = clkwait_next & ~Border_next & CPUClk & ioreqtw3;
wire contendAddr  = ((addr[15:14] == 2'b01) | (m128 & (addr[15:14] == 2'b11) & page_ram[0]));
wire memContend   = ioreq_n & mreqt23 & contendAddr;
wire ioContend    = ~ioreq_n;
wire next_clk     = hc_next[0] | (mZX & ulaContend & (memContend | ioContend));

reg  next_clk_r;

assign ce_cpu_sp = ce_7mn & (~CPUClk &  next_clk_r);
assign ce_cpu_sn = ce_7mn & ( CPUClk & ~next_clk_r);

always @(posedge clk_sys) begin
	if(ce_7mn) CPUClk <= next_clk;
	if(ce_7mp) next_clk_r <= next_clk;

	if(~CPUClk) begin
		// These are transparent latches!
		ioreqtw3 <= ioreq_n;
		mreqt23  <= nMREQ;
	end
end

/////////////////  ULA+, Timex  ///////////////////

assign     ulap_dout = ulap_group ? {6'd0, ulap_mono, ulap_ena} : palette_q;
assign     ulap_sel  = ulap_acc & addr[14] & ulap_avail;

wire       ulap_acc = ({addr[15], 1'b0, addr[13:0]} == 'hBF3B);
wire       io_wr = ~nIORQ & ~nWR;
reg  [5:0] pal_addr;
reg        ulap_group;
reg  [7:0] palette[64];
reg  [7:0] palette_q;

reg  [5:0] tmx_cfg;
reg        tmx_ena;
reg        tmx_using_ff;
wire       tmx_hi = &{tmx_ena, tmx_cfg[2:1]};

always @(posedge clk_sys) begin
	reg old_wr;
	old_wr <= io_wr;

	palette_q <= palette[pal_addr];

	if(reset) begin
		{ulap_ena, tmx_ena, tmx_using_ff, tmx_cfg} <= 0;
		palette <= '{default:0};
	end else if(~old_wr & io_wr) begin
		if(ulap_acc & ulap_avail) begin
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
		if((addr[7:0] == 'hFF) & tmx_avail) {tmx_using_ff, tmx_ena, tmx_cfg} <= {1'b1, |din[2:0], din[5:0]};
	end
end

endmodule
