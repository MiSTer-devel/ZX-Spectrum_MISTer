//============================================================================
//  YM2203 wrapper
//  Copyright (C) 2018 Sorgelig
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


module ym2203
(
	input         RESET,
	input         CLK,       // Global clock
	input         CE_CPU,    // CPU Clock enable
	input         CE_YM,     // YM2203 Master Clock enable x2 (due to YM2612 model!)

	input         A0,        // 0 - register number/read FM, 1 - data/read PSG
	input         WE,        // 0 - read , 1 - write
	input   [7:0] DI,        // Data In
	output  [7:0] DO,        // Data Out

	output  [7:0] CHANNEL_A, // PSG Output channel A
	output  [7:0] CHANNEL_B, // PSG Output channel B
	output  [7:0] CHANNEL_C, // PSG Output channel C
 	output [10:0] CHANNEL_FM,// FM Output channel

	output        PSG_ACTIVE,
	input         FM_ENA
);

reg [7:0] ymreg;
reg [1:0] pres;

always @(posedge CLK) begin

	if(RESET) pres <= 2;
	else if(CE_CPU & WE) begin
		if(FM_ENA) begin
			if(~A0) ymreg  <= DI;
			else begin
				case(ymreg)
					'h2d: pres[1] <= 1;
					'h2e: pres[0] <= 1;
					'h2f: pres    <= 0;
				endcase
			end
		end
	end
end

wire [2:0] opn_tbl[4] = '{1,1,5,2};
wire [2:0] opn_pres = opn_tbl[pres];

wire [2:0] psg_tbl[4] = '{0,0,3,1};
wire [2:0] psg_pres = psg_tbl[pres];

reg ce_psg_pre, ce_opn_pre;
always @(posedge CLK) begin
	reg [2:0] div_psg, div_opn;

	{ce_opn_pre, ce_psg_pre} <= 0;

	if(RESET) {div_opn, div_psg} <= 0;
	else if (CE_YM) begin
		div_opn <= div_opn + 1'd1;
		if(div_opn >= opn_pres) div_opn <= 0;
		ce_opn_pre <= !div_opn;

		div_psg <= div_psg + 1'd1;
		if(div_psg >= psg_pres) div_psg <= 0;
		ce_psg_pre <= !div_psg;
	end
end

reg ce_opn, ce_psg;
always @(negedge CLK) {ce_opn, ce_psg} <= {ce_opn_pre, ce_psg_pre};

wire [5:0] psg_active;
wire [7:0] psg_dout;
ym2149 ym2149
(
	.CLK(CLK),
	.CE(ce_psg),
	.RESET(RESET),
	.BDIR(WE),
	.BC(~A0),
	.DI(DI),
	.DO(psg_dout),
	.CHANNEL_A(CHANNEL_A),
	.CHANNEL_B(CHANNEL_B),
	.CHANNEL_C(CHANNEL_C),
	.ACTIVE(psg_active),
	.SEL(1'b0),
	.MODE(1'b0)
);

wire  [7:0] opn_dout;
wire [11:0] opn_audio;
jt12 jt12
(
	.rst(RESET),

	.cpu_clk(CLK & CE_CPU),
	.cpu_din(DI),
	.cpu_dout(opn_dout),
	.cpu_addr({1'b0,A0}),
	.cpu_cs_n(~FM_ENA),
	.cpu_wr_n(~WE),

	.syn_clk(CLK & ce_opn),
	.cpu_limiter_en(1'b1),
	.syn_snd_right(opn_audio)
);

assign DO = A0 ? psg_dout : opn_dout;
assign PSG_ACTIVE = |psg_active;
assign CHANNEL_FM = opn_audio[10:0];

endmodule
