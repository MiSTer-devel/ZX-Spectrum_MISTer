//============================================================================
//  Turbosound-FM
// 
//  Copyright (C) 2018 Ilia Sharin
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


module turbosound
(
	input         CLK,		 // Global clock
	input         CE_CPU,    // CPU Clock enable
	input         CE_PSG,    // PSG Clock enable
	input         CE_OPN,    // FM syn Clock enable
	input         RESET,	    // Chip RESET (set all Registers to '0', active high)
	input         BDIR,	    // Bus Direction (0 - read , 1 - write)
	input         BC,		    // Bus control
	input   [7:0] DI,	       // Data In
	output  [7:0] DO,	       // Data Out
	output [11:0] CHANNEL_L, // Output channel L
	output [11:0] CHANNEL_R, // Output channel R
	output        ACTIVE,
	input         SEL,
	input         MODE
);


// AY1 selected by default
reg ay_select = 1;

reg stat_sel = 1;
reg fm_ena = 0;

always_ff @(posedge CLK or posedge RESET) begin
	if (RESET) begin
		// Select AY1 after reset
		ay_select <= 1;
		fm_ena <= 0;
		stat_sel <= 1;
	end
	else if (BDIR & BC & &DI[7:3]) begin
		// Select AY0 or AY1 according to lower bit of data register (1111 1FSN)
		ay_select <= DI[0];
		stat_sel <= DI[1];
		fm_ena <= ~DI[2];
	end
end

wire BDIR_0 = ~ay_select & BDIR;

// AY0 channel output data
wire [7:0] psg_ch_a_0;
wire [7:0] psg_ch_b_0;
wire [7:0] psg_ch_c_0;
wire [5:0] ay0_active;
wire [7:0] DO_0;

ym2149 ym2149_0
(
	.CLK(CLK),
	.CE(CE_PSG),
	.RESET(RESET),
	.BDIR(BDIR_0),
	.BC(BC),
	.LIMIT_REG(fm_ena),
	.DI(DI),
	.DO(DO_0),
	.CHANNEL_A(psg_ch_a_0),
	.CHANNEL_B(psg_ch_b_0),
	.CHANNEL_C(psg_ch_c_0),
	.ACTIVE(ay0_active),
	.SEL(SEL),
	.MODE(MODE)
);


wire BDIR_1 =  ay_select & BDIR;

// AY1 channel output data
wire [7:0] psg_ch_a_1;
wire [7:0] psg_ch_b_1;
wire [7:0] psg_ch_c_1;
wire [5:0] ay1_active;
wire [7:0] DO_1;

// AY1 (Default AY)
ym2149 ym2149_1
(
	.CLK(CLK),
	.CE(CE_PSG),
	.RESET(RESET),
	.BDIR(BDIR_1),
	.BC(BC),
	.LIMIT_REG(fm_ena),
	.DI(DI),
	.DO(DO_1),
	.CHANNEL_A(psg_ch_a_1),
	.CHANNEL_B(psg_ch_b_1),
	.CHANNEL_C(psg_ch_c_1),
	.ACTIVE(ay1_active),
	.SEL(SEL),
	.MODE(MODE)
);


//only 11 bits are actually used due to half-chip usage
wire [11:0] opn_0, opn_1;
wire  [7:0] opn_dout_0, opn_dout_1;

jt12 fm_0
(
	.rst(RESET),

	.cpu_clk(CLK & CE_CPU),
	.cpu_din(DI),
	.cpu_dout(opn_dout_0),
	.cpu_addr(~BC),
	.cpu_cs_n(0),
	.cpu_wr_n(~BDIR_0),

	.syn_clk(CLK & CE_OPN),
	.cpu_limiter_en(1),
	.syn_snd_right(opn_0)
);

jt12 fm_1
(
	.rst(RESET),

	.cpu_clk(CLK & CE_CPU),
	.cpu_din(DI),
	.cpu_dout(opn_dout_1),
	.cpu_addr(~BC),
	.cpu_cs_n(0),
	.cpu_wr_n(~BDIR_1),

	.syn_clk(CLK & CE_OPN),
	.cpu_limiter_en(1),
	.syn_snd_right(opn_1)
);

assign DO = stat_sel ? (ay_select ? DO_1 : DO_0) : (ay_select ? opn_dout_1 : opn_dout_0);
assign ACTIVE = ay0_playing | ay1_playing | fm_ena;

// AY activity signals
wire ay0_playing = |ay0_active; // OR reduction (all bits of ay0_active OR'ed with each other)
wire ay1_playing = |ay1_active; // OR reduction (all bits of ay1_active OR'ed with each other)

// Mix channel signals from both AY/YM chips (extending to 9 bits width to prevent clipping)
wire [8:0] sum_ch_a = { 1'b0, psg_ch_a_1 } + { 1'b0, psg_ch_a_0 };
wire [8:0] sum_ch_b = { 1'b0, psg_ch_b_1 } + { 1'b0, psg_ch_b_0 };
wire [8:0] sum_ch_c = { 1'b0, psg_ch_c_1 } + { 1'b0, psg_ch_c_0 };

// Control output channels (Only AY_1 plays if not in TurboSound mode)
wire [7:0] psg_a = ~ay0_playing ? psg_ch_a_1 : sum_ch_a[8:1];
wire [7:0] psg_b = ~ay0_playing ? psg_ch_b_1 : sum_ch_b[8:1];
wire [7:0] psg_c = ~ay0_playing ? psg_ch_c_1 : sum_ch_c[8:1];

wire signed [11:0] psg_l = {3'b000, psg_a, 1'd0} + {4'b0000, psg_b};
wire signed [11:0] psg_r = {3'b000, psg_c, 1'd0} + {4'b0000, psg_b};
wire signed [11:0] opn_s = {opn_0[11], opn_0[11:1]} + {opn_1[11], opn_1[11:1]};

assign CHANNEL_L = fm_ena ? opn_s + psg_l : psg_l;
assign CHANNEL_R = fm_ena ? opn_s + psg_r : psg_r;

endmodule
