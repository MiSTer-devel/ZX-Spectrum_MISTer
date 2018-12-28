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
	input         RESET,	    // Chip RESET (set all Registers to '0', active high)
	input         CLK,		 // Global clock
	input         CE,        // YM2203 Master Clock enable x2 (due to YM2612 model!)

	input         BDIR,	    // Bus Direction (0 - read , 1 - write)
	input         BC,		    // Bus control
	input   [7:0] DI,	       // Data In
	output  [7:0] DO,	       // Data Out

	output [11:0] CHANNEL_L, // Output channel L
	output [11:0] CHANNEL_R  // Output channel R
);


// AY1 selected by default
reg ay_select = 1;
reg stat_sel  = 1;
reg fm_ena    = 0;
reg ym_wr     = 0;
reg [7:0] ym_di;

always_ff @(posedge CLK or posedge RESET) begin
	reg old_BDIR = 0;
	reg ym_acc = 0;

	if (RESET) begin
		ay_select <= 1;
		stat_sel  <= 1;
		fm_ena    <= 0;
		ym_acc    <= 0;
		ym_wr     <= 0;
		old_BDIR  <= 0;
	end
	else begin
		ym_wr <= 0;
		old_BDIR <= BDIR;
		if (~old_BDIR & BDIR) begin
			if(BC & &DI[7:3]) begin
				ay_select <=  DI[0];
				stat_sel  <=  DI[1];
				fm_ena    <= ~DI[2];
				ym_acc    <= 0;
			end
			else if(BC) begin
				ym_acc <= !DI[7:4] || fm_ena;
				ym_wr  <= !DI[7:4] || fm_ena;
			end
			else begin
				ym_wr <= ym_acc;
			end
			ym_di <= DI;
		end
	end
end


wire  [7:0] psg_ch_a_0;
wire  [7:0] psg_ch_b_0;
wire  [7:0] psg_ch_c_0;
wire [10:0] opn_0;
wire  [7:0] DO_0;

wire WE_0 = ~ay_select & ym_wr;

ym2203 ym2203_0
(
	.RESET(RESET),
	.CLK(CLK),
	.CE(CE),

	.A0((BDIR|ym_wr) ? ~BC : stat_sel),
	.WE(WE_0),
	.DI(ym_di),
	.DO(DO_0),

	.CHANNEL_A(psg_ch_a_0),
	.CHANNEL_B(psg_ch_b_0),
	.CHANNEL_C(psg_ch_c_0),
	.CHANNEL_FM(opn_0)
);

wire  [7:0] psg_ch_a_1;
wire  [7:0] psg_ch_b_1;
wire  [7:0] psg_ch_c_1;
wire [10:0] opn_1;
wire  [7:0] DO_1;

wire WE_1 = ay_select & ym_wr;

ym2203 ym2203_1
(
	.RESET(RESET),
	.CLK(CLK),
	.CE(CE),

	.A0((BDIR|ym_wr) ? ~BC : stat_sel),
	.WE(WE_1),
	.DI(ym_di),
	.DO(DO_1),

	.CHANNEL_A(psg_ch_a_1),
	.CHANNEL_B(psg_ch_b_1),
	.CHANNEL_C(psg_ch_c_1),
	.CHANNEL_FM(opn_1)
);

assign DO = ay_select ? DO_1 : DO_0;

// Mix channel signals from both AY/YM chips (extending to 9 bits width to prevent clipping)
wire [8:0] sum_ch_a = { 1'b0, psg_ch_a_1 } + { 1'b0, psg_ch_a_0 };
wire [8:0] sum_ch_b = { 1'b0, psg_ch_b_1 } + { 1'b0, psg_ch_b_0 };
wire [8:0] sum_ch_c = { 1'b0, psg_ch_c_1 } + { 1'b0, psg_ch_c_0 };

wire [7:0] psg_a = sum_ch_a[8] ? 8'hFF : sum_ch_a[7:0];
wire [7:0] psg_b = sum_ch_b[8] ? 8'hFF : sum_ch_b[7:0];
wire [7:0] psg_c = sum_ch_c[8] ? 8'hFF : sum_ch_c[7:0];

wire signed [11:0] psg_l = {3'b000, psg_a, 1'd0} + {4'b0000, psg_b};
wire signed [11:0] psg_r = {3'b000, psg_c, 1'd0} + {4'b0000, psg_b};
wire signed [11:0] opn_s = {{2{opn_0[10]}}, opn_0[10:1]} + {{2{opn_1[10]}}, opn_1[10:1]};

assign CHANNEL_L = fm_ena ? opn_s + psg_l : psg_l;
assign CHANNEL_R = fm_ena ? opn_s + psg_r : psg_r;

endmodule
