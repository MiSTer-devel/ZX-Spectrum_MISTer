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
	input         CE,        // YM2203 Master Clock enable

	input         BDIR,	    // Bus Direction (0 - read , 1 - write)
	input         BC,		    // Bus control
	input   [7:0] DI,	       // Data In
	output  [7:0] DO,	       // Data Out

	output [11:0] CHANNEL_L, // Output channel L
	output [11:0] CHANNEL_R  // Output channel R
);


reg       RESET_s;
reg       BDIR_s;
reg       BC_s;
reg [7:0] DI_s;

always_ff @(posedge CLK) begin
	reg       RESET_d;
	reg       BDIR_d;
	reg       BC_d;
	reg [7:0] DI_d;

	RESET_d <= RESET;
	BDIR_d <= BDIR;
	BC_d <= BC;
	DI_d <= DI;
	
	RESET_s <= RESET_d;
	BDIR_s <= BDIR_d;
	BC_s <= BC_d;
	DI_s <= DI_d;
end


// AY1 selected by default
reg ay_select = 1;
reg stat_sel  = 1;
reg fm_ena    = 0;
reg ym_wr     = 0;
reg [7:0] ym_di;

always_ff @(posedge CLK or posedge RESET_s) begin
	reg old_BDIR = 0;
	reg ym_acc = 0;

	if (RESET_s) begin
		ay_select <= 1;
		stat_sel  <= 1;
		fm_ena    <= 0;
		ym_acc    <= 0;
		ym_wr     <= 0;
		old_BDIR  <= 0;
	end
	else begin
		ym_wr <= 0;
		old_BDIR <= BDIR_s;
		if (~old_BDIR & BDIR_s) begin
			if(BC_s & &DI_s[7:3]) begin
				ay_select <=  DI_s[0];
				stat_sel  <=  DI_s[1];
				fm_ena    <= ~DI_s[2];
				ym_acc    <= 0;
			end
			else if(BC_s) begin
				ym_acc <= !DI_s[7:4] || fm_ena;
				ym_wr  <= !DI_s[7:4] || fm_ena;
			end
			else begin
				ym_wr <= ym_acc;
			end
			ym_di <= DI_s;
		end
	end
end

wire  [7:0] psg_ch_a_0;
wire  [7:0] psg_ch_b_0;
wire  [7:0] psg_ch_c_0;
wire [15:0] opn_0;
wire  [7:0] DO_0;

jt03 ym2203_0
(
	.rst(RESET_s),
	.clk(CLK),
	.cen(CE),
	.din(ym_di),
	.addr((BDIR_s|ym_wr) ? ~BC_s : stat_sel),
	.cs_n(ay_select),
	.wr_n(~ym_wr),
	.dout(DO_0),

	.psg_A(psg_ch_a_0),
	.psg_B(psg_ch_b_0),
	.psg_C(psg_ch_c_0),

	.fm_snd(opn_0)
);

wire  [7:0] psg_ch_a_1;
wire  [7:0] psg_ch_b_1;
wire  [7:0] psg_ch_c_1;
wire [15:0] opn_1;
wire  [7:0] DO_1;

jt03 ym2203_1
(
	.rst(RESET_s),
	.clk(CLK),
	.cen(CE),
	.din(ym_di),
	.addr((BDIR_s|ym_wr) ? ~BC_s : stat_sel),
	.cs_n(~ay_select),
	.wr_n(~ym_wr),
	.dout(DO_1),

	.psg_A(psg_ch_a_1),
	.psg_B(psg_ch_b_1),
	.psg_C(psg_ch_c_1),

	.fm_snd(opn_1)
);

assign DO = ay_select ? DO_1 : DO_0;

reg  [8:0] sum_ch_a,sum_ch_b,sum_ch_c;
reg  [7:0] psg_a,psg_b,psg_c;
reg [11:0] psg_l,psg_r,opn_s;
reg [11:0] ch_l, ch_r;

always @(posedge CLK) begin

	sum_ch_a <= { 1'b0, psg_ch_a_1 } + { 1'b0, psg_ch_a_0 };
	sum_ch_b <= { 1'b0, psg_ch_b_1 } + { 1'b0, psg_ch_b_0 };
	sum_ch_c <= { 1'b0, psg_ch_c_1 } + { 1'b0, psg_ch_c_0 };

	psg_a <= sum_ch_a[8] ? 8'hFF : sum_ch_a[7:0];
	psg_b <= sum_ch_b[8] ? 8'hFF : sum_ch_b[7:0];
	psg_c <= sum_ch_c[8] ? 8'hFF : sum_ch_c[7:0];

	psg_l <= {3'b000, psg_a, 1'd0} + {4'b0000, psg_b};
	psg_r <= {3'b000, psg_c, 1'd0} + {4'b0000, psg_b};
	opn_s <= {{2{opn_0[15]}}, opn_0[15:6]} + {{2{opn_1[15]}}, opn_1[15:6]};

	ch_l <= fm_ena ? $signed(opn_s) + $signed(psg_l) : $signed(psg_l);
	ch_r <= fm_ena ? $signed(opn_s) + $signed(psg_r) : $signed(psg_r);
end

assign CHANNEL_L = ch_l;
assign CHANNEL_R = ch_r;

endmodule
