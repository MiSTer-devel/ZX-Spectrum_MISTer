////////////////////////////////////////////////////////////////////////////////
//
//  PS2-to-Kempston Mouse
//  (C) 2016 Sorgelig
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
//
////////////////////////////////////////////////////////////////////////////////

module mouse
(
	input        clk_sys,
	input        ce_7mp,
	input        reset,

	input        ps2_mouse_clk,
	input        ps2_mouse_data,
	
	input  [2:0] addr,
	output       sel,
	output [7:0] dout
);

assign dout = data;
assign sel  = port_sel;

reg   [1:0] button;
reg  [11:0] dx;
reg  [11:0] dy;
reg  [32:0] q;

wire [11:0] newdx = dx + {{4{q[5]}},q[19:12]};
wire [11:0] newdy = dy + {{4{q[6]}},q[30:23]};

reg   [1:0] swap;
reg   [7:0] data;
reg         port_sel;
always @* begin
	port_sel = 1;
	casex(addr)
		 3'b011: data = dx[7:0];
		 3'b111: data = dy[7:0];
		 3'bX10: data = ~{6'b000000, button[~swap[1]], button[swap[1]]} ;
		default: {port_sel,data} = 8'hFF;
	endcase
end

always @(posedge clk_sys) begin
	integer   idle;
	reg       old_clk;

	if(reset) begin
		dx     <= 128; // dx != dy for better mouse detection
		dy     <= 0;
		button <= 0;
		idle   <= 0;
		swap   <= 0;
	end else begin
		old_clk <= ps2_mouse_clk;
		if(old_clk & ~ps2_mouse_clk) begin
			q <= {ps2_mouse_data, q[32:1]};
			idle <= 0;
		end else if(~old_clk & ps2_mouse_clk) begin
			if(q[32] & ^q[31:23] & ~q[22] & q[21] & ^q[20:12] & ~q[11] & q[10] & ^q[9:1] & ~q[0]) begin
				if(!swap) swap <= q[2:1];
				button <= q[2:1];
				dx <= |newdx[11:8] ? {8{~q[5]}} : newdx;
				dy <= |newdy[11:8] ? {8{~q[6]}} : newdy;
				q  <= ~0;
			end
		end else if(ps2_mouse_clk & ce_7mp) begin
			if(idle < 3500000) idle <= idle + 1;
				else q <= ~0;
		end
	end
end

endmodule
