//============================================================================
//  Z80 snapshot loader for ZX Spectrum
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

module snap_loader #(parameter ARCH_ZX48, ARCH_ZX128, ARCH_ZX3, ARCH_P128)
(
	input          clk_sys,

	input          ioctl_download,
	input   [24:0] ioctl_addr,
	input    [7:0] ioctl_data,
	input          ioctl_wr,
	output         ioctl_wait,

	input          snap_sna,

	input          ram_ready,

	output [211:0] REG,
	output         REGSet,

	output  [24:0] addr,
	output   [7:0] dout,
	output         wr,

	output         reset,
	output         hwset,
	output   [4:0] hw,
	input    [4:0] hw_ack,

	output   [2:0] border,
	output   [7:0] reg_1ffd,
	output   [7:0] reg_7ffd
);

/*
        https://www.worldofspectrum.org/faq/reference/z80format.htm

v1,v2,v3

        0       1       A register
        1       1       F register
        2       2       BC register pair (LSB, i.e. C, first)
        4       2       HL register pair
        6       2       Program counter  = 0 for v2,v3
        8       2       Stack pointer
        10      1       Interrupt register
        11      1       Refresh register (Bit 7 is not significant!)
        12      1       Bit 0  : Bit 7 of the R-register
                        Bit 1-3: Border colour
                        Bit 4  : 1=Basic SamRom switched in
                        Bit 5  : 1=Block of data is compressed
                        Bit 6-7: No meaning
        13      2       DE register pair
        15      2       BC' register pair
        17      2       DE' register pair
        19      2       HL' register pair
        21      1       A' register
        22      1       F' register
        23      2       IY register (Again LSB first)
        25      2       IX register
        27      1       Interrupt flipflop, 0=DI, otherwise EI
        28      1       IFF2 (not particularly important...)
        29      1       Bit 0-1: Interrupt mode (0, 1 or 2)
                        Bit 2  : 1=Issue 2 emulation
                        Bit 3  : 1=Double interrupt frequency
                        Bit 4-5: 1=High video synchronisation
                                 3=Low video synchronisation
                                 0,2=Normal
                        Bit 6-7: 0=Cursor/Protek/AGF joystick
                                 1=Kempston joystick
                                 2=Sinclair 2 Left joystick (or user
                                   defined, for version 3 .z80 files)
                                 3=Sinclair 2 Right joystick
v2,v3
      * 30      2       Length of additional header block (see below)
      * 32      2       Program counter
      * 34      1       Hardware mode (see below)
      * 35      1       If in SamRam mode, bitwise state of 74ls259.
                        For example, bit 6=1 after an OUT 31,13 (=2*6+1)
                        If in 128 mode, contains last OUT to 0x7ffd
			If in Timex mode, contains last OUT to 0xf4
      * 36      1       Contains 0xff if Interface I rom paged
			If in Timex mode, contains last OUT to 0xff
      * 37      1       Bit 0: 1 if R register emulation on
                        Bit 1: 1 if LDIR emulation on
			Bit 2: AY sound in use, even on 48K machines
			Bit 6: (if bit 2 set) Fuller Audio Box emulation
			Bit 7: Modify hardware (see below)
      * 38      1       Last OUT to port 0xfffd (soundchip register number)
      * 39      16      Contents of the sound chip registers
        55      2       Low T state counter
        57      1       Hi T state counter
        58      1       Flag byte used by Spectator (QL spec. emulator)
                        Ignored by Z80 when loading, zero when saving
        59      1       0xff if MGT Rom paged
        60      1       0xff if Multiface Rom paged. Should always be 0.
        61      1       0xff if 0-8191 is ROM, 0 if RAM
        62      1       0xff if 8192-16383 is ROM, 0 if RAM
        63      10      5 x keyboard mappings for user defined joystick
        73      10      5 x ASCII word: keys corresponding to mappings above
        83      1       MGT type: 0=Disciple+Epson,1=Disciple+HP,16=Plus D
        84      1       Disciple inhibit button status: 0=out, 0ff=in
        85      1       Disciple inhibit flag: 0=rom pageable, 0ff=not
     ** 86      1       Last OUT to port 0x1ffd
	  
	  
        Hardware mode:  Meaning in v2           Meaning in v3
        -----------------------------------------------------
          0             48k                     48k
          1             48k + If.1              48k + If.1
          2             SamRam                  SamRam
          3             128k                    48k + M.G.T.
          4             128k + If.1             128k
          5             -                       128k + If.1
          6             -                       128k + M.G.T.
          7                    Spectrum +3
          8                    [mistakenly used by some versions of XZX-Pro to indicate a +3]
          9                    Pentagon (128K)
         10                    Scorpion (256K)
         11                    Didaktik-Kompakt
         12                    Spectrum +2
         13                    Spectrum +2A
         14                    TC2048
         15                    TC2068
        128                    TS2068

	  
       Page    In '48 mode     In '128 mode    In SamRam mode
        ------------------------------------------------------
         0      48K rom         rom (basic)     48K rom
         1      Interface I, Disciple or Plus D rom, according to setting
         2      -               rom (reset)     samram rom (basic)
         3      -               page 0          samram rom (monitor,..)
         4      8000-bfff       page 1          Normal 8000-bfff
         5      c000-ffff       page 2          Normal c000-ffff
         6      -               page 3          Shadow 8000-bfff
         7      -               page 4          Shadow c000-ffff
         8      4000-7fff       page 5          4000-7fff
         9      -               page 6          -
        10      -               page 7          -
        11      Multiface rom   Multiface rom   -
	  
*/

/* SNA 48k
   0        1      byte   I
   1        8      word   HL',DE',BC',AF'
   9        10     word   HL,DE,BC,IY,IX
   19       1      byte   Interrupt (bit 2 contains IFF2, 1=EI/0=DI)
   20       1      byte   R
   21       4      words  AF,SP
   25       1      byte   IntMode (0=IM0/1=IM1/2=IM2)
   26       1      byte   BorderColor (0..7, not used by Spectrum 1.7)
   27       49152  bytes  RAM dump 16384..65535
*/

assign REG = snap_REG;
assign REGSet = snap_REGSet;
assign dout = snap_data;
assign wr = snap_wr;
assign reset = snap_reset;
assign hwset = snap_hwset;
assign hw = snap_hw;
assign border = snap_border;
assign reg_1ffd = snap_1ffd;
assign reg_7ffd = snap_7ffd;
assign ioctl_wait = snap_wait;

reg [211:0] snap_REG;
reg         snap_REGSet;

reg         snap_wait = 0;
wire [24:0] snap_addr;
reg   [7:0] snap_data;
reg         snap_wr;

reg         snap_reset;
reg         snap_hwset;
reg   [4:0] snap_hw;
reg  [31:0] snap_status;

reg   [2:0] snap_border;
reg   [7:0] snap_1ffd;
reg   [7:0] snap_7ffd;

always_comb begin
	addr = addr_pre;
	if(hdrv1 || snap_sna) begin
		case(addr_pre[17:14])
				0: addr[16:14] = 5;
				1: addr[16:14] = 2;
				2: addr[16:14] = 0;
		default: addr[16:14] = 1; //some unused page in 48K
		endcase
	end
end

reg        hdrv1;
reg  [7:0] snap_hdrlen;
reg [24:0] addr_pre;
always_ff @(posedge clk_sys) begin
	reg       old_download;
	//reg [7:0] snap_mhw;
	reg [7:0] snap61;
	reg [7:0] snap62;
	reg [2:0] comp_state;
	reg[24:0] addr;
	reg[15:0] sz;
	reg       compr;
	reg       wren;
	reg [7:0] cnt;
	reg       finish;
	reg [1:0] hold = 0;
	
	hdrv1 <= (snap_hdrlen == 30);

	snap_wr <= 0;
	old_download <= ioctl_download;

	//prepare to download snapshot with default v1 header
	if(~old_download && ioctl_download) begin
		snap_hdrlen <= snap_sna ? 8'd27 : 8'd30;
		snap_reset <= 1;
		snap_hw <= 0;
	end

	//finish download
	if(old_download && ~ioctl_download) begin
		if(snap_hw) begin
			snap_REGSet <= 1;
			snap_hwset <= 1;
			hold <= '1;
		end
		else snap_reset <= 0; // unsupported snapshot loaded - just exit from reset.
	end
	
	//wait for confirmation from HPS
	if(snap_hwset && (snap_hw == hw_ack)) begin
		snap_hwset <= 0;
		snap_reset <= 0;
	end

	//hold snap_REGSet for several clocks after reset
	if(~snap_reset) begin
		if(hold) hold <= hold - 1'd1;
		else snap_REGSet <= 0;
	end

	if(ioctl_download & ioctl_wr) begin
		if (ioctl_addr<snap_hdrlen) begin
			if (!snap_sna) case(ioctl_addr[6:0]) // Z80
				 0: snap_REG[7:0]     <= ioctl_data; //a
				 1: snap_REG[15:8]    <= ioctl_data; //f
				 2: snap_REG[87:80]   <= ioctl_data; //c
				 3: snap_REG[95:88]   <= ioctl_data; //b
				 4: snap_REG[119:112] <= ioctl_data; //l
				 5: snap_REG[127:120] <= ioctl_data; //h
				 6: snap_REG[71:64]   <= ioctl_data; //pcl
				 7: snap_REG[79:72]   <= ioctl_data; //pch
				 8: snap_REG[55:48]   <= ioctl_data; //spl
				 9: snap_REG[63:56]   <= ioctl_data; //sph
				10: snap_REG[39:32]   <= ioctl_data; //i
				11: snap_REG[47:40]   <= ioctl_data; //r
				12: begin
						snap_REG[47] <= ioctl_data[0];
						snap_border <= &ioctl_data ? 3'd0 : ioctl_data[3:1];
						snap_1ffd <= 0;
						comp_state <= 0;
						finish <= 0;
						if(!snap_REG[79:64]) begin
							snap_hdrlen <= 87;
							snap_hw <= 0;
						end
						else begin
							snap_hw <= ARCH_ZX48;
							addr <= 0;
							sz <= 'hC000;
							compr <= 0;
							comp_state <= 3;
							wren <= 1;
							if(~&ioctl_data & ioctl_data[5]) begin
								sz <= 0;
								compr <= 1;
							end
						end
					 end
				13: snap_REG[103:96]  <= ioctl_data; //e
				14: snap_REG[111:104] <= ioctl_data; //d
				15: snap_REG[151:144] <= ioctl_data; //c'
				16: snap_REG[159:152] <= ioctl_data; //b'
				17: snap_REG[167:160] <= ioctl_data; //e'
				18: snap_REG[175:168] <= ioctl_data; //d'
				19: snap_REG[183:176] <= ioctl_data; //l'
				20: snap_REG[191:184] <= ioctl_data; //h'
				21: snap_REG[23:16]   <= ioctl_data; //a'
				22: snap_REG[31:24]   <= ioctl_data; //f'
				23: snap_REG[199:192] <= ioctl_data; //yl
				24: snap_REG[207:200] <= ioctl_data; //yh
				25: snap_REG[135:128] <= ioctl_data; //xl
				26: snap_REG[143:136] <= ioctl_data; //xh
				27: snap_REG[211:210] <= ioctl_data ? 2'b11 : 2'b00; //iff2,iff1
				29: snap_REG[209:208] <= ioctl_data[1:0]; //im

				//v2,v3
				30: snap_hdrlen <= 8'd32+ioctl_data;
				32: snap_REG[71:64]   <= ioctl_data; //pcl
				33: snap_REG[79:72]   <= ioctl_data; //pch
				34: case(ioctl_data)
							  0,1: snap_hw <= ARCH_ZX48; //48K
								 3: snap_hw <= (snap_hdrlen <= 55) ? ARCH_ZX128 : ARCH_ZX48; //128K/48K
						4,5,6,12: snap_hw <= ARCH_ZX128; //128K
						  7,8,13: snap_hw <= ARCH_ZX3;   //+3
								 9: snap_hw <= ARCH_P128;  //P128
					 endcase
				35: snap_7ffd <= ioctl_data;
				//37: snap_mhw  <= ioctl_data[7];
				86: snap_1ffd <= ioctl_data;
			endcase
			else case(ioctl_addr[6:0]) // SNA
				 0: begin
						snap_REG[39:32]   <= ioctl_data; //i
						snap_REG[71:64]   <= 8'h72; //pcl - RETN in this address
						snap_REG[79:72]   <= 8'h00; //pch
						snap_1ffd <= 0;
						snap_hw <= ARCH_ZX48;
						finish <= 0;
						addr <= 0;
						sz <= 'hC000;
						compr <= 0;
						comp_state <= 3;
						wren <= 1;
					end
				 1: snap_REG[183:176] <= ioctl_data; //l'
				 2: snap_REG[191:184] <= ioctl_data; //h'
				 3: snap_REG[167:160] <= ioctl_data; //e'
				 4: snap_REG[175:168] <= ioctl_data; //d'
				 5: snap_REG[151:144] <= ioctl_data; //c'
				 6: snap_REG[159:152] <= ioctl_data; //b'
				 7: snap_REG[23:16]   <= ioctl_data; //a'
				 8: snap_REG[31:24]   <= ioctl_data; //f'
				 9: snap_REG[119:112] <= ioctl_data; //l
				10: snap_REG[127:120] <= ioctl_data; //h
				11: snap_REG[103:96]  <= ioctl_data; //e
				12: snap_REG[111:104] <= ioctl_data; //d
				13: snap_REG[87:80]   <= ioctl_data; //c
				14: snap_REG[95:88]   <= ioctl_data; //b
				15: snap_REG[199:192] <= ioctl_data; //yl
				16: snap_REG[207:200] <= ioctl_data; //yh
				17: snap_REG[135:128] <= ioctl_data; //xl
				18: snap_REG[143:136] <= ioctl_data; //xh
				19: snap_REG[211:210] <= {ioctl_data[2], 1'b0}; //iff2,iff1
				20: snap_REG[47:40]   <= ioctl_data; //r
				21: snap_REG[7:0]     <= ioctl_data; //a
				22: snap_REG[15:8]    <= ioctl_data; //f
				23: snap_REG[55:48]   <= ioctl_data; //spl
				24: snap_REG[63:56]   <= ioctl_data; //sph
				25: snap_REG[209:208] <= ioctl_data[1:0]; //im
				26: snap_border       <= ioctl_data[2:0];
			endcase
		end

		else if(snap_hw && !finish) begin
			case(comp_state)
				0: begin 
						sz[7:0] <= ioctl_data;
						comp_state <= comp_state + 1'd1;
					end
				1: begin 
						sz[15:8] <= ioctl_data;
						comp_state <= comp_state + 1'd1;
					end
				2: begin
						compr <= 1;
						if(&sz) begin
							sz <= 'h4000;
							compr <= 0;
						end
						wren <= 0;
						addr <= 0;
						if(snap_hw == ARCH_ZX48)
							case(ioctl_data)
								4: begin addr <= {4'd2, 14'd0}; wren <= 1; end
								5: begin addr <= {4'd0, 14'd0}; wren <= 1; end
								8: begin addr <= {4'd5, 14'd0}; wren <= 1; end
							endcase
						else if(ioctl_data>=3 && ioctl_data<=10) begin
							addr <= {ioctl_data[3:0]-3'd3, 14'd0};
							wren <= 1;
						end
						comp_state <= comp_state + 1'd1;
					end
				3: begin
						if(compr && ioctl_data == 'hED) comp_state <= comp_state + 1'd1;
						else begin
							addr_pre <= addr;
							snap_data <= ioctl_data;
							snap_wr <= wren;
							addr <= addr + 1'd1;
						end
					end
				4: begin
						if(ioctl_data == 'hED) comp_state <= comp_state + 1'd1;
						else begin
							snap_wait <= wren;
							addr_pre <= addr;
							addr <= addr + 1'd1;
							snap_data <= 'hED;
							snap_wr <= wren;
							comp_state <= 3;
							cnt <= 1;
						end
					end
				5: begin
						cnt <= ioctl_data - 1'd1;
						comp_state <= comp_state + 1'd1;
						if(!ioctl_data) finish <= 1;
					end
				6: begin
						snap_wait <= wren;
						addr_pre <= addr;
						addr <= addr + 1'd1;
						snap_data <= ioctl_data;
						snap_wr <= wren;
						comp_state <= 3;
					end
			endcase
			if(comp_state>=3) begin
				sz <= sz - 1'd1;
				if(sz == 1) begin
					if(snap_hdrlen == 30 || snap_sna) finish <= 1;
					else comp_state <= 0;
				end
			end
		end
	end

	if(~snap_wr & snap_wait & ram_ready) begin
		if(cnt) begin
			addr_pre <= addr;
			addr <= addr + 1'd1;
			snap_data <= ioctl_data;
			snap_wr <= 1;
			cnt <= cnt - 1'd1;
		end
		else begin
			snap_wait <= 0;
		end
	end
	
	//v1 - 49152 bytes max
	if(snap_wr && ((snap_hdrlen == 30) || snap_sna) && (addr_pre == 'hBFFF)) wren <= 0;
end

endmodule
