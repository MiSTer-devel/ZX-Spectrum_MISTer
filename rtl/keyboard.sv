// ZX Spectrum for Altera DE1
//
// Copyright (c) 2009-2011 Mike Stirling
// Copyright (c) 2015-2017 Sorgelig
// Rewritten 2020 by Julian Paolo Thiry
//
// All rights reserved
//
// Redistribution and use in source and synthezised forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// * Redistributions in synthesized form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the author nor the names of other contributors may
//   be used to endorse or promote products derived from this software without
//   specific prior written agreement from the author.
//
// * License is granted for non-commercial use only.  A fee may not be charged
//   for redistributions as source code or in synthesized/hardware form without 
//   specific prior written agreement from the author.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

/* PS/2 scancode to Spectrum matrix conversion

The ZX Spectrum has only 40 keys: 0-9, A-Z, SPACE, ENTER, CAPS SHIFT and SYMBOL
SHIFT. The ZX Spectrum+ raises this to 58, but all of the new keys function by
pressing two existing keys simultaneously. There is no physical difference
between pressing the new key and pressing the existing ones together.

This keyboard driver has been written with the general idea to have all 40
Spectrum keys match behavior exactly, but to also have the additional keys
behave in the most intuitive way possible. This can be boiled down to five
overall goals:

1. The A-Z, 0-9, and SPACE keys must behave exactly as they do on a real
	Spectrum. This is the easy one.

2. Both Shift keys must work as CAPS SHIFT, both Ctrl keys as SYMBOL SHIFT, and
   both Enter keys as ENTER. These keys must behave as if physically wired
	together. I.E., the following sequence must produce a SHIFT Q:
		Right Shift down, Left Shift down, Right Shift up, Q

Note that if 1 and 2 are met, this would be a fully functional driver, as every
Spectrum input can be accomplished. But we can do better!
		
3. Each additional key on the Spectrum+ keyboard that is wired to CAPS SHIFT
	must have an analogous key. These keys must behave as if they are physically
	wired together in	the same way. In particular, pressing and releasing
	different combinations of these keys with the keys they are wired to	must
	not cause any unexpected signals.
	These keys have been mapped as follows:
		BREAK        (CS+Space)  Escape
		EXTEND MODE  (CS+SS)     Tab
		EDIT         (CS+1)      Home
		CAPS LOCK    (CS+2)      Caps Lock
		TRUE VIDEO   (CS+3)      Page Up
		INV VIDEO    (CS+4)      Page Down
		Left Arrow   (CS+5)      Left Arrow or Numpad 4
		Down Arrow   (CS+6)      Down Arrow or Numpad 2
		Up Arrow     (CS+7)      Up Arrow or Numpad 8
		Right Arrow  (CS+8)      Right Arrow or Numpad 6
		GRAPH        (CS+9)      End
		DELETE       (CS+0)      BACKSPACE

4. Keys bearing symbols that can be produced on a Spectrum using the SYMBOL
	SHIFT key must produce those very symbols. This means that all of the
	following keys	must act as if they are wired directly to SYMBOL SHIFT and
	the appropriate second key:
		-_  =+  ;:  '"(US) '@(UK)  ,<  .>  /?  #~(UK) Num/  Num*  Num-  Num+
	For the first eight, a Shifted keypress must also send the appropriate keys.
	Additionally, toggling Shift after pressing any of these keys must not cause
	the Spectrum to see a different key being	pressed. For example, pressing
	Shift and ,< will send SS+R for the "<" symbol. Releasing Shift must not
	then cause SS+N (the "," symbol) to be sent, as this is clearly not the
	intent of the user.

Many might be tempted to stop there, but this driver goes beyond!

5. Keys bearing symbols that can be produced on a Spectrum in EXTEND MODE must
	produce those very symbols. This includes the following keys:
		[{  ]}  \|  `~(US) #~(UK)
	The Shifted versions must work appropriately as well. Since there is no `
	symbol on the Spectrum, this key will produce the only other extended mode
	symbol, ©.

	Note that this feature is meant as a convenience only. These symbols require
	three distinct events in order to be processed correctly:
		SYMBOL SHIFT + CAPS SHIFT
		Release of CAPS SHIFT
		SYMBOL SHIFT + Key
	Because of this, it is necessary to hold the key down long enough for all
	three events to occur. This is done quite quickly, but will definitely be
	noticed by fast typists. If the keypress is too short, the Spectrum will go
	into EXTEND MODE, but will not press the appropriate symbol key afterwards.
	Note also that these keys will result in the wrong things appearing on the
	screen if pressed when EXTEND MODE is already active. This means repeatedly
	pressing these keys quickly will likely produce unexpected keywords. This is
	NOT a bug, but merely the result of the combination of the prior two points.

Having been met, the above five goals should provide the ultimate combination
of Spectrum-like behavior with additional functionality provided by all those
extra keys.

In the above descriptions, there are a few references to (US) and (UK). This is
because US and UK keyboard layouts differ as follows:
    Layout         OSD Mode
	US    UK        US    UK
	`~    `¬        ©~    
	'"    '@        '"    '@
	\|    #~        \|    #~
	      \|              \|
You should change the OSD settings to match your keyboard's layout if you want
these keys to print what they display.

As a bonus, this driver also understands the Game Mode of the Recreated ZX
Spectrum keyboard. As this is a completely non-standard protocol, the OSD will
not be able to understand it at all. Qwerty Mode cannot be used because it does
not provide signals for CAPS SHIFT and SYMBOL SHIFT. Therefore, for best
results, you will need a second standard keyboard attached to the MiSTer to use
for controlling the OSD or using the Function Keys.

A final note for completeness: this driver also outputs the state of all
modifier keys and Function keys 1-11 for use as hotkeys outside the ULA.

*/
module keyboard
(
	input             reset,
	input             clk_sys,
	input      [10:0] ps2_key,
	input      [15:0] addr,
	input       [1:0] layout,

	output      [4:0] key_data,
	output     [11:1] Fn,
	output      [2:0] mod
);

reg        input_strobe = 0;
reg        extended = 0;
reg        key_down = 0;
reg  [7:0] code;

wire       us = layout == 0; // US layout
wire       uk = layout == 1; // UK layout
wire       rs = layout == 2; // Recreated Spectrum

reg [103:1] s; /* State of Keys. These registers are mostly indexed by PS/2 scan code,
with some additions for virtual keys, and codes above 67 packed into the unused codes.
    0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
0        F9   Vrt~ F5   F3   F1   F2   F12  Vt`# F10  F8   F6   F4   Tab  `~   Vrt_
1   F7   LAlt LShf Num6 LCtl Q    1    Vrt+ F11  Esc  Z    S    A    W    2    Vrt:
2   Num* C    X    D    E    4    3    Vt"@ Num2 Spc  V    F    T    R    5    Vrt<
3   Num+ N    B    H    G    Y    6    Vrt> RCtl Num8 M    J    U    7    8    Vrt?
4   Vrt[ ,    K    I    O    0    9    Vrt] Vrt{ .    /    L    ;    P    -    Vrt}
5   Num- Vrt| '    Num4 [{   =    RAlt Vrt\ CpLk RShf Entr ]}   Home \|   PgUp PgDn
6   Num/ Left Down Up   Rght NmEn BkSp End

In UK Layout, 61 is mapped to 5D, 5D is mapped to 0E, and signals are adjusted
to reflect what's printed on #~ and '@ (instead of `~ and '").

In RecSpec mode, 0E holds the state of SYMBOL SHIFT, which is not affected by
either of the Shift keys. The state of the Function keys and modifier keys is
still tracked so that a second keyboard may be used for hotkeys.
*/

wire    shift = s['h12] | s['h59];
wire      alt = s['h11] | s['h56];
wire     ctrl = s['h14] | s['h38];
assign mod[0] = shift;
assign mod[1] = alt;
assign mod[2] = ctrl;

                  //    F11     F10     F9      F8      F7      F6      F5      F4      F3      F2      F1
assign         Fn = { s['h18],s['h09],s['h01],s['h0A],s['h10],s['h0B],s['h03],s['h0C],s['h04],s['h06],s['h05] };

                  //    Num6    Num8    Num2    Num4    Tab     Esc     End     Rght 
wire [15:0] sCaps = { s['h13],s['h39],s['h28],s['h53],s['h0D],s['h19],s['h67],s['h64],
                  //    Up      Down    Left    PgDn    PgUp    CpLk    Home    BkSp
                      s['h63],s['h62],s['h61],s['h5F],s['h5E],s['h58],s['h5C],s['h66] };

                  //    /       .       ,       '       ;       =       -       Num+    Num-
wire [25:0] sSymb = { s['h4A],s['h49],s['h41],s['h52],s['h4C],s['h55],s['h4E],s['h30],s['h50],
                  //    Vrt?    Vrt>    Vrt<    Vt"@    Vrt:    Vrt+    Vrt_    Num*    Num/
                      s['h3F],s['h37],s['h2F],s['h27],s['h1F],s['h17],s['h0F],s['h20],s['h60],
                  //    Vr`#    Vrt~    Vrt\    Vrt|    Vrt]    Vrt}    Vrt[    Vrt{
                      s['h08],s['h02],s['h57],s['h51],s['h47],s['h4F],s['h40],s['h48] };

wire  [4:0] keys[7:0];
assign keys[0][0] = (rs & ~(s['h0E]))                            // In Recreated Spectrum mode, CAPS SHIFT is 0E
                 | (~rs & ~((shift | |sCaps[15:0]) & ~(|sSymb[25:0]))); // Shift or any CAPS key, except if any SYMBOL key
assign keys[0][1] = ~(s['h1A] | s['h1F]);                        // Z or Vrt:
assign keys[0][2] = ~(s['h22]);                                  // X
assign keys[0][3] = ~(s['h21] | s['h3F]);                        // C or Vrt?
assign keys[0][4] = ~(s['h2A] | s['h4A] | s['h60]);              // V or / or Num/

assign keys[1][0] = ~(s['h1C] | (s['h02] & s['h0E]));            // A or Vrt~ plus `~
assign keys[1][1] = ~(s['h1B] | (s['h51] & s['h5D]));            // S or Vrt| plus \|
assign keys[1][2] = ~(s['h23] | (s['h57] & s['h5D]));            // D or Vrt\ plus \|
assign keys[1][3] = ~(s['h2B] | (s['h48] & s['h54]));            // F or Vrt{ plus [{
assign keys[1][4] = ~(s['h34] | (s['h4F] & s['h5B]));            // G or Vrt} plus ]}

assign keys[2][0] = ~(s['h15]);                                  // Q
assign keys[2][1] = ~(s['h1D]);                                  // W
assign keys[2][2] = ~(s['h24]);                                  // E
assign keys[2][3] = ~(s['h2D] | s['h2F]);                        // R or Vrt<
assign keys[2][4] = ~(s['h2C] | s['h37]);                        // T or Vrt>

assign keys[3][0] = ~(s['h16] | s['h5C]);                        // 1 or Home
assign keys[3][1] = ~(s['h1E] | s['h58] | (uk & s['h27]));       // 2 or CpLk UK Vt"@
assign keys[3][2] = ~(s['h26] | s['h5E] | (uk & s['h08]));       // 3 or PgUp UK Vt`#
assign keys[3][3] = ~(s['h25] | s['h5F]);                        // 4 or PgDn
assign keys[3][4] = ~(s['h2E] | s['h61] | s['h53]);              // 5 or Left or Num4

assign keys[4][0] = ~(s['h45] | s['h0F] | s['h66]);              // 0 or Vrt_ or BkSp
assign keys[4][1] = ~(s['h46] | s['h67]);                        // 9 or End
assign keys[4][2] = ~(s['h3E] | s['h64] | s['h13]);              // 8 or Rght or Num6
assign keys[4][3] = ~(s['h3D] | s['h63] | s['h39] | s['h52]);    // 7 or Up or Num8 or '
assign keys[4][4] = ~(s['h36] | s['h62] | s['h28]);              // 6 or Down or Num2

assign keys[5][0] = ~(s['h4d] | (us & (s['h27] | (s['h08] & s['h0E])))); // P or US Vt"@ or US Vt`# plus `~
assign keys[5][1] = ~(s['h44] | s['h4C]);                        // O or ;
assign keys[5][2] = ~(s['h43]);                                  // I
assign keys[5][3] = ~(s['h3C] | (s['h47] & s['h5B]));            // U or Vrt] plus ]}
assign keys[5][4] = ~(s['h35] | (s['h40] & s['h54]));            // Y or Vrt[ plus [{

assign keys[6][0] = ~(s['h5A] | s['h65]);                        // Entr or NmEn
assign keys[6][1] = ~(s['h4B] | s['h55]);                        // L or =
assign keys[6][2] = ~(s['h42] | s['h17] | s['h30]);              // K or Vrt+ or Num+
assign keys[6][3] = ~(s['h3B] | s['h4E] | s['h50]);              // J or - or Num-
assign keys[6][4] = ~(s['h33]);                                  // H

assign keys[7][0] = ~(s['h29] | s['h19]);                        // Spc or Esc
assign keys[7][1] = ~(ctrl | |sSymb[25:0] | s['h0D]);            // Ctrl or any SYMBOL key or Tab
assign keys[7][2] = ~(s['h3A] | s['h49]);                        // M or .
assign keys[7][3] = ~(s['h31] | s['h41]);                        // N or ,
assign keys[7][4] = ~(s['h32] | s['h20]);                        // B or Num*


// Output addressed row to ULA
assign key_data = (!addr[8]  ? keys[0] : 5'b11111)
                 &(!addr[9]  ? keys[1] : 5'b11111)
                 &(!addr[10] ? keys[2] : 5'b11111)
                 &(!addr[11] ? keys[3] : 5'b11111)
                 &(!addr[12] ? keys[4] : 5'b11111)
                 &(!addr[13] ? keys[5] : 5'b11111)
                 &(!addr[14] ? keys[6] : 5'b11111)
                 &(!addr[15] ? keys[7] : 5'b11111);


always @(posedge clk_sys) begin
	reg old_reset = 0;
	old_reset <= reset;

	if(~old_reset & reset)begin
		s <= 0;
	end

	if(input_strobe & ~rs) begin
		case(code)
			8'h18,8'h09,8'h01,8'h0A,8'h0B,8'h03,8'h0C,8'h04,8'h06,8'h05, // F10-F1 except F7
			8'h15,8'h1D,8'h24,8'h2D,8'h2C,8'h35,8'h3C,8'h43,8'h44,8'h4D, // QWERTYUIOP
			8'h1C,8'h1B,8'h23,8'h2B,8'h34,8'h33,8'h3B,8'h42,8'h4B, // ASDFGHJKL
			8'h1A,8'h22,8'h21,8'h2A,8'h32,8'h31,8'h3A,8'h12,8'h59, // ZXCVBNM LShf RShf
			8'h16,8'h1E,8'h26,8'h25,8'h2E,8'h36,8'h3D,8'h3E,8'h46,8'h45, // 1234567890
			8'h66,8'h0D,8'h58,8'h29: // BkSp, Esc, Tab, CpLk, Spc
				s[code] <= key_down;

			// These are moved simply to fill in unused registers 
			8'h83: s['h10] <= key_down; // F7
			8'h78: s['h18] <= key_down; // F11
			8'h79: s['h30] <= key_down; // Num+
			8'h7B: s['h50] <= key_down; // Num-
			8'h7C: s['h20] <= key_down; // Num*
			8'h76: s['h19] <= key_down; // Esc
			
			// Track extended equivalents separately
			8'h11: if (extended) begin
					s['h56] <= key_down; // RAlt
				end else begin
					s['h11] <= key_down; // LAlt
				end
			8'h14: if (extended) begin
					s['h38] <= key_down; // RCtl
				end else begin
					s['h14] <= key_down; // LCtl
				end
			8'h6B: if (extended) begin
					s['h61] <= key_down; // Left
				end else begin
					s['h53] <= key_down; // Num4
				end
			8'h72: if (extended) begin
					s['h62] <= key_down; // Down
				end else begin
					s['h28] <= key_down; // Num2
				end
			8'h75: if (extended) begin
					s['h63] <= key_down; // Up
				end else begin
					s['h39] <= key_down; // Num8
				end
			8'h74: if (extended) begin
					s['h64] <= key_down; // Rght
				end else begin
					s['h13] <= key_down; // Num6
				end
			8'h5A: if (extended) begin
					s['h65] <= key_down; // NmEn
				end else begin
					s['h5A] <= key_down; // Entr
				end

			// Track only the extended (non-numpad) version of these to avoid
			// crowding the Numpad cursors.
			8'h69: if (extended) begin
					s['h67] <= key_down; // End
				end
			8'h6C: if (extended) begin
					s['h5C] <= key_down; // Home
				end
			8'h7D: if (extended) begin
					s['h5E] <= key_down; // PgUp
				end
			8'h7A: if (extended) begin
					s['h5F] <= key_down; // PgDn
				end
			
			// Symbol keys. These send the original code when pressed alone and
			// a virtual key when pressed with Shift. When released, both the
			// original and virtual key are cleared. This ensures that changing
			// the state of the Shift key does not change which Spectrum key
			// is being held down.
			8'h4E: if (key_down) begin   // - Vrt_
					if (shift) s['h0F] <= 1;
					else       s['h4E] <= 1;
				end else begin
					s['h0F] <= 0;
					s['h4E] <= 0;
				end
			8'h55: if (key_down) begin   // = Vrt+
					if (shift) s['h17] <= 1;
					else       s['h55] <= 1;
				end else begin
					s['h17] <= 0;
					s['h55] <= 0;
				end
			8'h4C: if (key_down) begin   // ; Vrt:
					if (shift) s['h1F] <= 1;
					else       s['h4C] <= 1;
				end else begin
					s['h1F] <= 0;
					s['h4C] <= 0;
				end
			8'h52: if (key_down) begin   // ' Vrt"
					if (shift) s['h27] <= 1;
					else       s['h52] <= 1;
				end else begin
					s['h27] <= 0;
					s['h52] <= 0;
				end
			8'h41: if (key_down) begin   // , Vrt<
					if (shift) s['h2F] <= 1;
					else       s['h41] <= 1;
				end else begin
					s['h2F] <= 0;
					s['h41] <= 0;
				end
			8'h49: if (key_down) begin   // . Vrt>
					if (shift) s['h37] <= 1;
					else       s['h49] <= 1;
				end else begin
					s['h37] <= 0;
					s['h49] <= 0;
				end
			8'h4A: if (extended) begin
					s['h60] <= key_down;  // Num/
				end else begin
					if (key_down) begin    // / Vrt?
						if (shift) s['h3F] <= 1;
						else       s['h4A] <= 1;
					end else begin
						s['h3F] <= 0;
						s['h4A] <= 0;
					end
				end
			
			// Extended Symbol keys. Both the original key and the virtual key
			// sent in the ghost key routine must be pressed to send the Spectrum
			// code. When the actual key is released, both possible virtual keys
			// must be released.
			8'h54: if (key_down) begin // [{
					s['h54] <= 1;
				end else begin
					s['h54] <= 0;
					s['h48] <= 0;
					s['h40] <= 0;
				end
			8'h5B: if (key_down) begin // ]}
					s['h5B] <= 1;
				end else begin
					s['h5B] <= 0;
					s['h4F] <= 0;
					s['h47] <= 0;
				end
			8'h61: if (uk) begin       // \| (UK only)
					if (key_down) begin
						s['h5D] <= 1;
					end else begin
						s['h5D] <= 0;
						s['h51] <= 0;
						s['h57] <= 0;
					end
				end
			8'h5D: if (us) begin       // \| (US only)
					if (key_down) begin
						s['h5D] <= 1;
					end else begin
						s['h5D] <= 0;
						s['h51] <= 0;
						s['h57] <= 0;
					end
				end else if (uk) begin  // #~ (UK only)
					if (key_down) begin
						if (shift) s['h0E] <= 1;
						else       s['h08] <= 1;
					end else begin
						s['h0E] <= 0;
						s['h02] <= 0;
						s['h08] <= 0;
					end
				end
			8'h0E: if (us) begin       // `~ (US only)
					if (key_down) begin
						s['h0E] <= 1;
					end else begin
						s['h0E] <= 0;
						s['h02] <= 0;
						s['h08] <= 0;
					end
				end
				
			// When an extended virtual key is pressed, only set the state if the
			// actual key is still pressed. This prevents getting the virtual key
			// stuck down since a "virtual key up" does not exist for these.
			8'h48,8'h40: if (s['h54]) s[code] <= key_down; // {[
			8'h4F,8'h47: if (s['h5B]) s[code] <= key_down; // }]
			8'h51,8'h57: if (s['h5D]) s[code] <= key_down; // |\
			8'h02,8'h08: if (s['h0E]) s[code] <= key_down; // ~`
					
			default: ;
		endcase
	end else if (input_strobe & rs) begin // Recreated ZX Spectrum
		if (key_down) begin
			case(code)
				8'h18,8'h09,8'h01,8'h0A,8'h0B,8'h03, // F10-F8, F6-F1, LShf, RShf
				8'h12,8'h59,8'h0C,8'h04,8'h06,8'h05: s[code] <= 1;
				8'h83: s['h10] <= 1; // F7
				8'h78: s['h18] <= 1; // F11
				8'h11: if (extended) s['h56] <= 1; else s['h11] <= 1; // RAlt, LAlt
				8'h14: if (extended) s['h38] <= 1; else s['h14] <= 1; // RCtl, LCtl

				8'h1C: if (shift) s['h2D] <= 1; else s['h16] <= 1; // A = R dn   a = 1 dn
				8'h32: if (shift) s['h2D] <= 0; else s['h16] <= 0; // B = R up   b = 1 up
				8'h21: if (shift) s['h2C] <= 1; else s['h1E] <= 1; // C = T dn   c = 2 dn
				8'h23: if (shift) s['h2C] <= 0; else s['h1E] <= 0; // D = T up   d = 2 up
				8'h24: if (shift) s['h35] <= 1; else s['h26] <= 1; // E = Y dn   e = 3 dn
				8'h2B: if (shift) s['h35] <= 0; else s['h26] <= 0; // F = Y up   f = 3 up
				8'h34: if (shift) s['h3C] <= 1; else s['h25] <= 1; // G = U dn   g = 4 dn
				8'h33: if (shift) s['h3C] <= 0; else s['h25] <= 0; // H = U up   h = 4 up
				8'h43: if (shift) s['h43] <= 1; else s['h2E] <= 1; // I = I dn   i = 5 dn
				8'h3B: if (shift) s['h43] <= 0; else s['h2E] <= 0; // J = I up   j = 5 up
				8'h42: if (shift) s['h44] <= 1; else s['h36] <= 1; // K = O dn   k = 6 dn
				8'h4B: if (shift) s['h44] <= 0; else s['h36] <= 0; // L = O up   l = 6 up
				8'h3A: if (shift) s['h4D] <= 1; else s['h3D] <= 1; // M = P dn   m = 7 dn
				8'h31: if (shift) s['h4D] <= 0; else s['h3D] <= 0; // N = P up   n = 7 up
				8'h44: if (shift) s['h1C] <= 1; else s['h3E] <= 1; // O = A dn   o = 8 dn
				8'h4D: if (shift) s['h1C] <= 0; else s['h3E] <= 0; // P = A up   p = 8 up
				8'h15: if (shift) s['h1B] <= 1; else s['h46] <= 1; // Q = S dn   q = 9 dn
				8'h2D: if (shift) s['h1B] <= 0; else s['h46] <= 0; // R = S up   r = 9 up
				8'h1B: if (shift) s['h23] <= 1; else s['h45] <= 1; // S = D dn   s = 0 dn
				8'h2C: if (shift) s['h23] <= 0; else s['h45] <= 0; // T = D up   t = 0 up
				8'h3C: if (shift) s['h2B] <= 1; else s['h15] <= 1; // U = F dn   u = Q dn
				8'h2A: if (shift) s['h2B] <= 0; else s['h15] <= 0; // V = F up   v = Q up
				8'h1D: if (shift) s['h34] <= 1; else s['h1D] <= 1; // W = G dn   w = W dn
				8'h22: if (shift) s['h34] <= 0; else s['h1D] <= 0; // X = G up   x = W up
				8'h35: if (shift) s['h33] <= 1; else s['h24] <= 1; // Y = H dn   y = E dn
				8'h1A: if (shift) s['h33] <= 0; else s['h24] <= 0; // Z = H up   z = E up
				8'h45:                   if (~shift) s['h3B] <= 1; //            0 = J dn
				8'h16: if (shift) s['h14] <= 1; else s['h3B] <= 0; // ! = SS dn  1 = J up
				8'h1E:                   if (~shift) s['h42] <= 1; //            2 = K dn
				8'h26:                   if (~shift) s['h42] <= 0; //            3 = K up
				8'h25: if (shift) s['h14] <= 0; else s['h4B] <= 1; // $ = SS up  4 = L dn
				8'h2E: if (shift) s['h29] <= 1; else s['h4B] <= 0; // % = Sp dn  5 = L up
				8'h36: if (shift) s['h29] <= 0; else s['h5A] <= 1; // ^ = Sp up  6 = En dn
				8'h3D:                   if (~shift) s['h5A] <= 0; //            7 = En up
				8'h3E:                   if (~shift) s['h0E] <= 1; //            8 = CS dn
				8'h46:                   if (~shift) s['h0E] <= 0; //            9 = CS up
				8'h41: if (shift) s['h1A] <= 1; else s['h32] <= 1; // < = Z dn   , = B dn
				8'h49: if (shift) s['h1A] <= 0; else s['h32] <= 0; // > = Z up   . = B up
				8'h4E:                   if (~shift) s['h22] <= 1; //            - = X dn
				8'h55:                   if (~shift) s['h22] <= 0; //            = = X up
				8'h54: if (shift) s['h3A] <= 1; else s['h21] <= 1; // { = M dn   [ = C dn
				8'h5B: if (shift) s['h3A] <= 0; else s['h21] <= 0; // } = M up   ] = C up
				8'h4C: if (shift) s['h2A] <= 0; else s['h2A] <= 1; // : = V up   ; = V dn
				8'h4A: if (shift) s['h31] <= 0; else s['h31] <= 1; // ? = N up   / = N dn
			endcase
		end else begin // key up
			case(code)
				8'h18,8'h09,8'h01,8'h0A,8'h0B,8'h03, // F10-F8, F6-F1, LShf, RShf
				8'h12,8'h59,8'h0C,8'h04,8'h06,8'h05: s[code] <= 0;
				8'h83: s['h10] <= 0; // F7
				8'h78: s['h18] <= 0; // F11
				8'h11: if (extended) s['h56] <= 0; else s['h11] <= 0; // RAlt, LAlt
				8'h14: if (extended) s['h38] <= 0; else s['h14] <= 0; // RCtl, LCtl
			endcase
		end
	end
end

reg [9:0] auto[55] = '{
	255,

	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,

	{2'b00, 8'h59}, // right shift
	{2'b00, 8'h12}, // left shift
	{2'b01, 8'h11}, // right alt
	{2'b00, 8'h11}, // left alt
	{2'b01, 8'h14}, // right ctrl
	{2'b00, 8'h14}, // left ctrl
	{2'b10, 8'h3b}, // J
	{2'b00, 8'h3b}, // J
	{2'b10, 8'h14}, // right ctrl
	{2'b10, 8'h4d}, // P
	{2'b00, 8'h4d}, // P
	0,
	{2'b10, 8'h4d}, // P
	{2'b00, 8'h4d}, // P
	{2'b00, 8'h14}, // right ctrl
	{2'b10, 8'h5a}, // enter
	{2'b00, 8'h5a}, // enter
	255,
	{2'b10, 8'h0d}, // tab
	{2'b00, 8'h0d}, // tab
	{2'b10, 8'h00}, // vkey
	255
};

always @(posedge clk_sys) begin
	integer div;
	reg [6:0] auto_pos = 0;
	reg old_reset = 0;
	reg old_s;

	input_strobe <= 0;
	old_reset <= reset;
	old_s <= ps2_key[10];

	if(~old_reset & reset)begin
		auto_pos <= 0;
	end else begin
		if(auto[auto_pos] == 255) begin
			div <=0;
			if(old_s != ps2_key[10]) begin
				{input_strobe, key_down, extended, code} <= {1'b1, ps2_key[9:0]};
				if(ps2_key[9:0] == 9) auto_pos <= 1; // F10
				if(ps2_key[9]) begin
					if (us) begin
						//The virtual keys are carefully arranged to make this math work.
						case({shift,ps2_key[8:0]})
							10'h254,10'h25B,10'h25D,10'h20E: begin
									auto_pos <= 51;
									auto[53] <= {2'b10, ps2_key[7:0] - 8'h0C}; // { } | ~
								end
							9'h54,9'h5B: begin
									auto_pos <= 51;
									auto[53] <= {2'b10, ps2_key[7:0] - 8'h14}; // [ ]
								end
							9'h5D,9'h0E: begin
									auto_pos <= 51;
									auto[53] <= {2'b10, ps2_key[7:0] - 8'h06}; // \ `
								end
						endcase
					end else if (uk) begin
						//Unfortunately, the careful arrangement only works for US mode.
						case({shift,ps2_key[8:0]})
							10'h261: begin
									auto_pos <= 51;
									auto[53] <= {2'b10, 8'h5D - 8'h0C};        // |
								end
							9'h61: begin
									auto_pos <= 51;
									auto[53] <= {2'b10, 8'h5D - 8'h06};        // \
								end
							10'h25D: begin
									auto_pos <= 51;
									auto[53] <= {2'b10, 8'h0E - 8'h0C};        // ~
								end
							10'h254,10'h25B: begin
									auto_pos <= 51;
									auto[53] <= {2'b10, ps2_key[7:0] - 8'h0C}; // { }
								end
							9'h54,9'h5B: begin
									auto_pos <= 51;
									auto[53] <= {2'b10, ps2_key[7:0] - 8'h14}; // [ ]
								end
						endcase
					end
				end
			end
		end else begin
			if(auto_pos >= 51) begin
				//This ghost writer cannot block the keyboard as we must be able to receive key up events.
				if(old_s != ps2_key[10]) begin
					{input_strobe, key_down, extended, code} <= {1'b1, ps2_key[9:0]};
				end else begin
					div <= div + 1;
					if(div == 2500000) begin
						div <=0;
						if(auto[auto_pos]) {input_strobe, key_down, extended, code} <= {1'b1, auto[auto_pos]};
						auto_pos <= auto_pos + 1'd1;
					end
				end
			end else begin
				div <= div + 1;
				if(div == 7000000) begin
					div <=0;
					if(auto[auto_pos]) {input_strobe, key_down, extended, code} <= {1'b1, auto[auto_pos]};
					auto_pos <= auto_pos + 1'd1;
				end
			end
		end
	end
end
endmodule
