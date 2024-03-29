/* This file is part of JT12.

 
    JT12 program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    JT12 program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with JT12.  If not, see <http://www.gnu.org/licenses/>.

    Author: Jose Tejada Gomez. Twitter: @topapate
    Version: 1.0
    Date: 27-12-2018
*/

// Wrapper to output only combined channels. Defaults to YM2203 mode.



module jt03(
    input           rst,        // rst should be at least 6 clk&cen cycles long
    input           clk,        // CPU clock
    input           cen,        // optional clock enable, it not needed leave as 1'b1
    input   [7:0]   din,
    input           addr,
    input           cs_n,
    input           wr_n,

    input           psg_type,

    output  [7:0]   dout,
    output          irq_n,

    // Separated output
    output          [ 7:0] psg_A,
    output          [ 7:0] psg_B,
    output          [ 7:0] psg_C,
    output  signed  [15:0] fm_snd,
    // combined output
    output          [ 9:0] psg_snd,    
    output  signed  [15:0] snd,
    output          snd_sample
);

jt12_top #(.use_lfo(0),.use_ssg(1), .num_ch(3), .use_pcm(0), .use_lr(0)) 
u_jt12(
    .rst            ( rst          ),        // rst should be at least 6 clk&cen cycles long
    .clk            ( clk          ),        // CPU clock
    .cen            ( cen          ),        // optional clock enable, it not needed leave as 1'b1
    .din            ( din          ),
    .addr           ( {1'b0, addr} ),
    .cs_n           ( cs_n         ),
    .wr_n           ( wr_n         ),

    .psg_type       ( psg_type     ),

    .dout           ( dout         ),
    .irq_n          ( irq_n        ),
    // Separated output
    .psg_A          ( psg_A        ),
    .psg_B          ( psg_B        ),
    .psg_C          ( psg_C        ),
    .psg_snd        ( psg_snd      ),    
    .fm_snd_left    ( fm_snd       ),
    .fm_snd_right   (),

    .snd_right      ( snd          ),
    .snd_left       (),
    .snd_sample     ( snd_sample   )
);

endmodule // jt03
