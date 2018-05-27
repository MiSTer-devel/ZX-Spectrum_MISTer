module turbosound
(
   input        CLK,		   // Global clock
	input        CE,        // PSG Clock enable
   input        RESET,	   // Chip RESET (set all Registers to '0', active high)
   input        BDIR,	   // Bus Direction (0 - read , 1 - write)
   input        BC,		   // Bus control
   input  [7:0] DI,	      // Data In
   output [7:0] DO,	      // Data Out
   output [7:0] CHANNEL_A, // PSG Output channel A
   output [7:0] CHANNEL_B, // PSG Output channel B
   output [7:0] CHANNEL_C, // PSG Output channel C

   input        SEL,
   input        MODE,

	input  [7:0] IOA_in,
	output [7:0] IOA_out,

	input  [7:0] IOB_in,
	output [7:0] IOB_out
);

// AY1 selected by default
reg ay_select = 1'b1;

// Bus control for each AY chips
wire BC_0;
wire BC_1;

// Data outputs for each AY chips
wire [7:0] DO_0;
wire [7:0] DO_1;

// AY activity signals
reg [5:0]ay0_active;
reg [5:0]ay1_active;
wire ay0_playing;
wire ay1_playing;

// AY0 channel output data
wire [7:0] psg_ch_a_0;
wire [7:0] psg_ch_b_0;
wire [7:0] psg_ch_c_0;

// AY1 channel output data
wire [7:0] psg_ch_a_1;
wire [7:0] psg_ch_b_1;
wire [7:0] psg_ch_c_1;

// Mixed channel data
wire [8:0] sum_ch_a;
wire [8:0] sum_ch_b;
wire [8:0] sum_ch_c;

// Mixed channel data (normalized)
wire [7:0] mix_ch_a;
wire [7:0] mix_ch_b;
wire [7:0] mix_ch_c;

always_ff @(posedge CLK or posedge RESET) begin
	if (RESET == 1'b1) begin
		// Select AY1 after reset
		ay_select <= 1'b1;
		
	end
	else if (BDIR && BC && DI[7:1] == 7'b1111111) begin
		// Select AY0 or AY1 according to lower bit of data register (1111 111N)
		ay_select <= DI[0];
	end
end

ym2149 ym2149_0
(
	.CLK(CLK),
	.CE(CE),
	.RESET(RESET),
	.BDIR(BDIR),
	.BC(BC_0),
	.DI(DI),
	.DO(DO_0),
	.CHANNEL_A(psg_ch_a_0),
	.CHANNEL_B(psg_ch_b_0),
	.CHANNEL_C(psg_ch_c_0),
	.ACTIVE(ay0_active),
	.SEL(SEL),
	.MODE(MODE),

	.IOA_in(IOA_in),
	.IOA_out(IOA_out),
	.IOB_in(IOB_in),
	.IOB_out(IOB_out)
);

// AY1 (Default AY)
ym2149 ym2149_1
(
	.CLK(CLK),
	.CE(CE),
	.RESET(RESET),
	.BDIR(BDIR),
	.BC(BC_1),
	.DI(DI),
	.DO(DO_1),
	.CHANNEL_A(psg_ch_a_1),
	.CHANNEL_B(psg_ch_b_1),
	.CHANNEL_C(psg_ch_c_1),
	.ACTIVE(ay1_active),
	.SEL(SEL),
	.MODE(MODE),

	.IOA_in(IOA_in),
	.IOA_out(IOA_out),
	.IOB_in(IOB_in),
	.IOB_out(IOB_out)
);

assign BC_0 = ~ay_select & BC;
assign BC_1 = ay_select & BC;
assign DO = ay_select ? DO_1 : DO_0;

// AY activity signals
assign ay0_playing = | ay0_active; // OR reduction (all bits of ay0_active OR'ed with each other)
assign ay1_playing = | ay1_active; // OR reduction (all bits of ay1_active OR'ed with each other)

// Mix channel signals from both AY/YM chips (extending to 9 bits width to prevent clipping)
assign sum_ch_a = { 1'b0, psg_ch_a_1 } + { 1'b0, psg_ch_a_0 };
assign sum_ch_b = { 1'b0, psg_ch_b_1 } + { 1'b0, psg_ch_b_0 };
assign sum_ch_c = { 1'b0, psg_ch_c_1 } + { 1'b0, psg_ch_c_0 };

// Fit samples back to 8-bit
assign mix_ch_a = sum_ch_a[8:1];
assign mix_ch_b = sum_ch_b[8:1];
assign mix_ch_c = sum_ch_c[8:1];


// Control output channels (Only AY_1 plays if not in TurboSound mode)
assign CHANNEL_A = ~ay0_playing ? psg_ch_a_1 : mix_ch_a;
assign CHANNEL_B = ~ay0_playing ? psg_ch_b_1 : mix_ch_b;
assign CHANNEL_C = ~ay0_playing ? psg_ch_c_1 : mix_ch_c;

//assign CHANNEL_A = mix_ch_a;
//assign CHANNEL_B = mix_ch_b;
//assign CHANNEL_C = mix_ch_c;
//assign CHANNEL_A = (ay1_active & ~ay0_active) ? psg_ch_a_1 : mix_ch_a;
//assign CHANNEL_B = (ay1_active & ~ay0_active) ? psg_ch_b_1 : mix_ch_b;
//assign CHANNEL_C = (ay1_active & ~ay0_active) ? psg_ch_c_1 : mix_ch_c;


endmodule