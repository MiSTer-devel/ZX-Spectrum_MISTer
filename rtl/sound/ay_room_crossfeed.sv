//
// AY-3-8910 Room Crossfeed
//
// 2 ms delayed opposite-channel blend, no low-pass
//
// Copyright (c) 2026 Ilia Sharin
//

module ay_room_crossfeed
(
    input  wire        clk,
    input  wire        ce,
    input  wire        reset,
    input  wire        enable,

    // 0=Off, 1=-15dB, 2=-14dB, 3=-13dB, 4=-12dB, 5=-9dB, 6=-6dB, 7=-3dB, 8=-2dB, 9=-1dB
    input  wire [3:0]  room_level,

    input  wire signed [31:0] in_left,    // Q4.28 input
    input  wire signed [31:0] in_right,
    output reg  signed [31:0] out_left,   // Q4.28 output
    output reg  signed [31:0] out_right
);

// 2 ms at 218.75 kHz
localparam DELAY_SAMPLES = 437;
localparam DELAY_BITS = 9;

// read and write addresses always differ: no read-during-write logic
(* ramstyle = "no_rw_check" *) reg signed [31:0] delay_l [0:511];
(* ramstyle = "no_rw_check" *) reg signed [31:0] delay_r [0:511];

integer i;
initial begin
    for (i = 0; i < 512; i = i + 1) begin
        delay_l[i] = 0;
        delay_r[i] = 0;
    end
end

reg [DELAY_BITS-1:0] delay_idx;

function signed [31:0] get_room_coef;
    input [3:0] level;
    begin
        case (level)
            4'd0:    get_room_coef = 32'h00000000;  // Off
            4'd1:    get_room_coef = 32'h16C8B439;  // 0.178 (-15dB)
            4'd2:    get_room_coef = 32'h19999999;  // 0.20  (-14dB)
            4'd3:    get_room_coef = 32'h1CAC0831;  // 0.224 (-13dB)
            4'd4:    get_room_coef = 32'h20000000;  // 0.25  (-12dB)
            4'd5:    get_room_coef = 32'h2CCCCCCD;  // 0.35  (-9dB)
            4'd6:    get_room_coef = 32'h40000000;  // 0.50  (-6dB)
            4'd7:    get_room_coef = 32'h5AE147AE;  // 0.71  (-3dB)
            4'd8:    get_room_coef = 32'h6528F5C3;  // 0.79  (-2dB)
            4'd9:    get_room_coef = 32'h71EB851F;  // 0.89  (-1dB)
            default: get_room_coef = 32'h00000000;
        endcase
    end
endfunction

reg [2:0] state;
localparam S_IDLE   = 3'd0;
localparam S_READ   = 3'd1;
localparam S_MULT_L = 3'd2;
localparam S_MULT_R = 3'd3;
localparam S_OUTPUT = 3'd4;

reg signed [31:0] in_left_r, in_right_r;
reg signed [31:0] delayed_l, delayed_r;
reg signed [31:0] crossfeed_l;
reg signed [31:0] room_coef;

reg signed [31:0] mult_a, mult_b;
wire signed [63:0] mult_result = $signed(mult_a) * $signed(mult_b);

wire [DELAY_BITS-1:0] read_idx = (delay_idx >= DELAY_SAMPLES) ?
                                  (delay_idx - DELAY_SAMPLES) :
                                  (delay_idx + 9'd512 - DELAY_SAMPLES);

// separate process with a registered read, so Quartus infers M10K
reg signed [31:0] delay_q_l, delay_q_r;
wire delay_we = ~reset & ce & (state == S_IDLE);

always @(posedge clk) begin
    if (delay_we) begin
        delay_l[delay_idx] <= in_left;
        delay_r[delay_idx] <= in_right;
    end
    delay_q_l <= delay_l[read_idx];
    delay_q_r <= delay_r[read_idx];
end

always @(posedge clk) begin
    if (reset) begin
        delay_idx <= 0;
        state <= S_IDLE;
        out_left <= 0;
        out_right <= 0;
        in_left_r <= 0;
        in_right_r <= 0;
        delayed_l <= 0;
        delayed_r <= 0;
        crossfeed_l <= 0;
        room_coef <= 0;
        mult_a <= 0;
        mult_b <= 0;
    end
    else begin
        case (state)
            S_IDLE: begin
                if (ce) begin
                    in_left_r <= in_left;
                    in_right_r <= in_right;

                    room_coef <= get_room_coef(room_level);

                    state <= S_READ;
                end
            end

            S_READ: begin
                delayed_l <= delay_q_r;
                delayed_r <= delay_q_l;

                delay_idx <= (delay_idx + 1) & 9'h1FF;

                if (!enable || room_level == 0) begin
                    out_left <= in_left_r;
                    out_right <= in_right_r;
                    state <= S_IDLE;
                end
                else begin
                    state <= S_MULT_L;
                end
            end

            S_MULT_L: begin
                mult_a <= delayed_l;
                mult_b <= room_coef;
                state <= S_MULT_R;
            end

            S_MULT_R: begin
                crossfeed_l <= mult_result[62:31];
                mult_a <= delayed_r;
                mult_b <= room_coef;
                state <= S_OUTPUT;
            end

            S_OUTPUT: begin
                out_left  <= in_left_r  + crossfeed_l;
                out_right <= in_right_r + mult_result[62:31];
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
