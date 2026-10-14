//
// AY-3-8910 DC Offset Filter
//
// 5 Hz one-pole RC high-pass (output coupling capacitor):
//   y = a * (y1 + x - x1), computed as y = s - k*s with k = 1 - a
//
// Copyright (c) 2026 Ilia Sharin
//

module ay_dc_filter
#(
    parameter SIGNED_IN = 0
)
(
    input  wire        clk,
    input  wire        ce,
    input  wire        reset,

    input  wire [31:0] in_sample,          // Q4.28 input, signed if SIGNED_IN
    output reg  signed [31:0] out_sample   // Q4.28 signed output
);

// k = 1 - a for 5 Hz at 218.75 kHz, Q0.40
localparam signed [29:0] DC_K = 30'sd157884418;

reg signed [32:0] x1;
reg signed [55:0] y;
reg signed [55:0] s;
reg signed [85:0] ks;
reg        [1:0]  state;

localparam S_IDLE = 2'd0;
localparam S_MUL  = 2'd1;
localparam S_OUT  = 2'd2;

wire signed [32:0] x_in = $signed({SIGNED_IN ? in_sample[31] : 1'b0, in_sample});
wire signed [55:0] y_new = s - $signed(ks[85:40]);
wire signed [55:0] y_rnd = y_new + 56'sd32768;

always @(posedge clk) begin
    if (reset) begin
        x1 <= 0;
        y <= 0;
        s <= 0;
        ks <= 0;
        out_sample <= 0;
        state <= S_IDLE;
    end
    else begin
        case (state)
            S_IDLE: begin
                if (ce) begin
                    s <= y + ($signed(x_in - x1) <<< 16);
                    x1 <= x_in;
                    state <= S_MUL;
                end
            end

            S_MUL: begin
                ks <= s * DC_K;
                state <= S_OUT;
            end

            S_OUT: begin
                y <= y_new;
                out_sample <= y_rnd[47:16];
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
