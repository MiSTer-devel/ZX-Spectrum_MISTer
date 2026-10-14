//
// AY-3-8910 Stereo Mixer
//
// ABC / ACB / Mono pans, Q1.31 in, Q4.28 out
//
// Copyright (c) 2026 Ilia Sharin
//

module ay_stereo_mixer
(
    input  wire        clk,
    input  wire        ce,
    input  wire [1:0]  stereo_mode,  // 0=ABC, 1=ACB, 2=Mono

    input  wire [31:0] ch_a,
    input  wire [31:0] ch_b,
    input  wire [31:0] ch_c,

    output reg  [31:0] out_left,
    output reg  [31:0] out_right
);

// pan gains with the final / 3 folded in: 0.9/3, 0.5/3, 0.1/3
localparam [31:0] PAN_LEFT   = 32'h26666666;
localparam [31:0] PAN_CENTER = 32'h15555555;
localparam [31:0] PAN_RIGHT  = 32'h04444444;

reg [31:0] pan_a_l, pan_a_r;
reg [31:0] pan_b_l, pan_b_r;
reg [31:0] pan_c_l, pan_c_r;

always @(*) begin
    case (stereo_mode)
        2'd0: begin // ABC: A=Left, B=Center, C=Right
            pan_a_l = PAN_LEFT;   pan_a_r = PAN_RIGHT;
            pan_b_l = PAN_CENTER; pan_b_r = PAN_CENTER;
            pan_c_l = PAN_RIGHT;  pan_c_r = PAN_LEFT;
        end
        2'd1: begin // ACB: A=Left, C=Center, B=Right
            pan_a_l = PAN_LEFT;   pan_a_r = PAN_RIGHT;
            pan_b_l = PAN_RIGHT;  pan_b_r = PAN_LEFT;
            pan_c_l = PAN_CENTER; pan_c_r = PAN_CENTER;
        end
        default: begin // Mono: All center
            pan_a_l = PAN_CENTER; pan_a_r = PAN_CENTER;
            pan_b_l = PAN_CENTER; pan_b_r = PAN_CENTER;
            pan_c_l = PAN_CENTER; pan_c_r = PAN_CENTER;
        end
    endcase
end

reg [63:0] prod_a_l, prod_a_r;
reg [63:0] prod_b_l, prod_b_r;
reg [63:0] prod_c_l, prod_c_r;

always @(posedge clk) begin
    if (ce) begin
        prod_a_l <= ch_a * pan_a_l;
        prod_a_r <= ch_a * pan_a_r;
        prod_b_l <= ch_b * pan_b_l;
        prod_b_r <= ch_b * pan_b_r;
        prod_c_l <= ch_c * pan_c_l;
        prod_c_r <= ch_c * pan_c_r;
    end
end

reg [33:0] sum_l, sum_r;

always @(posedge clk) begin
    if (ce) begin
        sum_l <= {2'b0, prod_a_l[63:32]} + {2'b0, prod_b_l[63:32]} + {2'b0, prod_c_l[63:32]};
        sum_r <= {2'b0, prod_a_r[63:32]} + {2'b0, prod_b_r[63:32]} + {2'b0, prod_c_r[63:32]};
    end
end

always @(posedge clk) begin
    if (ce) begin
        out_left  <= sum_l[33:2];
        out_right <= sum_r[33:2];
    end
end

endmodule
