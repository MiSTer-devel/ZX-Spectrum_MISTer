//
// AY-3-8910 Punch Enhancer
//
// transient emphasis:
//   diff = in - prev
//   env  = mag > env ? mag * attack + env * (1 - attack) : env * release
//   out  = in + diff * edge + diff * env * boost
//
// Copyright (c) 2026 Ilia Sharin
//

module ay_punch_enhancer
(
    input  wire        clk,
    input  wire        ce,
    input  wire        reset,
    input  wire        enable,
    input  wire        preset,       // 0 = AY, 1 = Paula/Beeper

    input  wire signed [31:0] in_left,    // Q4.28
    input  wire signed [31:0] in_right,
    output reg  signed [31:0] out_left,   // Q4.28
    output reg  signed [31:0] out_right
);

// rescaled for 218.75 kHz (r = 4.9603): attack / release as r-th roots,
// edge * r, boost * r^2 (held as mantissa * 2^shift)
localparam signed [31:0] ATTACK           = 32'h08E17CB6;
localparam signed [31:0] ONE_MINUS_ATTACK = 32'h771E834A;
localparam signed [31:0] RELEASE_AY       = 32'h7FFCB242;
localparam signed [31:0] RELEASE_PB       = 32'h7FF2C702;
localparam signed [31:0] EDGE_AY          = 32'h130C30C3;
localparam signed [31:0] EDGE_PB          = 32'h32CB2CB3;
localparam signed [31:0] BOOST_MANT       = 32'h4EBC35EC;

wire signed [31:0] release_coef = preset ? RELEASE_PB : RELEASE_AY;
wire signed [31:0] edge_coef    = preset ? EDGE_PB    : EDGE_AY;
wire signed [31:0] boost_coef   = BOOST_MANT;
wire [1:0]         boost_shift  = preset ? 2'd3 : 2'd2;

reg [4:0] state;
localparam S_IDLE          = 5'd0;
localparam S_DIFF          = 5'd1;
localparam S_MAG           = 5'd2;
localparam S_ENVL_LOAD     = 5'd3;
localparam S_ENVL_C1       = 5'd4;
localparam S_ENVL_C2       = 5'd5;
localparam S_ENVR_LOAD     = 5'd6;
localparam S_ENVR_C1       = 5'd7;
localparam S_ENVR_C2       = 5'd8;
localparam S_EDGEL_LOAD    = 5'd9;
localparam S_EDGER_LOAD    = 5'd10;
localparam S_TRANSL_LOAD   = 5'd11;
localparam S_TRANSL_C      = 5'd12;
localparam S_TRANSL_B_LOAD = 5'd13;
localparam S_TRANSR_LOAD   = 5'd14;
localparam S_TRANSR_C      = 5'd15;
localparam S_TRANSR_B_LOAD = 5'd16;
localparam S_OUT           = 5'd17;

reg signed [31:0] in_l_r, in_r_r;
reg signed [31:0] prev_l, prev_r;
reg signed [31:0] diff_l, diff_r;
reg signed [31:0] mag_l, mag_r;
reg signed [31:0] env_l, env_r;
reg signed [31:0] edge_l, edge_r;
reg signed [31:0] trans_l;
reg signed [31:0] t1;
reg att_l, att_r;

reg signed [31:0] mult_a, mult_b;
wire signed [63:0] mult_result = mult_a * mult_b;

wire signed [31:0] res31 = mult_result[62:31];
wire signed [31:0] res28 = mult_result[59:28];

always @(posedge clk) begin
    if (reset) begin
        state <= S_IDLE;
        out_left <= 0;
        out_right <= 0;
        in_l_r <= 0; in_r_r <= 0;
        prev_l <= 0; prev_r <= 0;
        diff_l <= 0; diff_r <= 0;
        mag_l <= 0;  mag_r <= 0;
        env_l <= 0;  env_r <= 0;
        edge_l <= 0; edge_r <= 0;
        trans_l <= 0;
        t1 <= 0;
        att_l <= 0; att_r <= 0;
        mult_a <= 0; mult_b <= 0;
    end
    else begin
        case (state)
            S_IDLE: begin
                if (ce) begin
                    in_l_r <= in_left;
                    in_r_r <= in_right;
                    if (enable) begin
                        state <= S_DIFF;
                    end
                    else begin
                        out_left  <= in_left;
                        out_right <= in_right;
                    end
                end
            end

            S_DIFF: begin
                diff_l <= in_l_r - prev_l;
                diff_r <= in_r_r - prev_r;
                state <= S_MAG;
            end

            S_MAG: begin
                mag_l <= diff_l[31] ? -diff_l : diff_l;
                mag_r <= diff_r[31] ? -diff_r : diff_r;
                state <= S_ENVL_LOAD;
            end

            S_ENVL_LOAD: begin
                att_l <= (mag_l > env_l);
                if (mag_l > env_l) begin
                    mult_a <= mag_l;  mult_b <= ATTACK;
                end
                else begin
                    mult_a <= env_l;  mult_b <= release_coef;
                end
                state <= S_ENVL_C1;
            end

            S_ENVL_C1: begin
                if (att_l) begin
                    t1 <= res31;
                    mult_a <= env_l;  mult_b <= ONE_MINUS_ATTACK;
                    state <= S_ENVL_C2;
                end
                else begin
                    env_l <= res31;
                    state <= S_ENVR_LOAD;
                end
            end

            S_ENVL_C2: begin
                env_l <= t1 + res31;
                state <= S_ENVR_LOAD;
            end

            S_ENVR_LOAD: begin
                att_r <= (mag_r > env_r);
                if (mag_r > env_r) begin
                    mult_a <= mag_r;  mult_b <= ATTACK;
                end
                else begin
                    mult_a <= env_r;  mult_b <= release_coef;
                end
                state <= S_ENVR_C1;
            end

            S_ENVR_C1: begin
                if (att_r) begin
                    t1 <= res31;
                    mult_a <= env_r;  mult_b <= ONE_MINUS_ATTACK;
                    state <= S_ENVR_C2;
                end
                else begin
                    env_r <= res31;
                    state <= S_EDGEL_LOAD;
                end
            end

            S_ENVR_C2: begin
                env_r <= t1 + res31;
                state <= S_EDGEL_LOAD;
            end

            S_EDGEL_LOAD: begin
                mult_a <= diff_l;  mult_b <= edge_coef;
                state <= S_EDGER_LOAD;
            end

            S_EDGER_LOAD: begin
                edge_l <= res31;
                mult_a <= diff_r;  mult_b <= edge_coef;
                state <= S_TRANSL_LOAD;
            end

            S_TRANSL_LOAD: begin
                edge_r <= res31;
                mult_a <= diff_l;  mult_b <= env_l;
                state <= S_TRANSL_C;
            end

            S_TRANSL_C: begin
                t1 <= res28;
                state <= S_TRANSL_B_LOAD;
            end

            S_TRANSL_B_LOAD: begin
                mult_a <= t1;  mult_b <= boost_coef;
                state <= S_TRANSR_LOAD;
            end

            S_TRANSR_LOAD: begin
                trans_l <= res31 <<< boost_shift;
                mult_a <= diff_r;  mult_b <= env_r;
                state <= S_TRANSR_C;
            end

            S_TRANSR_C: begin
                t1 <= res28;
                state <= S_TRANSR_B_LOAD;
            end

            S_TRANSR_B_LOAD: begin
                mult_a <= t1;  mult_b <= boost_coef;
                state <= S_OUT;
            end

            S_OUT: begin
                out_left  <= in_l_r + edge_l + trans_l;
                out_right <= in_r_r + edge_r + (res31 <<< boost_shift);
                prev_l <= in_l_r;
                prev_r <= in_r_r;
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
