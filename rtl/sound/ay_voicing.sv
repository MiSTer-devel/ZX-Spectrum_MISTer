//
// AY-3-8910 Tone Voicing (fixed tonal-balance EQ)
//
// Up to three biquads (HPF, peaking EQ, LPF) per channel.
// Profiles: 0 Flat, 1 Classic, 2 Headphones, 3 Warm, 4 TV, 5 Small speaker.
// Coefficients designed for 218.75 kHz (Q2.30); state Q6.40.
// One shared multiplier; outputs are valid LATENCY clocks after ce.
//
// Copyright (c) 2026 Ilia Sharin
//

module ay_voicing
(
    input  wire        clk,
    input  wire        ce,
    input  wire        reset,
    input  wire [2:0]  preset,         // 0..5 (6, 7 = Flat)

    input  wire signed [31:0] in_left,     // Q4.28
    input  wire signed [31:0] in_right,
    output reg  signed [31:0] out_left,    // Q4.28
    output reg  signed [31:0] out_right
);

localparam LATENCY = 40;

// {b0, b1, b2, a1, a2} per (profile, section)
function [159:0] voicing_coef;
    input [4:0] row;
    begin
        case (row)
            5'd0: voicing_coef = {32'sd1073741824, 32'sd0, 32'sd0, 32'sd0, 32'sd0}; // flat (identity)
            5'd1: voicing_coef = {32'sd1073741824, 32'sd0, 32'sd0, 32'sd0, 32'sd0}; // flat (identity)
            5'd2: voicing_coef = {32'sd1073741824, 32'sd0, 32'sd0, 32'sd0, 32'sd0}; // flat (identity)
            5'd3: voicing_coef = {32'sd1072752732, -32'sd1072752732, 32'sd0, -32'sd1071763640, 32'sd0}; // classic HPF
            5'd4: voicing_coef = {32'sd1074324827, -32'sd2144712642, 32'sd1070397925, -32'sd2144712642, 32'sd1070980928}; // classic peak
            5'd5: voicing_coef = {32'sd1073741824, 32'sd0, 32'sd0, 32'sd0, 32'sd0}; // classic (identity)
            5'd6: voicing_coef = {32'sd1072752732, -32'sd1072752732, 32'sd0, -32'sd1071763640, 32'sd0}; // headphones HPF
            5'd7: voicing_coef = {32'sd1074324827, -32'sd2144712642, 32'sd1070397925, -32'sd2144712642, 32'sd1070980928}; // headphones peak
            5'd8: voicing_coef = {32'sd52849607, 32'sd14077846, 32'sd0, -32'sd1611338874, 32'sd604524502}; // headphones LPF
            5'd9: voicing_coef = {32'sd1071431917, -32'sd2142863834, 32'sd1071431917, -32'sd2142860253, 32'sd1069125589}; // warm HPF
            5'd10: voicing_coef = {32'sd35719397, 32'sd9534832, 32'sd0, -32'sd1706614695, 32'sd678127100}; // warm LPF
            5'd11: voicing_coef = {32'sd1073741824, 32'sd0, 32'sd0, 32'sd0, 32'sd0}; // warm (identity)
            5'd12: voicing_coef = {32'sd1070910491, -32'sd2141820982, 32'sd1070910491, -32'sd2141813516, 32'sd1068086624}; // tv HPF
            5'd13: voicing_coef = {32'sd22263872, 32'sd5968138, 32'sd0, -32'sd1887003624, 32'sd841493810}; // tv LPF
            5'd14: voicing_coef = {32'sd1073741824, 32'sd0, 32'sd0, 32'sd0, 32'sd0}; // tv (identity)
            5'd15: voicing_coef = {32'sd1068303580, -32'sd2136607160, 32'sd1068303580, -32'sd2136579616, 32'sd1062892878}; // small_speaker HPF
            5'd16: voicing_coef = {32'sd1081625397, -32'sd2107306343, 32'sd1027638348, -32'sd2107306343, 32'sd1035521920}; // small_speaker peak
            5'd17: voicing_coef = {32'sd12911340, 32'sd3460416, 32'sd0, -32'sd1951731704, 32'sd894361636}; // small_speaker LPF
            default: voicing_coef = {32'sd1073741824, 32'sd0, 32'sd0, 32'sd0, 32'sd0};
        endcase
    end
endfunction

// Section state, index = channel * 3 + section
reg signed [45:0] x1 [0:5];
reg signed [45:0] x2 [0:5];
reg signed [45:0] y1 [0:5];
reg signed [45:0] y2 [0:5];

reg  [2:0]  preset_r;
reg  [1:0]  state;
reg         ch;
reg  [1:0]  sec;
reg  [2:0]  term;
reg signed [45:0] v;
reg signed [31:0] in_r_hold;
reg signed [81:0] acc;

localparam S_IDLE = 2'd0;
localparam S_MAC  = 2'd1;
localparam S_FIN  = 2'd2;

wire [2:0]   prof = (preset_r > 3'd5) ? 3'd0 : preset_r;
wire [4:0]   row  = prof * 3 + sec;
wire [159:0] coefs = voicing_coef(row);
wire [2:0]   idx  = {1'b0, sec} + (ch ? 3'd3 : 3'd0);

wire signed [31:0] c_term = (term == 3'd0) ? $signed(coefs[159:128]) :
                            (term == 3'd1) ? $signed(coefs[127:96])  :
                            (term == 3'd2) ? $signed(coefs[95:64])   :
                            (term == 3'd3) ? $signed(coefs[63:32])   :
                                             $signed(coefs[31:0]);
wire signed [45:0] op     = (term == 3'd0) ? v :
                            (term == 3'd1) ? x1[idx] :
                            (term == 3'd2) ? x2[idx] :
                            (term == 3'd3) ? y1[idx] :
                                             y2[idx];
wire signed [77:0] prod = c_term * op;

wire signed [81:0] acc_rnd = acc + 82'sd536870912;
wire signed [51:0] y_full  = acc_rnd[81:30];
wire signed [45:0] y_sat   = (y_full > 52'sh001FFFFFFFFFFF) ? 46'sh1FFFFFFFFFFF :
                             (y_full < -52'sh00200000000000) ? -46'sh200000000000 :
                             y_full[45:0];

wire signed [45:0] y_out_rnd = y_sat + 46'sd2048;
wire signed [33:0] y_out_full = y_out_rnd[45:12];
wire signed [31:0] y_out = (y_out_full > 34'sh07FFFFFFF) ? 32'sh7FFFFFFF :
                           (y_out_full < -34'sh080000000) ? -32'sh80000000 :
                           y_out_full[31:0];

wire signed [45:0] in_l_wide = {{2{in_left[31]}}, in_left, 12'd0};
wire signed [45:0] in_r_wide = {{2{in_r_hold[31]}}, in_r_hold, 12'd0};

integer k;
always @(posedge clk) begin
    if (reset) begin
        state <= S_IDLE;
        preset_r <= 3'd0;
        out_left <= 0;
        out_right <= 0;
        acc <= 0;
        for (k = 0; k < 6; k = k + 1) begin
            x1[k] <= 0; x2[k] <= 0; y1[k] <= 0; y2[k] <= 0;
        end
    end
    else begin
        case (state)
            S_IDLE: begin
                if (ce) begin
                    if (preset != preset_r) begin
                        for (k = 0; k < 6; k = k + 1) begin
                            x1[k] <= 0; x2[k] <= 0; y1[k] <= 0; y2[k] <= 0;
                        end
                    end
                    preset_r <= preset;
                    v <= in_l_wide;
                    in_r_hold <= in_right;
                    ch <= 1'b0;
                    sec <= 2'd0;
                    term <= 3'd0;
                    acc <= 0;
                    state <= S_MAC;
                end
            end

            S_MAC: begin
                // terms 0-2: + b * x, terms 3-4: - a * y
                if (term < 3'd3) acc <= acc + prod;
                else             acc <= acc - prod;
                if (term == 3'd4) state <= S_FIN;
                term <= term + 1'd1;
            end

            S_FIN: begin
                x2[idx] <= x1[idx];
                x1[idx] <= v;
                y2[idx] <= y1[idx];
                y1[idx] <= y_sat;
                acc <= 0;
                term <= 3'd0;
                if (sec == 2'd2) begin
                    if (!ch) begin
                        out_left <= y_out;
                        v <= in_r_wide;
                        ch <= 1'b1;
                        sec <= 2'd0;
                        state <= S_MAC;
                    end
                    else begin
                        out_right <= y_out;
                        state <= S_IDLE;
                    end
                end
                else begin
                    v <= y_sat;
                    sec <= sec + 1'd1;
                    state <= S_MAC;
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
