//
// One-pole DC blocker without multipliers: y = (1 - k) * (y1 + x - x1),
// k = 2^-13 + 2^-16 + 2^-17 (5.05 Hz at 218.75 kHz), Q4.28 in and out
//
// Copyright (c) 2026 Ilia Sharin
//

module hq_dc
(
    input  wire        clk,
    input  wire        reset,
    input  wire        ce,
    input  wire signed [31:0] in_sample,
    output reg  signed [31:0] out_sample
);

reg signed [31:0] x1;
reg signed [49:0] s, y;
reg               st;

always @(posedge clk) begin
    if (reset) begin
        x1 <= 0; s <= 0; y <= 0; st <= 0;
        out_sample <= 0;
    end
    else begin
        st <= 0;
        if (ce) begin
            s  <= y + ($signed({in_sample[31], in_sample} - {x1[31], x1}) <<< 16);
            x1 <= in_sample;
            st <= 1;
        end
        if (st) begin
            y <= s - (s >>> 13) - (s >>> 16) - (s >>> 17);
            out_sample <= (s - (s >>> 13) - (s >>> 16) - (s >>> 17) + 50'sd32768) >>> 16;
        end
    end
end

endmodule
