//
// AY-3-8910 20 kHz FIR lowpass (full-rate)
//
// 96-tap Kaiser (beta 5) 20 kHz low-pass at 218.75 kHz, one output per input
//
// Copyright (c) 2026 Ilia Sharin
//

module ay_fir_decimator
(
    input  wire        clk,
    input  wire        ce_in,
    input  wire        reset,

    input  wire signed [31:0] in_sample,   // Q4.28 input from mixer

    output reg         out_valid,
    output reg  signed [31:0] out_sample   // Q4.28 output at 218.75 kHz
);

localparam FIR_TAPS = 96;

(* ram_style = "block" *) reg signed [31:0] buffer [0:FIR_TAPS-1];
reg [6:0] buffer_idx;

// coefficients Q1.31, sum 1.0
(* ram_style = "block" *) reg signed [31:0] coeff_rom [0:95];

initial begin
    coeff_rom[ 0] = 32'h0006B9D9;
    coeff_rom[ 1] = 32'h000A84F4;
    coeff_rom[ 2] = 32'h000B435A;
    coeff_rom[ 3] = 32'h0006E94F;
    coeff_rom[ 4] = 32'hFFFD1CB6;
    coeff_rom[ 5] = 32'hFFF00A11;
    coeff_rom[ 6] = 32'hFFE44866;
    coeff_rom[ 7] = 32'hFFDF8FE0;
    coeff_rom[ 8] = 32'hFFE68253;
    coeff_rom[ 9] = 32'hFFFA4235;
    coeff_rom[10] = 32'h0016DCF8;
    coeff_rom[11] = 32'h003373F6;
    coeff_rom[12] = 32'h0044893E;
    coeff_rom[13] = 32'h00400451;
    coeff_rom[14] = 32'h0021AF59;
    coeff_rom[15] = 32'hFFEE7061;
    coeff_rom[16] = 32'hFFB4A0FC;
    coeff_rom[17] = 32'hFF88B62E;
    coeff_rom[18] = 32'hFF7EB865;
    coeff_rom[19] = 32'hFFA26B49;
    coeff_rom[20] = 32'hFFF0EDA6;
    coeff_rom[21] = 32'h00569870;
    coeff_rom[22] = 32'h00B2CE01;
    coeff_rom[23] = 32'h00E17976;
    coeff_rom[24] = 32'h00C7C91B;
    coeff_rom[25] = 32'h00600192;
    coeff_rom[26] = 32'hFFBFDA0A;
    coeff_rom[27] = 32'hFF16030D;
    coeff_rom[28] = 32'hFE9E1B99;
    coeff_rom[29] = 32'hFE8DAFAF;
    coeff_rom[30] = 32'hFEFFCE5A;
    coeff_rom[31] = 32'hFFE635FB;
    coeff_rom[32] = 32'h01064886;
    coeff_rom[33] = 32'h0204E546;
    coeff_rom[34] = 32'h027FAB6D;
    coeff_rom[35] = 32'h022D68F9;
    coeff_rom[36] = 32'h00FC2B39;
    coeff_rom[37] = 32'hFF22D1F6;
    coeff_rom[38] = 32'hFD1EAC9F;
    coeff_rom[39] = 32'hFB9B0625;
    coeff_rom[40] = 32'hFB472293;
    coeff_rom[41] = 32'hFCA512A1;
    coeff_rom[42] = 32'hFFDEF908;
    coeff_rom[43] = 32'h04AFB44D;
    coeff_rom[44] = 32'h0A674352;
    coeff_rom[45] = 32'h100BD3DC;
    coeff_rom[46] = 32'h14904FE9;
    coeff_rom[47] = 32'h1712D732;
    coeff_rom[48] = 32'h1712D732;
    coeff_rom[49] = 32'h14904FE9;
    coeff_rom[50] = 32'h100BD3DC;
    coeff_rom[51] = 32'h0A674352;
    coeff_rom[52] = 32'h04AFB44D;
    coeff_rom[53] = 32'hFFDEF908;
    coeff_rom[54] = 32'hFCA512A1;
    coeff_rom[55] = 32'hFB472293;
    coeff_rom[56] = 32'hFB9B0625;
    coeff_rom[57] = 32'hFD1EAC9F;
    coeff_rom[58] = 32'hFF22D1F6;
    coeff_rom[59] = 32'h00FC2B39;
    coeff_rom[60] = 32'h022D68F9;
    coeff_rom[61] = 32'h027FAB6D;
    coeff_rom[62] = 32'h0204E546;
    coeff_rom[63] = 32'h01064886;
    coeff_rom[64] = 32'hFFE635FB;
    coeff_rom[65] = 32'hFEFFCE5A;
    coeff_rom[66] = 32'hFE8DAFAF;
    coeff_rom[67] = 32'hFE9E1B99;
    coeff_rom[68] = 32'hFF16030D;
    coeff_rom[69] = 32'hFFBFDA0A;
    coeff_rom[70] = 32'h00600192;
    coeff_rom[71] = 32'h00C7C91B;
    coeff_rom[72] = 32'h00E17976;
    coeff_rom[73] = 32'h00B2CE01;
    coeff_rom[74] = 32'h00569870;
    coeff_rom[75] = 32'hFFF0EDA6;
    coeff_rom[76] = 32'hFFA26B49;
    coeff_rom[77] = 32'hFF7EB865;
    coeff_rom[78] = 32'hFF88B62E;
    coeff_rom[79] = 32'hFFB4A0FC;
    coeff_rom[80] = 32'hFFEE7061;
    coeff_rom[81] = 32'h0021AF59;
    coeff_rom[82] = 32'h00400451;
    coeff_rom[83] = 32'h0044893E;
    coeff_rom[84] = 32'h003373F6;
    coeff_rom[85] = 32'h0016DCF8;
    coeff_rom[86] = 32'hFFFA4235;
    coeff_rom[87] = 32'hFFE68253;
    coeff_rom[88] = 32'hFFDF8FE0;
    coeff_rom[89] = 32'hFFE44866;
    coeff_rom[90] = 32'hFFF00A11;
    coeff_rom[91] = 32'hFFFD1CB6;
    coeff_rom[92] = 32'h0006E94F;
    coeff_rom[93] = 32'h000B435A;
    coeff_rom[94] = 32'h000A84F4;
    coeff_rom[95] = 32'h0006B9D9;
end

reg [6:0] tap_idx;
reg signed [63:0] accumulator;
reg [1:0] state;

localparam S_IDLE    = 2'd0;
localparam S_COMPUTE = 2'd1;
localparam S_OUTPUT  = 2'd2;

reg signed [31:0] coeff_reg;
reg signed [31:0] sample_reg;
reg signed [63:0] mac_result;

reg [6:0] sample_idx_reg;

integer i;

always @(posedge clk) begin
    if (reset) begin
        buffer_idx <= 0;
        tap_idx <= 0;
        accumulator <= 0;
        state <= S_IDLE;
        out_valid <= 0;
        out_sample <= 0;
        coeff_reg <= 0;
        sample_reg <= 0;
        mac_result <= 0;
        sample_idx_reg <= 0;
        for (i = 0; i < FIR_TAPS; i = i + 1) begin
            buffer[i] <= 0;
        end
    end
    else begin
        out_valid <= 0;

        case (state)
            S_IDLE: begin
                if (ce_in) begin
                    buffer[buffer_idx] <= in_sample;
                    buffer_idx <= (buffer_idx == FIR_TAPS-1) ? 7'd0 : buffer_idx + 1'd1;

                    state <= S_COMPUTE;
                    tap_idx <= 0;
                    accumulator <= 0;
                    sample_idx_reg <= buffer_idx;
                end
            end

            S_COMPUTE: begin
                coeff_reg <= coeff_rom[tap_idx];
                sample_reg <= buffer[sample_idx_reg];

                if (sample_idx_reg == 0)
                    sample_idx_reg <= FIR_TAPS - 1;
                else
                    sample_idx_reg <= sample_idx_reg - 1'd1;

                mac_result <= $signed(sample_reg) * $signed(coeff_reg);

                if (tap_idx >= 2) begin
                    accumulator <= accumulator + mac_result;
                end

                tap_idx <= tap_idx + 1'd1;

                if (tap_idx == FIR_TAPS + 1) begin
                    state <= S_OUTPUT;
                    accumulator <= accumulator + mac_result;
                end
            end

            S_OUTPUT: begin
                state <= S_IDLE;
                out_valid <= 1;
                out_sample <= accumulator[62:31];
            end
        endcase
    end
end

endmodule
