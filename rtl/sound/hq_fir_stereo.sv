//
// Stereo 96-tap FIR on one 27x27 MAC: the ay_fir_decimator design
// (Kaiser beta 5, 20 kHz at 218.75 kHz) with history and coefficients in
// block RAM. L then R, 192 MAC cycles per sample (budget 256 clocks)
//
// Copyright (c) 2026 Ilia Sharin
//

module hq_fir_stereo
(
    input  wire        clk,
    input  wire        reset,
    input  wire        ce_in,

    input  wire signed [31:0] in_l,    // Q4.28
    input  wire signed [31:0] in_r,

    output reg         out_valid,
    output reg  signed [31:0] out_l,
    output reg  signed [31:0] out_r
);

// coefficients Q1.26 (ay_fir_decimator's Q1.31, rounded)
reg signed [26:0] coeff [0:95];
initial begin
    coeff[ 0] = 27'sd13775;
    coeff[ 1] = 27'sd21544;
    coeff[ 2] = 27'sd23067;
    coeff[ 3] = 27'sd14154;
    coeff[ 4] = -27'sd5914;
    coeff[ 5] = -27'sd32687;
    coeff[ 6] = -27'sd56765;
    coeff[ 7] = -27'sd66433;
    coeff[ 8] = -27'sd52205;
    coeff[ 9] = -27'sd11758;
    coeff[10] = 27'sd46824;
    coeff[11] = 27'sd105376;
    coeff[12] = 27'sd140362;
    coeff[13] = 27'sd131107;
    coeff[14] = 27'sd68987;
    coeff[15] = -27'sd35965;
    coeff[16] = -27'sd154360;
    coeff[17] = -27'sd244303;
    coeff[18] = -27'sd264765;
    coeff[19] = -27'sd191654;
    coeff[20] = -27'sd30867;
    coeff[21] = 27'sd177348;
    coeff[22] = 27'sd366192;
    coeff[23] = 27'sd461772;
    coeff[24] = 27'sd409161;
    coeff[25] = 27'sd196621;
    coeff[26] = -27'sd131376;
    coeff[27] = -27'sd479208;
    coeff[28] = -27'sd724771;
    coeff[29] = -27'sd758403;
    coeff[30] = -27'sd524685;
    coeff[31] = -27'sd52816;
    coeff[32] = 27'sd537156;
    coeff[33] = 27'sd1058602;
    coeff[34] = 27'sd1310043;
    coeff[35] = 27'sd1141576;
    coeff[36] = 27'sd516442;
    coeff[37] = -27'sd452976;
    coeff[38] = -27'sd1510043;
    coeff[39] = -27'sd2303951;
    coeff[40] = -27'sd2475755;
    coeff[41] = -27'sd1759083;
    coeff[42] = -27'sd67640;
    coeff[43] = 27'sd2456994;
    coeff[44] = 27'sd5454363;
    coeff[45] = 27'sd8412831;
    coeff[46] = 27'sd10781311;
    coeff[47] = 27'sd12097210;
    coeff[48] = 27'sd12097210;
    coeff[49] = 27'sd10781311;
    coeff[50] = 27'sd8412831;
    coeff[51] = 27'sd5454363;
    coeff[52] = 27'sd2456994;
    coeff[53] = -27'sd67640;
    coeff[54] = -27'sd1759083;
    coeff[55] = -27'sd2475755;
    coeff[56] = -27'sd2303951;
    coeff[57] = -27'sd1510043;
    coeff[58] = -27'sd452976;
    coeff[59] = 27'sd516442;
    coeff[60] = 27'sd1141576;
    coeff[61] = 27'sd1310043;
    coeff[62] = 27'sd1058602;
    coeff[63] = 27'sd537156;
    coeff[64] = -27'sd52816;
    coeff[65] = -27'sd524685;
    coeff[66] = -27'sd758403;
    coeff[67] = -27'sd724771;
    coeff[68] = -27'sd479208;
    coeff[69] = -27'sd131376;
    coeff[70] = 27'sd196621;
    coeff[71] = 27'sd409161;
    coeff[72] = 27'sd461772;
    coeff[73] = 27'sd366192;
    coeff[74] = 27'sd177348;
    coeff[75] = -27'sd30867;
    coeff[76] = -27'sd191654;
    coeff[77] = -27'sd264765;
    coeff[78] = -27'sd244303;
    coeff[79] = -27'sd154360;
    coeff[80] = -27'sd35965;
    coeff[81] = 27'sd68987;
    coeff[82] = 27'sd131107;
    coeff[83] = 27'sd140362;
    coeff[84] = 27'sd105376;
    coeff[85] = 27'sd46824;
    coeff[86] = -27'sd11758;
    coeff[87] = -27'sd52205;
    coeff[88] = -27'sd66433;
    coeff[89] = -27'sd56765;
    coeff[90] = -27'sd32687;
    coeff[91] = -27'sd5914;
    coeff[92] = 27'sd14154;
    coeff[93] = 27'sd23067;
    coeff[94] = 27'sd21544;
    coeff[95] = 27'sd13775;
end

// history: {channel, 7-bit circular index}, Q4.23
reg [26:0] hist [0:255];
reg  [7:0] waddr, raddr;
reg [26:0] wdata, rq;
reg        we;
reg signed [26:0] cq;
reg  [6:0] caddr;

always @(posedge clk) begin
    if (we) hist[waddr] <= wdata;
    rq <= hist[raddr];
    cq <= coeff[caddr];
end

localparam S_CLR = 3'd0, S_IDLE = 3'd1, S_WL = 3'd2, S_WR = 3'd3, S_MAC = 3'd4, S_END = 3'd5;

reg  [2:0] state;
reg  [6:0] wp, k;
reg        ch;
reg [26:0] xr;
reg  [3:0] p_valid, p_ch, p_last;
reg signed [53:0] prod;
reg signed [60:0] acc_l, acc_r;

always @(posedge clk) begin
    we <= 0;
    out_valid <= 0;
    p_valid <= {p_valid[2:0], 1'b0};
    p_ch    <= {p_ch[2:0], 1'b0};
    p_last  <= {p_last[2:0], 1'b0};

    if (reset) begin
        state <= S_CLR;
        waddr <= 0;
        wp <= 0;
        p_valid <= 0;
        acc_l <= 0; acc_r <= 0;
        out_l <= 0; out_r <= 0;
    end
    else begin
        case (state)
            S_CLR: begin
                we <= 1; wdata <= 0;
                waddr <= waddr + 1'd1;
                if (&waddr) state <= S_IDLE;
            end

            S_IDLE: if (ce_in) begin
                we <= 1; waddr <= {1'b0, wp}; wdata <= in_l[31:5];
                xr <= in_r[31:5];
                state <= S_WL;
            end

            S_WL: begin
                we <= 1; waddr <= {1'b1, wp}; wdata <= xr;
                k <= 0; ch <= 0;
                acc_l <= 0; acc_r <= 0;
                state <= S_WR;
            end

            S_WR, S_MAC: begin
                raddr <= {ch, wp - k};
                caddr <= k;
                p_valid[0] <= 1;
                p_ch[0] <= ch;
                p_last[0] <= ch && (k == 7'd95);
                state <= S_MAC;
                if (k == 7'd95) begin
                    k <= 0;
                    if (ch) state <= S_END;
                    ch <= 1;
                end
                else k <= k + 1'd1;
            end

            S_END: if (p_last[3]) begin
                out_l <= (acc_l + 61'sd1048576) >>> 21;
                out_r <= (acc_r + 61'sd1048576) >>> 21;
                out_valid <= 1;
                wp <= wp + 1'd1;
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase

        // pipeline: [0] address, [1] RAM/ROM out, [2] product, [3] accumulate
        if (p_valid[1]) prod <= $signed(rq) * cq;
        if (p_valid[2]) begin
            if (p_ch[2]) acc_r <= acc_r + prod;
            else         acc_l <= acc_l + prod;
        end
    end
end

endmodule
