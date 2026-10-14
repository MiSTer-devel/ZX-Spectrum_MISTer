//
// HQ mix bus: AY chain + FM, General Sound, Covox, SAA1099, beeper/tape
// at the reference balance, master DC blocker, compressor and limiter (README.md)
//
// Copyright (c) 2026 Ilia Sharin
//

module hq_mix
(
    input  wire        clk,
    input  wire        reset,
    input  wire        ce,          // 3.5 MHz, one T-state, >= 16 clocks apart
    input  wire        ce_gen,      // 218.75 kHz, coincides with ce

    input  wire        ay_valid,
    input  wire signed [31:0] ay_l, // Q4.28, 1.0 = full scale
    input  wire signed [31:0] ay_r,

    input  wire signed [15:0] fm_0,
    input  wire signed [15:0] fm_1,
    input  wire signed [14:0] gs_l,
    input  wire signed [14:0] gs_r,
    input  wire        [7:0]  covox_l,  // 0x80 = silence
    input  wire        [7:0]  covox_r,
    input  wire        [7:0]  saa_l,
    input  wire        [7:0]  saa_r,
    input  wire        ear,
    input  wire        mic,
    input  wire        tape,

    output reg  signed [15:0] out_l,
    output reg  signed [15:0] out_r
);

// ---------------------------------------------------------------------------
// Step sources, Q4.28 per T-state, through one shared multiplier
// ---------------------------------------------------------------------------
localparam signed [20:0] K_FM  = 21'sd5759;     // 0.703 * 2^13
localparam signed [20:0] K_GS  = 21'sd7875;     // 1.9226 / 2 * 2^13
localparam signed [20:0] K_SAA = 21'sd631613;   // 0.6 / 255 * 2^28

function signed [31:0] beeper;
    input e, m, t;
    begin
        case ({e, m})
            2'b00: beeper = 32'sd0;
            2'b01: beeper = 32'sd1600  * 8192;
            2'b10: beeper = 32'sd15400 * 8192;
            2'b11: beeper = 32'sd16000 * 8192;
        endcase
        if (t) beeper = beeper + 32'sd3850 * 8192;
    end
endfunction

reg signed [16:0] o_fm, o_gsl, o_gsr;
reg signed [8:0]  o_sal, o_sar;
reg signed [31:0] t_fm, t_gsl, t_gsr, t_cvl, t_cvr, t_sal, t_sar, t_bp;
reg signed [17:0] m_a;
reg signed [20:0] m_b;
reg signed [38:0] m_p;
reg [3:0] seq;
reg       gen_pend;

always @(posedge clk) begin
    if (reset) seq <= 0;
    else if (ce) begin
        seq <= 1;
        o_fm  <= $signed({fm_0[15], fm_0}) + $signed({fm_1[15], fm_1});
        o_gsl <= $signed({{2{gs_l[14]}}, gs_l}) + (gs_r >>> 1);
        o_gsr <= $signed({{2{gs_r[14]}}, gs_r}) + (gs_l >>> 1);
        o_sal <= $signed({1'b0, saa_l});
        o_sar <= $signed({1'b0, saa_r});
        t_cvl <= $signed({~covox_l[7], covox_l[6:0]}) <<< 20;
        t_cvr <= $signed({~covox_r[7], covox_r[6:0]}) <<< 20;
        t_bp  <= beeper(ear, mic, tape);
        gen_pend <= ce_gen;
    end
    else if (seq != 0) seq <= (seq == 4'd8) ? 4'd0 : seq + 1'd1;

    // operands at seq 1..5, product 1 clock later, result 2 clocks later
    case (seq)
        4'd1: begin m_a <= o_fm;  m_b <= K_FM;  end
        4'd2: begin m_a <= o_gsl; m_b <= K_GS;  end
        4'd3: begin m_a <= o_gsr; m_b <= K_GS;  end
        4'd4: begin m_a <= o_sal; m_b <= K_SAA; end
        4'd5: begin m_a <= o_sar; m_b <= K_SAA; end
        default: ;
    endcase
    m_p <= m_a * m_b;
    case (seq)
        4'd3: t_fm  <= m_p[31:0];
        4'd4: t_gsl <= m_p[31:0];
        4'd5: t_gsr <= m_p[31:0];
        4'd6: t_sal <= m_p[31:0];
        4'd7: t_sar <= m_p[31:0];
        default: ;
    endcase
end

wire signed [33:0] inst_l = t_fm + t_gsl + t_cvl + t_sal + t_bp;
wire signed [33:0] inst_r = t_fm + t_gsr + t_cvr + t_sar + t_bp;

// integrate-and-dump over the 16 T-states of a generator sample
reg signed [37:0] acc_l, acc_r;
reg signed [31:0] src_l, src_r;
reg src_valid;

function signed [31:0] sat32;
    input signed [37:0] v;
    sat32 = (v > 38'sh007FFFFFFF) ? 32'sh7FFFFFFF : (v < -38'sh0080000000) ? 32'sh80000000 : v[31:0];
endfunction

always @(posedge clk) begin
    src_valid <= 0;
    if (reset) begin
        acc_l <= 0; acc_r <= 0;
        src_l <= 0; src_r <= 0;
    end
    else if (seq == 4'd8) begin
        if (gen_pend) begin
            src_l <= sat32((acc_l + inst_l) >>> 4);
            src_r <= sat32((acc_r + inst_r) >>> 4);
            src_valid <= 1;
            acc_l <= 0; acc_r <= 0;
        end
        else begin
            acc_l <= acc_l + inst_l;
            acc_r <= acc_r + inst_r;
        end
    end
end

wire fir_valid;
wire signed [31:0] fir_l, fir_r;

hq_fir_stereo fir_src (
    .clk      (clk),
    .reset    (reset),
    .ce_in    (src_valid),
    .in_l     (src_l),
    .in_r     (src_r),
    .out_valid(fir_valid),
    .out_l    (fir_l),
    .out_r    (fir_r)
);

// ---------------------------------------------------------------------------
// Sum, latched once per sample: both inputs settle within the 256 clocks
// ---------------------------------------------------------------------------
reg signed [31:0] bus_l, bus_r, ayl, ayr;
reg signed [31:0] sum_l, sum_r;
reg sum_ce;

always @(posedge clk) begin
    sum_ce <= 0;
    if (reset) begin
        bus_l <= 0; bus_r <= 0; ayl <= 0; ayr <= 0;
        sum_l <= 0; sum_r <= 0;
    end
    else begin
        if (fir_valid) begin
            bus_l <= fir_l;
            bus_r <= fir_r;
        end
        if (ay_valid) begin
            ayl <= ay_l;
            ayr <= ay_r;
        end
        if (ce_gen) begin
            sum_l <= sat32(ayl + bus_l);
            sum_r <= sat32(ayr + bus_r);
            sum_ce <= 1;
        end
    end
end

// ---------------------------------------------------------------------------
// Master: DC blocker 5 Hz, makeup gain, look-ahead peak compressor, then the
// soft limiter as a safety
// ---------------------------------------------------------------------------
wire signed [31:0] dc_l, dc_r;

hq_dc dc_l_i (.clk(clk), .reset(reset), .ce(sum_ce), .in_sample(sum_l), .out_sample(dc_l));
hq_dc dc_r_i (.clk(clk), .reset(reset), .ce(sum_ce), .in_sample(sum_r), .out_sample(dc_r));

// x1.664 (+4.42 dB): the legacy mix level (compr() doubles small signals)
function signed [31:0] makeup;
    input signed [31:0] x;
    makeup = sat32(x + (x >>> 1) + (x >>> 3) + (x >>> 5) + (x >>> 7));
endfunction

// stereo-linked: gain T / peak above T = 0.7071 (-3 dBFS), held for the
// 256-sample (1.17 ms) look-ahead so the attack (1/32 per sample, 0.15 ms)
// completes before the peak leaves the delay; release 1/32768 per sample
// (150 ms); gain Q1.31
localparam [31:0] THRESH = 32'd189810711;      // 0.7071
localparam [31:0] UNITY  = 32'h80000000;

reg  [2:0] dc_sr;
reg  [7:0] cs;
reg signed [31:0] ml, mr, dq_l, dq_r, c_l, c_r;
reg signed [31:0] dly_l [0:255];
reg signed [31:0] dly_r [0:255];
reg  [7:0] wp;
reg  [9:0] c_addr;
reg  [1:0] c_sh;               // 0: << 1, 1: as is, 2: >> 1, 3: >> 2
reg        c_above;
reg [31:0] target, held, gain;
reg  [8:0] hold;
reg signed [51:0] pl, pr;
reg        c_valid;
wire [31:0] comp_q;

hq_comp_rom comp_rom (.clk(clk), .addr(c_addr), .q(comp_q));

wire [31:0] al = ml[31] ? -ml : ml;
wire [31:0] ar = mr[31] ? -mr : mr;
wire [31:0] pk0 = (al > ar) ? al : ar;
wire [31:0] pk  = pk0[31] ? 32'h7FFFFFFF : pk0;

always @(posedge clk) begin
    dc_sr <= reset ? 3'd0 : {dc_sr[1:0], sum_ce};
    cs    <= reset ? 8'd0 : {cs[6:0], dc_sr[2]};
    c_valid <= 0;
    if (reset) begin
        wp <= 0;
        gain <= UNITY;
        target <= UNITY;
        held <= UNITY;
        hold <= 0;
        c_l <= 0; c_r <= 0;
    end
    else begin
        if (dc_sr[2]) begin
            ml <= makeup(dc_l);
            mr <= makeup(dc_r);
        end
        if (cs[0]) begin
            dq_l <= dly_l[wp];
            dq_r <= dly_r[wp];
            c_above <= pk > THRESH;
            casez (pk[30:27])
                4'b1???: begin c_sh <= 3; c_addr <= pk[29:20]; end
                4'b01??: begin c_sh <= 2; c_addr <= pk[28:19]; end
                4'b001?: begin c_sh <= 1; c_addr <= pk[27:18]; end
                default: begin c_sh <= 0; c_addr <= pk[26:17]; end
            endcase
        end
        if (cs[1]) begin
            dly_l[wp] <= ml;
            dly_r[wp] <= mr;
            wp <= wp + 1'd1;
        end
        if (cs[2])
            target <= ~c_above ? UNITY :
                      (c_sh == 0) ? comp_q << 1 : (c_sh == 1) ? comp_q :
                      (c_sh == 2) ? comp_q >> 1 : comp_q >> 2;
        if (cs[3]) begin
            if (target <= held) begin
                held <= target;
                hold <= 9'd256;
            end
            else if (hold != 0) hold <= hold - 1'd1;
            else held <= target;
        end
        if (cs[4])
            gain <= (held < gain) ? gain - ((gain - held) >> 5) : gain + ((held - gain) >> 15);
        if (cs[5]) begin
            pl <= $signed(dq_l[31:5]) * $signed({1'b0, gain[31:8]});
            pr <= $signed(dq_r[31:5]) * $signed({1'b0, gain[31:8]});
        end
        if (cs[6]) begin
            c_l <= pl >>> 18;
            c_r <= pr >>> 18;
            c_valid <= 1;
        end
    end
end

// limiter, L then R on one ROM and one multiplier
localparam [31:0] KNEE = 32'd201326592;        // 0.75
localparam [27:0] TOP  = 28'd60817408;         // f(2.0), the curve end

reg  [3:0] ls;                 // 0 idle, 1..12 steps
reg        lch;
reg signed [31:0] lx;
reg        l_sg, l_lin, l_top;
reg [31:0] l_mag;
reg [18:0] l_fr;
reg [10:0] rom_addr;
wire [27:0] rom_q;
reg [27:0] q0, q1;
reg signed [39:0] l_step;
reg signed [31:0] y_l, y_r;
reg        y_valid;

hq_limiter_rom limiter_rom (.clk(clk), .addr(rom_addr), .q(rom_q));

wire [31:0] lx_abs = lx[31] ? -lx : lx;
wire [31:0] lx_d   = lx_abs - KNEE;
wire [31:0] l_m    = l_lin ? l_mag : KNEE + (l_top ? {4'd0, TOP} : {4'd0, q0} + l_step[31:0]);
wire signed [31:0] l_y = l_sg ? -$signed(l_m) : $signed(l_m);

always @(posedge clk) begin
    y_valid <= 0;
    if (reset) begin
        ls <= 0;
        y_l <= 0; y_r <= 0;
    end
    else begin
        case (ls)
            4'd0: if (c_valid) begin lx <= c_l; lch <= 0; ls <= 1; end
            4'd1, 4'd7: begin
                l_sg  <= lx[31];
                l_mag <= lx_abs;
                l_lin <= lx_abs <= KNEE;
                l_top <= |lx_d[31:29];
                l_fr  <= lx_d[18:0];
                rom_addr <= {1'b0, lx_d[28:19]};
                ls <= ls + 1'd1;
            end
            4'd2, 4'd8: begin rom_addr <= rom_addr + 1'd1; ls <= ls + 1'd1; end
            4'd3, 4'd9: begin q0 <= rom_q; ls <= ls + 1'd1; end
            4'd4, 4'd10: begin q1 <= rom_q; ls <= ls + 1'd1; end
            4'd5, 4'd11: begin
                l_step <= ($signed({1'b0, q1}) - $signed({1'b0, q0})) * $signed({1'b0, l_fr}) >>> 19;
                ls <= ls + 1'd1;
            end
            4'd6: begin y_l <= l_y; lx <= c_r; ls <= 7; end
            4'd12: begin y_r <= l_y; y_valid <= 1; ls <= 0; end
            default: ls <= 0;
        endcase
    end
end

// ---------------------------------------------------------------------------
// First-order hold to the 3.5 MHz CE rate, int16 out (1.0 = 32768)
// ---------------------------------------------------------------------------
reg signed [31:0] foh_l, foh_r, foh_dl, foh_dr;

function signed [15:0] sat16;
    input signed [31:0] v;
    reg   signed [31:0] r;
    begin
        r = v >>> 13;
        sat16 = (r > 32'sd32767) ? 16'sh7FFF : (r < -32'sd32768) ? 16'sh8000 : r[15:0];
    end
endfunction

always @(posedge clk) begin
    if (reset) begin
        foh_l <= 0; foh_r <= 0;
        foh_dl <= 0; foh_dr <= 0;
        out_l <= 0; out_r <= 0;
    end
    else begin
        if (y_valid) begin
            foh_dl <= (y_l - foh_l) >>> 4;
            foh_dr <= (y_r - foh_r) >>> 4;
        end
        if (ce) begin
            foh_l <= foh_l + foh_dl;
            foh_r <= foh_r + foh_dr;
            out_l <= sat16(foh_l + foh_dl);
            out_r <= sat16(foh_r + foh_dr);
        end
    end
end

endmodule
