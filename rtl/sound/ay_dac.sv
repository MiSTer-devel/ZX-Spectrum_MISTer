//
// AY-3-8910 / YM2149 DAC - exact amplitude tables
//
// Sum of the AY / YM DAC table values (Q1.31) of both TurboSound chips.
// One ay_dac_lut instance per chip: one table read at two addresses was
// mapped by Quartus to a RAM with the second address tied to 0.
//
// Copyright (c) 2026 Ilia Sharin
//

module ay_dac_lut
(
    input  wire        clk,
    input  wire        mode,      // 0 = YM2149, 1 = AY-3-8910
    input  wire [4:0]  level,
    output reg  [31:0] value      // unsigned Q1.31 [0.0, 1.0]
);

reg [4:0] level_r;
reg       mode_r;

always @(posedge clk) begin
    level_r <= level;
    mode_r  <= mode;

    if (mode_r) begin
        // AY-3-8910
        case (level_r)
            5'd0,  5'd1:  value <= 32'h00000000;
            5'd2,  5'd3:  value <= 32'h01478148;
            5'd4,  5'd5:  value <= 32'h01D981DA;
            5'd6,  5'd7:  value <= 32'h02B202B2;
            5'd8,  5'd9:  value <= 32'h03EE03EE;
            5'd10, 5'd11: value <= 32'h05D485D5;
            5'd12, 5'd13: value <= 32'h08418842;
            5'd14, 5'd15: value <= 32'h0DBE0DBE;
            5'd16, 5'd17: value <= 32'h10341034;
            5'd18, 5'd19: value <= 32'h1A3D1A3D;
            5'd20, 5'd21: value <= 32'h25672567;
            5'd22, 5'd23: value <= 32'h2FB92FB9;
            5'd24, 5'd25: value <= 32'h3F0B3F0B;
            5'd26, 5'd27: value <= 32'h51525152;
            5'd28, 5'd29: value <= 32'h671D671D;
            default:      value <= 32'h7FFFFFFF;
        endcase
    end
    else begin
        // YM2149
        case (level_r)
            5'd0,  5'd1:  value <= 32'h00000000;
            5'd2:  value <= 32'h00988099;
            5'd3:  value <= 32'h00FD00FD;
            5'd4:  value <= 32'h01670167;
            5'd5:  value <= 32'h01C981CA;
            5'd6:  value <= 32'h022D022D;
            5'd7:  value <= 32'h02900290;
            5'd8:  value <= 32'h031E831F;
            5'd9:  value <= 32'h03CD03CD;
            5'd10: value <= 32'h047D047D;
            5'd11: value <= 32'h052B852C;
            5'd12: value <= 32'h06368637;
            5'd13: value <= 32'h07778778;
            5'd14: value <= 32'h08B608B6;
            5'd15: value <= 32'h09F489F5;
            5'd16: value <= 32'h0BD78BD8;
            5'd17: value <= 32'h0E380E38;
            5'd18: value <= 32'h109B909C;
            5'd19: value <= 32'h13019302;
            5'd20: value <= 32'h169D169D;
            5'd21: value <= 32'h1B141B14;
            5'd22: value <= 32'h1F899F8A;
            5'd23: value <= 32'h23FB23FB;
            5'd24: value <= 32'h2AB7AAB8;
            5'd25: value <= 32'h33413341;
            5'd26: value <= 32'h3BD33BD3;
            5'd27: value <= 32'h44684468;
            5'd28: value <= 32'h514D514D;
            5'd29: value <= 32'h61066106;
            5'd30: value <= 32'h70A170A1;
            default: value <= 32'h7FFFFFFF;
        endcase
    end
end

endmodule

module ay_dac
(
    input  wire        clk,
    input  wire        mode,
    input  wire [4:0]  level_0,
    input  wire [4:0]  level_1,
    output reg  [31:0] dac_out
);

wire [31:0] val_0, val_1;

(* keep_hierarchy = "yes" *) ay_dac_lut lut_0 (.clk(clk), .mode(mode), .level(level_0), .value(val_0));
(* keep_hierarchy = "yes" *) ay_dac_lut lut_1 (.clk(clk), .mode(mode), .level(level_1), .value(val_1));

always @(posedge clk) begin
    dac_out <= val_0 + val_1;
end

endmodule
