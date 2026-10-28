module savestate_clock #(parameter CLK_HZ = 112_000_000)
(
	input             clk,
	input      [64:0] RTC,
	input      [32:0] TIMESTAMP,
	output reg [63:0] rtc = 0,
	output reg [31:0] timestamp = 0
);

function [7:0] bcd_inc(input [7:0] v);
	bcd_inc = (v[3:0] == 9) ? {v[7:4] + 4'd1, 4'd0} : v + 8'd1;
endfunction

wire       leap   = rtc[44] ? (rtc[43:40] == 2 || rtc[43:40] == 6)
                            : (rtc[43:40] == 0 || rtc[43:40] == 4 || rtc[43:40] == 8);
wire [7:0] month  = rtc[39:32];
wire [7:0] last   = (month == 8'h02) ? (leap ? 8'h29 : 8'h28) :
                    (month == 8'h04 || month == 8'h06 || month == 8'h09 || month == 8'h11) ? 8'h30 : 8'h31;

always @(posedge clk) begin
	reg old_rtc, old_ts;
	reg [31:0] div;

	old_rtc <= RTC[64];
	old_ts  <= TIMESTAMP[32];

	div <= div + 1'd1;
	if(div == CLK_HZ - 1) begin
		div <= 0;
		timestamp <= timestamp + 1'd1;
		rtc[7:0] <= bcd_inc(rtc[7:0]);
		if(rtc[7:0] == 8'h59) begin
			rtc[7:0]  <= 0;
			rtc[15:8] <= bcd_inc(rtc[15:8]);
			if(rtc[15:8] == 8'h59) begin
				rtc[15:8]  <= 0;
				rtc[23:16] <= bcd_inc(rtc[23:16]);
				if(rtc[23:16] == 8'h23) begin
					rtc[23:16] <= 0;
					rtc[55:48] <= (rtc[55:48] == 6) ? 8'd0 : rtc[55:48] + 1'd1;
					rtc[31:24] <= bcd_inc(rtc[31:24]);
					if(rtc[31:24] == last) begin
						rtc[31:24] <= 8'h01;
						rtc[39:32] <= bcd_inc(rtc[39:32]);
						if(rtc[39:32] == 8'h12) begin
							rtc[39:32] <= 8'h01;
							rtc[47:40] <= bcd_inc(rtc[47:40]);
							if(rtc[47:40] == 8'h99) rtc[47:40] <= 0;
						end
					end
				end
			end
		end
	end

	if(old_rtc != RTC[64]) begin
		rtc <= RTC[63:0];
		div <= 0;
	end
	if(old_ts != TIMESTAMP[32]) timestamp <= TIMESTAMP[31:0];
end

endmodule
