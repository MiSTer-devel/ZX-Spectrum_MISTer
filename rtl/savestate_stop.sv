module savestate_stop
(
	input        clk,
	input        m1,
	input  [7:0] cpu_din,
	input        ce_cpu_p,
	input        cpu_halted,
	input        freeze_req,
	input        stopped,
	input        no_stop,
	input        halt_only,
	output       stop_req,
	output reg   last_ei = 0
);

localparam [1:0] PF_NONE = 0, PF_CB = 1, PF_ED = 2, PF_XY = 3;

reg [1:0] pfx = PF_NONE;
reg [7:0] opc;

always @(posedge clk) begin
	reg old_m1;

	old_m1 <= m1;
	if(m1 & ce_cpu_p) opc <= cpu_din;
	if(old_m1 & ~m1) begin
		last_ei <= (pfx == PF_NONE) && (opc == 8'hFB);
		case(pfx)
			PF_NONE: pfx <= (opc == 8'hCB) ? PF_CB : (opc == 8'hED) ? PF_ED : (opc == 8'hDD || opc == 8'hFD) ? PF_XY : PF_NONE;
			PF_XY:   pfx <= (opc == 8'hDD || opc == 8'hFD) ? PF_XY : (opc == 8'hED) ? PF_ED : PF_NONE;
			default: pfx <= PF_NONE;
		endcase
		if(cpu_halted) pfx <= PF_NONE;
	end
end

assign stop_req = freeze_req & ~stopped & m1 & ~no_stop & ((pfx == PF_NONE) | cpu_halted) & (cpu_halted | ~halt_only);

endmodule
