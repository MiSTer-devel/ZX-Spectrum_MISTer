// Dandanator Mini: the pager from calypso-ports (dandanator_pager.sv, by Manuel
// Teira, unchanged) plus what this core needs: map nothing until a ROM has been
// loaded, go back to the cartridge menu, and stretch the reset it requests.

module dandanator
(
	input         clk,        // 112 MHz: the pager counts its timeouts in cycles
	input         reset,      // core reset, without reset_req
	input         en,
	input         dl,         // cartridge ROM download
	input         home_req,   // Reset & Apply or Alt+F11
	input         detach,     // Reset & Detach Cartridge

	input  [15:0] addr,
	input         nMREQ,
	input         nWR,

	output  [4:0] page,       // 16 KB page of the cartridge ROM
	output        rom,        // cartridge ROM at #0000 instead of the Spectrum one
	output        nmi_n,
	output        reset_req
);

// Back to the menu: with the CPU held in reset, reset the pager (clears locked
// commands) and make one write into the ROM, which is its command 1 (page 0).
// Forcing the page from outside is not enough: the menu's first command that
// does not change the page would bring back the pager's one.
localparam H_IDLE = 0, H_PEND = 1, H_RST = 2, H_PULSE = 3, H_WAIT = 4;

reg        loaded = 0;
wire       act = en & loaded;
reg  [2:0] hs = H_IDLE;
reg [14:0] hcnt;
reg  [9:0] rst_cnt = 0;      // the reset it requests (command 36) lasts one cycle

wire [5:0] pg;
wire       cmd, rst_n;
wire       fake = (hs == H_PULSE);
wire       prst = reset | ~act | (|rst_cnt) | (hs == H_RST);

dandanator_pager pager
(
	.clk(clk),
	.reset(prst),
	.ce(1'b1),
	.addr(fake ? 16'd0 : addr),
	.data(8'd0),
	.mreq_n(nMREQ & ~fake),
	.wr_n(nWR & ~fake),
	.page(pg),
	.nmi_n(nmi_n),
	.reset_n(rst_n),
	.has_cmd(cmd),
	.pulse_out(),
	.index_out()
);

always @(posedge clk) begin
	reg old_act, old_dl, old_cmd;
	old_act <= act;
	old_dl  <= dl;
	old_cmd <= cmd;

	if(old_dl & ~dl) loaded <= 1;
	if(detach) loaded <= 0;

	if(~rst_n | (old_act & ~act)) rst_cnt <= '1;
	else if(|rst_cnt) rst_cnt <= rst_cnt - 1'd1;

	hcnt <= hcnt + 1'd1;
	case(hs)
		H_PEND:  if(~reset) begin hs <= H_RST; hcnt <= 0; end
		H_RST:   if(&hcnt[9:0]) begin hs <= H_PULSE; hcnt <= 0; end
		H_PULSE: if(hcnt == 33) begin hs <= H_WAIT; hcnt <= 0; end
		H_WAIT:  if((cmd ^ old_cmd) | &hcnt) hs <= H_IDLE;
		default: ;
	endcase
	if(reset & (hs == H_PULSE || hs == H_WAIT)) hs <= H_PEND;
	if((act & ~old_act) | (old_dl & ~dl & en) | (home_req & act)) hs <= H_PEND;
	if(~act) hs <= H_IDLE;
end

assign reset_req = (|rst_cnt) | (hs != H_IDLE);
assign page      = pg[4:0];
assign rom       = act & ~pg[5];

endmodule
