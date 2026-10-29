module dandanator_pager(
    input clk,
    input reset,
    input ce,

    input [15:0] addr,
    input [7:0] data,
    input mreq_n,
    input wr_n,

    output reg [5:0] page = 6'd0,
    output reg nmi_n = 1'b1,
    output reg reset_n = 1'b1,
    
    output reg has_cmd = 1'b0,
    output pulse_out,
    output [1:0] index_out
);

assign pulse_out = pulse;
assign index_out = index;

parameter MHZ = 112;
localparam CMD_TOUT = MHZ * 130;        // 130us @112Mhz => 14560  ~ 14 bit
localparam SEQ_TOUT = MHZ * 5000;       //   5ms @112Mhz => 560000 ~ 20 bit
localparam ESP_TOUT = MHZ *  800;       // 800us @112Mhz
localparam CMD_DELAY = MHZ *  18;       //  18us @112Mhz => 2016   ~ 11 bit
localparam ESP_CMD_DELAY = MHZ * 9;     //   9us @112Mhz => 1008   ~ 10 bit
localparam [11:0] NMI_CYCLE_LEN = MHZ * 22; //  22us @112Mhz => 2464  -> 12 bit
localparam NMI_PULSE_LEN = 70;
// Real dandanator measured as: 22.083 us total NMI cycle with pulse of 625 ns (70 pulses at 112Mhz)

wire pulse = addr[15:14] == 2'b00 && mreq_n == 1'b0 && wr_n == 1'b0;
reg [19:0] timer = 'd0;
reg [7:0] cmd[3] = '{8'd0, 8'd0, 8'd0};
reg [7:0] pic_ram[8]; // Actually bigger, but this should be enough for game loading
reg commands_disabled = 1'b0;
reg commands_locked = 1'b0;
reg nmi_req = 1'b0;
reg reset_req = 1'b0;
reg [5:0] restart_page = 'd0;
reg restart_page_set = 1'b0;
reg [7:0] nmi_count = 8'd0;
reg [11:0] nmi_timer = 'd0;
reg [1:0] index;

always @(posedge clk) begin
    reg pulse_last = 1'b0;
    
    pulse_last <= pulse;
    if (~&timer) timer <= timer + 1'b1;
    
    if (reset) begin
        if (restart_page_set == 1'b1) page <= restart_page;
        restart_page_set <= 1'b0;
        reset_n <= 1'b1;
        index <= 'd0;
        {cmd[0], cmd[1], cmd[2]} <= 24'd0;
        timer <= 'd0;
        pulse_last <= 1'b0;
        commands_disabled <= 1'b0;
        commands_locked <= 1'b0;
        nmi_req <= 1'b0;
        reset_req <= 1'b0;
        has_cmd <= 1'b0;
        nmi_n <= 1'b1;
    end
    else begin
        if (~pulse_last & pulse) begin
            timer <= 'd0;
            if (cmd[index] == 8'd0 && index == 2'd0) cmd[index] <= cmd[index] + 1'd1;    // First pulse, don't take timer into account
            else if (timer < CMD_TOUT) cmd[index] <= cmd[index] + 1'd1; // Less than 130us since last pulse start
            else if (timer < ESP_TOUT) begin                            // Less than 800us (a bit empirical) since last pulse (special commands)
                case (index)
                    2'd0: begin
                        // Start of first data byte for special command
                        cmd[1] <= 8'd1;
                        index <= 2'd1;
                    end
                    2'd1: begin
                        // Start of second data byte for special command
                        cmd[2] <= 8'd1;
                        index <= 2'd2;
                    end
                    2'd2: begin
                        //Confirmation pulse for special command
                        index <= 2'd3;
                    end
                endcase
            end
            else begin
                // ESP_TOUT. Discard and start new command
                index <= 2'd0;
                cmd[0] <= 8'd1;
            end
        end
        else begin // Checks for timeout when no pulses
            if (timer >= SEQ_TOUT) begin
                index <= 2'd0;
                cmd[0] <= 'd0;
            end
            // cmd defined and passed 130us + 18us since last pulse start. Standard command
            else if (timer >= CMD_TOUT + CMD_DELAY && index == 2'd0 && |cmd[0] && cmd[0] < 8'd40) begin
                cmd[0] <= 'd0;             // Reset the command
                has_cmd <= ~has_cmd;       // Debug
                index <= 2'd0;
                // Only if commands are not locked or disabled (standard command cannot unlock commands)
                if (commands_disabled == 1'b0 && commands_locked == 1'b0) begin
                    // Page switch commands
                    if (cmd[0] < 8'd34) page <= cmd[0][5:0] == 6'd33 ? 6'b100000 : cmd[0][5:0] - 1'd1;
                    else case (cmd[0])
                        // Disable commands and enable Spectrum ROM
                        8'd34: begin
                            commands_disabled <= 1'b1;
                            page <= 6'b100000;
                        end
                        8'd36: reset_req <= 1'b1;                // Start reset request
                        8'd37: nmi_req <= 1'b1;                  // Start NMI request
                        8'd39: begin
                            restart_page <= page;               // Save restart page
                            restart_page_set <= 1'b1;
                        end
                    endcase
                end
            end
            // Special command. Execute 9us after the confirmation pulse
            else if (timer >= ESP_CMD_DELAY && index == 2'd3 && cmd[0] >= 8'd40) begin
                cmd[0] <= 'd0;              // Reset the command
                has_cmd <= ~has_cmd;        // Debug
                index <= 2'd0;
                if (commands_disabled == 1'b0) begin
                    // Command 46 must work even with locked commands
                    if (cmd[0] == 8'd46) begin
                        if (cmd[1] == cmd[2]) begin
                            case (cmd[1][4:0])
                                5'b11111: commands_disabled <= 1'b1;
                                5'b10000: commands_locked <= 1'b0;
                                5'b00001: commands_locked <= 1'b1;
                            endcase
                        end
                    end
                    // Any other command when the commands are not locked
                    else if (commands_locked == 1'b0) begin
                        case (cmd[0]) // Special commands
                            8'd40: begin
                                page <= cmd[1][5:0] == 6'd33 ? 6'b100000 : cmd[1][5:0] - 1'd1;
                                {commands_disabled, commands_locked, nmi_req, reset_req} <= cmd[2][3:0];
                            end
                            8'd44: pic_ram[cmd[1]] <= cmd[2];
                            8'd50: pic_ram[cmd[1]] <= cmd[2];
                            8'd45: begin
                                nmi_count <= pic_ram[cmd[1]];
                            end
                            default: begin end
                        endcase
                    end
                end
            end
        end
        if (|nmi_count) begin
            begin
                if (nmi_timer < NMI_CYCLE_LEN) begin
                    nmi_timer <= nmi_timer + 1'd1;
                    nmi_n <= nmi_timer < NMI_CYCLE_LEN - NMI_PULSE_LEN;
                end
                else begin
                    nmi_n <= 1'b1;
                    nmi_timer <= 'd0;
                    nmi_count <= nmi_count - 1'd1;
                end
            end
        end
        else begin
            nmi_timer <= 'd0;
            nmi_n <= 1'b1;
        end
        if (reset_req) reset_n <= 1'b0;
        if (nmi_req) begin
            nmi_req <= 1'b0;
            nmi_count <= 1'b1;
        end
    end
end

endmodule
