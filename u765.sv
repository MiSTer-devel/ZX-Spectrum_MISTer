// ====================================================================
//
//  NEC u765 FDC
//
//  Copyright (C) 2017 Gyorgy Szombathelyi <gyurco@freemail.hu>
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

//TODO:
//mt flags for READ
//WRITE DELETE should write the Deleted Address Mark to the SectorInfo
//SCAN commands
//time-accurate SEEK (based on the head stepping rate)
//real FORMAT (but this would require squeezing/expanding the image file)

module u765
(
	input        clk_sys,     // sys clock
	input        ce,          // chip enable
	input        reset,	     // async reset
	input        ready,
	input        a0,
	input        nRD,         // i/o read
	input        nWR,         // i/o write
	input  [7:0] din,         // i/o data in
	output [7:0] dout,        // i/o data out

	input        img_mounted, // signaling that new image has been mounted
	input [31:0] img_size,    // size of image in bytes
	output[31:0] sd_lba,
	output reg   sd_rd,
	output reg   sd_wr,
	input        sd_ack,
	input  [8:0] sd_buff_addr,
	input  [7:0] sd_buff_dout,
	output [7:0] sd_buff_din,
	input        sd_buff_wr
);

parameter COMMAND_TIMEOUT = 26'd35000000;

parameter UPD765_MAIN_D0B = 0;
parameter UPD765_MAIN_D1B = 1;
parameter UPD765_MAIN_D2B = 2;
parameter UPD765_MAIN_D3B = 3;
parameter UPD765_MAIN_CB = 4;
parameter UPD765_MAIN_EXM = 5;
parameter UPD765_MAIN_DIO = 6;
parameter UPD765_MAIN_RQM = 7;

parameter UPD765_ST3_US0 = 0;
parameter UPD765_ST3_US1 = 1;
parameter UPD765_ST3_HD = 2;
parameter UPD765_ST3_TS = 3;
parameter UPD765_ST3_T0 = 4;
parameter UPD765_ST3_RDY = 5;
parameter UPD765_ST3_WP = 6;
parameter UPD765_ST3_FT = 7;

parameter UPD765_SD_BUFF_TRACKINFO = 1'd0;
parameter UPD765_SD_BUFF_SECTOR = 1'd1;

typedef enum
{
 COMMAND_IDLE,

 COMMAND_READ_TRACK,

 COMMAND_WRITE_DELETED_DATA,
 COMMAND_WRITE_DATA,

 COMMAND_READ_DELETED_DATA,
 COMMAND_READ_DATA,

 COMMAND_RW_DATA_EXEC1,
 COMMAND_RW_DATA_EXEC2,
 COMMAND_RW_DATA_EXEC3,
 COMMAND_RW_DATA_EXEC4,
 COMMAND_RW_DATA_EXEC5,
 COMMAND_RW_DATA_EXEC6,
 COMMAND_RW_DATA_EXEC7,
 COMMAND_RW_DATA_EXEC8,

 COMMAND_READ_ID,
 COMMAND_READ_ID_EXEC1,
 COMMAND_READ_ID_EXEC2,
 COMMAND_READ_ID_EXEC3,

 COMMAND_FORMAT_TRACK,
 COMMAND_FORMAT_TRACK1,
 COMMAND_FORMAT_TRACK2,
 COMMAND_FORMAT_TRACK3,
 COMMAND_FORMAT_TRACK4,
 COMMAND_FORMAT_TRACK5,
 COMMAND_FORMAT_TRACK6,
 COMMAND_FORMAT_TRACK7,
 COMMAND_FORMAT_TRACK8,

 COMMAND_SCAN_EQUAL,
 COMMAND_SCAN_LOW_OR_EQUAL,
 COMMAND_SCAN_HIGH_OR_EQUAL,

 COMMAND_RECALIBRATE,

 COMMAND_SENSE_INTERRUPT_STATUS,
 COMMAND_SENSE_INTERRUPT_STATUS1,
 COMMAND_SENSE_INTERRUPT_STATUS2,

 COMMAND_SPECIFY,
 COMMAND_SPECIFY_WR,

 COMMAND_SENSE_DRIVE_STATUS,
 COMMAND_SENSE_DRIVE_STATUS_RD,

 COMMAND_SEEK,
 COMMAND_SEEK_EXEC1,

 COMMAND_SETUP,

 COMMAND_READ_RESULTS,

 COMMAND_INVALID,
 COMMAND_INVALID1
} state_t;

reg [19:0] image_size;
reg image_ready = 0;
reg [7:0] image_tracks;
reg [7:0] image_track_size;
reg image_sides; //1 side - 0, 2 sides - 1
reg [8:0] image_track_offsets_addr = 0;
reg image_track_offsets_wr;
reg [15:0] image_track_offsets_out, image_track_offsets_in;
reg image_edsk; //DSK - 0, EDSK - 1

//single port buffer in RAM
logic [15:0] image_track_offsets[0:511]; //offset of tracks * 256

reg [7:0] buff_data_in, buff_data_out;
reg [8:0] buff_addr;
wire sd_buff_type;
reg hds;

u765_dpram sbuf
(
	.clock(clk_sys),

	.address_a({sd_buff_type,hds,sd_buff_addr}),
	.data_a(sd_buff_dout),
	.wren_a(sd_buff_wr & sd_ack),
	.q_a(sd_buff_din),

	.address_b({sd_buff_type,hds,buff_addr}),
	.data_b(buff_data_out),
	.wren_b(buff_wr),
	.q_b(buff_data_in)
);
reg buff_wr, buff_wait;

wire rd = nWR & ~nRD;
wire wr = ~nWR & nRD;

always @(posedge clk_sys) begin
	if (image_track_offsets_wr) begin
		image_track_offsets[image_track_offsets_addr] <= image_track_offsets_out;
		image_track_offsets_in <= image_track_offsets_out;
	end else begin
		image_track_offsets_in <= image_track_offsets[image_track_offsets_addr];
	end
end

always @(posedge clk_sys) begin
	reg old_wr, old_rd;
	reg [31:0] seek_pos;
	reg [7:0] sector_c, sector_h, sector_r, sector_n;
	reg [7:0] sector_st1, sector_st2, total_sectors;
	reg [15:0] sector_size;
	reg [7:0] current_sector, last_sector;
	reg [14:0] bytes_to_read;
	reg [2:0] substate;
	reg [1:0] seek_state;
	reg [1:0] image_scan_state = 0;
	reg old_mounted;
	reg [15:0] track_offset;
	reg [5:0] ack;
	reg sd_busy;
	reg [26:0] timeout;
	reg rtrack, write, rw_deleted;
	reg [7:0] m_status;  //main status register
	reg [7:0] status[4] = '{0, 0, 0, 0}; //st0-3
	state_t state, command;
	reg [7:0] ncn; //new cylinder number
	reg [7:0] pcn; //present cylinder number
   reg ds0;
	reg [7:0] c;
	reg [7:0] h;
	reg [7:0] r;
	reg [7:0] n;
	reg [7:0] eot;
	//reg [7:0] gpl;
	reg [7:0] dtl;
	reg [7:0] sc;
	//reg [7:0] d;

	//reg mt;
	//reg mfm;
	reg sk;
	reg int_state;

	buff_wait <= 0;

	//new image mounted
	old_mounted <= img_mounted;
	if(old_mounted & ~img_mounted) begin
		image_size <= img_size[19:0];
		image_scan_state<=1;
		image_ready<=0;
		{ds0, ncn, pcn, c, h, r, n} <= 0;
		state <= COMMAND_IDLE;
		int_state <= 0;
		seek_state <= 0;
		last_sector <= 0;
	end

   //Process the image file
	if (ce) begin
		case (image_scan_state)
			0: ;//no new image
			1: //read the first 512 byte
				if (~sd_busy) begin
					sd_buff_type <= UPD765_SD_BUFF_SECTOR;
					sd_rd<=1;
					sd_lba<=0;
					sd_busy<=1;
					track_offset<=16'h1; //offset 100h
					image_track_offsets_addr <= 0;
					buff_addr<=0;
					buff_wait <= 1;
					image_scan_state<=2;
				end
			2: //process the header
				if (~sd_busy & ~buff_wait& ~image_ready) begin
					if (buff_addr == 0) begin
						if (buff_data_in == "E")
							image_edsk <= 1;
						else if (buff_data_in == "M")
							image_edsk <= 0;
						else begin
							image_ready <= 0;
							image_scan_state <= 0;
							status[3][UPD765_ST3_WP] <= 1;
							status[3][UPD765_ST3_RDY] <= 0;
						end
					end else if (buff_addr == 9'h30) image_tracks <= buff_data_in;
					else if (buff_addr == 9'h31) image_sides <= buff_data_in[1];
					else if (buff_addr == 9'h33) image_track_size <= buff_data_in;
					else if (buff_addr >= 9'h34) begin
						if (image_track_offsets_addr[8:1] != image_tracks) begin
							image_track_offsets_wr <= 1;
							if (image_edsk) begin
								image_track_offsets_out <= buff_data_in ? track_offset : 16'd0;
								track_offset <= track_offset + buff_data_in;
							end else begin
								image_track_offsets_out <= track_offset;
								track_offset <= track_offset + image_track_size;
							end
							image_scan_state <= 3;
						end else begin
							image_ready <= 1;
							status[3][UPD765_ST3_WP] <= 0;
							status[3][UPD765_ST3_RDY] <= 1;
							//Read the trackinfo because the host may not issue a seek
							//after the disk change, and the buffer will still contain
							//the data from the previous disk
							//seek will reset image_scan_state
							image_track_offsets_addr <= 9'd0;
							seek_state <= 1;
						end
					end
					buff_addr <= buff_addr + 1'd1;
					buff_wait <= 1;
				end
			3: begin
					image_track_offsets_wr <= 0;
					if (image_sides)
						image_track_offsets_addr <= image_track_offsets_addr + 1'd1;
					else
						image_track_offsets_addr <= image_track_offsets_addr + 2'd2;
					image_scan_state <= 2;
				end
		endcase
	end

	//the FDC
   if (reset & ~image_scan_state) begin
		m_status <= 8'h80;
		state <= COMMAND_IDLE;
		status[0] <= 0;
		status[1] <= 0;
		status[2] <= 0;
		status[3][UPD765_ST3_WP] <= ~image_ready;
		status[3][UPD765_ST3_RDY] <= image_ready;
		status[3][UPD765_ST3_T0] <= 1;
		{ds0, ncn, pcn, c, h, r, n} <= 0;
		int_state <= 0;
		seek_state <= 0;
		{ack, sd_wr, sd_rd, sd_busy} <= 0;
		image_track_offsets_wr <= 0;
	end else if (ce) begin

		ack <= {ack[4:0], sd_ack};
		if(ack[5:4] == 'b01) {sd_rd,sd_wr} <= 0;
		if(ack[5:4] == 'b10) sd_busy <= 0;

		old_wr <= wr;
		old_rd <= rd;

		//seek
		case(seek_state)
			0: ;//no seek in progress
			1: seek_state <= 2; //wait for image_track_offset_in
			2: if (image_ready && image_track_offsets_in) begin
				    if (~sd_busy) begin
				        sd_buff_type <= UPD765_SD_BUFF_TRACKINFO;
				        sd_rd <= 1;
				        sd_lba <= image_track_offsets_in[15:1];
				        sd_busy <= 1;
				        seek_state <= 3;
				    end
			   end else begin
					if (image_scan_state)
						image_scan_state <= 0;
					else begin
						int_state <= 1;
						pcn <= ncn;
						status[0] <= 8'h20; //it's legit to seek to an empty track
						status[3][UPD765_ST3_T0] <= !ncn;
					end
					seek_state <= 0;
			   end
			3: if (~sd_busy) begin
				    if (hds == image_sides) begin
						if (image_scan_state) //seek after disk image open
							image_scan_state <= 0;
						else begin
							int_state <= 1;
							status[0] <= 8'h20;
						end
						status[3][UPD765_ST3_T0] <= !ncn;
						seek_state <= 0;
						pcn <= ncn;
				    end else begin //read TrackInfo from the other head if 2 sided
				        hds <= ~hds;
				        image_track_offsets_addr <= image_track_offsets_addr + 1'd1;
				        seek_state <= 1;
				    end
			   end
		endcase

		case(state)
			COMMAND_IDLE:
			begin
				m_status[UPD765_MAIN_CB] <= 0;
				m_status[UPD765_MAIN_DIO] <= 0;
				m_status[UPD765_MAIN_RQM] <= 1;

				if (~old_wr & wr & a0) begin
					//mt <= din[7];
					//mfm <= din[6];
					sk <= din[5];
					substate <= 0;
					casex (din[7:0])
						8'bXXX00110: state <= COMMAND_READ_DATA;
						8'bXXX01100: state <= COMMAND_READ_DELETED_DATA;
						8'bXX000101: state <= COMMAND_WRITE_DATA;
						8'bXX001001: state <= COMMAND_WRITE_DELETED_DATA;
						8'b0XX00010: state <= COMMAND_READ_TRACK;
						8'b0X001010: state <= COMMAND_READ_ID;
						8'b0X001101: state <= COMMAND_FORMAT_TRACK;
						8'bXXX10001: state <= COMMAND_SCAN_EQUAL;
						8'bXXX11001: state <= COMMAND_SCAN_LOW_OR_EQUAL;
						8'bXXX11101: state <= COMMAND_SCAN_HIGH_OR_EQUAL;
						8'b00000111: state <= COMMAND_RECALIBRATE;
						8'b00001000: state <= COMMAND_SENSE_INTERRUPT_STATUS;
						8'b00000011: state <= COMMAND_SPECIFY;
						8'b00000100: state <= COMMAND_SENSE_DRIVE_STATUS;
						8'b00001111: state <= COMMAND_SEEK;
						default: state <= COMMAND_INVALID;
					endcase
				end else if(~old_rd & rd & a0) begin
					dout <= 8'hff;
				end
			end

			COMMAND_SENSE_INTERRUPT_STATUS:
			begin
				m_status[UPD765_MAIN_DIO] <= 1;
				m_status[UPD765_MAIN_CB] <= 1;
				int_state <= 0;
				state <= int_state ? COMMAND_SENSE_INTERRUPT_STATUS1 : COMMAND_INVALID;
			end

			COMMAND_SENSE_INTERRUPT_STATUS1:
			if (~old_rd & rd & a0) begin
				dout <= status[0];
				state <= COMMAND_SENSE_INTERRUPT_STATUS2;
			end

			COMMAND_SENSE_INTERRUPT_STATUS2:
			if (~old_rd & rd & a0) begin
				dout <= pcn;
				state <= COMMAND_IDLE;
			end

			COMMAND_SENSE_DRIVE_STATUS:
			begin
				int_state <= 0;
				if (~old_wr & wr & a0) begin
					state <= COMMAND_SENSE_DRIVE_STATUS_RD;
					m_status[UPD765_MAIN_DIO] <= 1;
					ds0 <= din[0];
				end
			end

			COMMAND_SENSE_DRIVE_STATUS_RD:
			if (~old_rd & rd & a0) begin
				dout <= ds0 ? 8'h1 : status[3];
				state <= COMMAND_IDLE;
			end

			COMMAND_SPECIFY:
			begin
				m_status[UPD765_MAIN_CB] <= 1;
				int_state <= 0;
				if (~old_wr & wr & a0) begin
					state <= COMMAND_SPECIFY_WR;
				end
			end

			COMMAND_SPECIFY_WR:
			if (~old_wr & wr & a0) begin
				state <= COMMAND_IDLE;
			end

			COMMAND_RECALIBRATE:
			begin
				last_sector <= 0;
				int_state <= 0;
				m_status[UPD765_MAIN_CB] <= 1;
				if (~old_wr & wr & a0) begin
					hds <= 0;
					ncn <= 0;
					image_track_offsets_addr <= 0;
					seek_state <= 1;
					state <= COMMAND_IDLE;
				end
			end

			COMMAND_SEEK:
			begin
				last_sector <= 0;
				int_state <= 0;
				m_status[UPD765_MAIN_CB] <= 1;
				if (~old_wr & wr & a0) begin
					hds <= 0;
					state <= COMMAND_SEEK_EXEC1;
				end
			end

			COMMAND_SEEK_EXEC1:
			if (~old_wr & wr & a0) begin
				if ((ready && image_ready && din<image_tracks) || !din) begin
					ncn <= din;
					image_track_offsets_addr <= { din, 1'b0 };
					seek_state <= 1;
					state <= COMMAND_IDLE;
				end else begin
					//Seek error
					int_state <= 1;
					status[0] <= 8'hE8;
					state <= COMMAND_IDLE;
				end
			end

			COMMAND_READ_ID:
			begin
				int_state<=0;
				m_status[UPD765_MAIN_CB] <= 1;
				if (~old_wr & wr & a0) begin
					if (~ready | ~image_ready) begin
						status[0] <= 8'h40;
						status[1] <= 8'b101;
						status[2] <= 0;
						state <= COMMAND_READ_RESULTS;
					end else	if (din[2] & ~image_sides) begin
						status[0] <= 8'h48; //no side B
						status[1] <= 0;
						status[2] <= 0;
						state <= COMMAND_READ_RESULTS;
					end else begin
						hds <= din[2];
						image_track_offsets_addr <= { pcn, din[2] };
						buff_wait <= 1;
						state <= COMMAND_READ_ID_EXEC1;
						m_status[UPD765_MAIN_RQM] <= 0;
					end
				end
			end

			COMMAND_READ_ID_EXEC1:
			if (~sd_busy & ~buff_wait) begin
				if (image_track_offsets_in) begin
					sd_buff_type <= UPD765_SD_BUFF_TRACKINFO;
					buff_addr <= {image_track_offsets_in[0], 8'h15}; //number of sectors
					buff_wait <= 1;
					state <= COMMAND_READ_ID_EXEC2;
				end else begin
					status[0] <= 8'h40;
					status[1] <= 8'b101;
					status[2] <= 0;
					state <= COMMAND_READ_RESULTS;
				end
			end

			COMMAND_READ_ID_EXEC2:
			if (~buff_wait) begin
				//cycle through sectors between adjacent READ ID commands
				//to imitate rotating media (and satisfy some copy protections)
				buff_addr[7:0] <= 8'h18 + (last_sector << 3); //choose the next sector
				buff_wait <= 1;
				last_sector <= last_sector == (buff_data_in - 1'd1) ? 8'h00: last_sector + 1'd1;
				state <= COMMAND_READ_ID_EXEC3;
			end

			COMMAND_READ_ID_EXEC3:
			if (~buff_wait) begin
				if (buff_addr[2:0] == 8'h00) sector_c <= buff_data_in;
				else if (buff_addr[2:0] == 8'h01) sector_h <= buff_data_in;
				else if (buff_addr[2:0] == 8'h02) sector_r <= buff_data_in;
				else if (buff_addr[2:0] == 8'h03) begin
					sector_n <= buff_data_in;
					status[0] <= 0;
					status[1] <= 0;
					status[2] <= 0;
					state <= COMMAND_READ_RESULTS;
				end
				buff_addr <= buff_addr + 1'd1;
				buff_wait <= 1;
			end

			COMMAND_READ_TRACK:
			begin
				int_state <= 0;
				m_status[UPD765_MAIN_CB] <= 1;
				command <= COMMAND_RW_DATA_EXEC1;
				state <= COMMAND_SETUP;
				{rtrack, write, rw_deleted} <= 3'b100;
			end

			COMMAND_WRITE_DATA:
			begin
				int_state <= 0;
				m_status[UPD765_MAIN_CB] <= 1;
				command <= COMMAND_RW_DATA_EXEC1;
				state <= COMMAND_SETUP;
				{rtrack, write, rw_deleted} <= 3'b010;
			end

			COMMAND_WRITE_DELETED_DATA:
			begin
				int_state<=0;
				m_status[UPD765_MAIN_CB] <= 1;
				command <= COMMAND_RW_DATA_EXEC1;
				state <= COMMAND_SETUP;
				{rtrack, write, rw_deleted} <= 3'b011;
			end

			COMMAND_READ_DATA:
			begin
				int_state <= 0;
				m_status[UPD765_MAIN_CB] <= 1;
				command <= COMMAND_RW_DATA_EXEC1;
				state <= COMMAND_SETUP;
				{rtrack, write, rw_deleted} <= 3'b000;
			end

			COMMAND_READ_DELETED_DATA:
			begin
				int_state<=0;
				m_status[UPD765_MAIN_CB] <= 1;
				command <= COMMAND_RW_DATA_EXEC1;
				state <= COMMAND_SETUP;
				{rtrack, write, rw_deleted} <= 3'b001;
			end

			COMMAND_RW_DATA_EXEC1:
			begin
				m_status[UPD765_MAIN_RQM] <= 0;
				m_status[UPD765_MAIN_EXM] <= 1;
				m_status[UPD765_MAIN_DIO] <= ~write;
				if (rtrack) r <= 1;
				// Read from the track stored at the last seek
				// even if different one is given in the command
				image_track_offsets_addr <= { pcn, hds };
				buff_wait <= 1;
				state <= COMMAND_RW_DATA_EXEC2;
			end

			COMMAND_RW_DATA_EXEC2:
			if (~sd_busy & ~buff_wait) begin
				current_sector <= 1'd1;
				sd_buff_type <= UPD765_SD_BUFF_TRACKINFO;
				seek_pos <= {image_track_offsets_in+1'd1,8'd0}; //TrackInfo+256bytes
				buff_addr <= {image_track_offsets_in[0], 8'h14}; //sector size
				buff_wait <= 1;
				state <= COMMAND_RW_DATA_EXEC3;
			end

			//process trackInfo + sectorInfo
			COMMAND_RW_DATA_EXEC3:
			if (~sd_busy & ~buff_wait) begin
				if (buff_addr[7:0] == 8'h14) begin
					if (!image_edsk) sector_size <= 8'h80 << buff_data_in[2:0];
					buff_addr[7:0] <= 8'h15; //number of sectors
					buff_wait <= 1;
				end else	if (buff_addr[7:0] == 8'h15) begin
					total_sectors <= buff_data_in;
					buff_addr[7:0] <= 8'h18; //sector info list
					buff_wait <= 1;
				end else if (current_sector > total_sectors) begin
					//sector not found or end of track
					m_status[UPD765_MAIN_EXM] <= 0;
					state <= COMMAND_READ_RESULTS;
					status[0] <= ^rtrack ? 8'h00 : 8'h40;
					status[1] <= rtrack ? 8'h00 : 8'h04;
					status[2] <= 0;
				end else begin
					//process sector info list
					case (buff_addr[2:0])
						0: sector_c <= buff_data_in;
						1: sector_h <= buff_data_in;
						2: sector_r <= buff_data_in;
						3: sector_n <= buff_data_in;
						4: sector_st1 <= buff_data_in;
						5: sector_st2 <= buff_data_in;
						6: if (image_edsk) sector_size[7:0] <= buff_data_in;
						7: begin
								if (image_edsk) sector_size[15:8] <= buff_data_in;
								state <= COMMAND_RW_DATA_EXEC4;
							end
					endcase
					buff_addr <= buff_addr + 1'd1;
					buff_wait <= 1;
				end
			end

			//found the sector?
			COMMAND_RW_DATA_EXEC4:
			if (sector_c != c && ~rtrack) begin
				m_status[UPD765_MAIN_EXM] <= 0;
				state <= COMMAND_READ_RESULTS;
				status[0] <= 8'h40;
				status[1] <= 8'h04; //no data
				status[2] <= sector_c == 8'hff ? 8'h02 : 8'h10; //bad/wrong cylinder
			end else if ((rtrack && current_sector == r) || 
							(~rtrack && sector_r == r && sector_h == h && sector_n == n)) begin
				//sector found in the sector info list
				if (sector_n == 6) bytes_to_read <= sector_size[14:0]; //speccial handling of 8k sectors
				else if (!sector_n) bytes_to_read <= dtl;
				else bytes_to_read <= 8'h80 << sector_n[2:0];
				timeout <= COMMAND_TIMEOUT;
				state <= COMMAND_RW_DATA_EXEC5;
			end else begin
				//try the next sector in the sectorinfo list
				current_sector <= current_sector + 1'd1;
				seek_pos <= seek_pos + sector_size;
				state <= COMMAND_RW_DATA_EXEC3;
			end

			//Read the LBA for the sector into the RAM
			COMMAND_RW_DATA_EXEC5:
			if (~sd_busy) begin
				sd_buff_type <= UPD765_SD_BUFF_SECTOR;
				sd_rd <= 1;
				sd_lba <= seek_pos[31:9];
				sd_busy <= 1;
				buff_addr <= seek_pos[8:0];
				buff_wait <= 1;
				state <= COMMAND_RW_DATA_EXEC6;
			end

			//Read from/write to Speccy
			COMMAND_RW_DATA_EXEC6:
			if (~sd_busy & ~buff_wait) begin
				if (!bytes_to_read) begin
					//end of the current sector
					m_status[UPD765_MAIN_RQM] <= 0;
					if (write && buff_addr && seek_pos < image_size) begin
						sd_lba <= seek_pos[31:9];
						sd_wr <= 1;
						sd_busy <= 1;
					end
					state <= COMMAND_RW_DATA_EXEC8;
				end else if (!timeout) begin
					m_status[UPD765_MAIN_EXM] <= 0;
					state <= COMMAND_READ_RESULTS;
					status[0] <= 8'h40;
					status[1] <= { sector_st1[7:5], !timeout, sector_st1[3:0] };
					status[2] <= sector_st2;
				end else if (~write & ~old_rd & rd & a0) begin
					if (&buff_addr) begin
						//sector continues on the next LBA
						m_status[UPD765_MAIN_RQM] <= 0;
						state <= COMMAND_RW_DATA_EXEC5;
					end
					//Speedlock: randomize 'weak' sectors last bytes
					//weak sector is cyl 0, head 0, sector 2
					dout <= (current_sector == 2 & !pcn & ~hds &
					         sector_st1[5] & sector_st2[5] & !bytes_to_read[14:2]) ?
								timeout[7:0] :
								buff_data_in;
					buff_addr <= buff_addr + 1'd1;
					buff_wait <= 1;
					bytes_to_read <= bytes_to_read - 1'd1;
					seek_pos <= seek_pos + 1'd1;
					timeout <= COMMAND_TIMEOUT;
				end else if (write & ~old_wr & wr & a0) begin
					buff_wr <= 1;
					buff_data_out <= din;
					timeout <= COMMAND_TIMEOUT;
					m_status[UPD765_MAIN_RQM] <= 0;
					state <= COMMAND_RW_DATA_EXEC7;
				end else begin
					m_status[UPD765_MAIN_RQM] <= 1;
					timeout <= timeout - 1'd1;
				end
			end else begin
				m_status[UPD765_MAIN_RQM] <= 0;
			end

			COMMAND_RW_DATA_EXEC7:
			begin
				buff_wr <= 0;
				buff_addr <= buff_addr + 1'd1;
				bytes_to_read <= bytes_to_read - 1'd1;
				seek_pos <= seek_pos + 1'd1;
				if (&buff_addr) begin
					//sector continues on the next LBA
					//so write out the current before reading the next
					if (seek_pos < image_size) begin
						sd_lba <= seek_pos[31:9];
						sd_wr <= 1;
						sd_busy <= 1;
					end
					state <= COMMAND_RW_DATA_EXEC5;
				end else begin
					m_status[UPD765_MAIN_RQM] <= 1;
					state <= COMMAND_RW_DATA_EXEC6;
				end
			end

			//End of reading/writing sector, what's next?
			COMMAND_RW_DATA_EXEC8:
			if (~sd_busy) begin
				if (~rtrack & ~sk & ((sector_st1[5] & sector_st2[5]) | (rw_deleted ^ sector_st2[6]))) begin
					//deleted mark or crc error
					m_status[UPD765_MAIN_EXM] <= 0;
					state <= COMMAND_READ_RESULTS;
					status[0] <= 8'h40;
					status[1] <= sector_st1;
					status[2] <= rw_deleted ? 8'h40 : sector_st2;
				end else	if ((rtrack ? current_sector : sector_r) == eot) begin
					//end of cylinder
					m_status[UPD765_MAIN_EXM] <= 0;
					state <= COMMAND_READ_RESULTS;
					status[0] <= rtrack ? 8'h00 : 8'h40;
					status[1] <= 8'h80;
					status[2] <= 0;
				end else begin
					//read the next sector (multi-sector transfer)
					r <= r + 1'd1;
					state <= COMMAND_RW_DATA_EXEC2;
				end
			end

			COMMAND_FORMAT_TRACK:
			begin
				int_state <= 0;
				m_status[UPD765_MAIN_CB] <= 1;
				state <= COMMAND_FORMAT_TRACK1;
			end

			COMMAND_FORMAT_TRACK1: //doesn't modify the media
			if (~old_wr & wr & a0) begin
				n <= din;
				state <= COMMAND_FORMAT_TRACK2;
			end

			COMMAND_FORMAT_TRACK2:
			if (~old_wr & wr & a0) begin
				sc <= din;
				state <= COMMAND_FORMAT_TRACK3;
			end

			COMMAND_FORMAT_TRACK3:
			if (~old_wr & wr & a0) begin
				//gpl <= din;
				state <= COMMAND_FORMAT_TRACK4;
			end

			COMMAND_FORMAT_TRACK4:
			if (~old_wr & wr & a0) begin
				//d <= din;
				m_status[UPD765_MAIN_EXM] <= 1;
				state <= COMMAND_FORMAT_TRACK5;
			end

			COMMAND_FORMAT_TRACK5:
			if (!sc) begin
				m_status[UPD765_MAIN_EXM] <= 0;
				status[0] <= 0;
				status[1] <= 0;
				status[2] <= 0;
				state <= COMMAND_READ_RESULTS;
			end else	if (~old_wr & wr & a0) begin
				c <= din;
				state <= COMMAND_FORMAT_TRACK6;
			end

			COMMAND_FORMAT_TRACK6:
			if (~old_wr & wr & a0) begin
				h <= din;
				state <= COMMAND_FORMAT_TRACK7;
			end

			COMMAND_FORMAT_TRACK7:
			if (~old_wr & wr & a0) begin
				r <= din;
				state <= COMMAND_FORMAT_TRACK8;
			end

			COMMAND_FORMAT_TRACK8:
			if (~old_wr & wr & a0) begin
				n <= din;
				sc <= sc - 1'd1;
				r <= r + 1'd1;
				state <= COMMAND_FORMAT_TRACK5;
			end

			COMMAND_SCAN_EQUAL:
			begin
				int_state <= 0;
				if (~old_wr & wr & a0) begin
					state <= COMMAND_IDLE;
				end
			end

			COMMAND_SCAN_HIGH_OR_EQUAL:
			begin
				int_state <= 0;
				if (~old_wr & wr & a0) begin
					state <= COMMAND_IDLE;
				end
			end

			COMMAND_SCAN_LOW_OR_EQUAL:
			begin
				int_state <= 0;
				if (~old_wr & wr & a0) begin
					state <= COMMAND_IDLE;
				end
			end

			COMMAND_SETUP:
			if (!old_wr & wr & a0) begin
				case (substate)
					0: begin
							hds <= din[2];
							substate <= 1;
						end
					1: begin
							c <= din;
							substate <= 2;
						end
					2:	begin
							h <= din;
							substate <= 3;
						end
					3: begin
							r <= din;
							substate <= 4;
						end
					4: begin
							n <= din;
							substate <= 5;
						end
					5: begin
							eot <= din;
							substate <= 6;
						end
					6:	begin
							//gpl <= din;
							substate <= 7;
						end
					7: begin
							dtl <= din;
							substate <= 0;
							if (~ready | ~image_ready) begin
								status[0] <= 8'h40;
								status[1] <= 8'b101;
								status[2] <= 0;
								state <= COMMAND_READ_RESULTS;
							end else if (hds & ~image_sides) begin
								hds <= 0;
								status[0] <= 8'h48; //no side B
								status[1] <= 0;
								status[2] <= 0;
								state <= COMMAND_READ_RESULTS;
							end else begin
								state <= command;
							end
						end
				endcase
			end

			COMMAND_READ_RESULTS:
			begin
				m_status[UPD765_MAIN_RQM] <= 1;
				m_status[UPD765_MAIN_DIO] <= 1;
				if (~old_rd & rd & a0) begin
					case (substate)
						0: begin
								dout <= {status[0][7:3], hds, status[0][1:0]};
								substate <= 1;
							end
						1: begin
								dout <= status[1];
								substate <= 2;
							end
						2: begin
								dout <= status[2];
								substate <= 3;
							end
						3: begin
								dout <= sector_c;
								substate <= 4;
							end
						4: begin
								dout <= sector_h;
								substate <= 5;
							end
						5: begin
								dout <= sector_r;
								substate <= 6;
							end
						6: begin
								dout <= sector_n;
								state <= COMMAND_IDLE;
							end
						7: ;//not happen
					endcase
				end
			end

			COMMAND_INVALID:
			begin
				int_state <= 0;
				m_status[UPD765_MAIN_DIO] <= 1;
				status[0] <= 8'h80;
				state <= COMMAND_INVALID1;
			end

			COMMAND_INVALID1:
			if (~old_rd & rd & a0) begin
				state <= COMMAND_IDLE;
				dout <= status[0];
			end

		endcase //status

		if (~old_rd & rd & ~a0) begin //read main status register
			dout <= m_status;
		end
	end
end

endmodule

module u765_dpram #(parameter DATAWIDTH=8, ADDRWIDTH=11)
(
	input	                clock,

	input	[ADDRWIDTH-1:0] address_a,
	input	[DATAWIDTH-1:0] data_a,
	input	                wren_a,
	output reg [DATAWIDTH-1:0] q_a,

	input	[ADDRWIDTH-1:0] address_b,
	input	[DATAWIDTH-1:0] data_b,
	input	                wren_b,
	output reg [DATAWIDTH-1:0] q_b
);

logic [DATAWIDTH-1:0] ram[0:(1<<ADDRWIDTH)-1];

always_ff@(posedge clock) begin
	if(wren_a) begin
		ram[address_a] <= data_a;
		q_a <= data_a;
	end else begin
		q_a <= ram[address_a];
	end
end

always_ff@(posedge clock) begin
	if(wren_b) begin
		ram[address_b] <= data_b;
		q_b <= data_b;
	end else begin
		q_b <= ram[address_b];
	end
end

endmodule