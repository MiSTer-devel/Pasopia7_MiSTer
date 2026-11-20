// A PAC2 RAM PAC (32 KiB, battery backed) whose contents live in a file on the SD card: a raw 32768-byte image, as
// CSCP's RAMPAC1.BIN / RAMPAC2.BIN. The file is read in whole when Main mounts it (an SC slot: remembered, so it
// comes back at the next core start), and every 512-byte block the CPU changes is written back once the CPU has
// left the RAM PAC alone for HOLD clocks (a BASIC SAVE writes many bytes; this writes each block once).
// Without a file (or with one that is not 32768 bytes) the RAM PAC starts formatted (INIT) and is lost at power off.
module p7_rampac #(
	parameter     INIT = "",
	parameter int HOLD = 5727272         // 0.1 s at 57.27 MHz
) (
	input             clk,

	// the CPU's side
	input      [14:0] addr,
	input             we,
	input       [7:0] d,
	output      [7:0] q,

	// hps_io, one virtual disk with 512-byte blocks
	input             img_mounted,
	input             img_readonly,
	input      [63:0] img_size,
	output reg [31:0] sd_lba = 0,
	output reg        sd_rd = 0,
	output reg        sd_wr = 0,
	input             sd_ack,
	input       [8:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din,
	input             sd_buff_wr,

	output reg        loaded = 0      // a file is behind the RAM PAC
);

localparam S_IDLE = 2'd0, S_LOAD = 2'd1, S_WB = 2'd2;
reg  [1:0] st = S_IDLE;
reg  [5:0] blk = 0;
reg        ro = 0, old_ack = 0;
reg [63:0] dirty = 0;
reg [22:0] quiet = 0;              // clocks since the last CPU write

dpram #(15, 8, INIT) ram
(
	.clock_a(clk), .enable_a(1'b1), .wren_a(we), .address_a(addr), .data_a(d), .q_a(q),
	.clock_b(clk), .enable_b(1'b1), .wren_b(st == S_LOAD && sd_ack && sd_buff_wr),
	.address_b({blk, sd_buff_addr}), .data_b(sd_buff_dout), .q_b(sd_buff_din)
);

reg  [5:0] first_dirty;
always_comb begin
	first_dirty = 6'd0;
	for (int i = 63; i >= 0; i--) if (dirty[i]) first_dirty = 6'(i);
end

always @(posedge clk) begin
	old_ack <= sd_ack;
	if (we) begin
		quiet <= 0;
		if (loaded && !ro) dirty[addr[14:9]] <= 1'b1;
	end
	else if (quiet != HOLD[22:0]) quiet <= quiet + 1'd1;

	case (st)
	S_LOAD: begin
		if (sd_ack) sd_rd <= 1'b0;
		if (old_ack && !sd_ack) begin
			if (blk == 6'd63) begin loaded <= 1'b1; st <= S_IDLE; end
			else begin blk <= blk + 1'd1; sd_lba <= {26'd0, blk + 1'd1}; sd_rd <= 1'b1; end
		end
	end
	S_WB: begin
		if (sd_ack) sd_wr <= 1'b0;
		if (old_ack && !sd_ack) st <= S_IDLE;
	end
	default:
		if (loaded && !ro && dirty != 0 && quiet == HOLD[22:0]) begin
			blk <= first_dirty;
			dirty[first_dirty] <= 1'b0;   // a write while it goes out marks it again
			sd_lba <= {26'd0, first_dirty};
			sd_wr <= 1'b1;
			st <= S_WB;
		end
	endcase

	if (img_mounted) begin
		loaded <= 1'b0;
		dirty <= 0;
		ro <= img_readonly;
		sd_rd <= 1'b0;
		sd_wr <= 1'b0;
		st <= S_IDLE;
		if (img_size == 64'd32768) begin
			blk <= 0;
			sd_lba <= 0;
			sd_rd <= 1'b1;
			st <= S_LOAD;
		end
	end
end

endmodule
