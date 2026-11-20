// Pasopia 7 floppy: a uPD765 (rtl/upd765_lite.sv) on two 2D drives whose disks are .d88 images in DDR3
// (rtl/d88_media.sv, from PC-8001_MiSTer). The glue is CSCP's pasopia7/floppy.cpp:
//   E0h write   TC off          E2h write   TC on (the end of a transfer)
//   E4h/E5h     uPD765 main status / data
//   E6h write   bit 7 resets the uPD765, bit 6 motor (on)
//   E6h read    bit 7 the uPD765's INT (in non-DMA mode also each data byte), the rest 0
// No DMA: the BIOS polls. The drives are not reset with the machine: a reset keeps the disks.
module p7_fdd #(parameter int CLK_HZ = 57272727)
(
	input             clk,
	input             reset,

	// the CPU's I/O: wr for one clk at the start of a write; rd_end for one clk at the end of a read
	input       [7:0] port,
	input             wr,
	input             rd_end,
	input       [7:0] din,
	output reg  [7:0] dout,
	output            sel,            // port is one of ours (E0-E7)

	output            motor,
	output            busy,           // a command is executing (the disk LED)

	// hps_io, two virtual disks with 512-byte blocks (slots 0 and 1)
	input       [1:0] img_mounted,
	input             img_readonly,
	input      [63:0] img_size,
	output     [31:0] sd_lba[2],
	output      [1:0] sd_rd,
	output      [1:0] sd_wr,
	input       [1:0] sd_ack,
	input       [8:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din[2],
	input             sd_buff_wr,

	output      [1:0] disk_ready,
	output      [1:0] disk_too_big,

	// DDR3 (sys/ddr_svc style, 64-bit words)
	input             ddr_busy,
	output      [7:0] ddr_burstcnt,
	output     [28:0] ddr_addr,
	input      [63:0] ddr_dout,
	input             ddr_dout_ready,
	output            ddr_rd,
	output     [63:0] ddr_din,
	output      [7:0] ddr_be,
	output            ddr_we
);

assign sel = port[7:3] == 5'b11100;
wire   sel_fdc = port[7:1] == 7'b1110010;     // E4, E5

reg motor_r = 0, fdc_rst = 0, tc = 0;
assign motor = motor_r;
always @(posedge clk) begin
	fdc_rst <= 1'b0;
	tc <= 1'b0;
	if (reset) motor_r <= 1'b0;
	else if (wr) begin
		if (port == 8'he2) tc <= 1'b1;
		if (port == 8'he6) begin
			fdc_rst <= din[7];
			motor_r <= din[6];
		end
	end
end

wire [19:0] m_addr;
wire  [7:0] m_q[0:1];
wire  [7:0] m_d, fdc_dout;
wire        m_rd, m_we, m_drv, fdc_int, fdc_exec;
wire  [1:0] m_done, m_ready, m_wprot;

upd765_lite #(.CLK_HZ(CLK_HZ)) fdc
(
	.clk(clk),
	.reset(reset | fdc_rst),
	.a0(port[0]),
	.rd(rd_end & sel_fdc),
	.wr(wr & sel_fdc),
	.din(din),
	.dout(fdc_dout),
	.tc(tc),
	.int_o(fdc_int),
	.exec(fdc_exec),
	.drq_o(),
	.msr_o(),
	.res_int(),
	.m_drv(m_drv),
	.m_addr(m_addr),
	.m_rd(m_rd),
	.m_q(m_q[m_drv]),
	.m_done(m_done[m_drv]),
	.m_we(m_we),
	.m_d(m_d),
	.m_ready(m_ready),
	.m_wprot(m_wprot)
);
assign busy = fdc_exec;

always @(*) begin
	dout = 8'hff;
	if (sel_fdc) dout = fdc_dout;
	else if (port == 8'he6) dout = {fdc_int, 7'd0};
end

// DDR3 shared by the two images: the grant changes hands when its holder is quiet and the other wants it
wire [28:0] d_addr[0:1];
wire [63:0] d_din[0:1];
wire  [7:0] d_be[0:1];
wire  [1:0] d_rd, d_we, d_quiet;
reg         g = 0;
always @(posedge clk) if (d_quiet[g] && (d_rd[~g] || d_we[~g])) g <= ~g;
assign ddr_burstcnt = 8'd1;
assign ddr_addr = d_addr[g];
assign ddr_din = d_din[g];
assign ddr_be = d_be[g];
assign ddr_rd = d_rd[g];
assign ddr_we = d_we[g];

genvar i;
generate for (i = 0; i < 2; i = i + 1) begin : drive
	d88_media #(.BASE(29'h06000000 + 29'(i) * 29'h10000)) media
	(
		.clk(clk),
		.reset(1'b0),
		.img_mounted(img_mounted[i]),
		.img_readonly(img_readonly),
		.img_size(img_size),
		.sd_lba(sd_lba[i]),
		.sd_rd(sd_rd[i]),
		.sd_wr(sd_wr[i]),
		.sd_ack(sd_ack[i]),
		.sd_buff_addr(sd_buff_addr),
		.sd_buff_dout(sd_buff_dout),
		.sd_buff_din(sd_buff_din[i]),
		.sd_buff_wr(sd_buff_wr & sd_ack[i]),
		.addr(m_addr),
		.rd(m_rd & (m_drv == i)),
		.we(m_we & (m_drv == i)),
		.d(m_d),
		.q(m_q[i]),
		.done(m_done[i]),
		.ready(m_ready[i]),
		.wprot(m_wprot[i]),
		.too_big(disk_too_big[i]),
		.ddr_busy(ddr_busy | (g != i)),
		.ddr_burstcnt(),
		.ddr_addr(d_addr[i]),
		.ddr_dout(ddr_dout),
		.ddr_dout_ready(ddr_dout_ready & (g == i)),
		.ddr_rd(d_rd[i]),
		.ddr_din(d_din[i]),
		.ddr_be(d_be[i]),
		.ddr_we(d_we[i]),
		.ddr_quiet(d_quiet[i])
	);
end endgenerate
assign disk_ready = m_ready;

endmodule
