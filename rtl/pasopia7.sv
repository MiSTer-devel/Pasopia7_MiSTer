//============================================================================
//  Toshiba Pasopia 7 (PA7007) machine
//
//  References: Common Source Code Project src/vm/pasopia7 (Takeda Toshiya's EmuPIA7: the behaviour the core is
//  compared against, see verilator/), MAME src/mame/toshiba/pasopia7.cpp (clocks, port names). PLAN.md section 3
//  has the memory and I/O map.
//
//  Clocks: the board has two crystals. 15.9744 MHz / 4 = 3.9936 MHz for the Z80, CTC and PIO (/8 for the two
//  SN76489s); 14.31818 MHz is the dot clock (80 columns; 40 columns and the CRTC character clock divide it). Here
//  clk_sys = CLK_MUL x 14.31818 MHz: 4 on the MiSTer (57.27 MHz), 1 for the fast simulation build. The dot clock
//  is every CLK_MUL clocks; the CPU enable comes from an exact fractional accumulator
//  (3.9936 / 14.31818 = 54912 / 196875).
//
//  Memory: 64 KiB RAM, 16 KiB BIOS ROM, 32 KiB BASIC ROM, 4 x 16 KiB VRAM (B, R, G/text code, attribute) and a
//  16 x 4-bit palette. From reset until the first write to port 3Ch the BIOS ROM is read everywhere and writes are
//  lost. Port 3Ch bits 1-0 select ROM or RAM in 0000-7FFF, bit 2 puts VRAM at 8000-BFFF, bit 3 (MIO) sends the next
//  I/O access to RAM at the 16-bit port address instead.
//
//  I/O (A7-A0): 08-0B 8255-0 (display), 0C-0F 8255-1 (display/memory), 10-11 HD46505, 18-1B PAC2 slot, 20-23 8255-2
//  (sound/tape/NMI), 28-2B CTC, 30-33 PIO (A data, B data, A control, B control), 3A/3B SN76489 #1/#2, 3C memory,
//  E0-E6 floppy (rtl/p7_fdd.sv). Ports 00-03 trap to NMI in Pasopia mode (8255-2 port C bit 6).
//
//  Interrupts: Z80 mode 2, CTC then PIO on the daisy chain. NMI: at reset, and the Pasopia I/O trap.
//
//  orig: the original Pasopia (PA7010; CSCP src/vm/pasopia). Same CPU, CTC, PIO, keyboard (its own matrix), PAC2
//  (RAM PAC #1 only, no select register), 8255-2 (tape, mute), floppy. Different: a 32 KiB T-BASIC ROM at 0000-7FFF
//  (port 3Ch bit 1 = RAM there; RAM always takes the writes; bit 2 resets the machine), no NMI, no SN76489s,
//  16 KiB VRAM + 1 attribute bit reached through two 8255s (00-03: A address low, B data, C read data; 08-0B: A
//  display mode, B status, C address high: a change of either address port writes the data, unless bit 14 is set;
//  bit 15 is the attribute), and its own display (rtl/paso_video.sv). Its ROMs come from boot1.rom (ioctl index
//  40h, or 1): tbasic.rom (32 KiB) + the 2 KiB font.
//============================================================================

module pasopia7 #(parameter CLK_MUL = 4, parameter TURBO = 5)
(
	input         clk_sys,
	input         reset,

	input  [10:0] ps2_key,
	input   [5:0] joy0,               // MiSTer bits: 0 right, 1 left, 2 down, 3 up, 4 button 1, 5 button 2
	input   [5:0] joy1,

	// boot.rom download (ioctl index 0): BIOS 0000-3FFF, BASIC 4000-BFFF, font C000-CFFF (the MAME files in order)
	input         ioctl_download,
	input   [7:0] ioctl_index,
	input         ioctl_wr,
	input  [24:0] ioctl_addr,
	input   [7:0] ioctl_dout,

	output        ce_pix,
	output reg [7:0] R,
	output reg [7:0] G,
	output reg [7:0] B,
	output reg    HSync,
	output reg    VSync,
	output reg    HBlank,
	output reg    VBlank,

	output [15:0] audio,              // signed

	input         tape_in,
	output        tape_motor,
	output        tape_out,
	input         turbo,              // the deck plays at TURBO x: the CPU runs TURBO x faster
	input         tape_audio,         // OSD Tape audio: the tape input in the sound
	input         lcd,                // the Pasopia 7 LCD model (PA7170): rtl/p7_lcd.sv instead of the CRT
	input         orig,               // the original Pasopia (PA7010)
	input         kbd_us,             // US keyboard symbols (rtl/p7_kbd.sv), else keys by position

	// image slots: 0 and 1 floppy drives 1 and 2 (.d88, held in DDR3: rtl/p7_fdd.sv), 2 and 3 RAM PAC #1 and #2
	// (32 KiB files: rtl/p7_rampac.sv)
	input   [3:0] img_mounted,
	input         img_readonly,
	input  [63:0] img_size,
	output [31:0] sd_lba[4],
	output  [3:0] sd_rd,
	output  [3:0] sd_wr,
	input   [3:0] sd_ack,
	input   [8:0] sd_buff_addr,
	input   [7:0] sd_buff_dout,
	output  [7:0] sd_buff_din[4],
	input         sd_buff_wr,
	output  [1:0] rampac_loaded,
	output  [1:0] disk_ready,
	output  [1:0] disk_too_big,
	output        disk_busy,

	input         ddr_busy,
	output  [7:0] ddr_burstcnt,
	output [28:0] ddr_addr,
	input  [63:0] ddr_dout,
	input         ddr_dout_ready,
	output        ddr_rd,
	output [63:0] ddr_din,
	output  [7:0] ddr_be,
	output        ddr_we,

	// debug / simulation
	output [15:0] cpu_addr,
	output        cpu_ce,
	output        cpu_m1_n,
	output        dbg_io_wr,
	output        dbg_io_rd,
	output [15:0] dbg_io_port,
	output  [7:0] dbg_io_data,
	output  [7:0] dbg_op              // the last opcode byte fetched (M1)
);

//------------------------------------------------------------------------------------------------
// Clock enables
//------------------------------------------------------------------------------------------------

reg [1:0] dot_div = 0;
always @(posedge clk_sys) dot_div <= (dot_div == CLK_MUL - 1) ? 2'd0 : dot_div + 1'd1;
wire ce_dot = dot_div == CLK_MUL - 1;     // 14.31818 MHz

// Turbo (~/dev2/tape-standard.md rule 7): while the deck plays, the CPU enable runs TURBO times faster. The BASIC
// tape routine (6932-6944) times the tape's pulses with CTC channel 0, which counts the CPU clock, so the CPU, CTC
// and PIO all speed up together with the tape. The PSGs keep their own enable and pitch. TURBO x 3.9936 MHz must
// leave at least two clk_sys clocks between CPU enables (5x at 57.27 MHz: every 2.87 clocks; the fast simulation
// build, CLK_MUL 1, has no room and uses TURBO 1).
// (21 bits: at CLK_MUL 4 the modulus is 787500 and acc + TURBO x INC reaches 1062059, past 20 bits; a 20-bit sum
// wrapped there and dropped CPU enables in turbo, 9% of them, so the tape ran fast against the CTC)
localparam [20:0] CPU_INC = 21'd54912;
localparam [20:0] CPU_MOD = 21'd196875 * CLK_MUL;
localparam [20:0] CPU_INC_T = CPU_INC * TURBO;
reg [20:0] cpu_acc = 0;
reg        ce_cpu = 0;
wire [20:0] cpu_inc = turbo ? CPU_INC_T : CPU_INC;
always @(posedge clk_sys) begin
	if (cpu_acc + cpu_inc >= CPU_MOD) begin
		cpu_acc <= cpu_acc + cpu_inc - CPU_MOD;
		ce_cpu  <= 1'b1;
	end
	else begin
		cpu_acc <= cpu_acc + cpu_inc;
		ce_cpu  <= 1'b0;
	end
end
// 1.9968 MHz for the SN76489s, never turbo
reg [20:0] psg_acc = 0;
reg        ce_psg = 0;
always @(posedge clk_sys) begin
	if (psg_acc + CPU_INC / 2 >= CPU_MOD) begin
		psg_acc <= psg_acc + CPU_INC / 2 - CPU_MOD;
		ce_psg  <= 1'b1;
	end
	else begin
		psg_acc <= psg_acc + CPU_INC / 2;
		ce_psg  <= 1'b0;
	end
end
assign cpu_ce = ce_cpu;

//------------------------------------------------------------------------------------------------
// CPU
//------------------------------------------------------------------------------------------------

wire        mreq_n, iorq_n, rd_n, wr_n, m1_n;
wire [15:0] A;
wire  [7:0] cpu_dout;
reg   [7:0] cpu_din;
wire        int_n;
wire        nmi_n;

reg  soft_rst = 0;                        // orig: port 3Ch bit 2
wire cpu_reset = reset | ioctl_download | soft_rst;

// T80 v350 (Sorgelig's MiSTer T80). The sim gets T80s as a ghdl synth netlist with these generics fixed
// (verilator/Makefile, -g...): keep both in step.
`ifdef VERILATOR
T80s cpu
`else
T80s #(.Mode(0), .T2Write(1), .IOWait(1)) cpu
`endif
(
	.RESET_n(~cpu_reset),
	.CLK(clk_sys),
	.CEN(ce_cpu),
	.OUT0(1'b0),
	.WAIT_n(1'b1),
	.INT_n(int_n),
	.NMI_n(nmi_n),
	.BUSRQ_n(1'b1),
	.M1_n(m1_n),
	.MREQ_n(mreq_n),
	.IORQ_n(iorq_n),
	.RD_n(rd_n),
	.WR_n(wr_n),
	.RFSH_n(),
	.HALT_n(),
	.BUSAK_n(),
	.A(A),
	.DI(cpu_din),
	.DO(cpu_dout)
);

assign cpu_addr = A;
assign cpu_m1_n = m1_n;

wire mem_rd = ~mreq_n & ~rd_n;
wire mem_wr = ~mreq_n & ~wr_n;
wire io_rd  = ~iorq_n & ~rd_n & m1_n;
wire io_wr  = ~iorq_n & ~wr_n;
wire inta   = ~iorq_n & ~m1_n;
wire fetch  = ~m1_n & ~mreq_n & ~rd_n;

reg mem_wr_d = 0, mem_rd_d = 0, io_wr_d = 0, io_rd_d = 0, inta_d = 0, fetch_d = 0;
always @(posedge clk_sys) begin
	mem_wr_d <= mem_wr;
	mem_rd_d <= mem_rd;
	io_wr_d  <= io_wr;
	io_rd_d  <= io_rd;
	inta_d   <= inta;
	fetch_d  <= fetch;
end
wire mem_wr_stb = mem_wr & ~mem_wr_d;
wire mem_rd_end = ~mem_rd & mem_rd_d;
wire io_wr_stb  = io_wr & ~io_wr_d;
wire io_rd_stb  = io_rd & ~io_rd_d;
wire io_rd_end  = ~io_rd & io_rd_d;
wire inta_stb   = inta & ~inta_d;
wire fetch_end  = ~fetch & fetch_d;

wire [7:0] port = A[7:0];

//------------------------------------------------------------------------------------------------
// ROMs (loaded from boot.rom) and RAM
//------------------------------------------------------------------------------------------------

wire rom_load = ioctl_download && ioctl_index == 8'd0 && ioctl_wr;
wire [7:0] bios_q, basic_q, ram_q;

spram #(14, 8) bios_rom
(
	.clock(clk_sys),
	.address(ioctl_download ? ioctl_addr[13:0] : A[13:0]),
	.data(ioctl_dout),
	.wren(rom_load && ioctl_addr[24:14] == 11'd0),
	.q(bios_q)
);

spram #(15, 8) basic_rom
(
	.clock(clk_sys),
	.address(ioctl_download ? ioctl_addr[14:0] - 15'h4000 : A[14:0]),
	.data(ioctl_dout),
	.wren(rom_load && ioctl_addr[24:16] == 9'd0 && (ioctl_addr[15:14] == 2'b01 || ioctl_addr[15:14] == 2'b10)),
	.q(basic_q)
);

// the original Pasopia's T-BASIC (boot1.rom 0000-7FFF) and font (8000-87FF)
wire orom_load = ioctl_download && (ioctl_index == 8'h40 || ioctl_index == 8'h01) && ioctl_wr;
wire [7:0] tbasic_q;
spram #(15, 8) tbasic_rom
(
	.clock(clk_sys),
	.address(ioctl_download ? ioctl_addr[14:0] : A[14:0]),
	.data(ioctl_dout),
	.wren(orom_load && ioctl_addr[24:15] == 10'd0),
	.q(tbasic_q)
);

// Memory map state (port 3Ch)
reg       mem_boot;                       // the BIOS everywhere, writes lost (until the first OUT to 3Ch)
reg [2:0] mem_map;
reg       mio, mio_cyc;                   // MIO armed / this I/O cycle goes to RAM
reg [1:0] pc2_lo;                         // 8255-2 port C bits 1-0: the last 3Ch write's bits 1-0
wire      vram_sel = ~mem_boot & mem_map[2];

wire in_vram = vram_sel && A[15:14] == 2'b10;

reg ram_we;
always @(*) begin
	ram_we = 1'b0;
	if (mem_wr_stb && !mem_boot && !in_vram) ram_we = 1'b1;
	if (io_wr_stb && mio) ram_we = 1'b1;  // MIO: the OUT goes to RAM
	if (orig) ram_we = mem_wr_stb;
end

spram #(16, 8) ram
(
	.clock(clk_sys),
	.address(A),
	.data(cpu_dout),
	.wren(ram_we),
	.q(ram_q)
);

//------------------------------------------------------------------------------------------------
// VRAM, palette, plane access (CSCP memory.cpp)
//------------------------------------------------------------------------------------------------

wire [7:0] ppi0_pa, ppi0_pb_o, ppi0_pc_o;
wire [7:0] ppi1_pa, ppi1_pb, ppi1_pc;
wire [7:0] ppi2_pa, ppi2_pb_o, ppi2_pc;

wire [7:0] plane     = ppi1_pa;           // bits 6-4: G/R/B write+read enable, bits 2-0: data (0 = write FFh)
wire [3:0] attr_data = ppi1_pb[3:0];
wire [2:0] text_page = ppi1_pb[6:4];
wire       attr_wrap = ppi1_pc[4];
wire       pal_sel   = |ppi1_pc[3:2];
wire       pal_dis   = ppi1_pc[3];
wire       blink     = ppi1_pc[5];
wire [7:0] mode      = ppi0_pa;           // bit 7 fine graphics, bit 5 80 columns, bit 3 one text colour (bits 2-0)

reg  [3:0] pal[16];
reg  [3:0] attr_latch;
wire       pal_access = pal_sel && plane[6:4] == 3'b000;

wire [7:0] vb_q, vr_q, vg_q, va_q;        // CPU side
wire [7:0] vb_v, vr_v, vg_v, va_v;        // video side
reg [13:0] v_gfx_addr, v_txt_addr;

wire vram_wr = mem_wr_stb && in_vram && !pal_access;
wire [3:0] attr_new = attr_wrap ? attr_latch : attr_data;

dpram #(14, 8) vram_b
(
	.clock_a(clk_sys), .enable_a(1'b1), .wren_a(orig ? o_vwr : vram_wr && plane[4]),
	.address_a(orig ? ovptr[13:0] : A[13:0]),
	.data_a(orig ? oppi0_pb : plane[0] ? cpu_dout : 8'hff), .q_a(vb_q),
	.clock_b(clk_sys), .enable_b(1'b1), .wren_b(1'b0), .address_b(v_gfx_addr), .data_b(8'h00), .q_b(vb_v)
);
dpram #(14, 8) vram_r
(
	.clock_a(clk_sys), .enable_a(1'b1), .wren_a(vram_wr && plane[5]), .address_a(A[13:0]),
	.data_a(plane[1] ? cpu_dout : 8'hff), .q_a(vr_q),
	.clock_b(clk_sys), .enable_b(1'b1), .wren_b(1'b0), .address_b(v_gfx_addr), .data_b(8'h00), .q_b(vr_v)
);
dpram #(14, 8) vram_g
(
	.clock_a(clk_sys), .enable_a(1'b1), .wren_a(vram_wr && plane[6]), .address_a(A[13:0]),
	.data_a(plane[2] ? cpu_dout : 8'hff), .q_a(vg_q),
	.clock_b(clk_sys), .enable_b(1'b1), .wren_b(1'b0), .address_b(v_txt_addr), .data_b(8'h00), .q_b(vg_v)
);
dpram #(14, 8) vram_a
(
	.clock_a(clk_sys), .enable_a(1'b1), .wren_a(orig ? o_vwr : vram_wr && plane[6]),
	.address_a(orig ? ovptr[13:0] : A[13:0]),
	.data_a(orig ? {7'd0, ovptr[15]} : {4'h0, attr_new}), .q_a(va_q),
	.clock_b(clk_sys), .enable_b(1'b1), .wren_b(1'b0), .address_b(v_txt_addr), .data_b(8'h00), .q_b(va_v)
);

assign ov_q = vb_q;
assign oa_q = va_q;
wire [7:0] vram_rd = (plane[4] & plane[0] ? vb_q : 8'hff) &
                     (plane[5] & plane[1] ? vr_q : 8'hff) &
                     (plane[6] & plane[2] ? vg_q : 8'hff);

always @(posedge clk_sys) begin
	if (cpu_reset) begin
		for (int i = 0; i < 16; i++) pal[i] <= i[3:0] & 4'd7;
		attr_latch <= 0;
	end
	else begin
		if (mem_wr_stb && in_vram && pal_access) pal[A[3:0]] <= cpu_dout[3:0];
		if (vram_wr && plane[6]) attr_latch <= attr_new;
		// a read of the G plane latches its attribute (8255-0 port B)
		if (mem_rd_end && in_vram && !pal_access && plane[6] && plane[2]) attr_latch <= va_q[3:0];
	end
end

// CPU memory read
always @(*) begin
	if (orig) cpu_din = !A[15] && !mem_map[1] ? tbasic_q : ram_q;
	else if (mem_boot) cpu_din = bios_q;
	else case (A[15:14])
	2'b00: cpu_din = mem_map[1] ? ram_q : basic_q;
	2'b01: cpu_din = mem_map[0] ? bios_q : mem_map[1] ? ram_q : basic_q;
	2'b10: cpu_din = !vram_sel ? ram_q : pal_access ? {4'h0, pal[A[3:0]]} : vram_rd;
	2'b11: cpu_din = ram_q;
	endcase
	if (inta) cpu_din = int_vec;
	else if (io_rd) cpu_din = io_dout;
end

//------------------------------------------------------------------------------------------------
// I/O
//------------------------------------------------------------------------------------------------

wire io_wr_dev = io_wr_stb && !mio;       // device strobes (not while MIO sends the cycle to RAM)

always @(posedge clk_sys) begin
	if (cpu_reset) begin
		mem_boot <= ~orig;
		if (!soft_rst) mem_map <= 3'd0;   // orig: the map survives its own reset (CSCP)
		mio      <= 1'b0;
		mio_cyc  <= 1'b0;
		pc2_lo   <= 2'd0;
	end
	else begin
		if (io_wr_stb || io_rd_stb) begin
			mio_cyc <= mio;
			mio <= 1'b0;
		end
		if (io_wr_dev && port == 8'h3c && orig) mem_map <= cpu_dout[2:0];
		else if (io_wr_dev && port == 8'h3c) begin
			mem_boot <= 1'b0;
			mem_map  <= cpu_dout[2:0];
			mio      <= cpu_dout[3];
			pc2_lo   <= cpu_dout[1:0];
		end
	end
end

// orig: a 3Ch write with bit 2 resets the machine (CSCP vm->reset(); the memory map stays)
reg [3:0] soft_cnt = 0;
always @(posedge clk_sys) begin
	if (reset) soft_cnt <= 0;
	else if (io_wr_dev && orig && port == 8'h3c && cpu_dout[2]) soft_cnt <= 4'd15;
	else if (soft_cnt != 0) soft_cnt <= soft_cnt - 1'd1;
	soft_rst <= soft_cnt != 0;
end

// 8255-0: display. A out: mode. B in: {attr3, 0, VSYNC, CRT (1), display, attr2-0}.
wire [7:0] ppi0_dout, ppi1_dout, ppi2_dout;
wire       crtc_vs, crtc_hs, crtc_de, crtc_hde, crtc_vde;
wire [7:0] ppi0_pb_i = {attr_latch[3], 1'b0, crtc_vs, ~lcd, crtc_hde & crtc_vde, attr_latch[2:0]};

i8255_m0 ppi0
(
	.clk(clk_sys), .reset(cpu_reset), .addr(port[1:0]), .rd(1'b0),
	.wr(io_wr_dev && !orig && port[7:2] == 6'b000010), .din(cpu_dout), .dout(ppi0_dout),
	.pa_i(8'h00), .pb_i(ppi0_pb_i), .pc_i(8'h00),
	.pa_o(ppi0_pa), .pb_o(ppi0_pb_o), .pc_o(ppi0_pc_o)
);

// 8255-1: display/memory, all outputs
i8255_m0 ppi1
(
	.clk(clk_sys), .reset(cpu_reset), .addr(port[1:0]), .rd(1'b0),
	.wr(io_wr_dev && !orig && port[7:2] == 6'b000011), .din(cpu_dout), .dout(ppi1_dout),
	.pa_i(8'h00), .pb_i(8'h00), .pc_i(8'h00),
	.pa_o(ppi1_pa), .pb_o(ppi1_pb), .pc_o(ppi1_pc)
);

// 8255-2: sound/tape/NMI. A out: bit 0 clears the NMI flags, bit 1 mutes the sound, bit 4 tape out, bit 5 motor
// (0 = on). B in: bit 5 tape in, bit 2 reset NMI flag, bit 1 trap NMI flag. C: bits 7 NMI mask, 6 Pasopia mode
// (out); bits 1-0 the 3Ch write (in).
reg        nmi_reset_flag, nmi_trap_flag;
wire [7:0] ppi2_pb_i = orig ? {2'b00, tape_in, 5'd0} : {2'b00, tape_in, 2'b00, nmi_reset_flag, nmi_trap_flag, 1'b0};

i8255_m0 ppi2
(
	.clk(clk_sys), .reset(cpu_reset), .addr(port[1:0]), .rd(1'b0),
	.wr(io_wr_dev && port[7:2] == 6'b001000), .din(cpu_dout), .dout(ppi2_dout),
	.pa_i(8'h00), .pb_i(ppi2_pb_i), .pc_i(orig ? {5'd0, mem_map[1], 2'b00} : {6'd0, pc2_lo}),
	.pa_o(ppi2_pa), .pb_o(ppi2_pb_o), .pc_o(ppi2_pc)
);

// The original Pasopia's 8255-0 (00-03: VRAM address low, data, read data) and 8255-1 (08-0B: display mode,
// status {attribute, HSYNC, VSYNC, CRT (1), display, 0, 0, 0}, VRAM address high). CSCP's 8255 passes a port's
// value on only when it changes (or at its first write): then the VRAM pointer moves and, with bit 14 = 0, the data
// latch is written there (bit 15 the attribute).
wire [7:0] oppi0_dout, oppi1_dout, oppi0_pa, oppi0_pb, oppi1_pa, oppi1_pc;
wire [7:0] ov_q, oa_q;
i8255_m0 oppi0
(
	.clk(clk_sys), .reset(cpu_reset), .addr(port[1:0]), .rd(1'b0),
	.wr(io_wr_dev && orig && port[7:2] == 6'b000000), .din(cpu_dout), .dout(oppi0_dout),
	.pa_i(8'h00), .pb_i(8'h00), .pc_i(ov_q),
	.pa_o(oppi0_pa), .pb_o(oppi0_pb), .pc_o()
);
i8255_m0 oppi1
(
	.clk(clk_sys), .reset(cpu_reset), .addr(port[1:0]), .rd(1'b0),
	.wr(io_wr_dev && orig && port[7:2] == 6'b000010), .din(cpu_dout), .dout(oppi1_dout),
	.pa_i(8'h00), .pb_i({oa_q[0], crtc_hs, crtc_vs, 1'b1, crtc_hde & crtc_vde, 3'd0}), .pc_i(8'h00),
	.pa_o(oppi1_pa), .pb_o(), .pc_o(oppi1_pc)
);
wire [15:0] ovptr = {oppi1_pc, oppi0_pa};
reg         o_wr_a, o_wr_c, o_first_a, o_first_c;
reg   [7:0] o_old_a, o_old_c;
always @(posedge clk_sys) begin
	o_wr_a <= io_wr_dev && orig && port == 8'h00;
	o_wr_c <= io_wr_dev && orig && (port == 8'h0a || (port == 8'h0b && !cpu_dout[7]));
	if (cpu_reset) begin
		o_first_a <= 1'b1; o_first_c <= 1'b1;
		o_old_a <= 0; o_old_c <= 0;
	end
	else begin
		if (o_wr_a) begin o_first_a <= 1'b0; o_old_a <= oppi0_pa; end
		if (o_wr_c) begin o_first_c <= 1'b0; o_old_c <= oppi1_pc; end
	end
end
wire o_vevt = (o_wr_a && (o_first_a || oppi0_pa != o_old_a)) || (o_wr_c && (o_first_c || oppi1_pc != o_old_c));
wire o_vwr  = o_vevt && !ovptr[14];

wire snd_mute = ppi2_pa[1];
assign tape_out   = ppi2_pa[4];
assign tape_motor = ~ppi2_pa[5];
wire nmi_mask = ppi2_pc[7];
wire paso_mode = ppi2_pc[6];

// NMI: reset, and the Pasopia I/O trap (accesses to 00-03h in Pasopia mode). T80 takes NMI on a falling edge.
wire trap = !orig && paso_mode && !mio && port[7:2] == 6'd0 && (io_wr_stb || io_rd_stb);
reg  [4:0] nmi_cnt;
reg        reset_d = 1;
assign nmi_n = orig || nmi_cnt == 0;
always @(posedge clk_sys) begin
	reset_d <= cpu_reset;
	if (cpu_reset) begin
		nmi_cnt <= 0;
		nmi_reset_flag <= 1'b1;
		nmi_trap_flag <= 1'b0;
	end
	else begin
		if (reset_d) nmi_cnt <= 5'd31;    // reset: the NMI the BIOS boots through (CSCP IOTRAP::do_reset)
		else if (trap) begin
			nmi_trap_flag <= 1'b1;
			if (!nmi_mask) nmi_cnt <= 5'd31;
		end
		else if (ce_cpu && nmi_cnt != 0) nmi_cnt <= nmi_cnt - 1'd1;
		if (io_wr_dev && port == 8'h20 && cpu_dout[0]) begin
			nmi_reset_flag <= 1'b0;
			nmi_trap_flag <= 1'b0;
		end
	end
end

// HD46505 register shadows (CSCP: R12-R17 read back on the odd port, the even port reads the address register)
reg  [4:0] crtc_ar;
reg  [7:0] crtc_r[18];
wire       crtc_wr = io_wr_dev && port[7:1] == 7'b0001000;
always @(posedge clk_sys) begin
	if (cpu_reset) begin
		crtc_ar <= 0;
		for (int i = 0; i < 18; i++) crtc_r[i] <= 8'h00;
	end
	else if (crtc_wr) begin
		if (!port[0]) crtc_ar <= cpu_dout[4:0];
		else if (crtc_ar < 5'd18) crtc_r[crtc_ar] <= cpu_dout;
	end
end
wire [7:0] crtc_dout = !port[0] ? {3'b000, crtc_ar} : (crtc_ar >= 5'd12 && crtc_ar < 5'd18) ? crtc_r[crtc_ar] : 8'hff;

// CTC: channels 0 and 2 time from the CPU clock; ZC0 -> TRG1, ZC2 -> TRG3, ZC1 -> flip-flop -> beeper
wire [7:0] ctc_dout, ctc_vector, pio_dout, pio_vector, pio_pa;
wire [3:0] ctc_zc;
wire       ctc_irq, ctc_ieo, ctc_in_svc, pio_irq, pio_ieo, pio_in_svc;
reg        ctc_ack, pio_ack, reti;

z80ctc ctc
(
	.clk(clk_sys), .reset(cpu_reset), .ce(ce_cpu),
	.wr(io_wr_dev && port[7:2] == 6'b001010), .channel(port[1:0]), .din(cpu_dout), .dout(ctc_dout),
	.trigger({ctc_zc[2], 1'b0, ctc_zc[0] & ~orig, 1'b0}),
	.iei(1'b1), .irq(ctc_irq), .ieo(ctc_ieo),
	.ack(ctc_ack), .reti(reti), .vector(ctc_vector), .zc(ctc_zc), .in_svc(ctc_in_svc)
);

reg beep_ff;
always @(posedge clk_sys) if (cpu_reset) beep_ff <= 0; else if (ctc_zc[1]) beep_ff <= ~beep_ff;

// PIO: A = keyboard row select (bit 7 gates the beeper), B = keyboard return lines (mode 3, interrupt)
wire [7:0] kbd_ret_n;
wire       pio_wr = io_wr_dev && port[7:2] == 6'b001100;
reg  [7:0] kbd_ret_d;
reg        vs_d;
always @(posedge clk_sys) begin
	kbd_ret_d <= kbd_ret_n;
	vs_d <= crtc_vs;
end
// CSCP re-reads the keyboard (and so re-checks the PIO's mode 3 condition) when port A changes and once a frame
wire pio_a_changed = pio_wr && port[1:0] == 2'b00 && cpu_dout != pio_pa;
reg  pio_a_changed_d;
always @(posedge clk_sys) pio_a_changed_d <= pio_a_changed;
wire kbd_evt = pio_a_changed_d || (kbd_ret_n != kbd_ret_d) || (crtc_vs & ~vs_d);

z80pio pio
(
	.clk(clk_sys), .reset(cpu_reset),
	.wr(pio_wr), .rd(1'b0), .cd(port[1]), .ba(port[0]), .din(cpu_dout), .dout(pio_dout),
	.pa_i(8'hff), .pb_i(kbd_ret_n), .pa_o(pio_pa), .pb_o(), .evt({kbd_evt, 1'b0}),
	.iei(ctc_ieo), .irq(pio_irq), .ieo(pio_ieo),
	.ack(pio_ack), .reti(reti & ~ctc_in_svc), .vector(pio_vector), .in_svc(pio_in_svc)
);

p7_kbd kbd
(
	.clk(clk_sys), .reset(reset), .ps2_key(ps2_key), .orig(orig), .us(kbd_us), .sel(pio_pa), .ret_n(kbd_ret_n)
);

// Interrupt acknowledge and RETI (ED 4D, seen on the opcode fetches)
reg [7:0] int_vec;
reg       pfx_ed, pfx_cb;
reg [7:0] fetch_byte;
assign int_n = ~(ctc_irq | pio_irq);
always @(posedge clk_sys) begin
	ctc_ack <= 1'b0;
	pio_ack <= 1'b0;
	reti    <= 1'b0;
	if (cpu_reset) begin
		int_vec <= 8'hff;
		pfx_ed  <= 1'b0;
		pfx_cb  <= 1'b0;
	end
	else begin
		if (inta_stb) begin
			if (ctc_irq) begin ctc_ack <= 1'b1; int_vec <= ctc_vector; end
			else if (pio_irq) begin pio_ack <= 1'b1; int_vec <= pio_vector; end
			else int_vec <= 8'hff;
		end
		if (fetch) fetch_byte <= cpu_din;
		if (fetch_end) begin
			if (pfx_ed) begin
				reti <= fetch_byte == 8'h4d;
				pfx_ed <= 1'b0;
			end
			else if (pfx_cb) pfx_cb <= 1'b0;
			else begin
				pfx_ed <= fetch_byte == 8'hed;
				pfx_cb <= fetch_byte == 8'hcb;
			end
		end
	end
end

// PAC2 slot (CSCP pac2.cpp): select register at 1Bh (bit 7 = 0); slot 4 RAM PAC #1, 3 RAM PAC #2, 2 Kanji ROM PAC
// (PA7246: 128 KiB kanji.rom from boot2.rom, ioctl index 80h or OSD 2; 18h-1Ah the 17-bit address; without a ROM it
// reads 00h, as CSCP without KANJI.ROM), 1 Joy PAC
reg  [2:0] pac_sel;
reg [14:0] rp_ptr[2];
reg [16:0] kj_ptr;
wire       pac_wr = io_wr_dev && port[7:2] == 6'b000110;
wire       pac_dev_wr = pac_wr && (port[1:0] != 2'b11 || cpu_dout[7] || orig);
wire [2:0] pac_s = orig ? 3'd4 : pac_sel;   // orig: one slot, the RAM PAC (CSCP's default device)
wire [7:0] rp_q[2];
wire       kj_load = ioctl_download && (ioctl_index == 8'h80 || ioctl_index == 8'h02) && ioctl_wr;
reg        kj_loaded = 0;
always @(posedge clk_sys) if (kj_load) kj_loaded <= 1'b1;
wire [7:0] kj_q;
spram #(17, 8) kanji_rom
(
	.clock(clk_sys),
	.address(ioctl_download ? ioctl_addr[16:0] : kj_ptr),
	.data(ioctl_dout),
	.wren(kj_load && ioctl_addr[24:17] == 8'd0),
	.q(kj_q)
);
always @(posedge clk_sys) begin
	if (cpu_reset) begin
		pac_sel <= 0;
		rp_ptr[0] <= 0; rp_ptr[1] <= 0;
		kj_ptr <= 0;
	end
	else if (pac_wr) begin
		if (port[1:0] == 2'b11 && !cpu_dout[7] && !orig) pac_sel <= cpu_dout[2:0];
		else if (pac_s == 3'd4 || pac_s == 3'd3) begin
			if (port[1:0] == 2'b00) rp_ptr[pac_s == 3'd3][7:0]  <= cpu_dout;
			if (port[1:0] == 2'b01) rp_ptr[pac_s == 3'd3][14:8] <= cpu_dout[6:0];
		end
		else if (pac_s == 3'd2) begin
			if (port[1:0] == 2'b00) kj_ptr[7:0]  <= cpu_dout;
			if (port[1:0] == 2'b01) kj_ptr[15:8] <= cpu_dout;
			if (port[1:0] == 2'b10) kj_ptr[16]   <= cpu_dout[0];
		end
	end
end

`ifdef VERILATOR
localparam RAMPAC_INIT = "../rtl/rampac_blank.hex";   // the sim runs in verilator/
`else
localparam RAMPAC_INIT = "rtl/rampac_blank.hex";
`endif

// RAM PAC #1 (select 4) and #2 (select 3), each with an optional file on the SD card
genvar rpi;
generate for (rpi = 0; rpi < 2; rpi = rpi + 1) begin : rampac
	p7_rampac #(.INIT(RAMPAC_INIT), .HOLD(CLK_MUL * 1431818)) rp
	(
		.clk(clk_sys),
		.addr(rp_ptr[rpi]), .d(cpu_dout), .q(rp_q[rpi]),
		.we(pac_dev_wr && pac_s == (rpi ? 3'd3 : 3'd4) && port[1:0] == 2'b10),
		.img_mounted(img_mounted[2 + rpi]), .img_readonly(img_readonly), .img_size(img_size),
		.sd_lba(sd_lba[2 + rpi]), .sd_rd(sd_rd[2 + rpi]), .sd_wr(sd_wr[2 + rpi]), .sd_ack(sd_ack[2 + rpi]),
		.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din[2 + rpi]),
		.sd_buff_wr(sd_buff_wr),
		.loaded(rampac_loaded[rpi])
	);
end endgenerate

// Joy PAC: 19h joystick 2, 1Ah joystick 1; bits (active low) 0 up, 1 down, 2 left, 3 right, 4/5 buttons
wire [5:0] jp0 = {joy0[5:4], joy0[0], joy0[1], joy0[2], joy0[3]};
wire [5:0] jp1 = {joy1[5:4], joy1[0], joy1[1], joy1[2], joy1[3]};

reg [7:0] pac_dout;
always @(*) begin
	case (pac_s)
	3'd4: pac_dout = rp_q[0];
	3'd3: pac_dout = rp_q[1];
	3'd2: pac_dout = kj_loaded ? kj_q : 8'h00;
	3'd1: pac_dout = port[1:0] == 2'b01 ? {2'b11, ~jp1} : port[1:0] == 2'b10 ? {2'b11, ~jp0} : 8'hff;
	default: pac_dout = 8'hff;
	endcase
end

// Floppy (E0-E7). A read of the uPD765's data register moves it on, so it acts at the end of the access, with the
// port latched at its start.
reg  [7:0] rd_port;
always @(posedge clk_sys) if (io_rd_stb) rd_port <= port;
wire [7:0] fdd_dout;
p7_fdd #(.CLK_HZ(CLK_MUL * 14318182)) p7_fdd
(
	.clk(clk_sys),
	.reset(reset),
	.port(io_rd_end ? rd_port : port),
	.wr(io_wr_dev),
	.rd_end(io_rd_end && !mio_cyc),
	.din(cpu_dout),
	.dout(fdd_dout),
	.sel(),
	.motor(),
	.busy(disk_busy),
	.img_mounted(img_mounted[1:0]), .img_readonly(img_readonly), .img_size(img_size),
	.sd_lba(sd_lba[0:1]), .sd_rd(sd_rd[1:0]), .sd_wr(sd_wr[1:0]), .sd_ack(sd_ack[1:0]),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din[0:1]), .sd_buff_wr(sd_buff_wr),
	.disk_ready(disk_ready), .disk_too_big(disk_too_big),
	.ddr_busy(ddr_busy), .ddr_burstcnt(ddr_burstcnt), .ddr_addr(ddr_addr), .ddr_dout(ddr_dout),
	.ddr_dout_ready(ddr_dout_ready), .ddr_rd(ddr_rd), .ddr_din(ddr_din), .ddr_be(ddr_be), .ddr_we(ddr_we)
);

// I/O read
reg [7:0] io_dout;
always @(*) begin
	io_dout = 8'hff;
	if (mio_cyc) io_dout = ram_q;
	else casez (port)
	8'b0000_00??: io_dout = orig ? oppi0_dout : 8'hff;
	8'b0000_10??: io_dout = orig ? oppi1_dout : ppi0_dout;
	8'b0000_11??: io_dout = orig ? 8'hff : ppi1_dout;
	8'b0001_000?: io_dout = crtc_dout;
	8'b0001_10??: io_dout = pac_dout;
	8'b0010_00??: io_dout = ppi2_dout;
	8'b0010_10??: io_dout = ctc_dout;
	8'h30, 8'h31: io_dout = pio_dout;
	8'b1110_0???: io_dout = fdd_dout;
	default:      io_dout = 8'hff;
	endcase
end

// Debug: one event per I/O cycle (reads at their end, with the data the CPU took)
reg [7:0] io_dout_l;
always @(posedge clk_sys) if (io_rd) io_dout_l <= io_dout;
assign dbg_io_wr   = io_wr_stb;
assign dbg_io_rd   = io_rd_end;
assign dbg_io_port = A;
assign dbg_io_data = io_wr_stb ? cpu_dout : io_dout_l;
assign dbg_op      = fetch_byte;

//------------------------------------------------------------------------------------------------
// HD46505 and video (CSCP display.cpp, drawn from the CRTC raster)
//------------------------------------------------------------------------------------------------

wire        col80 = orig ? oppi1_pa[5] : mode[5];
reg   [3:0] dcnt = 0;                     // dot within the character: 0-7 (80 columns) or 0-15 (40)
reg   [5:0] lcnt = 0;                     // LCD: the HD46505 runs at 1.79 MHz / 8 (one character per 64 dots)
always @(posedge clk_sys) if (ce_dot) lcnt <= lcnt + 1'd1;
wire        ce_char = lcd ? ce_dot && lcnt == 6'd63 : ce_dot && dcnt == (col80 ? 4'd7 : 4'd15);
always @(posedge clk_sys) if (ce_dot) dcnt <= ce_char ? 4'd0 : dcnt + 1'd1;

wire [13:0] ma;
wire  [4:0] ra;

mc6845 crtc
(
	.CLOCK(clk_sys),
	.CLKEN(ce_char),
	.CLKEN_CPU(1'b1),
	.nRESET(~cpu_reset),
	.ENABLE(crtc_wr),
	.R_nW(1'b0),
	.RS(port[0]),
	.DI(cpu_dout),
	.DO(),
	.VSYNC(crtc_vs),
	.HSYNC(crtc_hs),
	.DE(crtc_de),
	.HDE(crtc_hde),
	.VDE(crtc_vde),
	.CURSOR(),
	.LPSTB(1'b0),
	.VGA(1'b0),
	.MA(ma),
	.RA(ra),
	.test()
);

// Fetch during character period N (addresses from MA/RA; the font one clock after the code), show in N+1
wire fine = mode[7];
wire [13:0] lcd_vaddr, paso_vaddr;
wire [10:0] lcd_font_addr;
always @(*) begin
	v_gfx_addr = orig ? paso_vaddr : lcd ? lcd_vaddr : {ma[10:0], ra[2:0]};
	v_txt_addr = orig ? paso_vaddr : lcd ? lcd_vaddr : {ma[10:0], fine ? ra[2:0] : text_page};
end

wire [7:0] font_q;
dpram #(12, 8) font
(
	.clock_a(clk_sys), .enable_a(1'b1),
	.wren_a(rom_load && ioctl_addr[24:12] == 13'h000c), .address_a(ioctl_addr[11:0]), .data_a(ioctl_dout), .q_a(),
	.clock_b(clk_sys), .enable_b(1'b1), .wren_b(1'b0), .address_b(lcd ? {1'b0, lcd_font_addr} : {1'b0, vg_v, ra[2:0]}), .data_b(8'h00), .q_b(font_q)
);

// cursor (CSCP: the character whose address is R14/R15, lines from R10 bits 2-0 down; R10 bits 6-5 blink with the
// frame count)
reg [4:0] frame_cnt;
always @(posedge clk_sys) if (crtc_vs & ~vs_d) frame_cnt <= frame_cnt + 1'd1;
wire cursor_on = crtc_r[10][6:5] == 2'b00 ? 1'b1 :
                 crtc_r[10][6:5] == 2'b10 ? frame_cnt[3] :
                 crtc_r[10][6:5] == 2'b11 ? frame_cnt[4] : 1'b0;
wire cursor_here = cursor_on && crtc_r[8][7:6] != 2'b11 &&
                   ma[10:0] == {crtc_r[14][2:0], crtc_r[15]} && ra[2:0] >= crtc_r[10][2:0];

reg [7:0] s_b, s_r, s_g, s_pat, s_attr;
reg       s_cursor, s_de, s_hde, s_vde, s_hs, s_vs;
always @(posedge clk_sys) begin
	if (ce_char) begin
		s_b      <= pal_dis ? 8'h00 : vb_v;
		s_r      <= pal_dis ? 8'h00 : vr_v;
		s_g      <= pal_dis ? 8'h00 : vg_v;
		s_pat    <= font_q;
		s_attr   <= va_v;
		s_cursor <= cursor_here;
		s_de     <= crtc_de;
		s_hde    <= crtc_hde;
		s_vde    <= crtc_vde;
		s_hs     <= crtc_hs;
		s_vs     <= crtc_vs;
	end
end

wire [2:0] bitn   = 3'd7 - (col80 ? dcnt[2:0] : dcnt[3:1]);
wire       odd    = dcnt[0];
wire       pb_    = s_b[bitn];
wire       pr_    = s_r[bitn];
wire       pg_    = s_g[bitn];
wire [3:0] pal_g3 = pal[{odd, pg_, pr_, pb_}];   // fine graphics dot mode
wire [3:0] pal_t  = pal[{odd, 1'b0, pr_, pb_}];  // under text
wire       dotmode = fine & s_attr[3];
reg        tbit;
always @(*) begin
	tbit = s_pat[bitn];
	if (!fine && s_attr[3]) tbit = ~tbit;
	if (mode[3] && !s_attr[2] && blink) tbit = 1'b0;
end
wire [2:0] c_t = mode[3] ? mode[2:0] : s_attr[2:0];
wire [2:0] dot = !s_de    ? 3'd0 :
                 s_cursor ? 3'd7 :
                 dotmode  ? pal_g3[2:0] :
                 pal_t[3] ? pal_t[2:0] : tbit ? c_t : pal_t[2:0];

wire [7:0] lcd_r, lcd_g, lcd_b;
wire       lcd_hs, lcd_vs, lcd_hb, lcd_vb;
p7_lcd p7_lcd
(
	.clk(clk_sys), .ce_dot(ce_dot),
	.r8(crtc_r[8]), .r10(crtc_r[10]), .r12(crtc_r[12]), .r13(crtc_r[13]), .r14(crtc_r[14]), .r15(crtc_r[15]),
	.mode(mode), .pal(pal), .blink(blink), .cursor_on(cursor_on),
	.vaddr(lcd_vaddr), .vb(vb_v), .vr(vr_v), .vg(vg_v), .va(va_v),
	.font_addr(lcd_font_addr), .font_q(font_q),
	.R(lcd_r), .G(lcd_g), .B(lcd_b),
	.HSync(lcd_hs), .VSync(lcd_vs), .HBlank(lcd_hb), .VBlank(lcd_vb)
);

// the original Pasopia's display and font (boot1.rom 8000-87FF)
wire [10:0] paso_font_addr;
wire  [7:0] paso_font_q;
wire  [2:0] paso_dot;
wire        paso_hs, paso_vs, paso_hb, paso_vb;
dpram #(11, 8) paso_font
(
	.clock_a(clk_sys), .enable_a(1'b1),
	.wren_a(orom_load && ioctl_addr[24:11] == 14'h0010), .address_a(ioctl_addr[10:0]), .data_a(ioctl_dout), .q_a(),
	.clock_b(clk_sys), .enable_b(1'b1), .wren_b(1'b0), .address_b(paso_font_addr), .data_b(8'h00), .q_b(paso_font_q)
);
paso_video paso_video
(
	.clk(clk_sys), .ce_dot(ce_dot),
	.r1(crtc_r[1]), .r8(crtc_r[8]), .r10(crtc_r[10]), .r12(crtc_r[12]), .r13(crtc_r[13]), .r14(crtc_r[14]),
	.r15(crtc_r[15]), .mode(oppi1_pa), .blink8(frame_cnt[3]), .blink16(frame_cnt[4]),
	.vaddr(paso_vaddr), .vdata(vb_v), .vattr(va_v[0]),
	.font_addr(paso_font_addr), .font_q(paso_font_q),
	.dot(paso_dot), .HSync(paso_hs), .VSync(paso_vs), .HBlank(paso_hb), .VBlank(paso_vb)
);

assign ce_pix = ce_dot;
always @(posedge clk_sys) begin
	if (orig) begin
		// digital RGB: bit 0 blue, bit 1 red, bit 2 green
		R <= {8{paso_dot[1]}};
		G <= {8{paso_dot[2]}};
		B <= {8{paso_dot[0]}};
		HBlank <= paso_hb;
		VBlank <= paso_vb;
		HSync  <= paso_hs;
		VSync  <= paso_vs;
	end
	else if (lcd) begin
		R <= lcd_r;
		G <= lcd_g;
		B <= lcd_b;
		HBlank <= lcd_hb;
		VBlank <= lcd_vb;
		HSync  <= lcd_hs;
		VSync  <= lcd_vs;
	end
	else begin
		// digital RGB: bit 0 blue, bit 1 red, bit 2 green
		R <= {8{dot[1]}};
		G <= {8{dot[2]}};
		B <= {8{dot[0]}};
		HBlank <= ~s_hde;
		VBlank <= ~s_vde;
		HSync  <= s_hs;
		VSync  <= s_vs;
	end
end

//------------------------------------------------------------------------------------------------
// Sound: two SN76489AN at 3Ah/3Bh, the beeper (CTC ZC1 / 2, gated by PIO A bit 7); 8255-2 A bit 1 mutes all
//------------------------------------------------------------------------------------------------

wire [7:0] psg0_out, psg1_out;
SN76496 psg0
(
	.clk(clk_sys), .clken(ce_psg), .reset(cpu_reset), .ce(1'b1),
	.we(io_wr_dev && !orig && port == 8'h3a), .data(cpu_dout), .chmsk(4'hf),
	.sndout(psg0_out), .chactv(), .lreg()
);
SN76496 psg1
(
	.clk(clk_sys), .clken(ce_psg), .reset(cpu_reset), .ce(1'b1),
	.we(io_wr_dev && !orig && port == 8'h3b), .data(cpu_dout), .chmsk(4'hf),
	.sndout(psg1_out), .chactv(), .lreg()
);

wire       beep = beep_ff & pio_pa[7];
wire [15:0] mix = {3'b000, psg0_out, 5'd0} + {3'b000, psg1_out, 5'd0} + (beep ? 16'h1800 : 16'h0000);
// the cassette monitor (OSD Tape audio) plays the tape input while the motor runs, whatever the mute bit says
wire [15:0] tape_snd = tape_audio && tape_motor && tape_in ? 16'h0c00 : 16'h0000;
assign audio = (snd_mute ? 16'd0 : mix) + tape_snd;

endmodule
