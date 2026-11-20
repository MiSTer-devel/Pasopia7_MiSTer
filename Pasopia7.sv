//============================================================================
//  Toshiba Pasopia 7 for MiSTer
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

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Default values for ports not used in this core /////////

assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
assign DDRAM_CLK = clk_sys;                // the floppy images (rtl/p7_fdd.sv)

assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 1;                       // signed
assign AUDIO_MIX = 0;

assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

// Layout: ~/dev2/osd-standard.md, video-standard.md, tape-standard.md. boot.rom (index 0, games/Pasopia7/boot.rom):
// the MAME pasopia7 set's bios.rom (16 KiB), basic.rom (32 KiB) and 2732.ic146 (4 KiB font) in that order; Load ROM
// takes the same layout. Display waits for Apply and reset (cfg); Aspect ratio, Scale and Scandoubler Fx act at once
// (the monitor, not the machine); Input acts at once. Media: S0/S1 floppy drives 1 and 2 (.d88, removable: not
// remembered; the BIOS boots from drive 1), S2 the tape (CAS or WAV, rewound at every mount), SC3/SC4 RAM PAC #1 and
// #2 (32 KiB files, battery-backed storage: remembered, written back as BASIC changes them). Tape speed, Tape
// overlay, Tape audio and Rewind tape act at once.
`include "build_id.v"
localparam CONF_STR = {
	"Pasopia7;;",
	"-;",
	"S0,D88,Pri. FDD 1;",
	"S1,D88,FDD 2;",
	"S2,CASWAV,Tape;",
	"SC3,BIN,RAM PAC 1;",
	"SC4,BIN,RAM PAC 2;",
	"F0,ROM,Load ROM;",
	"F1,ROM,Load Pasopia ROM;",
	"F2,ROM,Load Kanji ROM;",
	"-;",
	"P1,Machine;",
	"P2,Video;",
	"P3,Input;",
	"P4,Audio;",
	"-;",
	"h0-,Changes pending;",
	"R[0],Apply and reset;",

	"P1O[12:11],Model,Pasopia 7,Pasopia 7 LCD,Pasopia;",
	"P1-;",
	"P1O[7],Tape speed,5x,Normal;",
	"P1O[13],Tape shaper,Off,On;",
	"P1-;",
	"P1T[10],Rewind tape;",

	"P2O[6:5],Display,Color,B&W,Green,Amber;",
	"P2-;",
	"P2O[20:19],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P2O[22:21],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"P2O[25:23],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"P2O[8],Tape overlay,On,Off;",

	"P3O[31],Keyboard symbols,US,By position (JP);",
	"P3O[30],Swap Alt/Win keys,No,Yes;",

	"P4O[9],Tape audio,Off,On;",

	// shown for 2 s at a tape mount (Main's Info box): 1 ready, 2 rejected
	"I,Tape ready (rewound)\nIn T-BASIC7 type CLOAD\nthen RUN,",
	"Tape format not supported;",

	"J,Button 1,Button 2;",
	"v,0;",
	"V,v",`BUILD_DATE
};

wire        forced_scandoubler;
wire [21:0] gamma_bus;
wire  [1:0] buttons;
wire [127:0] status;
wire [10:0] ps2_key;
wire [31:0] joystick_0, joystick_1;

wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;

wire        pending;
reg         info_req = 0;
reg   [7:0] info = 0;

// image slots, 512-byte blocks: S0/S1 floppy drives, S2 tape, S3/S4 RAM PACs
wire  [4:0] img_mounted;
wire        img_readonly;
wire [63:0] img_size;
wire [31:0] sd_lba[5];
wire  [4:0] sd_rd, sd_wr, sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din[5];
wire        sd_buff_wr;
wire  [5:0] sd_blk_cnt[5] = '{6'd0, 6'd0, 6'd0, 6'd0, 6'd0};

hps_io #(.CONF_STR(CONF_STR), .VDNUM(5)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),
	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask({15'd0, pending}),
	.info_req(info_req),
	.info(info),

	.ps2_key(ps2_key),
	.joystick_0(joystick_0),
	.joystick_1(joystick_1),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(1'b0),

	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),
	.sd_lba(sd_lba),
	.sd_blk_cnt(sd_blk_cnt),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr)
);

///////////////////////   CLOCKS   ///////////////////////////////

// clk_sys = 57.272727 MHz = 4 x the Pasopia 7's 14.31818 MHz dot clock (rtl/pasopia7.sv CLK_MUL = 4); the CPU's
// 3.9936 MHz comes from an exact fractional enable. The PLL is TK2000_MiSTer's (outclk_0 57.27 MHz).
wire clk_sys;
wire pll_locked;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(),
	.outclk_2(),
	.locked(pll_locked)
);

wire reset = RESET | status[0] | buttons[1] | ~pll_locked;

// the settings the machine runs with, taken from the OSD at every reset (osd-standard.md rule 1)
reg [127:0] cfg;
always @(posedge clk_sys) if (reset) cfg <= status;
assign pending = status[6:5] != cfg[6:5] || status[12:11] != cfg[12:11];
wire   lcd  = cfg[12:11] == 2'd1;         // the Pasopia 7 LCD model (PA7170)
wire   orig = cfg[12:11] == 2'd2;         // the original Pasopia (PA7010): its ROMs from boot1.rom

///////////////////////   INPUT   ///////////////////////////////

// Swap Alt/Win keys: the key next to the space bar sends the other one (GRAPH is left Alt, KANA right Alt)
reg [10:0] ps2_key_m;
always @(*) begin
	ps2_key_m = ps2_key;
	if (status[30])
		case (ps2_key[8:0])
		9'h011: ps2_key_m[8:0] = 9'h11f;  // left Alt -> left Win
		9'h11f: ps2_key_m[8:0] = 9'h011;
		9'h111: ps2_key_m[8:0] = 9'h127;  // right Alt -> right Win
		9'h127: ps2_key_m[8:0] = 9'h111;
		default: ;
		endcase
end

///////////////////////   CASSETTE   ///////////////////////////////

// A CAS or WAV on S2 (rtl/p7_tape.sv), else line-in through the ADC (as the other MiSTer computer cores).
wire tape_adc, tape_adc_act;
ltc2308_tape #(.CLK_RATE(57272727)) ltc2308_tape
(
	.reset(reset),
	.clk(clk_sys),
	.ADC_BUS(ADC_BUS),
	.dout(tape_adc),
	.active(tape_adc_act),
	.adc_sync(),
	.adc_data()
);

wire        tape_motor, tape_level, tape_loaded, tape_playing, tape_turbo, tape_ready, tape_bad, tape_underflow;
wire [17:0] tape_rate;
wire [31:0] tape_pos, tape_len;
reg         rewind_d;
always @(posedge clk_sys) rewind_d <= status[10];

p7_tape #(.CLK_HZ(57272727), .TURBO(5)) p7_tape
(
	.clk(clk_sys),
	.rewind(status[10] & ~rewind_d),
	.motor(tape_motor),
	.turbo_req(~status[7]),
	.shape(status[13]),
	.img_mounted(img_mounted[2]),
	.img_size(img_size),
	.sd_lba(sd_lba[2]),
	.sd_rd(sd_rd[2]),
	.sd_ack(sd_ack[2]),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_wr(sd_buff_wr),
	.level(tape_level),
	.loaded(tape_loaded),
	.playing(tape_playing),
	.turbo(tape_turbo),
	.ready(tape_ready),
	.bad(tape_bad),
	.underflow(tape_underflow),
	.rate(tape_rate),
	.position(tape_pos),
	.length(tape_len),
	.cas()
);
assign sd_wr[2] = 1'b0;
assign sd_buff_din[2] = 8'd0;

always @(posedge clk_sys) begin
	info_req <= tape_ready | tape_bad;
	if (tape_ready | tape_bad) info <= tape_bad ? 8'd2 : 8'd1;
end

wire tape_in = tape_loaded ? tape_level : tape_adc;

///////////////////////   MACHINE   ///////////////////////////////

// the machine's image slots 0-3 are hps_io's 0, 1, 3, 4 (2 is the tape)
wire [31:0] m_lba[4];
wire  [7:0] m_buff_din[4];
assign sd_lba[0] = m_lba[0];
assign sd_lba[1] = m_lba[1];
assign sd_lba[3] = m_lba[2];
assign sd_lba[4] = m_lba[3];
assign sd_buff_din[0] = m_buff_din[0];
assign sd_buff_din[1] = m_buff_din[1];
assign sd_buff_din[3] = m_buff_din[2];
assign sd_buff_din[4] = m_buff_din[3];
wire disk_busy;
assign LED_DISK = {1'b0, disk_busy};

wire HBlank, HSync, VBlank, VSync, ce_pix;
wire [7:0] mr, mg, mb;
wire [15:0] audio;

pasopia7 #(.CLK_MUL(4), .TURBO(5)) pasopia7
(
	.clk_sys(clk_sys),
	.reset(reset),
	.ps2_key(ps2_key_m),
	.joy0(joystick_0[5:0]),
	.joy1(joystick_1[5:0]),

	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index[7:0]),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr[24:0]),
	.ioctl_dout(ioctl_dout),

	.ce_pix(ce_pix),
	.R(mr), .G(mg), .B(mb),
	.HSync(HSync), .VSync(VSync),
	.HBlank(HBlank), .VBlank(VBlank),

	.audio(audio),

	.tape_in(tape_in),
	.tape_motor(tape_motor),
	.tape_out(),
	.turbo(tape_turbo),
	.tape_audio(status[9]),
	.lcd(lcd),
	.orig(orig),
	.kbd_us(~status[31]),

	.img_mounted({img_mounted[4:3], img_mounted[1:0]}),
	.img_readonly(img_readonly),
	.img_size(img_size),
	.sd_lba(m_lba),
	.sd_rd({sd_rd[4:3], sd_rd[1:0]}),
	.sd_wr({sd_wr[4:3], sd_wr[1:0]}),
	.sd_ack({sd_ack[4:3], sd_ack[1:0]}),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(m_buff_din),
	.sd_buff_wr(sd_buff_wr),
	.rampac_loaded(),
	.disk_ready(),
	.disk_too_big(),
	.disk_busy(disk_busy),

	.ddr_busy(DDRAM_BUSY),
	.ddr_burstcnt(DDRAM_BURSTCNT),
	.ddr_addr(DDRAM_ADDR),
	.ddr_dout(DDRAM_DOUT),
	.ddr_dout_ready(DDRAM_DOUT_READY),
	.ddr_rd(DDRAM_RD),
	.ddr_din(DDRAM_DIN),
	.ddr_be(DDRAM_BE),
	.ddr_we(DDRAM_WE),

	.cpu_addr(), .cpu_ce(), .cpu_m1_n(),
	.dbg_io_wr(), .dbg_io_rd(), .dbg_io_port(), .dbg_io_data(), .dbg_op()
);

assign AUDIO_L = audio;
assign AUDIO_R = audio;
assign LED_USER = ioctl_download | tape_playing;

///////////////////////   VIDEO   ///////////////////////////////

// The machine makes 15.7 kHz video: 640 x 200 active (40 columns repeat each dot), dots at 14.31818 MHz (ce_pix),
// 912 dots x 262 lines (59.92 Hz); the LCD model 640 x 64 in the same frame (rtl/p7_lcd.sv). Display: the digital
// RGB on a colour monitor, or its brightness on a monochrome one (luma of the 8 colours: 0.30 R + 0.59 G + 0.11 B).
wire [2:0] rgb = {mg[7], mr[7], mb[7]};
reg  [7:0] luma;
always @(*) begin
	case (rgb)                            // {G, R, B}
	3'd0: luma = 8'd0;    3'd1: luma = 8'd29;   3'd2: luma = 8'd76;   3'd3: luma = 8'd105;
	3'd4: luma = 8'd150;  3'd5: luma = 8'd179;  3'd6: luma = 8'd226;  3'd7: luma = 8'd255;
	endcase
end
wire [1:0] disp = cfg[6:5];
// the LCD model makes its own panel colours
wire [7:0] dr = lcd || disp == 2'd0 ? mr : disp == 2'd3 ? luma : disp == 2'd2 ? 8'd0 : luma;
wire [7:0] dg = lcd || disp == 2'd0 ? mg : disp == 2'd3 ? {1'b0, luma[7:1]} + {2'b00, luma[7:2]} : luma;
wire [7:0] db = lcd || disp == 2'd0 ? mb : disp == 2'd1 ? luma : 8'd0;

// the tape status bar over the picture, before the scaler (tape-standard.md rules 19-24)
wire [7:0] R, G, B;
p7_tape_overlay #(.TURBO(5)) p7_tape_overlay
(
	.clk(clk_sys),
	.loaded(tape_loaded), .playing(tape_playing), .fast(tape_turbo), .underflow(tape_underflow),
	.rate(tape_rate), .position(tape_pos), .length(tape_len),
	.enable(~status[8]),
	.ce_pix(ce_pix), .hblank(HBlank), .vblank(VBlank),
	.r_in(dr), .g_in(dg), .b_in(db),
	.r_out(R), .g_out(G), .b_out(B)
);

// video_mixer: scandoubler for 31 kHz VGA (Scandoubler Fx, or forced by the ini), HQ2x / CRT scanlines, gamma.
// video_freak: aspect ratio and integer scaling for the HDMI scaler. Aspect ratio, Scale and Scandoubler Fx act at
// once and stay out of "Changes pending" (~/dev2/video-standard.md).
wire  [1:0] ar  = status[20:19];
wire  [2:0] sfx = status[25:23];          // 0 None, 1 HQ2x, 2-4 CRT 25/50/75%
wire  [2:0] sl  = sfx ? sfx - 3'd1 : 3'd0;
wire        scandoubler = sfx != 3'd0 || forced_scandoubler;
assign VGA_SL = sl[1:0];
assign CLK_VIDEO = clk_sys;

video_mixer #(.LINE_LENGTH(660), .GAMMA(1)) video_mixer
(
	.*,
	.hq2x(sfx == 3'd1),
	.freeze_sync()
);

video_freak video_freak
(
	.*,
	.VGA_DE_IN(VGA_DE),
	.VGA_DE(),
	// Original: the real shape of the 640 x 200 picture. 14.318 MHz dots (4 fsc) have a pixel aspect of 3/7
	// (square-pixel rate 6.136 MHz / 14.318 MHz), so 640*3 : 200*7 = 1920:1400 = 48:35 (1.37:1), as the PC-8001.
	// The LCD model: 320 x 64 dots, taken as square (5:1).
	.ARX((!ar) ? (lcd ? 12'd5 : 12'd48) : (ar - 1'd1)),
	.ARY((!ar) ? (lcd ? 12'd1 : 12'd35) : 12'd0),
	.CROP_SIZE(12'd0),                     // 200 lines: 5x = 1000 already fits 1080
	.CROP_OFF(5'd0),
	.SCALE({1'b0, status[22:21]})
);

endmodule
