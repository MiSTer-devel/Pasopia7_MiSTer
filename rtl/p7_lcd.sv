// Pasopia 7 LCD model (PA7170): the 320 x 64 liquid crystal display, drawn from VRAM as CSCP display.cpp
// draw_text_lcd / draw_fine_lcd do. The BIOS sees 8255-0 port B bit 4 = 0 and programs the HD46505 for the LCD
// (R0 5Dh, R1 50h, R4 3, R6 4, R9 7: 32 lines of 80 characters at 1.79 MHz / 8, 74.38 Hz). The panel is driven in
// two halves: row y (0-7) of 40 characters starts at R12/R13 + 640 * (y & 3), + 8 for the lower half (y >= 4),
// and the characters of a row are 16 bytes apart (the even ones of the CRTC's line on top, the odd ones below).
// The text code is the G byte at the start of the cell (no text page), the attribute too.
//
// It makes its own 15.7 kHz picture: 912 dots x 262 lines at 14.318 MHz, 640 x 64 active (each LCD dot 2 dots wide,
// one line tall: nearly square, 6:7). Colours: CSCP's, a light panel (dot colour 0) and dark dots (1-7).
module p7_lcd
(
	input             clk,
	input             ce_dot,          // 14.31818 MHz

	input       [7:0] r8, r10, r12, r13, r14, r15,
	input       [7:0] mode,            // 8255-0 port A: bit 7 fine graphics, bit 3 one text colour (bits 2-0)
	input       [3:0] pal[16],
	input             blink,           // 8255-1 port C bit 5
	input             cursor_on,       // the cursor's blink phase (R10 bits 6-5 with the frame count)

	// VRAM (all four planes at vaddr, one clock of latency) and the font ROM
	output reg [13:0] vaddr,
	input       [7:0] vb, vr, vg, va,
	output reg [10:0] font_addr,
	input       [7:0] font_q,

	output reg  [7:0] R,
	output reg  [7:0] G,
	output reg  [7:0] B,
	output reg        HSync,
	output reg        VSync,
	output reg        HBlank,
	output reg        VBlank
);

localparam [8:0] V_ACT = 9'd100;       // the first active line (64 of them, about the middle of the screen)

reg  [9:0] hcnt = 0;
reg  [8:0] vcnt = 0;
always @(posedge clk) if (ce_dot) begin
	if (hcnt == 10'd911) begin
		hcnt <= 0;
		vcnt <= vcnt == 9'd261 ? 9'd0 : vcnt + 1'd1;
	end
	else hcnt <= hcnt + 1'd1;
end

wire [8:0] lyw = vcnt - V_ACT;
wire       vact = vcnt >= V_ACT && vcnt < V_ACT + 9'd64;
wire [5:0] ly = lyw[5:0];
wire [2:0] yy = ly[5:3];
wire [2:0] l  = ly[2:0];

// the cell of character x (hcnt[9:4]) on this line
wire [13:0] base = {r12[2:0], r13, 3'b000};
wire [13:0] src_g = base + 14'd640 * yy[1:0] + (yy[2] ? 14'd8 : 14'd0) + {4'd0, hcnt[9:4], 4'd0};
wire [13:0] cursor = {r14[2:0], r15, 3'b000};

// fetch character x during dots 16x..16x+15 (the code and attribute, then the line's planes and font), show it
// during 16x+16..16x+31
reg  [2:0] st = 0;
reg  [7:0] n_b, n_r, n_g, n_pat, n_attr, code;
reg        n_cur, cur_hit;
always @(posedge clk) begin
	if (st != 0) st <= st + 1'd1;
	case (st)
	3'd3: begin
		code    <= vg;
		n_attr  <= va;
		vaddr   <= src_g | {11'd0, l};
		font_addr <= {vg, l};
	end
	3'd6: begin
		n_b   <= vb;
		n_r   <= vr;
		n_g   <= vg;
		n_pat <= font_q;
		n_cur <= cur_hit && l >= r10[2:0];
		st    <= 0;
	end
	default: ;
	endcase
	if (ce_dot && hcnt[3:0] == 4'd0 && hcnt < 10'd640) begin
		vaddr   <= src_g;
		cur_hit <= cursor_on && r8[7:6] != 2'b11 && src_g == cursor;
		st      <= 3'd1;
	end
end

// the dots
reg  [7:0] s_b, s_r, s_g, s_pat, s_attr;
reg        s_cur;
always @(posedge clk) if (ce_dot && hcnt[3:0] == 4'd15) begin
	s_b <= n_b; s_r <= n_r; s_g <= n_g; s_pat <= n_pat; s_attr <= n_attr; s_cur <= n_cur;
end

wire [9:0] hx   = hcnt - 10'd16;
wire       hact = hcnt >= 10'd16 && hcnt < 10'd656;
wire [2:0] bitn = 3'd7 - hx[3:1];
wire       odd  = hx[1];
wire       fine = mode[7];
wire       pb_  = s_b[bitn], pr_ = s_r[bitn], pg_ = s_g[bitn];
wire [3:0] pal_g3 = pal[{odd, pg_, pr_, pb_}];
wire [3:0] pal_t  = pal[{odd, 1'b0, pr_, pb_}];
reg        tbit;
always @(*) begin
	tbit = s_pat[bitn];
	if (!fine && s_attr[3]) tbit = ~tbit;
	if (mode[3] && !s_attr[2] && blink) tbit = 1'b0;
end
wire [2:0] c_t = mode[3] ? mode[2:0] : s_attr[2:0];
wire [2:0] dot = r8[5:4] == 2'b11     ? 3'd0 :
                 s_cur                ? 3'd7 :
                 fine && s_attr[3]    ? pal_g3[2:0] :
                 pal_t[3] ? pal_t[2:0] : tbit ? c_t : pal_t[2:0];

always @(posedge clk) if (ce_dot) begin
	{R, G, B} <= dot == 3'd0 ? {8'd160, 8'd168, 8'd160} : {8'd48, 8'd56, 8'd16};
	HBlank <= ~hact;
	VBlank <= ~vact;
	HSync  <= hcnt >= 10'd720 && hcnt < 10'd787;
	VSync  <= vcnt >= 9'd230 && vcnt < 9'd233;
end

endmodule
