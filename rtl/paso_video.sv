// Original Pasopia (PA7010) video, as CSCP src/vm/pasopia/display.cpp draws it.
//
// VRAM: 16 KiB of bytes plus one attribute bit per byte (1 = graphics), eight 2 KiB line planes: line l (0-7) of a
// character cell is at plane l (+ l * 800h). The HD46505's R12/R13 give the start (bits 10-0), R1 the characters per
// row: the first byte of each row is not shown (it sets the row's colour), the R1 - 1 after it are. Rows follow on
// in VRAM. 25 rows of 8 lines (80 columns, mode bit 5 = 1) or 24 rows (40 columns: each dot twice).
// Mode (8255-1 port A): bits 7-6 screen 0 (text), 1 (text, or 2 x 4 coloured blocks per graphics byte, its colours in
// bits 6-4 and 2-0; the lines in pairs), 2 (text, or graphics bytes drawn as dots in the text colour), 3 ("1.5":
// screen 1 with every line its own); bit 5 80 columns; bits 2-0 the background colour.
// Text colour: white at the start of each frame; a text byte F8-FF (an "attribute character", drawn too) sets it to
// its bits 2-0 for the rest of the line (screen 0: of the row, from the row's first byte or white). In screens 1-3
// each of the 8 lines of a row keeps its own colour from row to row.
// Cursor (R14/R15, R10): only the blinking forms show.
//
// The picture: 912 dots x 262 lines at 14.318 MHz, 640 x 200 active (as CSCP draws it, whatever the CRTC's sync
// timing), from line V_ACT.
module paso_video
(
	input             clk,
	input             ce_dot,          // 14.31818 MHz

	input       [7:0] r1, r8, r10, r12, r13, r14, r15,
	input       [7:0] mode,
	input             blink8,          // the frame count's bits 3 / 4 (cursor blink)
	input             blink16,

	output reg [13:0] vaddr,
	input       [7:0] vdata,
	input             vattr,
	output reg [10:0] font_addr,
	input       [7:0] font_q,

	output reg  [2:0] dot,             // {G, R, B}
	output reg        HSync,
	output reg        VSync,
	output reg        HBlank,
	output reg        VBlank
);

localparam [8:0] V_ACT = 9'd30;

reg  [9:0] hcnt = 0;
reg  [8:0] vcnt = 0;
always @(posedge clk) if (ce_dot) begin
	if (hcnt == 10'd911) begin
		hcnt <= 0;
		vcnt <= vcnt == 9'd261 ? 9'd0 : vcnt + 1'd1;
	end
	else hcnt <= hcnt + 1'd1;
end

wire       col80 = mode[5];
wire [1:0] scr   = mode[7:6];
wire [2:0] c_b   = mode[2:0];
wire [8:0] yw    = vcnt - V_ACT;
wire       vact  = vcnt >= V_ACT && vcnt < V_ACT + 9'd200;
wire [4:0] row   = yw[7:3];
wire [2:0] l     = yw[2:0];
wire       row_ok = col80 ? row < 5'd25 : row < 5'd24;
wire [2:0] lsel  = scr == 2'd0 ? 3'd0 : scr == 2'd1 ? {l[2:1], 1'b0} : l;

// fetch slots of one character (8 dots, 16 at 40 columns) from hcnt = org: slot 0 the row's first byte, slot s the
// character s - 1, shown from hcnt 32 + (s - 1) * width
wire [4:0] cw   = col80 ? 5'd8 : 5'd16;
wire [9:0] org  = col80 ? 10'd16 : 10'd0;
wire [9:0] hrel = hcnt - org;
wire [6:0] slot = col80 ? hrel[9:3] : {1'b0, hrel[9:4]};
wire       slot_start = col80 ? hrel[2:0] == 3'd0 : hrel[3:0] == 4'd0;
wire       slot_end   = col80 ? hrel[2:0] == 3'd7 : hrel[3:0] == 4'd15;
wire [7:0] width = r1 - 8'd1;
wire       in_slots = hcnt >= org && {1'b0, slot} <= width && slot <= (col80 ? 7'd80 : 7'd40);   // what fits in 640 dots

wire [10:0] row_base = {r12[2:0], r13} + row * r1;
wire [10:0] a = row_base + {4'd0, slot};
wire [10:0] cursor = {r14[2:0], r15};

reg  [2:0] st = 0;
reg  [7:0] n_code, n_pat;
reg        n_attr, n_cur, cur_hit;
always @(posedge clk) begin
	if (st != 0) st <= st + 1'd1;
	case (st)
	3'd2: begin
		n_code    <= vdata;
		n_attr    <= vattr;
		font_addr <= {vdata, l};
	end
	3'd4: begin
		n_pat <= font_q;
		n_cur <= cur_hit && l >= r10[2:0];
		st    <= 0;
	end
	default: ;
	endcase
	if (ce_dot && slot_start && in_slots) begin
		vaddr   <= {lsel, a};
		cur_hit <= a == cursor && ((r10[6:5] == 2'b10 && blink8) || (r10[6:5] == 2'b11 && blink16));
		st      <= 3'd1;
	end
end

// the text colour: per line of the row (c_line), the current one (c_cur)
wire       is_attr = n_code[7:3] == 5'b11111;
reg  [2:0] c_line[8];
reg  [2:0] c_cur;
reg  [7:0] s_code, s_pat;
reg        s_attr, s_cur, s_valid;
always @(posedge clk) if (ce_dot) begin
	if (vcnt == 0 && hcnt == 0) for (int i = 0; i < 8; i++) c_line[i] <= 3'd7;
	if (slot_end && in_slots && vact && row_ok) begin
		if (slot == 0) begin
			s_valid <= 1'b0;
			if (scr == 2'd0) c_cur <= is_attr ? n_code[2:0] : 3'd7;
			else c_cur <= is_attr ? n_code[2:0] : c_line[l];
		end
		else begin
			s_valid <= 1'b1;
			s_code <= n_code; s_pat <= n_pat; s_attr <= n_attr; s_cur <= n_cur;
			if (is_attr && (scr == 2'd0 || !n_attr)) c_cur <= n_code[2:0];
		end
	end
	else if (slot_end) s_valid <= 1'b0;
	if (hcnt == 10'd700 && vact && row_ok) c_line[l] <= c_cur;
end

wire [9:0] hx   = hcnt - 10'd32;
wire       hact = hcnt >= 10'd32 && hcnt < 10'd672;
wire [2:0] bitn = 3'd7 - (col80 ? hx[2:0] : hx[3:1]);
wire       left = bitn[2];                  // bits 7-4: the left block
reg  [2:0] c;
always @(*) begin
	c = c_b;
	if (s_valid && vact && row_ok) begin
		case (scr)
		2'd0: c = s_pat[bitn] ? c_cur : c_b;
		2'd2: c = (s_attr ? s_code[bitn] : s_pat[bitn]) ? c_cur : c_b;
		default:
			if (s_attr) begin
				if (left) c = |s_code[7:4] ? s_code[6:4] : c_b;
				else      c = |s_code[3:0] ? s_code[2:0] : c_b;
			end
			else c = s_pat[bitn] ? c_cur : c_b;
		endcase
		if (s_cur) c = 3'd7;
	end
	if (r8[5:4] == 2'b11) c = 3'd0;
end

always @(posedge clk) if (ce_dot) begin
	// CSCP's colour bits: 0 blue, 1 red, 2 green
	dot    <= {c[2], c[1], c[0]};
	HBlank <= ~hact;
	VBlank <= ~vact;
	HSync  <= hcnt >= 10'd720 && hcnt < 10'd787;
	VSync  <= vcnt >= 9'd240 && vcnt < 9'd243;
end

endmodule
