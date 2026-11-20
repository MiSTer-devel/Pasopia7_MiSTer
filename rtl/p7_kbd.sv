// Pasopia 7 keyboard: a 12 x 8 key matrix behind the Z80 PIO.
//
// PIO port A selects (bits 6-4: groups of four rows, rows 0-3 / 4-7 / 8-11; bits 3-0: the rows in a group; several
// may be selected at once), port B reads the return lines, active low. The PS/2 to matrix table is generated from
// CSCP's key_map (tools/gen_kbd_table.py -> p7_kbd_table.svh), including CSCP's folded keys (Insert = SHIFT + DEL,
// Down = SHIFT + UP, Tab = SHIFT + ESC, Backspace = DEL). The original Pasopia (orig) has its own table
// (paso_kbd_table.svh: arrows, Insert, Home on other positions; ESC on row 3 bits 1 and 3).
//
// us: a US keyboard types the symbols printed on it. The 14 keys whose symbols differ from the JIS layout press the
// Pasopia key that has the symbol, with SHIFT forced on or off while they are held (Shift+2 '@' -> the @ key, ' ->
// Shift+7, Shift+' '"' -> Shift+2, = -> Shift+-, \ -> the yen key, ...). The choice is made at the press, from the
// Shift keys held then. Otherwise keys map by position.

module p7_kbd
(
	input            clk,
	input            reset,
	input     [10:0] ps2_key,         // hps_io: [10] toggles per event, [9] pressed, [8] E0, [7:0] code
	input            orig,            // the original Pasopia's matrix
	input            us,              // US symbol mapping
	input      [7:0] sel,             // PIO port A
	output     [7:0] ret_n            // to PIO port B
);

`include "p7_kbd_table.svh"
`include "paso_kbd_table.svh"

localparam SHIFT_ROW = 0, SHIFT_BIT = 1;

reg  [511:0] down;                    // PS/2 {E0, code} held
reg    [1:0] cnt[96];                 // keys held on each matrix position
reg    [2:0] extra_shift;             // folded keys held that add SHIFT
reg          toggle_d;

wire   [8:0] code  = ps2_key[8:0];

// US symbol keys: {index, JIS code and SHIFT unshifted, JIS code and SHIFT shifted} (JIS codes: the PS/2 code of the
// JP keyboard's key in that position: 54 @, 55 ^, 52 :, 4C ;, 4E -, 5B [, 5D ], 6A yen, 51 ro)
reg  [3:0] us_idx;
reg        us_hit;
reg  [8:0] us_t0, us_t1;
reg        us_f0, us_f1;
always @(*) begin
	us_hit = 1'b1; us_idx = 4'd0;
	us_t0 = code; us_f0 = 1'b0; us_t1 = code; us_f1 = 1'b1;
	case (code)
	9'h01e: begin us_idx = 4'd0;  us_t1 = 9'h054; us_f1 = 1'b0; end   // 2 @
	9'h036: begin us_idx = 4'd1;  us_t1 = 9'h055; us_f1 = 1'b0; end   // 6 ^
	9'h03d: begin us_idx = 4'd2;  us_t1 = 9'h036; end                 // 7 &
	9'h03e: begin us_idx = 4'd3;  us_t1 = 9'h052; end                 // 8 *
	9'h046: begin us_idx = 4'd4;  us_t1 = 9'h03e; end                 // 9 (
	9'h045: begin us_idx = 4'd5;  us_t1 = 9'h046; end                 // 0 )
	9'h04e: begin us_idx = 4'd6;  us_t1 = 9'h051; end                 // - _
	9'h055: begin us_idx = 4'd7;  us_t0 = 9'h04e; us_f0 = 1'b1; us_t1 = 9'h04c; end   // = +
	9'h054: begin us_idx = 4'd8;  us_t0 = 9'h05b; us_t1 = 9'h05b; end // [ {
	9'h05b: begin us_idx = 4'd9;  us_t0 = 9'h05d; us_t1 = 9'h05d; end // ] }
	9'h05d: begin us_idx = 4'd10; us_t0 = 9'h06a; us_t1 = 9'h06a; end // \ |  (yen)
	9'h04c: begin us_idx = 4'd11; us_t1 = 9'h052; us_f1 = 1'b0; end   // ; :
	9'h052: begin us_idx = 4'd12; us_t0 = 9'h03d; us_f0 = 1'b1; us_t1 = 9'h01e; end   // ' "
	9'h00e: begin us_idx = 4'd13; us_t0 = 9'h054; us_f0 = 1'b1; us_t1 = 9'h055; end   // ` ~
	default: us_hit = 1'b0;
	endcase
end
reg   [13:0] us_alt;                  // the variant each held US key pressed (1: shifted)
reg          ovr = 1'b0, ovr_shift;   // a held US key forces SHIFT
reg    [3:0] ovr_idx;
wire         host_shift = down[9'h012] | down[9'h059];
wire         us_key = us && us_hit;
wire         us_v   = ps2_key[9] ? host_shift : us_alt[us_idx];   // press: from Shift now; release: as pressed
wire   [8:0] tcode  = us_key ? (us_v ? us_t1 : us_t0) : code;
wire   [8:0] ent    = orig ? paso_key(tcode) : p7_key(tcode);
wire   [6:0] pos   = ent[6:3] * 7'd8 + ent[2:0];

always @(posedge clk) begin
	if (reset) begin
		down <= 0;
		ovr <= 1'b0;
		us_alt <= 0;
		for (int i = 0; i < 96; i++) cnt[i] <= 0;
		extra_shift <= 0;
		toggle_d <= ps2_key[10];
	end
	else begin
		toggle_d <= ps2_key[10];
		if (ps2_key[10] != toggle_d && ent[8] && down[code] != ps2_key[9]) begin
			down[code] <= ps2_key[9];
			if (us_key) begin
				if (ps2_key[9]) begin
					us_alt[us_idx] <= host_shift;
					ovr <= 1'b1;
					ovr_shift <= host_shift ? us_f1 : us_f0;
					ovr_idx <= us_idx;
				end
				else if (ovr && ovr_idx == us_idx) ovr <= 1'b0;
			end
			if (ps2_key[9]) begin
				cnt[pos] <= cnt[pos] + 2'd1;
				if (ent[7]) extra_shift <= extra_shift + 3'd1;
			end
			else begin
				if (cnt[pos] != 0) cnt[pos] <= cnt[pos] - 2'd1;
				if (ent[7] && extra_shift != 0) extra_shift <= extra_shift - 3'd1;
			end
		end
	end
end

reg [7:0] ret;
always @(*) begin
	ret = 0;
	for (int g = 0; g < 3; g++)
		if (sel[4 + g])
			for (int r = 0; r < 4; r++)
				if (sel[r])
					for (int b = 0; b < 8; b++)
						if ((g * 4 + r) == SHIFT_ROW && b == SHIFT_BIT) begin
							if (ovr ? ovr_shift : cnt[SHIFT_ROW * 8 + SHIFT_BIT] != 0 || extra_shift != 0) ret[b] = 1'b1;
						end
						else if (cnt[(g * 4 + r) * 8 + b] != 0 ||
						         (orig && (g * 4 + r) == 3 && b == 1 && cnt[3 * 8 + 3] != 0))
							ret[b] = 1'b1;
end
assign ret_n = ~ret;

endmodule
