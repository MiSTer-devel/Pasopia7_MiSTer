// Cassette deck for an image on an hps_io SD slot (~/dev2/tape-standard.md), from SMC777_MiSTer's smc777_tape.sv.
//
// The Pasopia 7 reads tape as the level of 8255-2 port B bit 5. T-BASIC7's tape routine (6932h-6944h) times each
// pulse with CTC channel 0, so a tape is plain audio. Two image formats:
//   - PCM WAV, 8- or 16-bit, mono or stereo (the first channel plays), 4-192 kHz. The level goes through a comparator
//     with a little hysteresis (5% of full scale), as the simulator's line-in model (verilator/sim_headless.cpp).
//   - CAS: the Common Source Code Project's own tape image (the TOSEC-style Pasopia 7 set uses it). Every byte is a
//     run at 48 kHz: bit 7 the level, bits 6-0 the number of samples (0: skipped). Anything that is not a RIFF/WAVE
//     file is taken as CAS. At mount the deck reads the image once to add up its length (for the status bar).
// The image is not held on chip: an 8-sector ring buffer is filled ahead of the playing position, so any length works.
// Each mount starts rewound.
//
// The deck plays while the motor relay (8255-2 port A bit 5) is on. Turbo (rule 7): while it plays, the CPU runs TURBO
// times faster (rtl/pasopia7.sv turbo) and the samples are played TURBO times faster, so the loader sees the same
// pulse widths in CTC counts. Turbo is chosen when the deck starts, and only if the stream fits MAX_BPS bytes per
// second from the SD card (rule 9); a faster WAV plays at 1x. 500 kB/s takes mono 16-bit WAVs up to 48 kHz at 5x
// (480 kB/s); measured on a MiSTer (2026-11-19): 882 kB/s (44.1 kHz stereo 16-bit at 5x) still loads, 1.76 MB/s
// does not. If the ring buffer runs dry the deck waits (no invented
// samples) and `underflow` is set until the next mount or rewind.
module p7_tape #(
	parameter int CLK_HZ  = 57272727,
	parameter int TURBO   = 5,
	parameter int MAX_BPS = 500000
) (
	input             clk,
	input             rewind,       // back to the first sample
	input             motor,        // the deck plays while the relay is on
	input             turbo_req,    // OSD: Tape speed = TURBO x (taken when the deck starts)
	input             shape,        // OSD: Tape shaper (WAV): each cycle redrawn as a clean square wave (see the player)

	// hps_io block device for this slot
	input             img_mounted,
	input      [63:0] img_size,
	output reg [31:0] sd_lba,
	output reg        sd_rd = 1'b0,
	input             sd_ack,
	input       [8:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	input             sd_buff_wr,

	output reg        level = 1'b0, // to the cassette input
	output reg        loaded = 1'b0,// a playable image is mounted
	output            playing,      // motor on, samples left
	output            turbo,        // the CPU runs fast now
	output reg        ready,        // one-clock pulses after a mount: OSD info messages 1 and 2
	output reg        bad,
	output reg        underflow = 1'b0,
	output reg [17:0] rate = 18'd0, // samples per second
	output reg [31:0] position = 32'd0, // samples played
	output reg [31:0] length = 32'd0,   // samples on the tape
	output reg        cas = 1'b0      // the image is CAS (else WAV)
);

// ---- sector ring: 8 x 512 bytes ------------------------------------------------------------------------------------
// Sectors base..head-1 have been fetched since the last refetch; the ring holds the last 8 of them. Fetches run
// ahead of the read position by up to 8 sectors. A fetch still in flight when head moves elsewhere is dropped.

reg  [31:0] rd = 32'd0;           // byte position read next
reg  [22:0] head = 23'd0, base = 23'd0, fetch_lba = 23'd0;
reg         busy = 1'b0;
reg         drop = 1'b0;          // the fetch in flight belongs to the image before a mount
reg  [31:0] size = 32'd0;
reg   [1:0] wait_q = 2'd0;        // clocks since rd changed (the ring's read port is registered)
wire [22:0] rd_sec = rd[31:9];
wire [22:0] low    = head - base > 23'd8 ? head - 23'd8 : base;
wire        avail  = rd_sec >= low && rd_sec < head;
wire        byte_ok = avail && wait_q == 2'd2;
wire  [7:0] buf_q;

dpram #(12, 8) ring
(
	.clock_a(clk), .enable_a(1'b1), .wren_a(sd_ack & sd_buff_wr), .address_a({fetch_lba[2:0], sd_buff_addr}),
	.data_a(sd_buff_dout), .q_a(),
	.clock_b(clk), .enable_b(1'b1), .wren_b(1'b0), .address_b(rd[11:0]), .data_b(8'h00), .q_b(buf_q)
);

// ---- WAV parser and player ------------------------------------------------------------------------------------------

localparam [31:0] ID_RIFF = 32'h46464952, ID_WAVE = 32'h45564157, ID_FMT = 32'h20746d66, ID_DATA = 32'h61746164;

// S_RIFF..S_FMT3 read one little-endian word each into w; S_WORD then acts on it (`what` = the state that read it)
localparam [3:0] S_IDLE = 0, S_RIFF = 1, S_RSIZE = 2, S_WAVE = 3, S_CHID = 4, S_CHSZ = 5, S_FMT0 = 6, S_FMT1 = 7,
                 S_FMT2 = 8, S_FMT3 = 9, S_WORD = 10, S_PLAY = 11, S_SCAN = 12, S_PRIME = 13;

reg   [3:0] state = S_IDLE, what = S_IDLE;
reg  [31:0] w = 32'd0;
reg   [1:0] nb = 2'd3;            // bytes still to read into w after this one
reg  [31:0] id = 32'd0, fmt_end = 32'd0, tag_ch = 32'd0;
reg  [31:0] data_start = 32'd0, data_end = 32'd0;
reg   [2:0] align = 3'd1;         // bytes per frame: 1, 2 or 4
reg         bits16 = 1'b0, fmt_ok = 1'b0;
reg   [5:0] chunks = 6'd0;        // chunk headers read: give up on garbage

// player
reg  [31:0] acc = 32'd0;
reg  [21:0] step = 22'd0;         // samples per second, times TURBO while turbo
reg         due = 1'b0, fast = 1'b0, motor_d = 1'b0;
reg         rd_hi = 1'b0;         // the low byte of a 16-bit sample has been read
reg   [7:0] lo_byte = 8'd0;
reg   [4:0] dry = 5'd0;           // clocks a due sample has waited for the SD card
reg   [6:0] run = 7'd0;           // CAS: samples left in the current run after this one
reg  [31:0] cas_len = 32'd0;      // CAS: samples on the tape (the scan at mount)

// WAV: a scanner reads samples ahead of the player as runs of one level (pieces of up to 255 samples, so a long gap
// starts playing before its end is found; longer than any shaped half up to 192 kHz) into a FIFO of 8; the
// player plays them at the sample rate. The shaper needs a whole low half before it plays it, and its output can run
// faster than the tape (a low shortened to its high), so the scanner reads ahead of the player rather than in step.
reg         sc_lvl = 1'b0;        // comparator state
reg  [11:0] sc_len = 12'd0;       // samples in the run being scanned
reg         sc_eof = 1'b0;        // the last run is in the FIFO
reg  [12:0] q_run [0:7];          // {level, samples}
reg   [2:0] q_wp = 3'd0, q_rp = 3'd0;
reg   [3:0] q_n = 4'd0;
reg  [11:0] p_left = 12'd0;       // samples left of the run playing
reg  [11:0] pend_len = 12'd0;     // shaper: the length the next low plays for
reg         pend = 1'b0;          // ... if set
reg         started = 1'b0;       // a run has played since the mount or rewind (no underflow before: the first fill)
// shaper thresholds in samples at this rate: 137 us (floor), 312 us (floor), 500 us (ceil); outputs: 208 and 417 us
reg  [11:0] t_min = 12'd0, t_half = 12'd0, t_max = 12'd0, n_short = 12'd1, n_long = 12'd1;
reg  [43:0] t_mul0 = 44'd0, t_mul1 = 44'd0, t_mul2 = 44'd0, t_mul3 = 44'd0, t_mul4 = 44'd0;

assign playing = loaded && motor && (cas ? rd < data_end || run != 7'd0 : !(sc_eof && q_n == 4'd0 && p_left == 12'd0));
assign turbo   = playing && fast;

wire [32:0] chunk_end = {1'b0, rd} + {1'b0, w} + w[0];   // after the size word: body plus pad byte
wire [31:0] chunk_end_sat = chunk_end[32] ? 32'hFFFFFFFF : chunk_end[31:0];
wire [32:0] data_lim = {1'b0, rd} + {1'b0, w};
wire [15:0] s16 = {buf_q, lo_byte};

// fmt: tag 1 (PCM), 1-2 channels, 8/16 bits, block align = channels * bytes per sample
wire [15:0] f_ch = tag_ch[31:16], f_bits = w[31:16], f_align = w[15:0];
wire        f_ok = tag_ch[15:0] == 16'd1 && (f_ch == 16'd1 || f_ch == 16'd2) && (f_bits == 16'd8 || f_bits == 16'd16) &&
                   f_align == (f_ch << (f_bits == 16'd16 ? 1 : 0)) && rate >= 18'd4000;

// move the read position; outside the resident sectors the ring refetches from there
task seek(input [31:0] to);
	rd <= to;
	wait_q <= 2'd0;
	if (!(to[31:9] >= low && to[31:9] < head)) begin head <= to[31:9]; base <= to[31:9]; end
endtask

// empty the WAV scanner and player
task flush;
	sc_lvl <= 1'b0;
	sc_len <= 12'd0;
	sc_eof <= 1'b0;
	q_wp <= 3'd0;
	q_rp <= 3'd0;
	q_n <= 4'd0;
	p_left <= 12'd0;
	pend <= 1'b0;
	started <= 1'b0;
endtask

reg        push, pop, nl;
reg [12:0] push_run, e, e2;
reg [11:0] plen;

// shaper thresholds: rate x seconds x 2^26, then / 2^26 (floor, or ceil / round with the constant added)
always @(posedge clk) begin
	t_mul0  <= {26'd0, rate} * 44'd9194;
	t_mul1  <= {26'd0, rate} * 44'd20938;
	t_mul2  <= {26'd0, rate} * 44'd33554 + 44'd67108863;
	t_mul3  <= {26'd0, rate} * 44'd13959 + 44'd33554432;
	t_mul4  <= {26'd0, rate} * 44'd27984 + 44'd33554432;
	t_min   <= t_mul0[37:26];
	t_half  <= t_mul1[37:26];
	t_max   <= t_mul2[37:26];
	n_short <= t_mul3[37:26] == 12'd0 ? 12'd1 : t_mul3[37:26];
	n_long  <= t_mul4[37:26] == 12'd0 ? 12'd1 : t_mul4[37:26];
end

always @(posedge clk) begin
	ready <= 1'b0;
	bad <= 1'b0;
	if (wait_q != 2'd2) wait_q <= wait_q + 2'd1;

	// sector fetch
	if (~busy && state != S_IDLE && head < rd_sec + 23'd8 && {head, 9'd0} < size) begin
		fetch_lba <= head;
		sd_lba <= {9'd0, head};
		sd_rd <= 1'b1;
		busy <= 1'b1;
	end
	if (busy && sd_ack) sd_rd <= 1'b0;
	if (busy && ~sd_rd && ~sd_ack) begin
		busy <= 1'b0;
		drop <= 1'b0;
		if (fetch_lba == head && !drop) head <= head + 23'd1;
	end

	motor_d <= motor;
	if (motor & ~motor_d) fast <= turbo_req && rate * align * TURBO <= MAX_BPS;
	step <= fast ? rate * TURBO : {4'd0, rate};

	case (state)
	S_RIFF, S_RSIZE, S_WAVE, S_CHID, S_CHSZ, S_FMT0, S_FMT1, S_FMT2, S_FMT3:
		if (rd >= size) begin
			state <= S_IDLE;
			bad <= 1'b1;
		end else if (byte_ok) begin
			w <= {buf_q, w[31:8]};
			rd <= rd + 32'd1;
			wait_q <= 2'd0;
			if (nb != 2'd0) nb <= nb - 2'd1;
			else begin
				nb <= 2'd3;
				what <= state;
				state <= S_WORD;
			end
		end

	S_WORD: begin
		state <= S_IDLE;
		case (what)
		S_RIFF:
			if (w == ID_RIFF) state <= S_RSIZE;
			else begin                                            // not a WAV: a CAS image; add up its runs first
				cas <= 1'b1;
				rate <= 18'd48000;
				align <= 3'd1;
				cas_len <= 32'd0;
				seek(32'd0);
				state <= S_SCAN;
			end
		S_RSIZE: state <= S_WAVE;
		S_WAVE:  state <= w == ID_WAVE ? S_CHID : S_IDLE;
		S_CHID:  begin id <= w; chunks <= chunks + 6'd1; state <= &chunks ? S_IDLE : S_CHSZ; end
		S_CHSZ:
			if (id == ID_FMT && w >= 32'd16) begin
				fmt_end <= chunk_end_sat;
				state <= S_FMT0;
			end else if (id == ID_DATA) begin
				data_start <= rd;
				data_end <= data_lim > {1'b0, size} ? size : data_lim[31:0];
				state <= fmt_ok ? S_PLAY : S_IDLE;
				loaded <= fmt_ok;
				ready <= fmt_ok;
			end else begin
				seek(chunk_end_sat);
				state <= S_CHID;
			end
		S_FMT0:  begin tag_ch <= w; state <= S_FMT1; end
		S_FMT1:  begin rate <= w > 32'd192000 ? 18'd0 : w[17:0]; state <= S_FMT2; end
		S_FMT2:  state <= S_FMT3;                                     // byte rate
		S_FMT3:  begin
			fmt_ok <= f_ok;
			align <= f_align[2:0];
			bits16 <= f_bits == 16'd16;
			seek(fmt_end);
			state <= S_CHID;
		end
		default: ;
		endcase
		if ((what == S_WAVE && w != ID_WAVE) ||
		    (what == S_CHID && &chunks) || (what == S_CHSZ && id == ID_DATA && !fmt_ok)) bad <= 1'b1;
	end

	S_SCAN:
		if (rd >= size) begin
			data_start <= 32'd0;
			data_end <= size;
			seek(32'd0);
			bad <= cas_len == 32'd0;
			state <= cas_len != 32'd0 ? S_PRIME : S_IDLE;
		end else if (byte_ok) begin
			cas_len <= cas_len + {25'd0, buf_q[6:0]};
			rd <= rd + 32'd1;
			wait_q <= 2'd0;
		end

	S_PRIME:                                              // CAS: play once the first sector is back
		if (byte_ok) begin
			loaded <= 1'b1;
			ready <= 1'b1;
			state <= S_PLAY;
		end

	S_PLAY: begin
		if (playing && !due) begin
			if (acc + step >= CLK_HZ) begin acc <= acc + step - CLK_HZ; due <= 1'b1; end
			else acc <= acc + step;
		end
		if (due && cas) begin
			if (run != 7'd0) begin
				run <= run - 7'd1;
				due <= 1'b0;
				position <= position + 32'd1;
			end else begin
				if (!avail) begin
					if (dry != 5'd31) dry <= dry + 5'd1;
					if (dry == 5'd16) underflow <= 1'b1;
				end
				if (byte_ok) begin
					rd <= rd + 32'd1;
					wait_q <= 2'd0;
					dry <= 5'd0;
					if (buf_q[6:0] != 7'd0) begin
						level <= buf_q[7];
						run <= buf_q[6:0] - 7'd1;
						due <= 1'b0;
						position <= position + 32'd1;
					end
				end
			end
		end else if (!cas) begin
			// scanner: one sample into the run being scanned; a finished run goes into the FIFO
			push = 1'b0;
			push_run = {sc_lvl, sc_len};
			if (!sc_eof && q_n < 4'd7) begin
				if (!rd_hi && rd + align > data_end) begin
					sc_eof <= 1'b1;
					push = sc_len != 12'd0;
				end else if (byte_ok) begin
					if (bits16 && !rd_hi) begin
						lo_byte <= buf_q;
						rd_hi <= 1'b1;
						rd <= rd + 32'd1;
						wait_q <= 2'd0;
					end else begin
						nl = sc_lvl;
						if (bits16) begin
							if ($signed(s16) >= 16'sd1639) nl = 1'b1;
							else if ($signed(s16) <= -16'sd1639) nl = 1'b0;
							rd <= rd + align - 32'd1;
						end else begin
							if (buf_q >= 8'd135) nl = 1'b1;
							else if (buf_q <= 8'd121) nl = 1'b0;
							rd <= rd + align;
						end
						wait_q <= 2'd0;
						rd_hi <= 1'b0;
						sc_lvl <= nl;
						if (sc_len != 12'd0 && (nl != sc_lvl || sc_len == 12'd255)) begin
							push = 1'b1;
							sc_len <= 12'd1;
						end else sc_len <= sc_len + 12'd1;
					end
				end
			end
			// player: one sample per tick
			pop = 1'b0;
			if (due) begin
				if (p_left != 12'd0) begin
					p_left <= p_left - 12'd1;
					due <= 1'b0;
				end else if (q_n != 4'd0 && (q_n != 4'd1 || sc_eof || !q_run[q_rp][12] || !shape)) begin
					// the Common Source Code Project's wave shaper (src/vm/datarec.cpp) with the Pasopia's nominal
					// lengths: a high of 137-500 us followed by a low over 137 us becomes a short (208 us) or long
					// (417 us) half, split at 312 us; the low takes the same length unless it is 500 us or more (a gap)
					pop = 1'b1;
					started <= 1'b1;
					e = q_run[q_rp];
					e2 = q_run[q_rp + 3'd1];
					plen = e[11:0];
					if (!e[12] && pend) plen = pend_len;
					pend <= 1'b0;
					if (shape && e[12] && q_n != 4'd1 && e[11:0] > t_min && e[11:0] < t_max && e2[11:0] > t_min) begin
						plen = e[11:0] > t_half ? n_long : n_short;
						pend <= e2[11:0] < t_max;
						pend_len <= plen;
					end
					level <= e[12];
					p_left <= plen - 12'd1;
					position <= position + {20'd0, e[11:0]};
					due <= 1'b0;
					dry <= 5'd0;
				end else if (!avail && started) begin
					if (dry != 5'd31) dry <= dry + 5'd1;
					if (dry == 5'd16) underflow <= 1'b1;
				end
			end
			if (push) begin
				q_run[q_wp] <= push_run;
				q_wp <= q_wp + 3'd1;
			end
			if (pop) q_rp <= q_rp + 3'd1;
			q_n <= q_n + {3'd0, push} - {3'd0, pop};
		end
		if (rewind) begin
			seek(data_start);
			position <= 32'd0;
			acc <= 32'd0;
			due <= 1'b0;
			rd_hi <= 1'b0;
			run <= 7'd0;
			level <= 1'b0;
			underflow <= 1'b0;
			flush();
		end
	end

	default: ;
	endcase

	if (img_mounted) begin
		size <= img_size[63:32] != 32'd0 ? 32'hFFFFFFFF : img_size[31:0];
		rd <= 32'd0;
		head <= 23'd0;
		base <= 23'd0;
		wait_q <= 2'd0;
		nb <= 2'd3;
		chunks <= 6'd0;
		fmt_ok <= 1'b0;
		loaded <= 1'b0;
		rate <= 18'd0;
		position <= 32'd0;
		acc <= 32'd0;
		due <= 1'b0;
		rd_hi <= 1'b0;
		level <= 1'b0;
		underflow <= 1'b0;
		dry <= 5'd0;
		run <= 7'd0;
		cas <= 1'b0;
		flush();
		drop <= busy;
		state <= img_size == 64'd0 ? S_IDLE : S_RIFF;
	end
end

// samples on the tape: the data bytes over the frame size (1, 2 or 4)
always @(posedge clk) length <= cas ? cas_len : (data_end - data_start) >> (align == 3'd4 ? 2 : align == 3'd2 ? 1 : 0);

endmodule
