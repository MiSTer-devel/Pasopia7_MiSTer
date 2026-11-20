// Z80 PIO, as the Pasopia 7 uses it: port A output (keyboard row select, beeper gate), port B in bit-control mode
// (mode 3) with an interrupt on the keyboard return lines.
//
// Behaviour follows the Common Source Code Project's src/vm/z80pio.cpp (Takeda Toshiya), the reference the core is
// compared against:
//   - control words: vector (bit 0 = 0), interrupt control xxxx0111 (bit 7 enable, 6 AND/OR, 5 high/low, 4 mask
//     follows; a mask write re-enables), interrupt enable xxxx0011, mode xx001111 (mode 3: direction word follows).
//   - mode 0 (output) data write, and mode 1 (input) new data, request an interrupt at once (no handshake wired).
//   - mode 3: the condition is re-evaluated on a data write to that port and on each 'evt' strobe (the machine
//     raises it when the input lines may have changed: CSCP re-reads the keyboard on every port A change and once
//     a frame). A true condition requests an interrupt; a false one withdraws an un-acknowledged request.
//   - data reads return input bits for input lines and the output latch for output lines.
//   - control reads return {A mode, B mode >> 4} (CSCP's Sharp-style status).
//
// Bus side, like rtl/z80ctc.sv: wr/rd are one-clk events; ack and reti are one-clk events from the machine's
// interrupt bridge, which latches 'vector' at ack. irq is the INT request; ieo feeds the next device in the chain.

module z80pio
(
	input            clk,
	input            reset,

	input            wr,
	input            rd,
	input            cd,              // 1 control, 0 data  (C/D)
	input            ba,              // 1 port B, 0 port A (B/A)
	input      [7:0] din,
	output reg [7:0] dout,

	input      [7:0] pa_i,
	input      [7:0] pb_i,
	output     [7:0] pa_o,            // output latch (input lines read as the latch too; the machine uses A as output)
	output     [7:0] pb_o,
	input      [1:0] evt,             // [ch]: re-evaluate a mode 3 condition / new input data

	input            iei,
	output           irq,
	output           ieo,
	input            ack,
	input            reti,
	output     [7:0] vector,
	output           in_svc           // a channel is being serviced (gates RETI for devices further down)
);

localparam M_OUT = 2'd0, M_IN = 2'd1, M_BI = 2'd2, M_CTL = 2'd3;

reg [7:0] wreg[2], dir[2], mask[2], vect[2], ctrl1[2];
reg [1:0] mode[2];
reg [1:0] set_dir, set_mask, enb, enb_tmp, req, svc;

wire [7:0] rreg[2];
assign rreg[0] = pa_i;
assign rreg[1] = pb_i;
wire [7:0] val[2];
assign val[0] = (rreg[0] & dir[0]) | (wreg[0] & ~dir[0]);
assign val[1] = (rreg[1] & dir[1]) | (wreg[1] & ~dir[1]);

assign pa_o = wreg[0];
assign pb_o = wreg[1];

// CSCP check_mode3_intr(): bits 6-5 of the interrupt control word (00 OR/low, 01 OR/high, 10 AND/low, 11 AND/high)
function automatic cond(input [7:0] v, input [7:0] m, input [7:0] c1);
	reg [7:0] mk, x;
	begin
		mk = ~m;
		x  = v & mk;
		case (c1[6:5])
		2'b00: cond = x != mk;
		2'b01: cond = x != 8'd0;
		2'b10: cond = x == 8'd0;
		2'b11: cond = x == mk;
		endcase
	end
endfunction

wire any_svc = |svc;
wire ch_a_irq = enb[0] & req[0];
wire ch_b_irq = enb[1] & req[1];
assign irq    = iei & ~any_svc & (ch_a_irq | ch_b_irq);
assign ieo    = iei & ~any_svc;
assign vector = ch_a_irq ? vect[0] : vect[1];
assign in_svc = any_svc;

wire ch = ba;

always @(posedge clk) begin
	if (reset) begin
		for (int i = 0; i < 2; i++) begin
			wreg[i]  <= 8'h00;
			mode[i]  <= M_IN;
			ctrl1[i] <= 8'h00;
			dir[i]   <= 8'hff;
			mask[i]  <= 8'hff;
			vect[i]  <= 8'h00;
		end
		set_dir <= 0; set_mask <= 0; enb <= 0; enb_tmp <= 0; req <= 0; svc <= 0;
	end
	else begin
		// new input data / condition re-check
		for (int i = 0; i < 2; i++)
			if (evt[i]) begin
				if (mode[i] == M_IN || mode[i] == M_BI) req[i] <= 1'b1;
				else if (mode[i] == M_CTL) req[i] <= cond(val[i], mask[i], ctrl1[i]);
			end

		if (wr && !cd) begin
			wreg[ch] <= din;
			if (mode[ch] == M_OUT || mode[ch] == M_BI) req[ch] <= 1'b1;
			else if (mode[ch] == M_CTL)
				req[ch] <= cond((rreg[ch] & dir[ch]) | (din & ~dir[ch]), mask[ch], ctrl1[ch]);
		end
		else if (wr && cd) begin
			if (set_dir[ch]) begin
				dir[ch] <= din;
				set_dir[ch] <= 1'b0;
			end
			else if (set_mask[ch]) begin
				mask[ch] <= din;
				set_mask[ch] <= 1'b0;
				enb[ch] <= enb_tmp[ch];
			end
			else if (!din[0]) vect[ch] <= din;
			else if (din[3:0] == 4'h3) enb[ch] <= din[7];
			else if (din[3:0] == 4'h7) begin
				ctrl1[ch] <= din;
				if (din[4]) begin
					set_mask[ch] <= 1'b1;
					req[ch] <= 1'b0;
					enb_tmp[ch] <= din[7];
					enb[ch] <= 1'b0;
				end
				else enb[ch] <= din[7];
			end
			else if (din[3:0] == 4'hf) begin
				mode[ch] <= din[7:6];
				case (din[7:6])
				M_OUT: dir[ch] <= 8'h00;
				M_IN, M_BI: dir[ch] <= 8'hff;
				M_CTL: set_dir[ch] <= 1'b1;
				endcase
			end
		end

		if (ack && irq) begin
			if (ch_a_irq) begin req[0] <= 1'b0; svc[0] <= 1'b1; end
			else          begin req[1] <= 1'b0; svc[1] <= 1'b1; end
		end
		if (reti) begin
			if (svc[0]) svc[0] <= 1'b0;
			else if (svc[1]) svc[1] <= 1'b0;
		end
	end
end

always @(*) begin
	if (cd) dout = {mode[0], 6'd0} | {4'd0, mode[1], 2'd0};
	else    dout = val[ch];
end

endmodule
