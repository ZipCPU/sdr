////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	qpskrcvr.v
//
// Project:	SDR, a basic Soft(Gate)ware Defined Radio architecture
//
// Purpose:	
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2020, Gisselquist Technology, LLC
//
// This program is free software (firmware): you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
// target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	GPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/gpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype	none
//
module	qpskrcvr #(
		parameter	CLOCK_FREQUENCY_HZ = 36_000_000,
		parameter	BASEBAND_SAMPLE_RATE_HZ = 512_000,
		parameter	CIC_SAMPLE_RATE_HZ = BASEBAND_SAMPLE_RATE_HZ* 4,
		localparam	CIC_DOWN = CLOCK_FREQUENCY_HZ
						/ CIC_SAMPLE_RATE_HZ,
		localparam	RESAMPLE_DOWN = CIC_SAMPLE_RATE_HZ
						/ BASEBAND_SAMPLE_RATE_HZ,
		//
		localparam	CIC_BITS = 7
	) (
		// {{{
		input	wire		i_clk, i_reset,
		input	wire		i_audio_en, i_rf_en,
		//
		input	wire		i_wb_cyc, i_wb_stb, i_wb_we,
		input	wire	[1:0]	i_wb_addr,
		input	wire	[31:0]	i_wb_data,
		input	wire	[3:0]	i_wb_sel,
		output	reg		o_wb_stall, o_wb_ack,
		output	reg	[31:0]	o_wb_data,
		//
		// Receive interface
		input	wire	[1:0]	i_rf_data,
		//
		// Outgoing PWM interface
		output	reg	o_pwm_audio,
		//
		// Debug interface
		input	wire	[1:0]	i_dbg_sel,
		output	reg		o_dbg_ce,
		output	reg	[31:0]	o_dbg_data,
		output	reg [HIST_BITS-1:0]	o_dbg_hist
		// }}}
	);
	//

	////////////////////////////////////////////////////////////////////////
	//
	// Incoming bus processing
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	localparam	PHASE_BITS=16;
	localparam [PHASE_BITS-1:0] SYMBOL_STEP
				= { 3'b001, {(PHASE_BITS-3){1'b0}} };
	localparam	HIST_BITS=10;
	localparam	BB_BITS=7;
	localparam	SOFT_BITS = 2*BB_BITS+2;
	localparam	FC_BITS = 8;
	localparam	PWM_BITS = 8;
	localparam	NUM_DOWNSAMPLE_COEFFS = 31 * 4;

	reg	[2:0]			high_symbol_phase;
	reg				load_pll;
	reg	[PHASE_BITS-1:0]	new_pll_step;
	reg	[4:0]			pll_lgcoeff;
	reg				reset_downsampler, write_downsampler;
	reg	[15:0]			write_coeff;

	always @(*)
		high_symbol_phase = 3'h0;

	initial	load_pll     = 0;
	initial	pll_lgcoeff  = 5'h2;
	initial	new_pll_step = { 3'h1, {(PHASE_BITS-3){1'b0}} };
	always @(posedge i_clk)
	if (i_wb_stb && i_wb_we && i_wb_addr == 2'b00)
	begin
		load_pll  <= |i_wb_sel[1:0];
		pll_lgcoeff  <= i_wb_data[20:16];
		new_pll_step <= i_wb_data[15: 0];
	end else
		load_pll <= 1'b0;

	always @(posedge i_clk)
	if (i_wb_stb && i_wb_we && i_wb_addr == 2'b01)
	begin
		reset_downsampler <= i_wb_sel[3] && i_wb_data[31];
		write_downsampler <= |i_wb_sel[1:0];
		write_coeff <= 0;
		if (i_wb_sel[1])
			write_coeff[15:8] <= i_wb_data[15:8];
		if (i_wb_sel[0])
			write_coeff[7:0] <= i_wb_data[7:0];
	end else begin
		reset_downsampler <= 0;
		write_downsampler <= 0;
	end

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// CIC filtering
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	// Come down from CLOCK_FREQUENCY_HZ down to 512ksps
	wire			cic_ce, cic_ign;
	wire	[CIC_BITS-1:0]	cic_sample_i, cic_sample_q;

	cicfil #(.IW(2), .OW(CIC_BITS), .STAGES(4),
		.LGMEM(28), .SHIFT(10)
	) cici(i_clk, 1'b0, CIC_DOWN[6:0], 1'b1, i_rf_data[1] ? 2'b01: 2'b11,
			cic_ce, cic_sample_i);

	cicfil #(.IW(2), .OW(CIC_BITS), .STAGES(4),
		.LGMEM(28), .SHIFT(10)
	) cicq(i_clk, 1'b0, CIC_DOWN[6:0], 1'b1, i_rf_data[0] ? 2'b01: 2'b11,
			cic_ign, cic_sample_q);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Downsampling, combined with Matched filtering
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	// Need to come down to 8x per symbol, or 512ksps total

	wire				baseband_ce;
	wire	signed	[BB_BITS-1:0]	baseband_i, baseband_q;

	subfildowniq #(.IW(CIC_BITS), .OW(BB_BITS), .CW(12), .SHIFT(10),
		.INITIAL_COEFFS(0),
		.NDOWN(RESAMPLE_DOWN),
		.FIXED_COEFFS(1'b0), .NCOEFFS(NUM_DOWNSAMPLE_COEFFS)
	) resample(i_clk,
		reset_downsampler, write_downsampler, write_coeff[15:4],
		cic_ce, cic_sample_i, cic_sample_q,
		baseband_ce, baseband_i, baseband_q);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Symbol tracking
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// Measure | demod_i^2  + demod_q^2 |
	//
	// Filter with (1 + sqrt(2)*z^-1 + z^-2) * (1 - z^-4)
	//	Sqrt(2) * 256 ~= 1_0110_1010, so sqrt(2) ~= 1.011....
	//	So we can try approximating 1.414 with 1.5
	reg	signed	[2*BB_BITS-1:0]	baseband_i_squared, baseband_q_squared;
	reg				square_sum_ce;
	reg	signed	[2*BB_BITS:0]	am_detect;
	reg	signed	[2*BB_BITS:0]	am_lag [0:1];
	reg	signed	[2*BB_BITS+1:0]	am_sum [0:5];
	reg	signed	[2*BB_BITS+2:0]	filtered_detect;
	wire	[2:0]			past_high_symbol_phase;
	reg				symbol_ce;
	reg	signed [BB_BITS-1:0]	symbol_i, symbol_q;
	wire				amfil_sample;
	wire	[PHASE_BITS-1:0]	sym_phase;
	reg	[2:0]			last_sym_phase, short_phase;
	wire	[1:0]			sym_err;

	always @(posedge i_clk)
	if (baseband_ce)
	begin
		baseband_i_squared <= baseband_i * baseband_i;
		baseband_q_squared <= baseband_q * baseband_q;
		square_sum_ce <= 1'b1;
	end else
		square_sum_ce <= 1'b0;

	always @(posedge i_clk)
	if (square_sum_ce)
		am_detect <= baseband_i_squared + baseband_q_squared;

	always @(posedge i_clk)
	if (baseband_ce)
	begin
		am_lag[0] <= am_detect;
		am_lag[1] <= am_lag[0];
	end

	//
	// Generate a filtered_detect signal
	//
	always @(posedge i_clk)
	if (baseband_ce)
	begin
		//	reg	signed	[2*BB_BITS:0]	am_detect;
		//	reg	signed	[2*BB_BITS:0]	am_lag [0:1];
		// Verilator lint_off WIDTH
		am_sum[0] <= am_detect + (am_lag[0] >>> 1);
		// Verilator lint_on  WIDTH
		am_sum[1] <= am_lag[0] + am_lag[1];
		am_sum[2] <= am_sum[1];
		am_sum[3] <= am_sum[2];
		am_sum[4] <= am_sum[3];
		am_sum[5] <= am_sum[4];
		filtered_detect <= am_sum[5] - am_sum[1];
	end

	assign	amfil_sample = filtered_detect[2*BB_BITS+2];

	// Apply a PLL
	sdpll	#(
		.PHASE_BITS(PHASE_BITS),
		.INITIAL_PHASE_STEP(SYMBOL_STEP)
	) symbol_pll(i_clk, load_pll, new_pll_step[PHASE_BITS-2:0],
		baseband_ce, amfil_sample, pll_lgcoeff, sym_phase, sym_err);

	assign	short_phase = sym_phase[PHASE_BITS-1:PHASE_BITS-3];
	
	always @(posedge i_clk)
	if (baseband_ce)
		last_sym_phase <= short_phase;

	assign	past_high_symbol_phase = high_symbol_phase+1;

	always @(posedge i_clk)
	if (baseband_ce)
	begin
		if (short_phase == high_symbol_phase)
			symbol_ce <= 1'b1;
		if (short_phase == past_high_symbol_phase)
			symbol_ce <= 1'b1;

		// Be careful not to repeat our sample
		if (last_sym_phase == high_symbol_phase)
			symbol_ce <= 1'b0;
	end else
		symbol_ce <= 1'b0;

	always @(posedge i_clk)
	if (symbol_ce)
		{ symbol_i, symbol_q } <= { baseband_i, baseband_q };

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// FM Demod to get baseband symbols
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	reg	signed	[BB_BITS-1:0]	past_symbol_i, past_symbol_q;
	reg	[2:0]			mpy_count, past_mpy_count,
					past_past_mpy_count;
	reg	signed	[BB_BITS-1:0]	dmd_multiplicand_a, dmd_multiplicand_b;
	reg	signed	[2*BB_BITS-1:0]	dmd_multiply_out;
	reg	signed	[2*BB_BITS:0]	pre_soft_dmd_i, pre_soft_dmd_q;
	reg	signed	[SOFT_BITS-1:0]	soft_dmd_i, soft_dmd_q;
	wire				dmd_i, dmd_q;

	// demod = current * conj($past(current,4))
	//		ci * pi + cq * pq
	//		ci * pq - cq * pi

	// demod_i = $past(demod_i,4)
	// demod_q
	//

	always @(posedge i_clk)
	if (symbol_ce)
		{ past_symbol_i, past_symbol_q } <= { symbol_i, symbol_q };

	//
	// We'll multiplex our multiply across several clock cycles, so that we
	// can get by with only using one.  Since we're *WAY* oversampled (by
	// about 70x or more), no one will notice our multiplexed multiply here.
	//
	always @(posedge i_clk)
	if (symbol_ce)
		mpy_count <= 3'h4;
	else if (mpy_count > 0)
		mpy_count <= mpy_count - 1;

	always @(posedge i_clk)
	case(mpy_count[1:0])
	2'b00: begin
		dmd_multiplicand_a <=        symbol_i;
		dmd_multiplicand_b <=   past_symbol_i;
		end
	2'b11: begin
		dmd_multiplicand_a <=        symbol_q;
		dmd_multiplicand_b <=   past_symbol_q;
		end
	2'b10: begin
		dmd_multiplicand_a <=        symbol_i;
		dmd_multiplicand_b <=   past_symbol_q;
		end
	2'b01: begin
		dmd_multiplicand_a <=        symbol_q;
		dmd_multiplicand_b <= - past_symbol_i;
		end
	endcase

	always @(posedge i_clk)
	begin
		past_mpy_count <= mpy_count;
		past_past_mpy_count <= past_mpy_count;
	end

	always @(posedge i_clk)
	if (past_mpy_count > 0)
		dmd_multiply_out <= dmd_multiplicand_a * dmd_multiplicand_b;

	always @(posedge i_clk)
	case(past_past_mpy_count)
	// Verilator lint_off WIDTH
	3'b100: pre_soft_dmd_i <= dmd_multiply_out;
	3'b011: pre_soft_dmd_i <= pre_soft_dmd_i + dmd_multiply_out;
	3'b010: pre_soft_dmd_q <= dmd_multiply_out;
	3'b001: pre_soft_dmd_q <= pre_soft_dmd_q + dmd_multiply_out;
	// Verilator lint_on  WIDTH
	default: begin end
	endcase

	//
	// Apply an extra +45 degree rotation, so we're no longer centered on
	// the various axes.  This will also make it possible for us to just
	// strip off the top bit and call it our demodulated bit.
	//
	always @(posedge i_clk)
	if (symbol_ce)
	begin
		soft_dmd_i <= pre_soft_dmd_q + pre_soft_dmd_i;
		soft_dmd_q <= pre_soft_dmd_q - pre_soft_dmd_i;
	end

	assign	dmd_i = !soft_dmd_i[SOFT_BITS-1];
	assign	dmd_q = !soft_dmd_q[SOFT_BITS-1];

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// (Residual carrier removal?)
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	//
	// We'll skip this step.  With the differential encoding we've chosen,
	// this really isn't required.
	//

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Frame synchronization
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	reg	[1:0]	frame_count, frame_pos;
	reg	[3:0]	frame_match;
	reg	[FC_BITS-1:0]	frame_check	[0:3];
	reg	[7:0]	frame_sreg;
	reg		frame_ce;
	reg	[6:0]	scrambled_sample;
	genvar		gk;

	// Add 1 on match, -1 on fail
	// Look for overflow

	always @(posedge i_clk)
	if (symbol_ce)
		frame_count <= frame_count + 1;

	generate for(gk=0; gk<4; gk=gk+1)
	begin
		always @(posedge i_clk)
		if (symbol_ce && frame_count == gk)
		begin
			if (dmd_i != dmd_q)
			begin
				if (&frame_check[gk])
					frame_match[gk] <= 1'b1;
				else
					frame_check[gk] <= frame_check[gk] + 1;
			end else begin
				frame_match[gk] <= 0;
				if (frame_check[gk] != 0)
					frame_check[gk] <= frame_check[gk] - 1;
			end
		end
	end endgenerate

	always @(posedge i_clk)
	if (symbol_ce)
	begin
		case(frame_match)
		4'b0001: frame_pos <= 2'b00;
		4'b0010: frame_pos <= 2'b01;
		4'b0100: frame_pos <= 2'b10;
		4'b1000: frame_pos <= 2'b11;
		default: begin end ///  No lock, keep position
		endcase
	end


	always @(posedge i_clk)
	if (symbol_ce)
		frame_sreg <= { frame_sreg[5:0], dmd_i, dmd_q };

	always @(posedge i_clk)
	if (symbol_ce && frame_pos == frame_count)
	begin
		frame_ce <= 1'b1;
		scrambled_sample <= frame_sreg[6:0];
	end else
		frame_ce <= 1'b0;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Descrambling
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	wire	[6:0]	audio_sample;

	descrambler
	recover(i_clk, 1'b0, frame_ce, scrambled_sample, audio_sample);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Sigma Delta output
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	reg	[PWM_BITS-1:0]	sigma_delta;

	always @(posedge i_clk)
	if (i_audio_en)
		sigma_delta <= sigma_delta
			+ { {(PWM_BITS-7){audio_sample[6]}}, audio_sample }
			+ { !sigma_delta[PWM_BITS-1], {(PWM_BITS-2){1'b0}} };
	else
		sigma_delta <= 0;

	always @(posedge i_clk)
	if (i_audio_en)
		o_pwm_audio <= sigma_delta[PWM_BITS-1];
	else
		o_pwm_audio <= !o_pwm_audio;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Debug symbol generation
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(posedge i_clk)
	case(i_dbg_sel)
	2'b00: // Downsampled results
		{ o_dbg_ce, o_dbg_data, o_dbg_hist } <= {
			baseband_ce,
			{(16-BB_BITS){baseband_i[BB_BITS-1]}}, baseband_i,
			{(16-BB_BITS){baseband_q[BB_BITS-1]}}, baseband_q,
			baseband_i[BB_BITS-1:BB_BITS-HIST_BITS/2],
			baseband_q[BB_BITS-1:BB_BITS-HIST_BITS/2] };
	2'b01: // Symbols pre-FM
		{ o_dbg_ce, o_dbg_data, o_dbg_hist } <= {
			symbol_ce,
			{(16-BB_BITS){baseband_i[BB_BITS-1]}}, baseband_i,
			{(16-BB_BITS){baseband_q[BB_BITS-1]}}, baseband_q,
			baseband_i[BB_BITS-1:BB_BITS-HIST_BITS/2],
			baseband_q[BB_BITS-1:BB_BITS-HIST_BITS/2] };
	2'b10: // Symbols post-FM
		{ o_dbg_ce, o_dbg_data, o_dbg_hist } <= {
			symbol_ce,
			{(16-BB_BITS){soft_dmd_i[SOFT_BITS-1] }},
				soft_dmd_i[SOFT_BITS-1:SOFT_BITS-BB_BITS],
			{(16-BB_BITS){soft_dmd_q[SOFT_BITS-1] }},
				soft_dmd_q[SOFT_BITS-1:SOFT_BITS-BB_BITS],
			soft_dmd_i[SOFT_BITS-1:SOFT_BITS-HIST_BITS/2],
			soft_dmd_q[SOFT_BITS-1:SOFT_BITS-HIST_BITS/2] };
			// [2*BB_BITS+1:0]
	2'b11: // Frame detect and sample output
		{ o_dbg_ce, o_dbg_data, o_dbg_hist } <= { frame_ce,
			frame_match,				// 4 bits
			frame_check[0][FC_BITS-1:FC_BITS-4],	// 4 bits
			frame_check[1][FC_BITS-1:FC_BITS-4],	// 4 bits
			frame_check[2][FC_BITS-1:FC_BITS-4],	// 4 bits
			frame_check[3][FC_BITS-1:FC_BITS-4],	// 4 bits
			{(32-20-7){ audio_sample[6] } },
				audio_sample,			// 7 bits
			{ (HIST_BITS-7){audio_sample[6]} }, audio_sample[6:0] };
	endcase

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Return Wishbone
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(*)
		o_wb_stall = 1'b0;

	always @(posedge i_clk)
		o_wb_ack <= i_wb_stb && !i_reset;

	always @(posedge i_clk)
		o_wb_data <= o_dbg_data;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Make Verilator happy
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// Verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0, i_wb_cyc, i_rf_en, cic_ign, new_pll_step[15],
			i_wb_data[30:16], i_wb_sel[3:2],
			write_coeff[3:0], sym_phase[PHASE_BITS-4:0], sym_err,
			frame_sreg[7] };
	// Verilator lint_on  UNUSED
	// }}}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Formal properties (if any)
// {{{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	FORMAL
`endif
// }}}
endmodule
