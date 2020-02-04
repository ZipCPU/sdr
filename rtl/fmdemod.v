////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	fmdemod.v
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
module	fmdemod(i_clk, i_reset, i_audio_en, i_rf_en,
		// Wishbone interface
		i_wb_cyc, i_wb_stb, i_wb_we, i_wb_addr, i_wb_data, i_wb_sel,
			o_wb_stall, o_wb_ack, o_wb_data,
		//
		// Receive interface
		i_rf_data,
		//
		// PWM interface
		o_pwm_audio,
		//
		// Debug interface
		i_dbg_sel, o_dbg_ce, o_dbg_data, o_dbg_hist);
	//
	parameter [31:0]	CLOCK_FREQUENCY_HZ = 36_000_000;
	parameter [31:0]	RAW_DATA_RATE_HZ = 960_000;
	parameter [31:0]	AUDIO_SAMPLE_RATE_HZ = 48_000;
	parameter		NUM_AUDIO_COEFFS = 666;
	parameter		HIST_BITS = 10;
	//
	// localparam [31:0]	RAW_AUDIO_DOWNSAMPLE_RATIO = CLOCK_FREQUENCY_HZ / AUDIO_SAMPLE_RATE_HZ; // == 750
	localparam [31:0]	RAW_AUDIO_DOWNSAMPLE_RATIO = 750;
	// localparam		CIC_DOWN = 25;	// About 200kHz B/W
	localparam		CIC_DOWN = 5;
	localparam [31:0]	SECONDARY_DOWNSAMPLE_RATIO = 750 / CIC_DOWN;
	//
	input	wire		i_clk, i_reset;
	input	wire		i_audio_en, i_rf_en;
	//
	// Wishbone interface
	input	wire		i_wb_cyc, i_wb_stb, i_wb_we;
	input	wire [1:0]	i_wb_addr;
	input	wire [31:0]	i_wb_data;
	input	wire	[3:0]	i_wb_sel;
	output	reg		o_wb_stall;
	output	reg		o_wb_ack;
	output	reg	[31:0]	o_wb_data;
	//
	// Receive interface
	input	wire	[1:0]	i_rf_data;
	//
	// Outgoing PWM interface
	output	reg		o_pwm_audio;
	//
	// Debug interface
	input	wire	[1:0]	i_dbg_sel;
	output	reg		o_dbg_ce;
	output	reg	[31:0]	o_dbg_data;
	output reg [HIST_BITS-1:0] o_dbg_hist;
	//
	integer	k;

	localparam	CIC_BITS = 7;
	localparam	BB_BITS  = 16;
	localparam	PWM_BITS = 16;
	localparam	PLL_BITS = 24;

	wire			cic_ce, cic_ign, baseband_ce;
	wire	[CIC_BITS-1:0]	cic_sample_i, cic_sample_q;
	wire signed [BB_BITS-1:0] baseband_sample;
	reg	[PWM_BITS-1:0]	pwm_counter, brev_counter;
	reg signed [31:0]	amplified_sample;
	reg	[15:0]		audio_sample_off;
	// wire			splr_busy, splr_done;
	wire	[PLL_BITS-1:0]	pll_phase;
	reg	[PLL_BITS-1:0]	last_pll_phase, fm_step;
	reg	[4:0]		pll_lgcoeff;
	reg signed [15:0]	r_gain;
	reg			load_pll;
	reg	[22:0]		new_pll_step;
	reg			write_audio_filter, reset_filter;
	reg	[15:0]		write_coeff;
	wire	[1:0]		pll_err;

	////////////////////////////////////
	//
	// Bus
	//
	initial	pll_lgcoeff  = 5'h02;
	initial	new_pll_step = 0;
	initial	load_pll     = 1;
	initial	reset_filter = 1;
	initial	write_audio_filter = 0;
	always @(posedge i_clk)
	if (i_wb_stb && i_wb_we)
	begin
		{ reset_filter, load_pll, write_coeff } <= 0;

		case(i_wb_addr)
		2'b00: r_gain <= i_wb_data[15:0];
		2'b01: { load_pll, new_pll_step } <= { 1'b1,  i_wb_data[22:0] };
		2'b10: { reset_filter, write_audio_filter, write_coeff }
				<= { i_wb_data[31], !i_wb_data[31],
						i_wb_data[15:0] };
		2'b11: pll_lgcoeff <= i_wb_data[4:0];
		default: begin end
		endcase
	end else
		{ reset_filter, load_pll, write_coeff } <= 0;

	always @(*)
		o_wb_stall = 1'b0;

	always @(posedge i_clk)
		o_wb_ack <= !i_reset && i_wb_stb;

	always @(posedge i_clk)
		o_wb_data <= 0;

	////////////////////////////////////
	//
	// CIC Filters
	//

	cicfil #(.IW(2), .OW(CIC_BITS), .STAGES(4), .LGMEM(28), .SHIFT(6))
	cici(i_clk, reset_filter, CIC_DOWN[6:0], 1'b1, i_rf_data[1] ? 2'b11 : 2'b01, cic_ce, cic_sample_i);

	cicfil #(.IW(2), .OW(CIC_BITS), .STAGES(4), .LGMEM(28), .SHIFT(6))
	cicq(i_clk, reset_filter, CIC_DOWN[6:0], 1'b1, i_rf_data[0] ? 2'b11 : 2'b01, cic_ign, cic_sample_q);

	////////////////////////////////////
	//
	// Track the FM carrier
	//

	quadpll #(.PHASE_BITS(24), .OPT_TRACK_FREQUENCY(1'b1),
		.INITIAL_PHASE_STEP(24'h00_00_00)) // i.e. TRACKING FREQUENCY
	carrier_track(i_clk, load_pll, new_pll_step,
		cic_ce, { cic_sample_i[CIC_BITS-1], cic_sample_q[CIC_BITS-1] },
			pll_lgcoeff, pll_phase, pll_err);

	always @(posedge i_clk)
	if (cic_ce)
	begin
		fm_step <= pll_phase - last_pll_phase;
		last_pll_phase <= pll_phase;
	end

	////////////////////////////////////
	//
	// Downsampling (PLL cleanup)
	//

	subfildown #(.IW(BB_BITS), .OW(BB_BITS), .CW(12),
		.NDOWN(SECONDARY_DOWNSAMPLE_RATIO),
		.FIXED_COEFFS(1'b0), .NCOEFFS(NUM_AUDIO_COEFFS))
	resample(i_clk, reset_filter,
		write_audio_filter, write_coeff[15:4],
		cic_ce, fm_step[PLL_BITS-1:PLL_BITS-BB_BITS],
		baseband_ce, baseband_sample);

	////////////////////////////////////
	//
	// Amplify the result
	//

	(* mul2dsp *)
	always @(posedge  i_clk)
	// if (splr_done)
		amplified_sample <= baseband_sample[15:0] * r_gain;

	////////////////////////////////////
	//
	// Convert to PWM
	//

	always @(posedge  i_clk)
	// if (splr_done)
	begin
		audio_sample_off[14:0] <=  amplified_sample[30:16];
		audio_sample_off[15]   <= !amplified_sample[31];
	end

	always @(posedge i_clk)
		pwm_counter <= pwm_counter + 1;

	always @(*)
	for(k=0; k<PWM_BITS; k=k+1)
		brev_counter[k] = pwm_counter[PWM_BITS-1-k];

	always @(posedge i_clk)
	if (!i_audio_en)
		o_pwm_audio <= !o_pwm_audio;
	else
		o_pwm_audio <= (brev_counter < audio_sample_off);

	////////////////////////////////////
	//
	// Create the debugging outputs
	//

	always @(posedge i_clk)
	case(i_dbg_sel)
	2'b00: { o_dbg_ce, o_dbg_data, o_dbg_hist } <= { cic_ce,
			{(16-CIC_BITS){cic_sample_i[CIC_BITS-1]}}, cic_sample_i,
			{(16-CIC_BITS){cic_sample_q[CIC_BITS-1]}}, cic_sample_q,
			cic_sample_i[CIC_BITS-1:CIC_BITS-(HIST_BITS/2)],
			cic_sample_q[CIC_BITS-1:CIC_BITS-(HIST_BITS/2)] };
	2'b10: { o_dbg_ce, o_dbg_data, o_dbg_hist } <= { cic_ce,
				{(32-PLL_BITS){1'b0}}, fm_step,
				fm_step[PLL_BITS-1:PLL_BITS-HIST_BITS] };
	2'b01: { o_dbg_ce, o_dbg_data, o_dbg_hist } <= { baseband_ce,
				{(32-BB_BITS){1'b0}}, baseband_sample,
				baseband_sample[BB_BITS-1:BB_BITS-HIST_BITS] };
	2'b11: { o_dbg_ce, o_dbg_data, o_dbg_hist }
			<= { baseband_ce, amplified_sample,
			amplified_sample[32-1:32-HIST_BITS] };
	default: { o_dbg_ce, o_dbg_data, o_dbg_hist } <= 0;
	endcase

	always @(posedge i_clk)
		o_dbg_hist <= 0;

	// Verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0, i_wb_cyc, i_wb_sel,
			cic_ign,
			i_rf_en, write_coeff[3:0], pll_err, i_wb_data[30:23],
			amplified_sample[15:0], fm_step[23:16],
			write_audio_filter, write_coeff };
	// Verilator lint_on  UNUSED
endmodule
