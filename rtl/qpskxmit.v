////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	qpskxmit.txt
//
// Project:	SDR, a basic Soft(Gate)ware Defined Radio architecture
//
// Purpose:	QPSK Audio transmitter
//
// Registers:
//	0:	Audio downsampling filter control
//	1:	Pulseshaping filter control
//	2:	(Reserved)
//	3:	(Reserved)
//
// Signal parameters:
//
//	Modulation:		QPSK
//	Audio Sample Rate:	16 kHz
//	Bandwidth:		72 kHz
//	Symbol Rate:		64 kHz
//	First stage audio filter:
//		Running on 960kHz sampled stream
//		Generating a 16kHz audio sample stream
//		1999 Coefficients
//		Passband 7.2kHz, Stopband 8.8 kHz
//		% genfil -r -s 960 1999 7.2 8.8	 # 61 dB stop band (audio8k.fl)
//	LFSR: 7-data bits per clock, Feedthrough, coefficients 31, 13, 0
//	Data Frame Length:	 8 bits = 1 5-bit sample plus 1 bit framing
//		Sync bit prevents 180 degree transition once every 4 symbols
//	Pulse shaping filter:	(pshape.fl)
//		Running at 256ksps, TMIC, Null at 80kHz, 32 taps
//		% genfil -r 32 tmic 2
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
module	qpskxmit(i_clk, i_reset, i_audio_en, i_rf_en,
		// Wishbone interface
		i_wb_cyc, i_wb_stb, i_wb_we, i_wb_addr, i_wb_data, i_wb_sel,
			o_wb_stall, o_wb_ack, o_wb_data,
		//
		// Microphone (SPI) interface
		o_mic_csn, o_mic_sck, i_mic_miso,
		//
		// Transmit interface
		o_rf_data,
		// Debug interface
		i_dbg_sel, o_dbg_ce, o_dbg_data, o_dbg_hist);
	//
	parameter [31:0]	CLOCK_FREQUENCY_HZ = 36_000_000;
	parameter [31:0]	MICROPHONE_SAMPLE_RATE_HZ = 960_000;
	parameter [31:0]	AUDIO_SAMPLE_RATE_HZ = 16_000;
	parameter [31:0]	BASEBAND_SAMPLE_RATE_HZ = 4 * 64_000;
	parameter		NUM_AUDIO_COEFFS = 1999;
	parameter		HIST_BITS = 10;
	localparam		PWM_BITS  = 16;
	//
	//
	localparam 	SYNC_BITS       = 8;
	localparam 	DATA_FRAME_LEN  = 1024*12;
	localparam 	RADIO_FRAME_LEN = DATA_FRAME_LEN + SYNC_BITS;
	parameter real	SYMBOL_RATE_HZ  = AUDIO_SAMPLE_RATE_HZ * RADIO_FRAME_LEN / DATA_FRAME_LEN * 12.0 / 4.0;
	//
	// Verilator lint_off REALCVT
	localparam real 	MIC_STEP_R = (4.0 * (1<<30)) * MICROPHONE_SAMPLE_RATE_HZ
					* 1.0 / CLOCK_FREQUENCY_HZ ;
	localparam [31:0]	MIC_STEP = MIC_STEP_R;
	localparam real 	SYMBOL_STEP_R = (SYMBOL_RATE_HZ * 1.0 / CLOCK_FREQUENCY_HZ)
					* 4.0 * (1<<30);
	localparam [31:0]	SYMBOL_STEP = SYMBOL_STEP_R;
	// Verilator lint_on REALCVT
	localparam [31:0]	RAW_AUDIO_DOWNSAMPLE_RATIO = MICROPHONE_SAMPLE_RATE_HZ / AUDIO_SAMPLE_RATE_HZ; // == 20
	//
	input	wire	i_clk, i_reset;
	input	wire	i_audio_en, i_rf_en;
	//
	// Wishbone interface
	input	wire	i_wb_cyc, i_wb_stb, i_wb_we;
	input	wire [1:0]	i_wb_addr;
	input	wire [31:0]	i_wb_data;
	input	wire	[3:0]	i_wb_sel;
	output	reg		o_wb_stall;
	output	reg		o_wb_ack;
	output	reg	[31:0]	o_wb_data;
	//
	//
	output	wire		o_mic_csn, o_mic_sck;
	input	wire		i_mic_miso;
	//
	// Transmit interface
	output	reg	[1:0]	o_rf_data;
	//
	// Debug interface
	input	wire	[1:0]	i_dbg_sel;
	output	reg		o_dbg_ce;
	output	reg	[31:0]	o_dbg_data;
	output	reg	[HIST_BITS-1:0]	o_dbg_hist;
	//

	localparam	MIC_BITS  = 12;
	localparam	AUDIO_BITS= 7;
	localparam	BB_BITS   = 12;
	// localparam	XMIT_AUDIO_BITS= 12;
	localparam	LGFLEN = 3;

	reg			mic_ce;
	reg	[31:0]		mic_ce_counter;
	wire			mic_ignore, mic_valid;
	wire	[MIC_BITS-1:0]	mic_sample;

	wire				audio_ce;
	wire	[AUDIO_BITS-1:0]	audio_sample;

	reg		symbol_ce;
	reg	[31:0]	symbol_ce_counter;

	reg			reset_audio, reset_pulse,
				write_audio, write_pulse;
	reg [15:0]		write_coeff;

	reg	[1:0]		last_cons;
	reg	[PWM_BITS-1:0]	sd_counter_i, sd_counter_q;

	////////////////////////////////////////////////////////////////////////
	//
	// Incoming wishbone interface
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(posedge i_clk)
		reset_audio <= i_wb_stb && i_wb_we
					&& i_wb_addr == 0 && i_wb_data[31];
	always @(posedge i_clk)
		reset_pulse <= i_wb_stb && i_wb_we
					&& i_wb_addr == 1 && i_wb_data[31];

	always @(posedge i_clk)
	begin
		write_audio <= (i_wb_stb && i_wb_we && i_wb_addr == 0);
		write_pulse <= (i_wb_stb && i_wb_we && i_wb_addr == 1);
		write_coeff <= i_wb_data[15:0];
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Generate a RAW_DATA_RATE CE signal
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	initial	{ mic_ce, mic_ce_counter } = 0;
	always @(posedge i_clk)
	if (!i_audio_en)
		{ mic_ce, mic_ce_counter } <= 0;
	else
		{ mic_ce, mic_ce_counter } <= mic_ce_counter + MIC_STEP;

	//
	// A/D to get audio in
	//
	smpladc #(.CKPCK(2))
	audio_adc(i_clk, mic_ce, mic_ce, i_audio_en,
		o_mic_csn, o_mic_sck, i_mic_miso,
		{ mic_ignore, mic_valid, mic_sample });

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Downsampler -- to go from RAW_DATA_RATE_HZ to our internal data rate
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	subfildown #(.IW(MIC_BITS), .OW(AUDIO_BITS), .CW(12),
		.NDOWN(RAW_AUDIO_DOWNSAMPLE_RATIO),
		.FIXED_COEFFS(1'b0),
		.NCOEFFS(NUM_AUDIO_COEFFS))
	resample_audio(i_clk, reset_audio, write_audio, write_coeff[15:4],
		mic_ce, mic_sample, audio_ce, audio_sample);
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// LFSR Scrambler
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	wire	[6:0]	scrambled_sample;

	scrambler #(.WS(7), .LN(31), .TAPS(31'h80_00_02_00))
	randomizer(i_clk, i_reset, audio_ce, audio_sample, scrambled_sample);

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Radio framing
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	reg		qpsk_ce;
	reg	[1:0]	qpsk_bits, qpsk_symbol;
	reg	[5:0]	qpsk_sreg;

	always @(posedge i_clk)
	if (audio_ce)
	begin
		qpsk_bits <= { !scrambled_sample[6], scrambled_sample[6] };
		qpsk_sreg <= scrambled_sample[5:0];
	end else if (qpsk_ce)
	begin
		qpsk_bits <= scrambled_sample[5:4];
		qpsk_sreg   <= { scrambled_sample[5:0], 2'b00 };
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Symbol to bitmap
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// Our new bits specify how to rotate our position from the last
	// constellation point.
	//
	//	01	|	00
	//	------------------
	//	11	|	10
	//
	always @(posedge i_clk)
	if (qpsk_ce)
	begin
		case({ qpsk_symbol, qpsk_bits })
		// No rotation
		4'b??00: qpsk_symbol <= qpsk_symbol;
		// Rotate 90 degrees clockwise
		4'b0001: qpsk_symbol <= 2'b01;
		4'b0101: qpsk_symbol <= 2'b11;
		4'b1001: qpsk_symbol <= 2'b10;
		4'b1101: qpsk_symbol <= 2'b00;
		// Rotate 180 degrees clockwise
		4'b??11: qpsk_symbol <= ~qpsk_symbol;
		// Rotate 270 degrees clockwise
		4'b0010: qpsk_symbol <= 2'b10;
		4'b0110: qpsk_symbol <= 2'b00;
		4'b1010: qpsk_symbol <= 2'b01;
		4'b1110: qpsk_symbol <= 2'b11;
		endcase
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Pulse shaping, baseband waveform generation
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	localparam	LGFIFO = 4;
	wire	[LGFIFO:0]	bbfifo_fill;
	wire			bbfifo_full, bbfifo_empty;
	reg			baseband_ce;
	reg	[$clog2(CLOCK_FREQUENCY_HZ / BASEBAND_SAMPLE_RATE_HZ):0]
				baseband_counter;
	reg	[1:0]		baseband_pulses;
	wire	[BB_BITS-1:0]	pulse_i, pulse_q, baseband_i, baseband_q;
	wire			pulse_ready, pulse_ce;

	pulseshaperiq #(.NUP(LGFLEN), .NCOEFFS(31), .IW(2), .OW(BB_BITS),
		.FIXED_COEFFS(1'b0)
		//,  .INITIAL_COEFFS(PULSE_SHAPE_FILTER)
	) genpulseiq(i_clk, reset_pulse, write_pulse, write_coeff,
			qpsk_ce, pulse_ready,
					{ qpsk_symbol[1], 1'b1 },
					{ qpsk_symbol[0], 1'b1 },
				pulse_ce, pulse_i, pulse_q);

	sfifo #(.BW(2*BB_BITS), .LGFLEN(LGFIFO),
		.OPT_ASYNC_READ(1'b0)
		// .OPT_WRITE_ON_FULL(1'b0),
		// .OPT_READ_ON_EMPTY(1'b0)
	) stretch( i_clk, i_reset, pulse_ce, { pulse_i, pulse_q }, bbfifo_full,
			bbfifo_fill,
			baseband_ce, { baseband_i, baseband_q }, bbfifo_empty);

	initial	baseband_ce = 1'b0;
	always @(posedge i_clk)
	if (qpsk_ce)
	begin
		baseband_ce <= 1'b1;
		baseband_counter <= (CLOCK_FREQUENCY_HZ
						/ BASEBAND_SAMPLE_RATE_HZ);
		baseband_pulses <= 3;
	end else if (baseband_pulses > 0)
	begin
		baseband_ce <= 1'b0;
		if (baseband_counter <= 1)
		begin
			baseband_ce <= 1;
			baseband_counter <= (CLOCK_FREQUENCY_HZ
						/ BASEBAND_SAMPLE_RATE_HZ);
			baseband_pulses <= baseband_pulses - 1;
		end else
			baseband_counter <= baseband_counter - 1;
	end else
		baseband_ce <= 1'b0;

	// Verilator lint_on UNUSED
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Module the radio framing signals up to bandpass
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	//
	// No baseband to bandpass translation necessary--this radio chip
	// accepts baseband I+Q waveforms as is
	//
	// assign	bandpass_ce = baseband_ce;
	// assign	bandpass_i  = baseband_i;
	// assign	bandpass_q  = baseband_q;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Sigma delta conversion
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	reg	[PWM_BITS-1:0]	sdi_integrator, sdq_integrator;

	always @(posedge i_clk)
		sdi_integrator <= sdi_integrator
			+ { {(PWM_BITS-BB_BITS){baseband_i[BB_BITS-1]}},
					baseband_i }
				+ { !sdi_integrator[15], {(PWM_BITS-1){1'b0}} };

	always @(posedge i_clk)
		sdq_integrator <= sdq_integrator
			+ { {(PWM_BITS-BB_BITS){baseband_q[BB_BITS-1]}},
					baseband_q }
				+ { !sdq_integrator[15], {(PWM_BITS-1){1'b0}} };

	always @(posedge i_clk)
	if (i_rf_en)
		o_rf_data <= { sdi_integrator[15], sdq_integrator[15] };
	else
		o_rf_data <= ~o_rf_data;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Debug signal choices
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(posedge i_clk)
	case(i_dbg_sel)
	// Microphone
	2'b00: { o_dbg_ce, o_dbg_data, o_dbg_hist } <= { mic_ce,
			{(32-MIC_BITS){mic_sample[MIC_BITS-1]}}, mic_sample,
			mic_sample[MIC_BITS-1:MIC_BITS-HIST_BITS] };
	// Downsampled Audio
	2'b01: { o_dbg_ce, o_dbg_data, o_dbg_hist } <= { audio_ce,
			{(32-AUDIO_BITS){audio_sample[AUDIO_BITS-1]}},  
			audio_sample,
			audio_sample[AUDIO_BITS-1:AUDIO_BITS-HIST_BITS] };
	// QPSK Bitstream
	2'b10: { o_dbg_ce, o_dbg_data, o_dbg_hist } <= { qpsk_ce,
			14'h0, qpsk_symbol, 14'h0, qpsk_bits,
			qpsk_symbol[1], {(HIST_BITS/2-1){1'b0}},
			qpsk_symbol[0], {(HIST_BITS/2-1){1'b0}} };
	// Baseband sample stream
	2'b11: { o_dbg_ce, o_dbg_data, o_dbg_hist } <= { baseband_ce,
			{(16-BB_BITS){baseband_i[BB_BITS-1]}},
				baseband_i[BB_BITS-1:0],
			{(16-BB_BITS){baseband_q[BB_BITS-1]}},
				baseband_q[BB_BITS-1:0],
			baseband_i[BB_BITS-1:BB_BITS-HIST_BITS/2],
			baseband_q[BB_BITS-1:BB_BITS-HIST_BITS/2] };
	endcase
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Wishbone return
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(*)
		o_wb_stall = 1'b0;

	initial	o_wb_ack = 0;
	always @(posedge i_clk)
		o_wb_ack <= i_wb_stb && !i_reset;

	always @(*)
		o_wb_data = o_dbg_data;
	// }}}
	// Verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0 };
	// Verilator lint_on  UNUSED
endmodule
