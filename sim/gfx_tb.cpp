////////////////////////////////////////////////////////////////////////////////
//
// Filename:	gfx_tb.cpp
// {{{
// Project:	SDR, a basic Soft(Gate)ware Defined Radio architecture
//
// Purpose:	This file is identical to automaster_tb, and a replacement
//		for it, save that it works with a GtkMM graphics library.
//	As such, it calls and accesses the main.v function via the MAIN_TB
//	class found in main_tb.cpp.  When put together with the other
//	components here, this file will simulate (all of) the host's
//	interaction with the FPGA circuit board and plot various components,
//	parts and pieces  of that interaction, onto a local window.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2019-2021, Gisselquist Technology, LLC
// {{{
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
// }}}
// License:	GPL, v3, as defined and found on www.gnu.org,
// {{{
//		http://www.gnu.org/licenses/gpl.html
//
////////////////////////////////////////////////////////////////////////////////
//
// }}}
#include <signal.h>
#include <time.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>

#include <gtkmm.h>

#include <sigwin.h>
#include <plotwin.h>
#include <conplot.h>
#include <cxspecan.h>
#include <idler.h>

#include "verilated.h"
#include "design.h"

// #define	TRACE_FST

#include "testb.h"
#include "twoc.h"
// #include "twoc.h"

#include "port.h"

#include "main_tb.cpp"

#define	TXBITS	0
#define	TXCON	1
#define	TXBB	2
#define	TXRAW	3
#define	RXRAW	4
#define	RXCIC	5
#define	RXBB	6
#define	RXCON	7
#define	RXBITS	8

#define	RAWLEN		(65536*8)
#define	TXCONLEN	(RAWLEN/4)
#define	TXBBLEN		(RAWLEN/4)
#define	RXCICLEN	(RAWLEN/16)
#define	RXBBLEN		(RAWLEN/16)
#define	RXCONLEN	(RAWLEN/16)
#define	CORLLEN		(RAWLEN/64)

void	usage(void) {
	fprintf(stderr, "USAGE: main_tb <options>\n");
	fprintf(stderr,
// -h
// -p # command port
// -s # serial port
// -f # profile file
"\t-d\tSets the debugging flag\n"
);
}

class	TBTASK : public IDLER {
// {{{
public:
	// Public declarations
	// {{{
	char	*m_name;
	MAINTB	*m_tb;
	
	SIGWIN<CONPLOT>	*m_rxconplot;
	SIGWIN<PLOTWIN>	*m_spectra, *m_corl;
	CXSPECAN	*m_txbb_specan, *m_raw_specan,
			*m_rxcic_specan, *m_rxbb_specan;
	CONPLOT		*m_rxcon;
	COMPLEX		*m_txbbsyms, *m_txbbcon;
	COMPLEX		*m_rawsyms;
	COMPLEX		*m_rxcicsyms;
	COMPLEX		*m_rxbbsyms;
	COMPLEX		*m_rxconsyms, *m_rxsyms;
	double		*m_corlre, *m_corlim;
	// }}}

	TBTASK(void) {
		// {{{
		m_tb = new MAINTB;

		m_rxconplot = new SIGWIN<CONPLOT>(m_rxcon = new CONPLOT(1024));
		m_spectra   = new SIGWIN<PLOTWIN>(new PLOTWIN(RXBITS+1));
		if (false)
			m_corl      = new SIGWIN<PLOTWIN>(new PLOTWIN(RXBITS+1));
		else
			m_corl = NULL;

		m_txbb_specan = new CXSPECAN(m_spectra->m_plot, RXBITS+1, NULL);
		m_txbb_specan->sample_rate(1./17./8.);
		m_txbb_specan->dB(true);
		m_raw_specan = new CXSPECAN(m_spectra->m_plot, RXBITS+1, NULL);
		m_raw_specan->sample_rate(1.);
		m_raw_specan->dB(true);
		m_rxcic_specan = new CXSPECAN(m_spectra->m_plot, RXBITS+1, NULL);
		m_rxcic_specan->sample_rate(1./17.);
		m_rxcic_specan->dB(true);
		m_rxbb_specan  = new CXSPECAN(m_spectra->m_plot, RXBITS+1, NULL);
		m_rxbb_specan->sample_rate(1./17./4.);
		m_rxbb_specan->dB(true);

		m_spectra->m_plot->set_color(TXBB,  0.00, 0.00, 0.00);
		m_spectra->m_plot->set_color(TXRAW, 0.70, 0.70, 0.70);
		m_spectra->m_plot->set_color(RXCIC, 0.75, 0.75, 0.25);
		m_spectra->m_plot->set_color(RXBB,  1.00, 0.00, 0.00);
		m_spectra->title("Frequency Analysis");
		m_rxconplot->title("RX Constellation");
		if (m_corl) {
			m_corl->m_plot->set_color(0, 1.00, 0.00, 0.00);
			m_corl->m_plot->set_color(0, 0.00, 1.00, 0.00);
			m_corl->title("Correlations");
		}

		m_txbbcon   = new COMPLEX[TXCONLEN];
		m_txbbsyms  = new COMPLEX[TXBBLEN];
		m_rawsyms   = new COMPLEX[RAWLEN];	// 17 * RXCIC
		m_rxcicsyms = new COMPLEX[RXCICLEN];	//  8 * TXBB
		m_rxbbsyms  = new COMPLEX[RXBBLEN];	// 
		m_rxsyms    = new COMPLEX[RXCONLEN];
		m_rxconsyms = new COMPLEX[RXCONLEN];
		m_corlre    = new double[CORLLEN];
		m_corlim    = new double[CORLLEN];
		// }}}
	}

	void	opentrace(const char *vcdname) {
		m_tb->opentrace(vcdname);
	}

	void	reset(void) {
		m_tb->reset();
	}

	// quantum() -- main idler work function, called by window system
	// {{{
	virtual	bool	quantum(void) {
		int	txconpts = 0, txbbpts=0, rawpts=0,
			rxcicpts=0, rxbbpts=0, rxsympts = 0, rxconpts = 0;

		for(int k=0; k<RAWLEN; k++) {
			m_tb->tick();

			if (m_tb->m_core->main__DOT__qpskxmiti__DOT__qpsk_ce) {
				m_txbbcon[txconpts] = COMPLEX(
					(m_tb->m_core->main__DOT__qpskxmiti__DOT__qpsk_symbol & 2) ? -1.:1.,
					(m_tb->m_core->main__DOT__qpskxmiti__DOT__qpsk_symbol & 1) ? -1.:1.);
				txconpts++;
			}

			if (m_tb->m_core->main__DOT__qpskxmiti__DOT__baseband_ce) {
				m_txbbsyms[txbbpts] = COMPLEX(
					(double)(sbits(m_tb->m_core->main__DOT__qpskxmiti__DOT__baseband_i,12)),
					(double)(sbits(m_tb->m_core->main__DOT__qpskxmiti__DOT__baseband_q,12)));
				txbbpts++;
			}

			m_rawsyms[rawpts] = COMPLEX(
				(m_tb->m_core->o_rf_data & 2) ? -1.0 : 1.0,
				(m_tb->m_core->o_rf_data & 1) ? -1.0 : 1.0);
			rawpts++;

			if (m_tb->m_core->main__DOT__qpskrcvri__DOT__cic_ce) {
				m_rxcicsyms[rxcicpts] = COMPLEX(
					(double)(sbits(m_tb->m_core->main__DOT__qpskrcvri__DOT__cic_sample_i,7)),
					(double)(sbits(m_tb->m_core->main__DOT__qpskrcvri__DOT__cic_sample_q,7)));
				rxcicpts++;
			}

			if (m_tb->m_core->main__DOT__qpskrcvri__DOT__baseband_ce) {
				m_rxbbsyms[rxbbpts] = COMPLEX(
					(double)(sbits(m_tb->m_core->main__DOT__qpskrcvri__DOT__baseband_i,7)),
					(double)(sbits(m_tb->m_core->main__DOT__qpskrcvri__DOT__baseband_q,7)));
				rxbbpts++;
			}

			if (m_tb->m_core->main__DOT__qpskrcvri__DOT__symbol_ce) {
				m_rxsyms[rxsympts] = COMPLEX(
					(double)(sbits(m_tb->m_core->main__DOT__qpskrcvri__DOT__symbol_i,7)),
					(double)(sbits(m_tb->m_core->main__DOT__qpskrcvri__DOT__symbol_q,7)));
				rxsympts++;
			}

			// if (m_tb->m_core->main__DOT__qpskrcvri__DOT__symbol_ce)
			if (m_tb->m_core->main__DOT__qpskrcvri__DOT__rmc_done){
				m_rxconsyms[rxconpts] = COMPLEX(
					(double)(sbits(m_tb->m_core->main__DOT__qpskrcvri__DOT__cons_i,8)),
					(double)(sbits(m_tb->m_core->main__DOT__qpskrcvri__DOT__cons_q,8)));
				m_rxconsyms[rxconpts] /= 32.0;
				rxconpts++;
			}

		}

		assert(rawpts   <= RAWLEN);
		assert(txbbpts  <= TXBBLEN);
		assert(txconpts <= TXCONLEN);
		assert(rxcicpts <= RXCICLEN);
		assert(rxbbpts  <= RXBBLEN);
		assert(rxsympts <= RXCONLEN);
		assert(rxconpts <= RXCONLEN);

		m_raw_specan->write(  TXRAW, rawpts,   m_rawsyms);
		m_txbb_specan->write( TXBB,  txbbpts,  m_txbbsyms);
		m_rxcic_specan->write(RXCIC, rxcicpts, m_rxcicsyms);
		m_rxbb_specan->write( RXBB,  rxbbpts,  m_rxbbsyms);
		m_rxcon->write(rxconpts, m_rxconsyms);

		if (m_corl && !m_corl->m_plot->paused())  {
			COMPLEX	*cbuf;
#ifdef	CORRELATE_RESULTS
			COMPLEX	txc[CORLLEN];

			for(int k=0; k< CORLLEN; k++)
				txc[k] = 0.0;
			for(int k=0; k<txconpts && k * 8 < CORLLEN; k++)
				txc[8*k] = m_txbbcon[k];
			for(int k=rxbbpts; k<CORLLEN; k++)
				m_rxbbsyms[k] = 0;

			cfft(m_txc, CORLLEN);
			cfft(m_rxbbsyms, CORLLEN);

			for(int k=0; k<CORLLEN; k++)
				m_rxbbsyms[k] *= txc[k].conj();
			cbuf = m_rxbbsyms;
#else
			for(int k=txconpts; k<CORLLEN; k++)
				m_txbbcon[k] = 0;
			// for(int k=0; k<rxconpts; k++)
			//	m_rxconsyms[k].im() = -m_rxconsyms[k].im();
			for(int k=rxsympts; k<CORLLEN; k++)
				m_rxsyms[k] = 0;

			cfft(m_txbbcon, CORLLEN);
			cfft(m_rxsyms, CORLLEN);

			for(int k=0; k<CORLLEN; k++)
				m_rxsyms[k] *= m_txbbcon[k].conj();
			cbuf = m_rxsyms;
#endif
			icfft(cbuf, CORLLEN);
			for(int k=0; k<CORLLEN; k++) {
				m_corlre[k] = cbuf[k].cre();
				m_corlim[k] = cbuf[k].cim();
			}
			m_corl->m_plot->write(0, 0, 1., CORLLEN/2., m_corlre, true);
			m_corl->m_plot->write(1, 0, 1., CORLLEN/2., m_corlim, true);
		}

		printf("TRACK: SYM %4x/%d/%d, FC %4x\n",
			m_tb->m_core->main__DOT__qpskrcvri__DOT__symbol_pll__DOT__r_step,
			m_tb->m_core->main__DOT__qpskrcvri__DOT__pll_lgcoeff,
			m_tb->m_core->main__DOT__qpskrcvri__DOT__high_symbol_phase,
			m_tb->m_core->main__DOT__qpskrcvri__DOT__carrier_step);
			
		return true;
	}
	// }}}
// }}}
};

int	main(int argc, char **argv) {
	Gtk::Main	main_instance(argc, argv);
	Verilated::commandArgs(argc, argv);

	// whateverWIN	win(intro);
	const	char *trace_file = NULL; // "trace.vcd";
	bool	debug_flag = false;

	// Argument processing
	// {{{
	for(int argn=1; argn < argc; argn++) {
		if (argv[argn][0] == '-') for(int j=1;
					(j<512)&&(argv[argn][j]);j++) {
			switch(tolower(argv[argn][j])) {
			case 'd': debug_flag = true;
				if (trace_file == NULL)
					trace_file = "trace.vcd";
				break;
			// case 't': trace_file = argv[++argn]; j=1000; break;
			case 'h': usage(); exit(0); break;
			default:
				fprintf(stderr, "ERR: Unexpected flag, -%c\n\n",
					argv[argn][j]);
				usage();
				break;
			}
		} else {
			fprintf(stderr, "ERR: Unknown argument, %s\n", argv[argn]);
			exit(EXIT_FAILURE);
		}
	}
	// }}}

	if (debug_flag) {
		printf("Opening design with\n");
		printf("\tDebug Access port = %d\n", FPGAPORT); // fpga_port);
		printf("\tVCD File         = %s\n", trace_file);
	}

	TBTASK	*task = new TBTASK;

	if (trace_file)
		task->opentrace(trace_file);

/*
	while(true) {
		tb->tick();
	}

	tb->close();
	delete tb;
*/
	Gtk::Main::run();

	return	EXIT_SUCCESS;
}
