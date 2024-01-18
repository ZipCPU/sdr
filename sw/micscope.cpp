////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	micscope.cpp
// {{{
// Project:	SDR, a basic Soft(Gate)ware Defined Radio architecture
//
// Purpose:	
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2019-2024, Gisselquist Technology, LLC
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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <strings.h>
#include <ctype.h>
#include <string.h>
#include <signal.h>
#include <assert.h>

#include "port.h"
#include "regdefs.h"
#include "scopecls.h"

#ifdef	R_RFSCOPE

#include "hexbus.h"

#define	WBSCOPE		R_RFSCOPE
#define	WBSCOPEDATA	R_RFSCOPED

FPGA	*m_fpga;
void	closeup(int v) {
	m_fpga->kill();
	exit(0);
}

#define	BIT(V,N)	((V>>N)&1)
#define	BITV(N)		BIT(val,N)

class	MICSCOPE : public SCOPE {
public:
	MICSCOPE(FPGA *fpga, unsigned addr, bool vecread)
		: SCOPE(fpga, addr, false, false) {};
	~MICSCOPE(void) {}
	virtual	void	decode(DEVBUS::BUSW val) const {
		// {{{
		// int	trig;
		int	rf, sample, csn, sck, miso, ce, valid, audioen, rfen,
			micdata;

		rf        = (val >> 29) & 0x03;
		sample    = (val >> 20) & 0x03ff;
		csn       = BITV(18);
		sck       = BITV(17);
		miso      = BITV(16);
		ce        = BITV(15);
		valid     = BITV(14);
		audioen   = BITV(13);
		rfen      = BITV(12);
		micdata = val & 0x0fff;

		printf("%s%s %s | %s%s (%s%s) -> %s%3x%s | %3x -> %s%s\n",
			(csn)?"   ":"CSN", (sck)?"SCK":"   ", miso ? "1":"0",
			(ce) ? "CE":"  ",(valid) ?"VL":"  ",
			(audioen)?"AU":"--", (rfen)?"RF":"--",
			(ce)?"0x":"(  ", micdata, (ce)?" ": "?",
			sample, (rf & 2)?"I":"-", (rf&1)?"Q":"-");
		// }}}
	}

	virtual	void	define_traces(void) {
		// {{{
		register_trace("o_rf_data",  2, 29);
		register_trace("sample_data_off", 10, 20);
		register_trace("o_mic_csn",  1, 18);
		register_trace("o_mic_sck",  1, 17);
		register_trace("i_mic_miso", 1, 16);
		register_trace("mic_ce",     1, 15);
		register_trace("mic_valid",  1, 14);
		register_trace("i_audio_en", 1, 13);
		register_trace("i_rf_en",    1, 12);
		register_trace("mic_data",  12,  0);
		// }}}
	}
};

void	usage(void) {
	printf("USAGE: micscope\n");
}

int main(int argc, char **argv) {
	const char *host = FPGAHOST;
	int	port=FPGAPORT;

	m_fpga = new FPGA(new NETCOMMS(host, port));

	MICSCOPE *scope = new MICSCOPE(m_fpga, WBSCOPE, false);
	scope->set_clkfreq_hz(36000000);
	if (!scope->ready()) {
		printf("Scope is not yet ready:\n");
		scope->decode_control();
	} else {
		scope->print();
		scope->writevcd("micscope.vcd");
	}
	delete	m_fpga;
}

#else // RFSCOPE
int main(int argc, char **argv) {
	fprintf(stderr, "No RF Scope enabled\n");
	exit(EXIT_FAILURE);
}
#endif
