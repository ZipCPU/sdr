////////////////////////////////////////////////////////////////////////////////
//
// Filename:	histogram.cpp
//
// Project:	SDR, a basic Soft(Gate)ware Defined Radio architecture
//
// Purpose:	To give a user access, via a command line program, to read
//		and write wishbone registers one at a time.  Thus this program
//	implements readio() and writeio() but nothing more.
//
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
#include "hexbus.h"

FPGA	*m_fpga;
void	closeup(int v) {
	m_fpga->kill();
	exit(0);
}

void	usage(void) {
	printf("USAGE: histogram\n");
}

int main(int argc, char **argv) {
	const char *host = FPGAHOST;
	int	port=FPGAPORT;
	unsigned	hbuf[1024];
	int		lastzero = 0, sum = 0;

	m_fpga = new FPGA(new NETCOMMS(host, port));

	signal(SIGSTOP, closeup);
	signal(SIGHUP, closeup);

	m_fpga->readi(R_HISTOGRAM, 1024, hbuf);
	lastzero = 0;
	sum = 0;
	for(int k=0; k<1024; k++) {
		sum = sum + hbuf[k];
		if (hbuf[k] == 0)
			lastzero = 1;
		else {
			if (lastzero)
				printf("  *****\n");
			printf("@%4d #%6d: ", k, hbuf[k]);
			for(int j=0; j<64; j++)
				if (hbuf[k] > (unsigned)j*(2048/ 64))
					printf("+");
			printf("\n");
			lastzero = 0;
		}
	} if (lastzero)
		printf("  *****\n");

	printf("Total sum: %5d\n", sum);

	FILE	*hp;
	hp = fopen("hist.bin","w");
	fwrite(hbuf, sizeof(int), 1024, hp);
	fclose(hp);

	if (m_fpga->poll())
		printf("FPGA was interrupted\n");
	delete	m_fpga;
}

