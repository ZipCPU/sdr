////////////////////////////////////////////////////////////////////////////////
//
// Filename:	constellation.cpp
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
// Copyright (C) 2020-2024, Gisselquist Technology, LLC
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
#include "hexbus.h"

FPGA	*m_fpga;
void	closeup(int v) {
	m_fpga->kill();
	exit(0);
}

void	usage(void) {
	printf("USAGE: constellation\n");
}

int main(int argc, char **argv) {
	const char *host = FPGAHOST;
	int	port=FPGAPORT;
	unsigned	hbuf[1024];
	int	x, y;
	int	con[32][32];


	m_fpga = new FPGA(new NETCOMMS(host, port));

	signal(SIGSTOP, closeup);
	signal(SIGHUP, closeup);

	m_fpga->readi(R_HISTOGRAM, 1024, hbuf);

	for(int k=0; k<1024; k++) {
		x = (k>>5) & 0x1f;
		y = k & 0x1f;

		con[y][x] = hbuf[k];
	}

	for(int y=0; y<0x20; y++) {
		// {{{
		int	cy = y - 0x10;

		if (cy == 0)
			printf("-- ");
		else
			printf("   ");

		for(int x=0; x<0x20; x++) {
			int	cx = x - 0x10, cp;

			cp = con[cy][cx];
			if (cp == 0)
				printf(" ");
			else if (cp > 16)
				printf("X");
			else if (cp > 8)
				printf("x");
			else if (cp > 4)
				printf("o");
			else if (cp > 2)
				printf("*");
			else
				printf(".");
		}

		if (cy == 0)
			printf("-- ");
		else
			printf("   ");

		printf("\n");
		// }}}
	}

	FILE	*hp;
	hp = fopen("cons.bin","w");
	fwrite(hbuf, sizeof(int), 1024, hp);
	fclose(hp);

	if (m_fpga->poll())
		printf("FPGA was interrupted\n");
	delete	m_fpga;
}

