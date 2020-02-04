////////////////////////////////////////////////////////////////////////////////
//
// Filename:	automaster_tb.cpp
//
// Project:	SDR, a basic Soft(Gate)ware Defined Radio architecture
//
// Purpose:	This file calls and accesses the main.v function via the
//		MAIN_TB class found in main_tb.cpp.  When put together with
//	the other components here, this file will simulate (all of) the
//	host's interaction with the FPGA circuit board.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2019-2020, Gisselquist Technology, LLC
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
#include <signal.h>
#include <time.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>

#include "verilated.h"
#include "design.h"

// #define	TRACE_FST

#include "testb.h"
// #include "twoc.h"

#include "port.h"

#include "main_tb.cpp"

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

int	main(int argc, char **argv) {
	Verilated::commandArgs(argc, argv);

	const	char *trace_file = NULL; // "trace.vcd";
	bool	debug_flag = false;

	MAINTB	*tb = new MAINTB;

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

	if (debug_flag) {
		printf("Opening design with\n");
		printf("\tDebug Access port = %d\n", FPGAPORT); // fpga_port);
		printf("\tVCD File         = %s\n", trace_file);
	} if (trace_file)
		tb->opentrace(trace_file);

	tb->reset();

	while(true)
		tb->tick();

	tb->close();
	delete tb;

	return	EXIT_SUCCESS;
}
