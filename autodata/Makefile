################################################################################
##
## Filename:	auto-data/Makefile
## {{{
## Project:	GDR, a basic Gateware Defined Radio architecture
##
## Purpose:	To coordinate the build of the glue logic holding this design
##		together via autofpga.
##
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2019-2024, Gisselquist Technology, LLC
## {{{
## This program is free software (firmware): you can redistribute it and/or
## modify it under the terms of the GNU General Public License as published
## by the Free Software Foundation, either version 3 of the License, or (at
## your option) any later version.
##
## This program is distributed in the hope that it will be useful, but WITHOUT
## ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
## for more details.
##
## You should have received a copy of the GNU General Public License along
## with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
## target there if the PDF file isn't present.)  If not, see
## <http://www.gnu.org/licenses/> for a copy.
## }}}
## License:	GPL, v3, as defined and found on www.gnu.org,
## {{{
##		http://www.gnu.org/licenses/gpl.html
##
################################################################################
##
## }}}
.PHONY: all
all:	data
#
# Could also depend upon load, if desired, but not necessary
#
# This is the list of components that will form our project.  Each file consists
# of parts and pieces of our project that autofpga will copy/paste into our
# main project files.
#

## Potential Transmitters
#
# RF := qamxmit.txt
# RF := amxmit.txt
# RF := fmxmit.txt
# RF := qpskxmit.txt

## Potential Receiver
#
# RF := amdemod.txt
# RF := fmdemod.txt
# RF := qpskrcvr.txt

## Full transmit/receiver chain
#
# RF := wavsim.txt
# RF := amsim.txt
# RF := fmsim.txt
RF := qpsksim.txt

DEBUG := histogram.txt rfscope.txt samplerate.txt
DATA := global.txt clock36.txt version.txt hexbus.txt gpio.txt $(RF) $(DEBUG)

AUTOFPGA := autofpga
.PHONY: data
data: $(DATA)
	$(AUTOFPGA) -d -o . $(DATA)

clean:
	rm -f toplevel.v main.v regdefs.h regdefs.cpp board.h board.ld
	rm -f rtl.make.inc main_tb.cpp
	rm -f autofpga.dbg
