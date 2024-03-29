################################################################################
##
## Filename:	Makefile
## {{{
## Project:	SDR, a basic Soft(Gate)ware Defined Radio architecture
##
## Purpose:	To coordinate the build of a cycle accurate, Verilator based,
##		simulation of the main module.  Access to the simulation is
##	provided via the same software commands that will access the board, save
##	that the parameters are a touch different.  (See the access software for
##	more information ...)
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
# Make certain the "all" target is the first and therefore the default target
all:
CXX	:= g++
OBJDIR	:= obj-pc
RTLD	:= ../rtl
VOBJDR	:= $(RTLD)/obj_dir
ifneq ($(VERILATOR_ROOT),)
VERILATOR:=$(VERILATOR_ROOT)/bin/verilator
else
VERILATOR_ROOT ?= $(shell bash -c 'verilator -V|grep VERILATOR_ROOT | head -1 | sed -e " s/^.*=\s*//"')
endif
export	$(VERILATOR)
VROOT   := $(VERILATOR_ROOT)
VDEFS   := $(shell ./vversion.sh)
SIGLIBD := $(HOME)/src/siglib
SIGDPYD := $(HOME)/src/sigdisplay
GFXFLAGS:= -I $(SIGLIBD) -I $(SIGDPYD) `pkg-config gtkmm-3.0 --cflags`
GFXLIBS := $(SIGDPYD)/sigdpy.a $(SIGLIBD)/siglib.a `pkg-config gtkmm-3.0 --cflags --libs`
FLAGS	:= -Wall -Og -g $(VDEFS)
VINCD   := $(VROOT)/include
VINC	:= -I$(VINCD) -I$(VINCD)/vltstd -I$(VOBJDR)
INCS	:= -I. -I../sw -I$(RTLD) $(VINC)
#
# A list of our sources and headers
#
SOURCES := automaster_tb.cpp main_tb.cpp uartsim.cpp micnco.cpp
HEADERS := ../sw/port.h testb.h uartsim.h micnco.h
VOBJDR	:= $(RTLD)/obj_dir
VOBJS   := $(OBJDIR)/verilated.o $(OBJDIR)/verilated_vcd_c.o $(OBJDIR)/verilated_threads.o
VMAIN	:= $(VOBJDR)/Vmain__ALL.a
SIMSRCS := uartsim.cpp micnco.cpp twoc.cpp
SIMOBJ := $(subst .cpp,.o,$(SIMSRCS))
SIMOBJS:= $(addprefix $(OBJDIR)/,$(SIMOBJ))
#
PROGRAMS := main_tb
# Now the return to the "all" target, and fill in some details
all:	$(PROGRAMS) hex

$(OBJDIR)/%.o: %.cpp
	$(mk-objdir)
	$(CXX) $(FLAGS) $(INCS) -c $< -o $@

$(OBJDIR)/%.o: $(VINCD)/%.cpp
	$(mk-objdir)
	$(CXX) $(FLAGS) $(INCS) -c $< -o $@

.PHONY: hex
# hex: $(subst $(RTLD)/,,$(wildcard $(RTLD)/*.hex))
hex: sintable.hex amdemod.hex
# %.hex: $(RTLD)/%.hex
sintable.hex: $(RTLD)/sintable.hex
	@bash -c "if [ ! -e $@ ]; then ln -s $< $@ ; fi"
amdemod.hex: $(RTLD)/amdemod.hex
	@bash -c "if [ ! -e $@ ]; then ln -s $< $@ ; fi"

#
$(OBJDIR)/main_tb.o: automaster_tb.cpp
	$(mk-objdir)
	$(CXX) $(FLAGS) $(INCS) -c $< -o $@

$(OBJDIR)/gfx_tb.o: gfx_tb.cpp
	$(mk-objdir)
	$(CXX) $(FLAGS) $(GFXFLAGS) $(INCS) -c $< -o $@


main_tb: $(OBJDIR)/main_tb.o $(SIMOBJS) $(VMAIN) $(VOBJS)
	$(CXX) $(FLAGS) $(INCS) $^ -lelf -lpthread -o $@

gfx_tb: $(OBJDIR)/gfx_tb.o $(SIMOBJS) $(VMAIN) $(VOBJS)
	$(CXX) $(FLAGS) $(GFXFLAGS) $(INCS) $^ $(GFXLIBS) -lelf -lpthread -o $@

#
# The "clean" target, removing any and all remaining build products
#
.PHONY: clean
clean:
	rm -f *.vcd
	rm -f $(PROGRAMS)
	rm -rf $(OBJDIR)/

#
# The "depends" target, to know what files things depend upon.  The depends
# file itself is kept in $(OBJDIR)/depends.txt
#
define build-depends
	$(mk-objdir)
	@echo "Building dependency file"
	@$(CXX) $(VDEFS) $(GFXFLAGS) $(INCS) -MM $(SOURCES) > $(OBJDIR)/xdepends.txt
	@sed -e 's/^.*.o: /$(OBJDIR)\/&/' < $(OBJDIR)/xdepends.txt > $(OBJDIR)/depends.txt
	@rm $(OBJDIR)/xdepends.txt
endef

.PHONY: depends
depends: tags
	$(build-depends)

$(OBJDIR)/depends.txt: depends

#
define	mk-objdir
	@bash -c "if [ ! -e $(OBJDIR) ]; then mkdir -p $(OBJDIR); fi"
endef


#
# The "tags" target
#
tags:	$(SOURCES) $(HEADERS)
	@echo "Generating tags"
	@ctags $(SOURCES) $(HEADERS)


ifneq ($(MAKECMDGOALS),clean)
-include $(OBJDIR)/depends.txt
endif
