################################################################################
##
## Filename:	Makefile
## {{{
## Project:	SDR, a basic Soft(Gate)ware Defined Radio architecture
##
## Purpose:	
## Targets:	
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2019-2021, Gisselquist Technology, LLC
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
PROGRAMS := wbregs netuart rfregs histogram constellation
SCOPES := micscope
all: $(PROGRAMS) $(SCOPES)
CXX := g++
OBJDIR := obj-pc
BUS := hexbus
EXTSRCS := $(BUS).cpp
LCLSRCS := llcomms.cpp regdefs.cpp
BUSSRCS := $(LCLSRCS) hexbus.cpp llcomms.cpp
DEPSRCS := wbregs.cpp netuart.cpp histogram.cpp constellation.cpp $(BUSSRCS)
HEADERS := llcomms.h port.h scopecls.h devbus.h $(wildcard ../$(BUS)/sw/*.h)
BUSOBJS := $(addprefix $(OBJDIR)/,$(subst .cpp,.o,$(LCLSRCS) $(EXTSRCS)))
CFLAGS := -g -Wall -I. -I../rtl
LIBS :=
SUBMAKE := $(MAKE) --no-print-directory -C

.PHONY: objects
objects:
	echo $(BUSOBJS)

%.o: $(OBJDIR)/%.o
$(OBJDIR)/%.o: %.cpp
	$(mk-objdir)
	$(CXX) $(CFLAGS) -c $< -o $@
$(OBJDIR)/%.o: %.c
	$(mk-objdir)
	$(CXX) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -rf $(OBJDIR)/ $(PROGRAMS) a.out tags *.o

netuart: $(OBJDIR)/netuart.o
	$(CXX) $(CFLAGS) $^ -o $@
#
# Some simple programs that just depend upon the ability to talk to the FPGA,
# and little more. 
#
wbregs: $(OBJDIR)/wbregs.o $(BUSOBJS)
	$(CXX) $(CFLAGS) $^ $(LIBS) -o $@

rfregs: $(OBJDIR)/rfregs.o $(BUSOBJS)
	$(CXX) $(CFLAGS) $^ $(LIBS) -o $@

histogram: $(OBJDIR)/histogram.o $(BUSOBJS)
	$(CXX) $(CFLAGS) $^ $(LIBS) -o $@

constellation: $(OBJDIR)/constellation.o $(BUSOBJS)
	$(CXX) $(CFLAGS) $^ $(LIBS) -o $@

## SCOPES
# These depend upon the scopecls.o, the bus objects, as well as their
# main file(s).
# memscope: $(OBJDIR)/memscope.o $(OBJDIR)/scopecls.o $(BUSOBJS)
#	$(CXX) $(CFLAGS) $^ $(LIBS) -o $@
micscope: $(OBJDIR)/micscope.o $(OBJDIR)/scopecls.o $(BUSOBJS)
	$(CXX) $(CFLAGS) $^ $(LIBS) -o $@

define	mk-objdir
	@bash -c "if [ ! -e $(OBJDIR) ]; then mkdir -p $(OBJDIR); fi"
endef

define	build-depends
	@echo "Building dependency file(s)"
	$(mk-objdir)
	$(CXX) $(CFLAGS) -MM $(DEPSRCS) > $(OBJDIR)/xdepends.txt
	@sed -e 's/^.*.o: /$(OBJDIR)\/&/' < $(OBJDIR)/xdepends.txt > $(OBJDIR)/depends.txt
	@rm $(OBJDIR)/xdepends.txt
endef

tags: $(DEPSRCS) $(HEADERS)
	@echo "Generating tags"
	@ctags $(DEPSRCS) $(HEADERS)

.PHONY: depends
depends: $(OBJDIR)/depends.txt

$(OBJDIR)/depends.txt: $(DEPSRCS) $(HEADERS)
	$(build-depends)

-include $(OBJDIR)/depends.txt
