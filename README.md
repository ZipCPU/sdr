# A Software Defined Radio Project for FPGAs

Okay, so ... FPGA designs aren't really software.  Perhaps this should more
appropriately be named a "Gateware Defined Radio" project--but that spoils the
idea.

The goal of this project is to build a gateware design that can transfer audio
from one radio to another.  The design is built around the
[icebreaker](https://github.com/icebreaker-fpga/icebreaker) FPGA board,
the [SX1257 Radio PMod](https://github.com/xil-se/SX1257-PMOD),
the [Digilent MEMS microphone](https://store.digilentinc.com/pmod-mic3-mems-microphone-with-adjustable-gain/)
and the [Digilent AMP2 PMod](https://store.digilentinc.com/pmod-amp2-audio-amplifier/).
You can see [a picture of this setup here](doc/radio-set.jpg).

## Building the design

This repository actually contains several designs.  Only one of them at present
even (partially) works.  There's an AM transmitter, an FM transmitter, an AM
receiver, and an FM receiver.  Which design gets built into the repository
is controlled by which line "RF" line in the [AutoFPGA
Makefile](autodata/Makefile) is uncommented.

To build, you'll first need [AutoFPGA (dev
branch)](https://github.com/ZipCPU/autofpga) installed and in your path.
You can then run `make autodata` to build the top level design, PCF, simulation,
and software data files.  From here, `make rtl` will build the design,
`make sim` will build the simulator, and `make sw` will build the host support
software.

Beware, if you use any of the composed simulations, such as the AM transmitter
followed by the AM demodulator, you will run into a problem where the microphone
and the amplifier are both attempting to use the same pins.  The solution
is not to build the composed (transmit/receive) design in rtl.  Instead,
run `make rtl-sim` to build the full simulation.

## Running in Simulation

To run the design in simulation, simply run the `main_tb` file in the `sim/`
directory.  This isn't very useful at present, however, unless you add the
`-d` switch to turn on VCD file generation.  You can then examine all the
traces from within the design.

Be aware, the simulation has no channel model.  Outputs to the
SX1257
are simply fed back into the simulation receiver.

## Running on hardware

To run the design on an icebreaker, you can load the design via
`sudo iceprog rtl/sdr.bin`.  Once you do that, you'll want to then run
`sw/netuart /dev/ttyUSB?` (replace with your serial port device).  At this
point, you can interact with your design using `wbregs`.  Perhaps more
importantly, you can interact with the registers on the SX1257
using the `rfregs`.  In particular, `txconfig.sh` will turn the transmitter
on, and set it up at 915MHz.

## Debugging within hardware

This design offers two methods for debugging: capturing traces, and capturing
histograms from within the data flow.  Two bits of the GPIO register (three
for the TX/RX composed simulations) control a tap point selection within the
design.  This tap point contains a CE signal, 32'bits of data, and 10 bits
of data for a histogram.

To record a trace from wtihin the hardware, map the signals you wish to capture
to a Wishbone Scope C++ file--such as [this one for the
microphone](sw/micscope.cpp).  You can then use `wbregs rfscope 0` to reset
the scope any time you want to make a new capture, and
[micscope](sw/micscope.cpp) to extract the captured dasta and then to turn
it into a VCD file you can then analyze with GTKWave.

Alternatively, you can capture histograms from within the design.  These
can be read with the [histogram](sw/histogram.cpp) program.  This program will
also output a binary file of 32-bit integers containing your histogram.
(The histogram is currently hardcoded at 1024 words, due to hardware limitations
in the iCE40 up5k.)

With a little creativity, the histogram capture utility can be turned into
a constellation capture utility.  By sending 5-bits of I and 5-bits of Q
into the histogram, and then interpreting the results accordingly, you can
capture (and then plot) a constellation diagram.  This is the purpose of
the [constellation](sw/constellation.cpp) program.

## Project Status

As of 20200204, ...

1. The AM receiver works when composed with the transmitter in simulation.
	The simulation has no channel model, is only so good, etc., etc.
	It's a start.
2. Both FM transmit/receive pair work when composed together in simulation.
3. The AM transmitter works, and the results can be heard using a Lime
	SDR radio receiver.  There's an annoying tone present which I haven't
	yet chased down.

## License

This project and all of its files are licensed under the GNU General
Public License, v3.
