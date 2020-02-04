#!/bin/bash

./rfregs -c
# ./rfregs regmode	0x01	# Enable standby
# ./rfregs frftx	0xcb5555	# = 915MHz, can be offset in steps of 68.66455Hz
# ./rfregs txbw		0x00	# Transmit PLL bandwidth = 75kHz
# ./rfregs diomapping	0
# ./rfregs clkselect	1
# ./rfregs regmode	0x0c	# Enable transmit/broadcast

./rfregs regmode   0x8d
./rfregs clkselect 0x92

#
# Turn on the audio, transmitter, and LED
./wbregs gpio 0x340034

grep AMGAIN regdefs.h > /dev/null && \
	echo "To start the AM modulator, type: ./wbregs amgain 0x10003e00"
