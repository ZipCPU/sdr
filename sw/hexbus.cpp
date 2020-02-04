////////////////////////////////////////////////////////////////////////////////
//
// Filename:	hexbus.cpp
//
// Project:	SDR, a basic Soft(Gate)ware Defined Radio architecture
//
// Purpose:	This is the C++ program on the command side that will interact
//		with a UART on an FPGA, to command the WISHBONE on that same
//	FPGA to ... whatever we wish to command it to do.  Interaction will
//	take place according to the hexbus protocol.
//
//	This code does not run on an FPGA, is not a test bench, neither is it a
//	simulator.  It is a portion of a command program for commanding an FPGA.
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
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h> 
#include <assert.h> 
#include <strings.h> 
#include <poll.h> 
#include <ctype.h> 

#include "hexbus.h"

#define	HEXB_ADDR	'A'
#define	HEXB_READ	'R'
#define	HEXB_IDLE	'Z'
#define	HEXB_ACK	'K'
#define	HEXB_RESET	'T'
#define	HEXB_INT	'I'
#define	HEXB_ERR	'E'

//
// Three mutually exclusive possibilities are here for tracing what's going on:
//
// 1. You can define DBGPRINTF to be printf
//	The trace file will be sent to stdout
//
// 2. You can define DBGPRINTF to be filedump
//	The tracefil will be sent to hexdebug.txt
//
// 3. You can leave DBGPRINTF undefined, or define it to be null
//	Either way, no debugging output will be produced
//
// #define	DBGPRINTF	printf
// #define	DBGPRINTF	filedump
//
//
#ifndef	DBGPRINTF
#define	DBGPRINTF	null
#else
#warning "HEXBUS DEBUG IS TURNED ON"
#endif

void	null(...) {}

bool	gbl_last_readidle = true;

#include <stdarg.h> // replaces the (defunct) varargs.h include file
void	filedump(const char *fmt, ...) {
	static	FILE *dbgfp = NULL;
	va_list	args;

	// Open the file if it isn't already open
	if (!dbgfp)
		dbgfp = fopen("hexdebug.txt", "w");
	if (!dbgfp) {
		fprintf(stderr, "ERR: Software trace file not opened\n");
		perror("O/S Err: ");
	}
	va_start(args, fmt);
	vfprintf(dbgfp, fmt, args);
	va_end(args);
	fflush(dbgfp);
	gbl_last_readidle = false;

	// If you want the debug output to go to stderr as well, you can
	// uncomment the next couple of lines
	// va_start(args, fmt);
	// vfprintf(stderr, fmt, args);
	// va_end(args);
}

/*
 * lclreadcode
 *
 * Read from our interface, and drop any idle characters (bottom seven bits
 * set) from any interaction.
 */
int	HEXBUS::lclreadcode(char *buf, int len) {
	char	*sp, *dp;
	int	nr, ret;

	nr = m_dev->read(buf, len);
	m_total_nread += nr;
	ret = nr; sp = buf; dp = buf;
	for(int i=0; i<nr; i++) {
		if ((buf[i]&0x7f)==0x7f) {
			// Idle insert, not a valid code word, skip it
			sp++;
		} else {
			*sp++ = *dp++;
		}
	} return ret;
}

/*
 * bufalloc
 *
 * Allocate4 a buffer of at least length (len).  This is similar to realloc().
 *
 */
void	HEXBUS::bufalloc(int len) {
	if ((m_buf)&&(m_buflen >= len))
		return;
	if (m_buf)
		delete[] m_buf;
	m_buflen = (len&(-0x3f))+0x40;
	m_buf = new char[m_buflen];
}

/*
 * writeio
 *
 * Write a single value to the debugging interface
 */
void	HEXBUS::writeio(const BUSW a, const BUSW v) {

	// We do all of our interaction using writev.  Here, we just set up a
	// writev call.
	writev(a, 0, 1, &v);
	m_lastaddr = a; m_addr_set = true;
}

/*
 * writev
 *
 * This internal write function.  This writes a buffer of information to our
 * interface, and the place to study how a write works.
 *
 * Parameters:
 *	a	is the address to write to
 *	p	=1 to increment address, 0 otherwise
 *	len	The number of values to write to the bus
 *	buf	A memory pointer to the information to write
 *
 * Notice that this routine can only write complete 32-bit words.  It doesn't
 * really have any 8-bit byte support, although you might be able to create such
 * by readio()'ing a word, modifying it, and then calling writeio() to write the
 * modified word back.
 */
void	HEXBUS::writev(const BUSW a, const int p, const int len,
		const BUSW *buf) {
	char	*ptr;
	unsigned	nw = 0;

	DBGPRINTF("WRITEV(%08x,%d,#%d,0x%08x ...)\n", a, p, len, buf[0]);

	// Encode the address
	ptr = encode_address(a|((p)?0:1));
	m_lastaddr = a; m_addr_set = true;
	m_nacks = 0;

	while(nw < (unsigned)len) {
		*ptr++ = 'W'; *ptr = '\0';
		if (buf[nw] != 0) {
			sprintf(ptr, "%x\n", buf[nw]);
			ptr += strlen(ptr);
		} else {
			*ptr++ = '\n';
			*ptr = '\0';
		}

		DBGPRINTF("WRITEV-SUB(%08x%s,&buf[%d] = 0x%08x,ACKS=%d)\n", a+(nw<<2), (p)?"++":"", nw, buf[nw], m_nacks);
		m_dev->write(m_buf, ptr-m_buf);
		DBGPRINTF(">> %s", m_buf);

		while(m_nacks < (unsigned)nw)
			readidle();

		nw ++;
		ptr = m_buf;
	}

	DBGPRINTF("Missing %d acks still\n", (unsigned)len-m_nacks);
	while(m_nacks < (unsigned)len)
		readidle();

	if (p)
		m_lastaddr += (len<<2);
	DBGPRINTF("WR: LAST ADDRESS LEFT AT %08x\n", m_lastaddr);
}

/*
 * writez
 *
 * Write a buffer of values to a single address.
 */
void	HEXBUS::writez(const BUSW a, const int len, const BUSW *buf) {
	writev(a, 0, len, buf);
}

/*
 * writei
 *
 * Write a buffer of values to a memory range.  Unlike writez, this function
 * increments the address pointer after every memory write.
 */
void	HEXBUS::writei(const BUSW a, const int len, const BUSW *buf) {
	writev(a, 1, len, buf);
}

/*
 * readio
 *
 * Read a single value from the bus.
 *
 * If the bus returns an error, this routine will print a comment and throw
 * the error up the chain.  If the address of the value read doesn't match
 * the address requested (an internal check), then an error message will be
 * sent to the log file and the interface will exit with an error condition.
 * This should only happen if there are bugs in the interface, and hopefully
 * I've gotten rid of all of those.
 *
 */
HEXBUS::BUSW	HEXBUS::readio(const HEXBUS::BUSW a) {
	BUSW	v;

	// I/O reads are now the same as vector reads, but with a vector length
	// of one.
	DBGPRINTF("READIO(0x%08x)\n", a);
	try {
		readv(a, 0, 1, &v);
	} catch(BUSERR b) {
		DBGPRINTF("READIO::BUSERR trying to read %08x\n", a);
		throw BUSERR(a);
	}

	if (m_lastaddr != a) {
		DBGPRINTF("LAST-ADDR MIS-MATCH: (RCVD) %08x != %08x (XPECTED)\n", m_lastaddr, a);
		m_addr_set = false;

		exit(EXIT_FAILURE);
	}

	return v;
}

/*
 * encode_address
 *
 * Creates a message to be sent across the bus with a new address value
 * in it.  If the low order bit of the address is set, then the address
 * will not increment as operations are applied.
 *
 * For the hexbus interface, this is just about as simple as a sprintf(),
 * although other interfaces are more complicated.
 */
char	*HEXBUS::encode_address(const HEXBUS::BUSW a) {
	char	*ptr = m_buf;


	if ((m_addr_set)&&(a == m_lastaddr)&&(m_inc == ((a&1)^1))) {
		DBGPRINTF("Address is already set to %08x\n", a);
		return ptr;
	}

	// An address starts with an address command word indicator
	*ptr++ = HEXB_ADDR;

	// Followed by the address in lower-case hex
	// While I hate providing *ALL EIGHT* hex digits to this function,
	// failing to do so causes overflows within the hexbus right now.
	sprintf(ptr, "%08x", a);

/*
	// If we can use an address difference, will it be valuable to do so?
	// Not quite yet in our current design.
	if (m_addr_set) {
		char	diff[9];

		sprintf(diff, "%x", a-m_lastaddr);

		if (strlen(diff) < strlen(ptr))
			strcpy(ptr, diff);
	}
*/
	ptr += strlen(ptr);
	*ptr = '\0';

	DBGPRINTF("ADDR-CMD: \'%s\' (a was %08x)\n", m_buf, a);

	return ptr;
}


/*
 * readv
 *
 * This is the main worker routine for read calls.  readio, readz, readi, all
 * end up here.  readv() reads a buffer of data from the given address, and
 * optionally increments (or not) the address after every read.
 *
 * Parameters:
 *	a	The address to start reading from
 *	inc	'1' if we want to increment the address following each read,
 *		'0' otherwise
 *	len	The number of words to read
 *	buf	A memory buffer storage location to place the results into
 * 
 */
void	HEXBUS::readv(const HEXBUS::BUSW a, const int inc, const int len, HEXBUS::BUSW *buf) {
	int	nread = 0;
	char	*ptr = m_buf;

	if (len <= 0)
		return;
	DBGPRINTF("READV(%08x,%d,#%4d)\n", a, inc, len);

	ptr = encode_address(a | ((inc)?0:1));
	m_lastaddr = a; m_addr_set = true; m_inc = inc;
	try {
	    while(nread < len) {
		*ptr++ = HEXB_READ; // This will be a read request

		// No other characters needed.  However, without a FIFO
		// we'll need to terminate this command and wait for a
		// response.
		*ptr++ = '\n';
		*ptr = '\0';

		m_dev->write(m_buf, (ptr-m_buf));

		// Read the result from the bus
		buf[nread++] = readword();
		DBGPRINTF("READV [%08x/%08x] = %08x\n", nread-1, len, buf[nread-1]);

		// Clear the command buffer so we can start over
		ptr = m_buf;
	    }
	} catch(BUSERR b) {
		DBGPRINTF("READV::BUSERR trying to read %08x\n", a+((inc)?(nread<<2):0));
		throw BUSERR(a+((inc)?(nread<<2):0));
	} catch(...) {
		DBGPRINTF("Some other error caught\n");
		assert(0);
	}

	// Make sure the address(es) we received were what we were expecting
	if ((unsigned)m_lastaddr != (a+((inc)?(len<<2):0))) {
		// Create and exit on an error if not.
		//
		// This will only ever happen if there is an error in the
		// dbgbus code
		DBGPRINTF("HEXBUS::READV(a=%08x,inc=%d,len=%4x,x) ERR: (Last) %08x != %08x + %08x (Expected)\n", a, inc, len<<2, m_lastaddr, a, (inc)?(len<<2):0);
		printf("HEXBUS::READV(a=%08x,inc=%d,len=%4x,x) ERR: (Last) %08x != %08x + %08x (Expected)\n", a, inc, len<<2, m_lastaddr, a, (inc)?(len<<2):0);
		fflush(stdout);
		assert((int)m_lastaddr == (a+(inc)?(len<<2):0));
		exit(EXIT_FAILURE);
	}

	DBGPRINTF("READV::COMPLETE, [%08x] -> %08x%s\n", a, buf[0],
		(len>1)?", ...":"");
}

/*
 * readi
 *
 * Read a series of values from bus addresses starting at address a,
 * incrementing the address to read from subsequent addresses along the way.
 *
 * Works by just calling readv to do the heavy lifting.
 */
void	HEXBUS::readi(const HEXBUS::BUSW a, const int len, HEXBUS::BUSW *buf) {
	readv(a, 1, len, buf);
}

/*
 * readi
 *
 * Read a series of values from the bus, with all the values coming from the
 * same address: a.  The address is not incremented between individual word
 * reads.
 *
 * Also calls readv to do the heavy lifting.
 */
void	HEXBUS::readz(const HEXBUS::BUSW a, const int len, HEXBUS::BUSW *buf) {
	readv(a, 0, len, buf);
}

/*
 * readword()
 *
 * Once the read command has been issued, readword() is called to read each
 * word's response from the bus.  This also processes any out of bounds
 * characters, such as interrupt notifications or bus error condition
 * notifications.
 */
HEXBUS::BUSW	HEXBUS::readword(void) {
	int		nr;
	unsigned	word, result, abort_countdown;
	bool		done = false;
	const bool	dbg  = false;

	if (dbg) DBGPRINTF("READ-WORD()\n");

	abort_countdown = 3;
	word = 0;
	do {
		// Blocking read (for now)
		do {
			nr = lclreadcode(&m_buf[0], 1);
		} while (nr < 1);
		if (dbg) DBGPRINTF("READWORD: -- lclreadcode, nr = %d, m_buf[0] = %c (%02x)\n",
				nr, isgraph(m_buf[0])?m_buf[0]:'.',
				m_buf[0] & 0x0ff);

		// If the character is a lower case hexadecimal digit, shift our
		// word by four bits and set the lower four bits with this
		// value.
		if (isdigit(m_buf[0])) {
			m_isspace = false;
			word = (word << 4) | (m_buf[0] & 0x0f);
		} else if ((m_buf[0] >= 'a')&&(m_buf[0] <= 'f')) {
			m_isspace = false;
			word = (word << 4) | ((m_buf[0] - 'a' + 10)&0x0f);
		} else {
			if (dbg) DBGPRINTF("RCVD OTHER-CHAR(%c), m_cmd = %02x (%c), word=0x%08x\n",
				isgraph(m_buf[0])?m_buf[0]:'.',
				m_cmd & 0x0ff, isgraph(m_cmd)?m_cmd:'.', word);
			if (m_isspace) {
				// Ignore multiple spaces
			} else if (m_cmd == HEXB_READ) {
				if (m_inc)
					m_lastaddr += 4;
				if (dbg) DBGPRINTF("RCVD WORD: 0x%08x\n", word);
				result = word;
				done = true;
			} else if (m_cmd == HEXB_ACK) {
				// Write acknowledgement
				if (m_inc)
					m_lastaddr += 4;
				m_nacks++;
			} else if (m_cmd == HEXB_INT) {
				m_interrupt_flag = true;
			} else if (m_cmd == HEXB_ERR) {
				if (dbg) DBGPRINTF("Bus error(0x%08x)-readword\n",m_lastaddr);
				m_bus_err = true;
				m_isspace = isspace(m_buf[0]);
				if (!m_isspace)
					m_cmd = m_buf[0];
				throw BUSERR(m_lastaddr);
			} else if (m_cmd == HEXB_IDLE) {
				abort_countdown--;
				if (0 == abort_countdown) {
					if (dbg) DBGPRINTF("Bus error(0x%08x,ABORT)\n",
						m_lastaddr);
					throw BUSERR(0);
				}
			} else if (m_cmd == HEXB_ADDR) {
				m_addr_set  = true;
				m_inc       = (word & 1) ? 0:1;
				m_lastaddr  = word & -4;
				if (dbg) DBGPRINTF("RCVD ADDR: 0x%08x%s\n", (word&-4),
					(m_inc)?" INC":"");
			} else if (m_cmd == HEXB_RESET) {
				m_addr_set = false;
			} else {
				if (dbg) DBGPRINTF("Other OOB info read, CMD = %c (%02x)\n", isgraph(m_cmd)?m_cmd:'.', m_cmd & 0x0ff);
			}

			// Any out of band character other than a newline is
			// a new command that we start here
			if (!isspace(m_buf[0])) {
				m_cmd = m_buf[0];
				if (dbg) DBGPRINTF("SETTING-NEW-CMD VALUE, CMD = %c (%02x)\n",
					isgraph(m_cmd)?m_cmd:'.', m_cmd & 0x0ff);
				m_isspace = false;
			} else m_isspace = true;

			// Clear the register so we can receive the next word
			word = 0;
		}
	} while(!done);

	return result;
}

/*
 * readidle()
 *
 * Reads until the bus becomes idle.  This is called by writev to make sure
 * any write acknowledgements are sufficiently flushed from the stream.  In
 * case anything else is in the stream ... we mostly ignore that here too.
 */
void	HEXBUS::readidle(void) {
	unsigned	word;

	if (!gbl_last_readidle) {
		DBGPRINTF("READ-IDLE()\n");
		gbl_last_readidle = true;
	}

	// Start by clearing the register
	word = 0;

	// Repeat as long as there are values to be read
	while(m_dev->available()) {
		// Read one character from the interface
		lclreadcode(&m_buf[0], 1);	// Number read must be one

		// If it's a hexadecimal digit, adjust our word register
		if (isdigit(m_buf[0]))
			word = (word << 4) | (m_buf[0] & 0x0f);
		else if ((m_buf[0] >= 'a')&&(m_buf[0] <= 'f'))
			word = (word << 4) | ((m_buf[0] - 'a' + 10)&0x0f);
		else if ((isspace(m_buf[0]))&&(m_isspace)) {
			// Ignore multiple spaces in a row
		} else {
			// Any thing else identifies the beginning (or end)
			// of a response word.  Deal with it based upon the
			// last response m_cmd received.
			if (m_isspace) {
				// Ignore the previous command if we've
				// already had a space between use
			} else if (m_cmd == HEXB_ADDR) {
				// Received an address word
				m_addr_set  = true;
				m_inc       = (word & 1) ? 0:1;
				m_lastaddr  = word & -4;
				DBGPRINTF("RCVD ADDR: 0x%08x%s\n", word&-3,
					(m_inc)?" INC":"");
			} else if (m_cmd == HEXB_READ) {
				// Read data ... doesn't make sense in this
				// context, so we'll just ignore it
				if (m_inc)
					m_lastaddr += 4;
			} else if (m_cmd == HEXB_INT) {
				// On an interrupt, just set the flag to note
				// we've received one.
				m_interrupt_flag = true;
			} else if (m_cmd == HEXB_ACK) {
				// Write acknowledgement.  writev() will check
				// whether the correct number of
				// acknoweledgments has been received before
				// moving on.  Read and note it here.
				if (m_inc)
					m_lastaddr += 4;
				m_nacks++;
			} else if (m_cmd == HEXB_ERR) {
				// On an err, throw a BUSERR exception
				DBGPRINTF("Bus error(%08x)-readidle\n", m_lastaddr);
				m_bus_err = true;
				throw BUSERR(m_lastaddr);
			} else if (m_cmd == HEXB_RESET) {
				DBGPRINTF("BUS RESET\n");
				// On any reset, clear the address set flag
				// and any unacknowledged bus error condition
				m_addr_set = false;
				m_bus_err = false;
			}

			// Any out of band character other than a whitespace
			// is a new command starting--keep track of which
			// command it is.
			if (!isspace(m_buf[0]))
				m_cmd = m_buf[0];
			m_isspace = (isspace(m_buf[0]))?true:false;
			word = 0;
		}
	}
}

/*
 * usleep()
 *
 * Called to implement some form of time-limited wait on a response from the
 * bus.
 */
void	HEXBUS::usleep(unsigned ms) {
	if (m_dev->poll(ms)) {
		int	nr;
		nr = m_dev->read(m_buf, 16);
		m_total_nread += nr;
		if (nr == 0) {
			// Connection closed, let it drop
			DBGPRINTF("Connection closed!!\n");
			m_dev->close();
			exit(-1);
		} for(int i=0; i<nr; i++) {
			if (m_buf[i] == HEXB_INT) {
				m_interrupt_flag = true;
				DBGPRINTF("!!!!!!!!!!!!!!!!! ----- INTERRUPT!\n");
			} else if (m_buf[i] == HEXB_IDLE) {
				DBGPRINTF("Interface is now idle\n");
			} else if (m_buf[i] == HEXB_ACK) {
			} else if (m_buf[i] == HEXB_RESET) {
				DBGPRINTF("Bus was RESET!\n");
				m_bus_err = false;
			} else if (m_buf[i] == HEXB_ERR) {
				DBGPRINTF("Bus error\n");
				m_bus_err = true;
			}
			// else if (m_buf[nr] == 'Q')
			// else if (m_buf[nr] == 'W')
			// else if (m_buf[nr] == '\n')
		}
	}
}

/*
 * wait()
 *
 * Wait for an interrupt condition.
 */
void	HEXBUS::wait(void) {
	if (m_interrupt_flag)
		DBGPRINTF("INTERRUPTED PRIOR TO WAIT()\n");
	do {
		// Here's where the real work is getting done
		usleep(200);
	} while(!m_interrupt_flag);
}

// HEXBUS:  3503421 ~= 3.3 MB, stopwatch = 1:18.5 seconds, vs 53.8 secs
//	If you issue two 512 word reads at once, time drops to 41.6 secs.
// PORTBUS: 6408320 ~= 6.1 MB, ... 26% improvement, 53 seconds real time

