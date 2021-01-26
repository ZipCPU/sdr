////////////////////////////////////////////////////////////////////////////////
//
// Filename:	rfregs.cpp
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
// Copyright (C) 2020-2021, Gisselquist Technology, LLC
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
#include <signal.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <strings.h>
#include <ctype.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

#include "port.h"
#include "regdefs.h"
#include "hexbus.h"


#define	SLAVE_ADDRESS	0x50
#define	MASTER_WR	0
#define	MASTER_RD	1
#define	RF_I2C_WRITE	SLAVE_ADDRESS | MASTER_WR
#define	RF_I2C_READ	SLAVE_ADDRESS | MASTER_RD

const	int	MAX_I2C_RETRIES = 1;

#define	RF_REGMODE		0x00
#define	RF_REGFRFRXMSB		0x01
#define	RF_REGFRFRXMIB		0x02
#define	RF_REGFRFRXLSB		0x03
#define	RF_REGFRFTXMSB		0x04
#define	RF_REGFRFTXMIB		0x05
#define	RF_REGFRFTXLSB		0x06
#define	RF_REGTXGAIN		0x08
#define	RF_REGTXBW		0x0a
#define	RF_REGTXDACBW		0x0b
#define	RF_REGRXANAGAIN		0x0c
#define	RF_REGRXBW		0x0d
#define	RF_REGRXPLLBW		0x0e
#define	RF_REGDIOMAPPING	0x0f
#define	RF_REGCLKSELECT		0x10
#define	RF_REGMODESTATUS	0x11
#define	RF_REGLOWBATTHRESH	0x1a

#define	RF_SPI_WRITE		0x01
#define	RF_SX_RESETW		0x01
#define	RF_SPI_MODE_FN		0xf0
#define	RF_IDLE_MODE_FN		0xf2
#define	RF_GPIO_WRITE_FN	0xf4
#define	RF_GPIO_READ_FN		0xf5
#define	RF_GPIO_ENABLE_FN	0xf6
#define	RF_GPIO_CONFIG_FN	0xf7
//
#define	RF_SPI_CONFIG		0x01	// MSB first, CPOL=0,CPHA=0, 461kHz
#define	RF_GPIO_ENABLE_CONFIG	0x02	// SX RESET is GPIO pin #1
// #define	RF_GPIO_CONFIG		0xa5	// Push/pull on 0-1, 'bz on 2-3
#define	RF_GPIO_CONFIG		0xa5	// Push/pull on 0-1, 'bz on 2-3

	////////////////////////////////////////////////////////////////////////
	//
	//
typedef	struct	{
	unsigned	rf_addr, rf_bytes;
	const char	*rf_name;
} RFNAME;

#define	RFREGS	15
RFNAME rfregs[RFREGS] = {
	RF_REGMODE,         1, "RegMode",
	RF_REGFRFRXMSB,     3, "FRFRX",
	RF_REGFRFTXMSB,     3, "FRFTx",
	RF_REGTXGAIN,       1, "TxGain",
	RF_REGTXBW,         1, "TxBandwidth",
	RF_REGTXBW,         1, "TxBW",
	RF_REGTXDACBW,      1, "TxDACBandwidth",
	RF_REGRXANAGAIN,    1, "RxAnalogGain",
	RF_REGRXBW,         1, "RxBandwidth",
	RF_REGRXBW,         1, "RxBW",
	RF_REGRXPLLBW,      1, "RxPLLBW",
	RF_REGDIOMAPPING,   1, "DioMapping",
	RF_REGCLKSELECT,    1, "ClkSelect",
	RF_REGMODESTATUS,   1, "ModeStatus",
	RF_REGLOWBATTHRESH, 1, "LowBatThresh"
};

unsigned	rfaddrdecode(const char *v) {
	if (isalpha(v[0])) {
		for(int i=0; i<RFREGS; i++)
			if (strcasecmp(v, rfregs[i].rf_name)==0)
				return rfregs[i].rf_addr;
		fprintf(stderr, "Unknown register: %s\n", v);
		exit(-2);
	} else
		return strtoul(v, NULL, 0);
}

const char *rfaddrname(const unsigned v) {
	for(int i=0; i<RFREGS; i++)
		if (rfregs[i].rf_addr == v)
			return rfregs[i].rf_name;
	return NULL;
}

unsigned	rfaddrbytes(const unsigned v) {
	for(int i=0; i<RFREGS; i++)
		if (rfregs[i].rf_addr == v)
			return rfregs[i].rf_bytes;
	return 1;
}

	////////////////////////////////////////////////////////////////////////
	//
	//

// define	RF_SX_RESETR	0x53

#define	SCL_BIT		1
#define	SDA_BIT		2

#define	SCL_INPUT	(SCL_BIT << 16)
#define	SDA_INPUT	(SDA_BIT << 16)

#define	SET_GPIO(A)	((A<<16)|A)
#define	CLR_GPIO(A)	(A<<16)

#define	SDA_OFF(A)	A->writeio(R_GPIO, CLR_GPIO(SDA_BIT))
#define	SDA_ON(A)	A->writeio(R_GPIO, SET_GPIO(SDA_BIT))

#define	SCL_OFF(A)	A->writeio(R_GPIO, CLR_GPIO(SCL_BIT))
#define	SCL_ON(A)	A->writeio(R_GPIO, SET_GPIO(SCL_BIT))

void	i2c_start(FPGA *m_fpga) {
	// {{{
	SDA_OFF(m_fpga);
	m_fpga->readio(R_GPIO);
	SCL_OFF(m_fpga);
	// }}}
}

void	i2c_stop(FPGA *m_fpga) {
	// {{{
	SDA_OFF(m_fpga);
	m_fpga->readio(R_GPIO);
	SCL_ON(m_fpga);
	m_fpga->readio(R_GPIO);
	SDA_ON(m_fpga);
	// }}}
}

int	i2c_read_byte(FPGA *m_fpga, int ack = 1) {
	// {{{
	int	result = 0, v;

	SDA_ON(m_fpga);
	for(int k=0; k<8; k++) {
		SCL_ON(m_fpga);
		do {
			v = m_fpga->readio(R_GPIO);
		} while((v & SCL_INPUT) == 0);
		
		result = (result << 1)
			| ((m_fpga->readio(R_GPIO) & SDA_INPUT) ? 1:0);
		SCL_OFF(m_fpga);
	}

	if (ack)
		SDA_OFF(m_fpga);
	SCL_ON(m_fpga);
	do {
		v = m_fpga->readio(R_GPIO);
	} while((v & SCL_INPUT) == 0);
	SCL_OFF(m_fpga);
	SDA_ON(m_fpga);

	return result;
	// }}}
}

int	i2c_write_byte(FPGA *m_fpga, unsigned byte) {
	// {{{
	int	v;

	for(int k=0; k<8; k++) {
		if (byte & (128 >> k)) {
			// fprintf(stderr, "WRITE BIT[%d] = 1\n", 7-k);
			SDA_ON(m_fpga);
		} else {
			// fprintf(stderr, "WRITE BIT[%d] = 0\n", 7-k);
			SDA_OFF(m_fpga);
		}
		SCL_ON(m_fpga);
		do {
			v = m_fpga->readio(R_GPIO);
		} while((v & SCL_INPUT) == 0);

		if (((v & SDA_BIT)==0) && ((byte & (128 >> k))!=0)) {
			fprintf(stderr, "ERR GPIO = %05x\n", v);
			return 1;
		}
		SCL_OFF(m_fpga);
	}

	SDA_ON(m_fpga);
	m_fpga->readio(R_GPIO);
	SCL_ON(m_fpga);
	do {
		v = m_fpga->readio(R_GPIO);
	} while((v & SCL_INPUT) == 0);
	SCL_OFF(m_fpga);

	if ((v & SDA_INPUT)!=0)
		return 1;
	return 0;
	// }}}
}

int	i2c_read(FPGA *m_fpga, int msglen, char *msg) {
	// {{{
	int	retries = 0;
	int	err = 0;

	do {
		i2c_start(m_fpga);
		err = i2c_write_byte(m_fpga, RF_I2C_READ);
		if (err)
			printf("I2C: RETRY-READ\n");
	} while(err && ++retries < MAX_I2C_RETRIES);

	if (err) {
		i2c_stop(m_fpga);
		return 1;
	}

	for(int k=0; !err && (k<msglen-2); k++)
		msg[k] = i2c_read_byte(m_fpga);
	if (!err)
		msg[msglen-2] = i2c_read_byte(m_fpga, 0);
	i2c_stop(m_fpga);

	return 0;
	// }}}
}

int	i2c_write(FPGA *m_fpga, int msglen, char *msg) {
	// {{{
	int	retries = 0;
	int	err = 0;

	do {
		i2c_start(m_fpga);
		err = i2c_write_byte(m_fpga, (RF_I2C_WRITE));
		if (err)
			printf("I2C: RETRY-WRITE\n");
	} while(err && ++retries < MAX_I2C_RETRIES);
	if (err) {
		i2c_stop(m_fpga);
		return 1;
	}

	for(int k=0; !err && k<msglen; k++)
		err |= i2c_write_byte(m_fpga, msg[k]);
	i2c_stop(m_fpga);

	return err;
	// }}}
}


unsigned	read_rfreg(FPGA *m_fpga, unsigned addr, unsigned  count = 1) {
	// {{{
	char		msg[32];
	int		msglen = 0;
	unsigned	result;

	msg[msglen++] = RF_SPI_WRITE;
	msg[msglen++] = addr & 0x07f; // Leave the top bit clear for a read
	for(int k=0; k<(int)count && (msglen < 32); k++)
		msg[msglen++] = 0;
	i2c_write(m_fpga, msglen, msg);

	for(int k=0; k<msglen+2; k++)
		msg[k] = 0;

	i2c_read(m_fpga, msglen, msg);

	result = 0;
	for(int k=0; k<(int)count; k++)
		result = (result << 8) | (msg[1+k] & 0x0ff);
	return result;
	// }}}
}

void	write_rfreg(FPGA *m_fpga, unsigned addr, unsigned value, int count=1) {
	// {{{
	char		msg[32];
	int		msglen = 0;

	msg[msglen++] = RF_SPI_WRITE;
	msg[msglen++] = addr | 0x80; // Set the top bit for a write
	for(int k=0; k<count && (msglen < 32); k++)
		msg[msglen++] = (value >> (count-1-k)*8);;

	i2c_write(m_fpga, msglen, msg);
	// }}}
}

void	rf_config(FPGA *m_fpga) {
	// {{{
	char	msg[32];
	int	msglen;

/*
	msg[0] = RF_SPI_MODE_FN;
	msg[1] = RF_SPI_CONFIG;
	msglen = 2;

	i2c_write(m_fpga, msglen, msg);
*/

	msg[0] = RF_GPIO_ENABLE_FN;
	msg[1] = RF_GPIO_ENABLE_CONFIG;
	msglen = 2;

	i2c_write(m_fpga, msglen, msg);

/*
	msg[0] = RF_GPIO_CONFIG_FN;
	msg[1] = RF_GPIO_CONFIG;
	msglen = 2;

	//
	// Toggle the SX_RESET pin high
	//
	i2c_write(m_fpga, msglen, msg);

	msg[0] = RF_GPIO_WRITE_FN;
	msg[1] = 0x02;
	msglen = 2;

	i2c_write(m_fpga, msglen, msg);

*/
	// And low again
	msg[0] = RF_GPIO_WRITE_FN;
	msg[1] = 0x00;
	msglen = 2;

	i2c_write(m_fpga, msglen, msg);

/*
	//
	// Set the reset to high impedence
	//
	msg[0] = RF_GPIO_CONFIG_FN;
	msg[1] = 0xa9;
	msglen = 2;

	i2c_write(m_fpga, msglen, msg);
*/
	// }}}
}

FPGA	*m_fpga;
void	closeup(int v) {
	m_fpga->kill();
	exit(0);
}

bool	isvalue(const char *v) {
	// {{{
	const char *ptr = v;

	while(isspace(*ptr))
		ptr++;

	if ((*ptr == '+')||(*ptr == '-'))
		ptr++;
	if (*ptr == '+')
		ptr++;
	if (*ptr == '0') {
		ptr++;
		if (tolower(*ptr) == 'x')
			ptr++;
	}

	return (isdigit(*ptr));
	// }}}
}

void	usage(void) {
	printf("USAGE: rfregs [-d] address [value]\n");
}

int main(int argc, char **argv) {
	const char *host = FPGAHOST;
	int	port=FPGAPORT;
	bool	config_flag = false;
	int	skp;

	// Argument processing
	// {{{
	skp = 1;
	for(int argn=0; argn<argc-skp; argn++) {
		skp++;
		if (strcmp(argv[argn+skp-1],"-c")==0) {
			config_flag = true;
		} else {
			skp--;
			argv[argn] = argv[argn+skp];
		}
	} argc -= skp;
	// }}}

	m_fpga = new FPGA(new NETCOMMS(host, port));

	signal(SIGSTOP, closeup);
	signal(SIGHUP, closeup);

	if (config_flag) {
		rf_config(m_fpga);
		if (argc < 1)
			exit(EXIT_SUCCESS);
	}

	if ((argc < 1)||(argc > 2)) {
		usage();
		// printf("USAGE: rfregs address [value]\n");
		exit(EXIT_FAILURE);
	}

	const char *nm = NULL, *named_address = argv[0];
	unsigned address, value;

	if (isvalue(named_address)) {
		address = strtoul(named_address, NULL, 0);
		nm = rfaddrname(address);
	} else {
		address = rfaddrdecode(named_address);
		nm = rfaddrname(address);
	}

	if (NULL == nm)
		nm = "";

	if (argc < 2) { // Read a register
		// {{{
		FPGA::BUSW	v;
		try {
			unsigned char a, b, c, d, msglen;

			msglen = rfaddrbytes(address);
			v = read_rfreg(m_fpga, address, msglen);
			a = (v>>24)&0x0ff;
			b = (v>>16)&0x0ff;
			c = (v>> 8)&0x0ff;
			d = (v    )&0x0ff;

			printf("%08x (%8s) : [%c%c%c%c] %08x\n", address, nm, 
				isgraph(a)?a:'.', isgraph(b)?b:'.',
				isgraph(c)?c:'.', isgraph(d)?d:'.', v);

		} catch(BUSERR b) {
			printf("%08x (%8s) : BUS-ERROR\n", address, nm);
		} catch(const char *er) {
			printf("Caught bug: %s\n", er);
			exit(EXIT_FAILURE);
		}
		// }}}
	} else { // Write to a register
		// {{{
		try {
			value = strtoul(argv[1], NULL, 0);
			write_rfreg(m_fpga, address, value);
			printf("%08x (%8s)-> %08x\n", address, nm, value);
		} catch(BUSERR b) {
			printf("%08x (%8s) : BUS-ERR)R\n", address, nm);
			exit(EXIT_FAILURE);
		} catch(const char *er) {
			printf("Caught bug on write: %s\n", er);
			exit(EXIT_FAILURE);
		}
		// }}}
	}

	if (m_fpga->poll())
		printf("FPGA was interrupted\n");
	delete	m_fpga;
}
