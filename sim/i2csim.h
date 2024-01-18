////////////////////////////////////////////////////////////////////////////////
//
// Filename:	i2csim.h
// {{{
// Project:	SDR, a basic Soft(Gate)ware Defined Radio architecture
//
// Purpose:	To create an I2C slave simulation that can then be driven and
//		tested by an RTL I2C master.
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
#ifndef	I2CSIM_H
#define	I2CSIM_H

#include <stdio.h>
#include <assert.h>
#include <string.h>

class	I2CBUS {
// {{{
public:
	unsigned int	m_scl:1;
	unsigned int	m_sda:1;
	I2CBUS(int scl=1, int sda=1) : m_scl(scl), m_sda(sda) {};
	I2CBUS	operator+(const I2CBUS b) const {
		return I2CBUS(m_scl&b.m_scl, m_sda&b.m_sda); }
	I2CBUS	operator+=(const I2CBUS b) {
		m_scl &= b.m_scl; m_sda &= b.m_sda;
		return *this;
	}
// }}}
};

typedef	enum { I2CIDLE=0, I2CDEVADDR, I2CDEVACK,
	I2CADDR, I2CSACK, I2CSRX, I2CSTX, I2CMACK, I2CLOSTBUS, I2CILLEGAL
} I2CSTATE;

class	I2CSIMSLAVE {
	// {{{
	char	*m_data;
	int	m_addr, m_daddr, m_abits, m_dbits, m_dreg, m_ack,
			m_last_sda, m_last_scl, m_counter, m_devword,
			m_memsz, m_adrmsk,
		m_devaddr;
	bool	m_illegal;
	unsigned long	m_tick, m_last_change_tick, m_speed;
	I2CBUS	m_bus; // My inputs

	I2CSTATE	m_state;
	// }}}

	volatile int	getack(int addr) {
		// {{{
		m_ack = 0;
		return m_ack;
	} // }}}
	volatile char	read(int addr) {
		// {{{
		// printf("SETTING READ ADDRESS TO %02x\n", m_daddr & m_adrmsk);
		m_daddr = addr;
		return m_data[m_daddr];
	} // }}}
	volatile char	read(void) {
		// {{{
		char	vl = m_data[m_daddr];
		// printf("READING FROM ADDRESS %02x\n", m_daddr & m_adrmsk);
		m_daddr = (m_daddr+1)&m_adrmsk;
		return vl;
	} // }}}
	volatile void	write(int addr, char data) {
		// {{{
		m_daddr = addr & m_adrmsk;
		m_data[m_daddr] = data;
	} volatile void	write(char data) {
		// {{{
		m_daddr = (m_daddr+1) & m_adrmsk;
		m_data[m_daddr] = data;
	} // }}}
	// }}}
public:
	I2CSIMSLAVE(const int ADDRESS = 0x050, const int nbits = 7) {
		// {{{
		m_memsz = (1<<nbits);
		m_adrmsk = m_memsz-1;
		m_data = new char[m_memsz];
		m_last_sda = 1;
		m_last_scl = 1;
		for(int i=0; i<m_memsz; i++)
			m_data[i] = 0;
		m_addr = 0;
		m_illegal = false;
		m_state = I2CIDLE;

		m_tick = m_last_change_tick = 0;
		m_speed= 20;
		
		m_devaddr = ADDRESS;
		m_daddr = 0;

		memset(m_data, 0, m_memsz);
	} // }}}

	I2CBUS	operator()(int scl, int sda);
	I2CBUS	operator()(const I2CBUS b) { return (*this)(b.m_scl, b.m_sda); }
	char	&operator[](const int a) {
		return m_data[a&m_adrmsk]; }

	unsigned vstate(void) const {
		return m_state;
	}
};

#endif
