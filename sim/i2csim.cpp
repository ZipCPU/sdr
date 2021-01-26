////////////////////////////////////////////////////////////////////////////////
//
// Filename:	i2csim.cpp
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
#include "i2csim.h"

I2CBUS	I2CSIMSLAVE::operator()(int scl, int sda) {
	// {{{
	I2CBUS	r(scl, sda); // Our default result

	if ((scl & m_bus.m_scl)&&(m_last_scl)
			&&(sda & m_bus.m_sda)&&(!m_last_sda)) {
		// Stop bit: Low to high transition with scl high
		// {{{
		// Leave the bus as is
		// printf("START BIT: Setting state to idle\n");
		m_state = I2CIDLE;
		m_illegal = false;

		m_bus.m_scl = m_bus.m_sda = 1;
		// }}}
	} else {
		m_bus.m_scl = m_bus.m_sda = 1;
		switch(m_state) {
		case I2CIDLE:
			// {{{
			if (!scl) {
				m_state = I2CILLEGAL;
			} else if (!sda) {
				m_state = I2CDEVADDR;
				m_addr  = 0;
				m_abits = 0;
				m_ack   = 1;
				m_dbits = 0;
			} // The the bus as it was on entry
			break;
			// }}}
		case	I2CDEVADDR:
			// {{{
			if ((scl)&&(!m_last_scl)) {
				m_addr = (m_addr<<1)|sda;
				m_abits++;
				if (m_abits == 8) {
					m_addr &= 0x0ff;
					if ((m_addr >> 1)==(m_devaddr)) {
						m_state = I2CDEVACK;
						m_ack = 0;
						m_devword = m_addr;
					} else
						m_state = I2CLOSTBUS;
				} m_counter = 0;
			} else if (scl) {
				// Can't change when the clock is high
				assert(sda == m_last_sda);
			} // The the bus as it was on entry
			break;
			// }}}
		case	I2CDEVACK:
			// {{{
			// Ack the master's device request, it's for us.  We
			// come in here before the negative edge of the last
			// bit, though
			if ((m_counter == 0)&&(r.m_scl)) {
				// Wait for the first negative edge, from the
				// last bit
				// printf("Waiting on negative edge before ack\n");
			} else {
				m_bus.m_sda = m_ack;
				if (scl) {
					// Neither the Master (nor anyone else)
					// is allowed to pull the line low
					// during our ack period
					if (!r.m_sda) {
						assert(r.m_sda);
					}
				}
				if (m_counter++ < 400) {
					m_bus.m_scl = 0;
				} else if ((r.m_scl==0)&&(m_last_scl)) {
					if (m_devword&1) {
						m_state = I2CSTX;
						m_dreg = read();
						// printf("I2C: Sending %02x next\n", m_dreg & 0x0ff);
					} else {
						m_state = I2CADDR;
						m_abits = 0;
						m_addr  = 0;
					}
				}
			} m_dbits = 0;
			break;
			// }}}
		case	I2CADDR:
			// {{{
			if ((scl)&&(!m_last_scl)) {
				m_addr = (m_addr<<1)|sda;
				m_abits++;
				if (m_abits >= 8) {
					m_state = I2CSACK;
					m_daddr = m_addr;
					m_ack = getack(m_addr);
				} m_counter = 0;
			} else if (scl) {
				// Can't change when the clock is high
				assert(sda == m_last_sda);
			} // The the bus as it was on entry
			break;
			// }}}
		case	I2CSACK:
			// {{{
			// Ack the master
			if ((m_counter == 0)&&(r.m_scl)) {
				// Wait for the first negative edge, from the
				// last bit.
			} else {
				m_bus.m_sda = m_ack;
				if (r.m_scl)
					// Master is not allowed to pull the
					// line low, that's our task
					assert(r.m_sda);
				m_bus.m_sda = m_ack;
				// Let's stretch the clock a touch here
				if (m_counter++ < 400) {
					m_bus.m_scl = 0;
				} else if ((!r.m_scl)&&(m_last_scl)) {
					m_state = I2CSRX;
				}
			} m_dbits = 0;
			break;
			// }}}
		case	I2CSRX:	// Master is writing to us, we are receiving
			// {{{
			if (r.m_scl) {
				// Not allowed to change when clock is high
				if (m_last_scl)
					assert(sda == m_last_sda);
				if (!m_last_scl) {
					m_dreg = ((m_dreg<<1) | r.m_sda)&0x0ff;
					m_dbits++;
					if (m_dbits == 8) {
						// Get an ack from the master
						m_state = I2CSACK;
						write(m_addr, m_dreg);
						m_addr = (m_addr + 1)&m_adrmsk;
					}
				} m_counter = 0;
			} break;
			// }}}
		case	I2CSTX: // Master is reading from us, we are txmitting
			// {{{
			//if (!sda) { // assert(sda); }
			if ((m_counter == 0)&&(r.m_scl)) {
			} else if (m_counter++ < 20) {
				// Wait some time before changing
			} if (r.m_scl) {
				// Not allowed to change when clock is high
				m_bus.m_sda = m_last_sda;
			} else if (!m_last_scl) {
				m_bus.m_sda = m_dreg>>(7-(m_dbits&0x07));
			} else if (m_last_scl) {
				m_dbits++;
				m_bus.m_sda = m_last_sda;
				if (m_dbits == 8) {
					// Get an ack from the master
					m_state = I2CMACK;
					m_dbits = 0;
				}
			} break;
			// }}}
		case	I2CMACK:
			// {{{
			// Insist that the master actually ACK
			//
			// Sadly, we can't.  The master can NAK and ... that's
			// the end.
			// Give the master a chance to ACK
			if ((!r.m_scl)&&(m_last_scl)) {
				if (!sda) {
					// master ACK'd.  Go on
					m_state = I2CSTX;
					m_dreg = read();
					// printf("I2C: Sending %02x next\n", m_dreg & 0x0ff);
				} else {
					m_state = I2CILLEGAL;
				}
			} m_dbits = 0;
			break;
			// }}}
		case	I2CLOSTBUS:
			// {{{
			// Not a problem, but ... someone else is driving the
			// bus.  We let them respond to the bus
			m_state = I2CLOSTBUS;
			break;
			// }}}
		case	I2CILLEGAL:	// fall through
		default:
			// {{{
			m_bus.m_scl = 1;
			m_bus.m_sda = 1;
			if (!m_illegal) {
				fprintf(stderr, "I2C: Illegal state!!\n");
				m_illegal = true;
				m_state = I2CILLEGAL;
				assert(0);
			}
			break;
		// }}}
	}}

	// printf("TICK: SCL,SDA = %d,%d\n", scl, sda);
	m_tick++;
	r += m_bus;
	// m_last_change_tick
	// {{{
	if ((r.m_scl != m_last_scl)||(r.m_sda != m_last_sda)) {
		/*
		if ((m_last_change_tick>0)&&(m_tick - m_last_change_tick < m_speed)) {
			fprintf(stderr, "ERR-SHORT-CHANGE: ONLY %ld CLOCKS BETWEN CHANGES\n",
				m_tick-m_last_change_tick);
			fprintf(stderr, "LAST CHANGE AT %ld TICKS, CHANGING AGAIN AT %ld TICKS\n", m_last_change_tick, m_tick);
			fprintf(stderr, "LAST-STATE: %d,%d\n", m_last_scl, m_last_sda);
			fprintf(stderr, "THIS-STATE: %d,%d\n", r.m_scl, r.m_sda);
			// assert(m_tick - m_last_change_tick >= m_speed);
		} */
		m_last_change_tick = m_tick;
	}
	// }}}

	m_last_scl = r.m_scl;
	m_last_sda = r.m_sda;
	// printf("TICK: LAST SCL,SDA = %d,%d\n", m_last_scl, m_last_sda);

	return r;
	// }}}
}

