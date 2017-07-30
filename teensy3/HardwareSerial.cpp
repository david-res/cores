/* Teensyduino Core Library
* http://www.pjrc.com/teensy/
* Copyright (c) 2017 PJRC.COM, LLC.
*
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject to
* the following conditions:
*
* 1. The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* 2. If the Software is incorporated into a build system that allows
* selection among a list of target devices, then similar target
* devices manufactured by PJRC.COM must be included in the list of
* target devices and selectable in the same manner.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
// Temporary HardwareSerial will be HardwareSerial once done...
#include "HardwareSerial.h"

#include "Arduino.h"
#include "Stream.h"
#include "pins_arduino.h"

#define C2_TX_ACTIVE		(hardware().c2_enable | UART_C2_TIE)
#define C2_TX_COMPLETING	(hardware().c2_enable  | UART_C2_TCIE)
#define C2_TX_INACTIVE		hardware().c2_enable 

#ifndef SERIAL1_TX_BUFFER_SIZE
#define SERIAL1_TX_BUFFER_SIZE     64 // number of outgoing bytes to buffer
#endif
#ifndef SERIAL1_RX_BUFFER_SIZE
#define SERIAL1_RX_BUFFER_SIZE     64 // number of incoming bytes to buffer
#endif

#ifndef SERIAL2_TX_BUFFER_SIZE
#define SERIAL2_TX_BUFFER_SIZE     40 // number of outgoing bytes to buffer
#endif
#ifndef SERIAL2_RX_BUFFER_SIZE
#define SERIAL2_RX_BUFFER_SIZE     64 // number of incoming bytes to buffer
#endif
#ifndef SERIAL3_TX_BUFFER_SIZE
#define SERIAL3_TX_BUFFER_SIZE     40 // number of outgoing bytes to buffer
#endif
#ifndef SERIAL3_RX_BUFFER_SIZE
#define SERIAL3_RX_BUFFER_SIZE     64 // number of incoming bytes to buffer
#endif
#ifndef SERIAL4_TX_BUFFER_SIZE
#define SERIAL4_TX_BUFFER_SIZE     40 // number of outgoing bytes to buffer
#endif
#ifndef SERIAL4_RX_BUFFER_SIZE
#define SERIAL4_RX_BUFFER_SIZE     64 // number of incoming bytes to buffer
#endif

#ifndef SERIAL5_TX_BUFFER_SIZE
#define SERIAL5_TX_BUFFER_SIZE     40 // number of outgoing bytes to buffer
#endif
#ifndef SERIAL5_RX_BUFFER_SIZE
#define SERIAL5_RX_BUFFER_SIZE     64 // number of incoming bytes to buffer
#endif
#ifndef SERIAL6_TX_BUFFER_SIZE
#define SERIAL6_TX_BUFFER_SIZE     40 // number of outgoing bytes to buffer
#endif
#ifndef SERIAL6_RX_BUFFER_SIZE
#define SERIAL6_RX_BUFFER_SIZE     64 // number of incoming bytes to buffer
#endif

#define IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest



void HardwareSerial::begin(uint32_t baud)
{
	hardware().clock_gate_register |= hardware().clock_gate_mask;
	rx_buffer_head_ = 0;
	rx_buffer_tail_ = 0;
	tx_buffer_head_ = 0;
	tx_buffer_tail_ = 0;
	transmitting_ = 0;

	rts_low_watermark_ = rx_buffer_total_size_ - hardware().rts_low_watermark;
	rts_high_watermark_ = rx_buffer_total_size_ - hardware().rts_high_watermark;

	volatile uint32_t *reg;
	reg = portConfigRegister(hardware().rx_pin[rx_pin_index_]);
	*reg = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(hardware().rx_mux[rx_pin_index_]);

	reg = portConfigRegister(hardware().tx_pin[tx_pin_index_]);
	if (tx_opendrain_)
		*reg = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_MUX(hardware().tx_mux[tx_pin_index_]);
	else
		*reg = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(hardware().tx_mux[tx_pin_index_]);

#if defined(KINETISK)
	uint32_t divisor = (((hardware().uart_clock * 2) + ((baud) >> 1)) / (baud));

	port().BDH = (divisor >> 13) & 0x1F;
	port().BDL = (divisor >> 5) & 0xFF;
	port().C4 = divisor & 0x1F;
	if (hardware().uart_fifo) {
		port().C1 = UART_C1_ILT;
		port().TWFIFO = 2; // tx watermark, causes S1_TDRE to set
		port().RWFIFO = 4; // rx watermark, causes S1_RDRF to set
		port().PFIFO = UART_PFIFO_TXFE | UART_PFIFO_RXFE;
	} else {
		port().C1 = 0;
		port().PFIFO = 0;
	}
#elif defined(KINETISL)
	uint32_t divisor = (((hardware().uart_clock) + ((baud) >> 1)) / (baud));
	Serial.printf("%d %d %d %d %d\n", hardware().rx_pin[rx_pin_index_], hardware().tx_pin[tx_pin_index_], 
		baud, hardware().uart_clock, divisor);
	port().BDH = (divisor >> 8) & 0x1F;
	port().BDL = divisor & 0xFF;
	port().C1 = 0;
#endif
	port().C2 = C2_TX_INACTIVE;
	NVIC_SET_PRIORITY(hardware().irq, hardware().irq_priority);
	NVIC_ENABLE_IRQ(hardware().irq);
}

void HardwareSerial::begin(uint32_t baud, uint32_t format)
{
	begin(baud);


	uint8_t c;

	c = port().C1;
	c = (c & ~0x13) | (format & 0x03);	// configure parity
	if (format & 0x04) c |= 0x10;		// 9 bits (might include parity)
	port().C1 = c;
	if ((format & 0x0F) == 0x04) port().C3 |= 0x40; // 8N2 is 9 bit with 9th bit always 1
	c = port().S2 & ~0x10;
	if (format & 0x10) c |= 0x10;		// rx invert
	port().S2 = c;
	c = port().C3 & ~0x10;
	if (format & 0x20) c |= 0x10;		// tx invert
	port().C3 = c;
#ifdef SERIAL_9BIT_SUPPORT
	c = port().C4 & 0x1F;
	if (format & 0x08) c |= 0x20;		// 9 bit mode with parity (requires 10 bits)
	port().C4 = c;
	use9Bits_ = format & 0x80;
#endif
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(KINETISL)
	// For T3.5/T3.6/TLC See about turning on 2 stop bit mode
	if ( format & 0x100) {
		uint8_t bdl = port().BDL;
		port().BDH |= UART_BDH_SBNS;		// Turn on 2 stop bits - was turned off by set baud
		port().BDL = bdl;		// Says BDH not acted on until BDL is written
	}
#endif
}

void HardwareSerial::end(void)
{
	if (!(hardware().clock_gate_register & hardware().clock_gate_mask)) return;
	while (transmitting_) yield();  // wait for buffered data to send
	NVIC_DISABLE_IRQ(hardware().irq);
	port().C2 = 0;

	// Switch both RX and TX pins to GPIO pins
	volatile uint32_t *reg;
	reg = portConfigRegister(hardware().rx_pin[rx_pin_index_]);
	*reg = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);

	reg = portConfigRegister(hardware().tx_pin[tx_pin_index_]);
	*reg = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);

	rx_buffer_head_ = 0;
	rx_buffer_tail_ = 0;
	if (rts_pin_) rts_deassert();
}

void HardwareSerial::transmitterEnable(uint8_t pin)
{
	while (transmitting_) ;
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	transmit_pin_ = portOutputRegister(pin);
	#if defined(KINETISL)
	transmit_mask_ = digitalPinToBitMask(pin);
	#endif
}

void HardwareSerial::setRX(uint8_t pin)
{
	if (pin != hardware().rx_pin[rx_pin_index_]) {
		for (unsigned int i = 0; i < sizeof(hardware().rx_pin); i++) {
			if  (pin == hardware().rx_pin[i]) {
				if (hardware().clock_gate_register & hardware().clock_gate_mask) {
					volatile uint32_t *reg;
					reg = portConfigRegister(hardware().rx_pin[rx_pin_index_]);
					*reg = 0;
					reg = portConfigRegister(hardware().rx_pin[i]);
					*reg = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(hardware().rx_mux[i]);
				}	
				rx_pin_index_ = i;
				return;
			}
		}
	}
}

void HardwareSerial::setTX(uint8_t pin, bool opendrain)
{
	if ((pin != hardware().tx_pin[tx_pin_index_]) || (tx_opendrain_ != opendrain)) {
		for (unsigned int i = 0; i < sizeof(hardware().tx_pin); i++) {
			if  (pin == hardware().tx_pin[i]) {
				if (hardware().clock_gate_register & hardware().clock_gate_mask) {
					volatile uint32_t *reg;
					if (tx_pin_index_ != i ) {
						reg = portConfigRegister(hardware().tx_pin[tx_pin_index_]);
						*reg = 0;
					}

					reg = portConfigRegister(hardware().tx_pin[i]);
					if (opendrain)
						*reg = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_MUX(hardware().tx_mux[i]);
					else
						*reg = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(hardware().tx_mux[i]);
				}	
				tx_pin_index_ = i;
				tx_opendrain_ = opendrain; 
				return;
			}
		}
	}
}

bool HardwareSerial::attachRts(uint8_t pin)
{
	if (!(hardware().clock_gate_register & hardware().clock_gate_mask)) return 0;
	if (pin < CORE_NUM_DIGITAL) {
		rts_pin_ = portOutputRegister(pin);
		#if defined(KINETISL)
		rts_mask_ = digitalPinToBitMask(pin);
		#endif
		pinMode(pin, OUTPUT);
		rts_assert();
	} else {
		rts_pin_ = NULL;
		return 0;
	}
	return 1;
}

bool HardwareSerial::attachCts(uint8_t pin)
{
#if defined(KINETISK)
	if (pin != hardware().cts_pin[cts_pin_index_]) {
		for (unsigned int i = 0; i < sizeof(hardware().cts_pin); i++) {
			if  (pin == hardware().cts_pin[i]) {
				if (hardware().clock_gate_register & hardware().clock_gate_mask) {
					volatile uint32_t *reg;
					reg = portConfigRegister(hardware().cts_pin[cts_pin_index_]);
					*reg = 0;
					reg = portConfigRegister(hardware().cts_pin[i]);
					*reg = PORT_PCR_MUX(hardware().cts_mux[i]) | PORT_PCR_PE ;
				}	
				cts_pin_index_ = i;
				port().MODEM |= UART_MODEM_TXCTSE;
				return 1;
			}
		}
	}
	port().MODEM &= ~UART_MODEM_TXCTSE;
#endif
	return 0;

}

int HardwareSerial::available(void)
{
	uint32_t head, tail;

	head = rx_buffer_head_;
	tail = rx_buffer_tail_;
	if (head >= tail) return head - tail;
	return rx_buffer_total_size_ + head - tail;
}

void HardwareSerial::addStorageForRead(void *buffer, size_t length) 
{
	rx_buffer_storage_ = (BUFTYPE*)buffer;
	if (buffer) {
		rx_buffer_total_size_ = rx_buffer_total_size_ + length;
	} else {
		rx_buffer_total_size_ = rx_buffer_total_size_;
	} 

	rts_low_watermark_ = rx_buffer_total_size_ - hardware().rts_low_watermark;
	rts_high_watermark_ = rx_buffer_total_size_ - hardware().rts_high_watermark;
}

void HardwareSerial::addStorageForWrite(void *buffer, size_t length) 
{
	tx_buffer_storage_ = (BUFTYPE*)buffer;
	if (buffer) {
		tx_buffer_total_size_ = tx_buffer_total_size_ + length;
	} else {
		tx_buffer_total_size_ = tx_buffer_total_size_;
	} 
}


int HardwareSerial::peek(void)
{
	uint32_t head, tail;

	head = rx_buffer_head_;
	tail = rx_buffer_tail_;
	if (head == tail) return -1;
	if (++tail >= rx_buffer_total_size_) tail = 0;
	if (tail < rx_buffer_size_) {
		return rx_buffer_[tail];
	} else {
		return rx_buffer_storage_[tail-rx_buffer_size_];
	}
}

int HardwareSerial::read(void)
{
	uint32_t head, tail;
	int c;

	head = rx_buffer_head_;
	tail = rx_buffer_tail_;
	if (head == tail) return -1;
	if (++tail >= rx_buffer_total_size_) tail = 0;
	if (tail < rx_buffer_size_) {
		c = rx_buffer_[tail];
	} else {
		c = rx_buffer_storage_[tail-rx_buffer_size_];
	}
	rx_buffer_tail_ = tail;
	if (rts_pin_) {
		uint32_t avail;
		if (head >= tail) avail = head - tail;
		else avail = rx_buffer_total_size_ + head - tail;
		if (avail <= rts_low_watermark_) rts_assert();
	}
	return c;
}

void HardwareSerial::flush(void)
{
	while (transmitting_) yield(); // wait
}

void HardwareSerial::clear(void)
{
#ifdef KINETISK	
	if (hardware().uart_fifo) {
		if (!(hardware().clock_gate_register & hardware().clock_gate_mask)) return;
		port().C2 &= ~(UART_C2_RE | UART_C2_RIE | UART_C2_ILIE);
		port().CFIFO = UART_CFIFO_RXFLUSH;
		port().C2 |= (UART_C2_RE | UART_C2_RIE | UART_C2_ILIE);
	}
#endif
	rx_buffer_head_ = rx_buffer_tail_;
	if (rts_pin_) rts_assert();
}

int HardwareSerial::availableForWrite(void)
{
	uint32_t head, tail;

	head = tx_buffer_head_;
	tail = tx_buffer_tail_;
	if (head >= tail) return tx_buffer_total_size_ - 1 - head + tail;
	return tail - head - 1;
}

size_t HardwareSerial::write(uint8_t c)
{
	uint32_t head, n;

	if (!(hardware().clock_gate_register & hardware().clock_gate_mask)) return 0;
	if (transmit_pin_) transmit_assert();
	head = tx_buffer_head_;
	if (++head >= tx_buffer_total_size_) head = 0;
	while (tx_buffer_tail_ == head) {
		int priority = nvic_execution_priority();
		if (priority <= hardware().irq_priority) {
			if ((port().S1 & UART_S1_TDRE)) {
				uint32_t tail = tx_buffer_tail_;
				if (++tail >= tx_buffer_total_size_) tail = 0;
				if (tail < tx_buffer_size_) {
					n = tx_buffer_[tail];
				} else {
					n = tx_buffer_storage_[tail-tx_buffer_size_];
				}

				if (use9Bits_) port().C3 = (port().C3 & ~0x40) | ((n & 0x100) >> 2);
				port().D = n;
				tx_buffer_tail_ = tail;
			}
		} else if (priority >= 256) {
			yield(); // wait
		}
	}
	if (head < tx_buffer_size_) {
		tx_buffer_[head] = c;
	} else {
		tx_buffer_storage_[head - tx_buffer_size_] = c;
	}
	transmitting_ = 1;
	tx_buffer_head_ = head;
	port().C2 = C2_TX_ACTIVE;
	return 1;
}

size_t HardwareSerial::write(const uint8_t * buffer, size_t count)
{
	if (!(hardware().clock_gate_register & hardware().clock_gate_mask)) return 0;
#ifdef KINETISK
	if (hardware().uart_fifo) {
		const uint8_t *p = (const uint8_t *)buffer;
		const uint8_t *end = p + count;
		uint32_t head, n;

		if (transmit_pin_) transmit_assert();
		while (p < end) {
			head = tx_buffer_head_;
			if (++head >= tx_buffer_total_size_) head = 0;
			if (tx_buffer_tail_ == head) {
				port().C2 = C2_TX_ACTIVE;
				do {
					int priority = nvic_execution_priority();
					if (priority <= hardware().irq_priority) {
						if ((port().S1 & UART_S1_TDRE)) {
							uint32_t tail = tx_buffer_tail_;
							if (++tail >= tx_buffer_total_size_) tail = 0;
							if (tail < tx_buffer_size_) {
								n = tx_buffer_[tail];
							} else {
								n = tx_buffer_storage_[tail-tx_buffer_size_];
							}
							if (use9Bits_) port().C3 = (port().C3 & ~0x40) | ((n & 0x100) >> 2);
							port().D = n;
							tx_buffer_tail_ = tail;
						}
					} else if (priority >= 256) {
						yield();
					}
				} while (tx_buffer_tail_ == head);
			}
			if (head < tx_buffer_size_) {
				tx_buffer_[head] = *p++;
			} else {
				tx_buffer_storage_[head - tx_buffer_size_] = *p++;
			}
			transmitting_ = 1;
			tx_buffer_head_ = head;
		}
		port().C2 = C2_TX_ACTIVE;
		return count;
	}
#endif	
	// not fifo. 
	const uint8_t *p = (const uint8_t *)buffer;
	while (count-- > 0) write(*p++);
	return count;
}
size_t HardwareSerial::write(const char * str)
{
	size_t len = strlen(str);
	write((const uint8_t *) str, len);
	return len;
}


// status interrupt combines
//   Transmit data below watermark  UART_S1_TDRE
//   Transmit complete		    UART_S1_TC
//   Idle line			    UART_S1_IDLE
//   Receive data above watermark   UART_S1_RDRF
//   LIN break detect		    UART_S2_LBKDIF
//   RxD pin active edge	    UART_S2_RXEDGIF

// The non-fifo queues will use this version
void HardwareSerial::status_isr(void)
{
	uint32_t head, tail, n;
	uint8_t c;

	if (port().S1 & UART_S1_RDRF) {
		if (use9Bits_ && (port().C3 & 0x80)) {
			n = port().D | 0x100;
		} else {
			n = port().D;
		}
		head = rx_buffer_head_ + 1;
		if (head >= rx_buffer_total_size_) head = 0;
		if (head != rx_buffer_tail_) {
			if (head < rx_buffer_size_) {
				rx_buffer_[head] = n;
			} else {
				rx_buffer_storage_[head-rx_buffer_size_] = n;
			}
			rx_buffer_head_ = head;
		}
	}
	c = port().C2;
	if ((c & UART_C2_TIE) && (port().S1 & UART_S1_TDRE)) {
		head = tx_buffer_head_;
		tail = tx_buffer_tail_;
		if (head == tail) {
			port().C2 = C2_TX_COMPLETING;
		} else {
			if (++tail >= tx_buffer_total_size_) tail = 0;
			if (tail < tx_buffer_size_) {
				n = tx_buffer_[tail];
			} else {
				n = tx_buffer_storage_[tail-tx_buffer_size_];
			}
			if (use9Bits_) port().C3 = (port().C3 & ~0x40) | ((n & 0x100) >> 2);
			port().D = n;
			tx_buffer_tail_ = tail;
		}
	}
	if ((c & UART_C2_TCIE) && (port().S1 & UART_S1_TC)) {
		transmitting_ = 0;
		if (transmit_pin_) transmit_deassert();
		port().C2 = C2_TX_INACTIVE;
	}
}

#ifdef KINETISK
// We setup the fifo ISR at compile time...
void HardwareSerial::status_isr_fifo(void)
{
	uint32_t head, tail, n;
	uint8_t c;

	uint32_t newhead;
	uint8_t avail;

	if (port().S1 & (UART_S1_RDRF | UART_S1_IDLE)) {
		__disable_irq();
		avail = port().RCFIFO;
		if (avail == 0) {
			// The only way to clear the IDLE interrupt flag is
			// to read the data register.  But reading with no
			// data causes a FIFO underrun, which causes the
			// FIFO to return corrupted data.  If anyone from
			// Freescale reads this, what a poor design!  There
			// write should be a write-1-to-clear for IDLE.
			c = port().D;
			// flushing the fifo recovers from the underrun,
			// but there's a possible race condition where a
			// new character could be received between reading
			// RCFIFO == 0 and flushing the FIFO.  To minimize
			// the chance, interrupts are disabled so a higher
			// priority interrupt (hopefully) doesn't delay.
			// TODO: change this to disabling the IDLE interrupt
			// which won't be simple, since we already manage
			// which transmit interrupts are enabled.
			port().CFIFO = UART_CFIFO_RXFLUSH;
			__enable_irq();
		} else {
			__enable_irq();
			head = rx_buffer_head_;
			tail = rx_buffer_tail_;
			do {
				if (use9Bits_ && (port().C3 & 0x80)) {
					n = port().D | 0x100;
				} else {
					n = port().D;
				}
				newhead = head + 1;
				if (newhead >= rx_buffer_total_size_) newhead = 0;
				if (newhead != tail) {
					head = newhead;
				}
				if (head < rx_buffer_size_) {
					rx_buffer_[head] = n;
				
				} else {
					rx_buffer_storage_[head-rx_buffer_size_] = n;
				}
			} while (--avail > 0);
			rx_buffer_head_ = head;
			if (rts_pin_) {
				uint32_t avail;
				if (head >= tail) avail = head - tail;
				else avail = rx_buffer_total_size_ + head - tail;
				if (avail >= rts_high_watermark_) rts_deassert();
			}
		}
	}
	c = port().C2;
	if ((c & UART_C2_TIE) && (port().S1 & UART_S1_TDRE)) {
		head = tx_buffer_head_;
		tail = tx_buffer_tail_;
		do {
			if (tail == head) break;
			if (++tail >= tx_buffer_total_size_) tail = 0;
			avail = port().S1;
			if (tail < tx_buffer_size_) {
				n = tx_buffer_[tail];
			} else {
				n = tx_buffer_storage_[tail-tx_buffer_size_];
			}
			if (use9Bits_) port().C3 = (port().C3 & ~0x40) | ((n & 0x100) >> 2);
			port().D = n;
		} while (port().TCFIFO < 8);
		tx_buffer_tail_ = tail;
		if (port().S1 & UART_S1_TDRE) port().C2 = C2_TX_COMPLETING;
	}

	if ((c & UART_C2_TCIE) && (port().S1 & UART_S1_TC)) {
		transmitting_ = 0;
		if (transmit_pin_) transmit_deassert();
		port().C2 = C2_TX_INACTIVE;
	}
}
#endif

//==========================================================
// Define LPUart version for Serial6 of T3.6
//==========================================================
#if defined(__MK66FX1M0__)
#define GPIO_BITBAND_ADDR(reg, bit) (((uint32_t)&(reg) - 0x40000000) * 32 + (bit) * 4 + 0x42000000)
#define GPIO_BITBAND_PTR(reg, bit) ((uint32_t *)GPIO_BITBAND_ADDR((reg), (bit)))
#define BITBAND_SET_BIT(reg, bit) (*GPIO_BITBAND_PTR((reg), (bit)) = 1)
#define BITBAND_CLR_BIT(reg, bit) (*GPIO_BITBAND_PTR((reg), (bit)) = 0)
#define TCIE_BIT 22
#define TIE_BIT  23


void HardwareSerial_LPUART::begin(uint32_t baud) {
	hardware().clock_gate_register |= hardware().clock_gate_mask;
	rx_buffer_head_ = 0;
	rx_buffer_tail_ = 0;
	tx_buffer_head_ = 0;
	tx_buffer_tail_ = 0;
	transmitting_ = 0;

	volatile uint32_t *reg;
	reg = portConfigRegister(hardware().rx_pin[rx_pin_index_]);
	*reg = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(hardware().rx_mux[rx_pin_index_]);

	reg = portConfigRegister(hardware().tx_pin[tx_pin_index_]);
	if (tx_opendrain_)
		*reg = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_MUX(hardware().tx_mux[tx_pin_index_]);
	else
		*reg = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(hardware().tx_mux[tx_pin_index_]);

	#define F_LPUART_CLOCK_SPEED  48000000 //F_BUS
	// Make sure the clock for this uart is enabled, else the registers are not
	// Convert the baud rate to best divisor and OSR, based off of code I found in posting
	// try to find an OSR > 4 with the minimum difference from the actual disired baud rate. 
    uint16_t sbr, sbrTemp, osrCheck;
    uint32_t osr, baudDiffCheck, calculatedBaud, baudDiff;
    uint32_t clockSpeed;

    // First lets figure out what the LPUART Clock speed is. 
    uint32_t PLLFLLSEL = SIM_SOPT2 & SIM_SOPT2_IRC48SEL;	// Note: Bot bits on here

    if (PLLFLLSEL == SIM_SOPT2_IRC48SEL)
    	clockSpeed = 48000000;  // Fixed to 48mhz
    else if (PLLFLLSEL == SIM_SOPT2_PLLFLLSEL)
    	clockSpeed = F_PLL;		// Using PLL clock
    else
    	clockSpeed = F_CPU/4;	// FLL clock, guessing
 
    osr = 4;
    sbr = (clockSpeed/(baud * osr));
    /*set sbr to 1 if the clockSpeed can not satisfy the desired baud rate*/
    if(sbr == 0) {
    	// Maybe print something.
    	return;	// can not initialize
    }

     // With integer math the divide*muliply implies the calculated baud will be >= desired baud
    calculatedBaud = (clockSpeed / (osr * sbr));
    baudDiff = calculatedBaud - baud;

    // Check if better off with sbr+1
    if (baudDiff != 0) {
      calculatedBaud = (clockSpeed / (osr * (sbr + 1)));
      baudDiffCheck = baud - calculatedBaud ;
      if (baudDiffCheck < baudDiff) {
        sbr++;  // use the higher sbr
        baudDiff = baudDiffCheck;
      }
    }

    // loop to find the best osr value possible, one that generates minimum baudDiff
    for (osrCheck = 5; osrCheck <= 32; osrCheck++)     {
        sbrTemp = (clockSpeed/(baud * osrCheck));

        if(sbrTemp == 0)
          break;    // higher divisor returns 0 so can not use...

        // Remember integer math so (X/Y)*Y will always be <=X
        calculatedBaud = (clockSpeed / (osrCheck * sbrTemp));
        baudDiffCheck = calculatedBaud - baud;
        if (baudDiffCheck <= baudDiff) {
            baudDiff = baudDiffCheck;
            osr = osrCheck;
            sbr = sbrTemp; 
        }
        // Lets try the rounded up one as well
        if (baudDiffCheck) {
          calculatedBaud = (clockSpeed / (osrCheck * ++sbrTemp));
          baudDiffCheck = baud - calculatedBaud;
          if (baudDiffCheck <= baudDiff) {
              baudDiff = baudDiffCheck;
              osr = osrCheck;
              sbr = sbrTemp; 
          }
        }
    }
	// for lower OSR <= 7x turn on both edge sampling
	uint32_t lpb = LPUART_BAUD_OSR(osr-1) | LPUART_BAUD_SBR(sbr);
    if (osr < 8) {
      lpb |= LPUART_BAUD_BOTHEDGE; 
    }
	LPUART0_BAUD = lpb;

	SIM_SOPT2 |= SIM_SOPT2_LPUARTSRC(1);	// Lets use PLL?

	LPUART0_CTRL = 0;
	LPUART0_MATCH = 0;
	LPUART0_STAT = 0;

	// Enable the transmitter, receiver and enable receiver interrupt
	LPUART0_CTRL |= LPUART_CTRL_RIE | LPUART_CTRL_TE | LPUART_CTRL_RE;
	NVIC_SET_PRIORITY(hardware().irq, hardware().irq_priority);
	NVIC_ENABLE_IRQ(hardware().irq);

}

void HardwareSerial_LPUART::begin(uint32_t baud, uint32_t format) {
	begin(baud);


	uint32_t c;

	// Bits 0-2 - Parity plus 9  bit. 
	c = LPUART0_CTRL;
	//c = (c & ~(LPUART_CTRL_M | LPUART_CTRL_PE | LPUART_CTRL_PT)) | (format & (LPUART_CTRL_PE | LPUART_CTRL_PT));	// configure parity
	//if (format & 0x04) c |= LPUART_CTRL_M;		// 9 bits (might include parity)
	c = (c & ~0x13) | (format & 0x03);	// configure parity
	if (format & 0x04) c |= 0x10;		// 9 bits (might include parity)
	LPUART0_CTRL = c;
	if ((format & 0x0F) == 0x04) LPUART0_CTRL |= LPUART_CTRL_T8; // 8N2 is 9 bit with 9th bit always 1

	// Bit 3 10 bit - Will assume that begin already cleared it.
	if (format & 0x08)
		LPUART0_BAUD |= LPUART_BAUD_M10;
	
	// Bit 4 RXINVERT 
	c = LPUART0_STAT & ~LPUART_STAT_RXINV;
	if (format & 0x10) c |= LPUART_STAT_RXINV;		// rx invert
	LPUART0_STAT = c;

	// Bit 5 TXINVERT
	c = LPUART0_CTRL & ~LPUART_CTRL_TXINV;
	if (format & 0x20) c |= LPUART_CTRL_TXINV;		// tx invert
	LPUART0_CTRL = c;

	// For T3.6 See about turning on 2 stop bit mode
	if ( format & 0x100) LPUART0_BAUD |= LPUART_BAUD_SBNS;	

}

void HardwareSerial_LPUART::end(void) {
	if (!(hardware().clock_gate_register & hardware().clock_gate_mask)) return;
	while (transmitting_) yield();  // wait for buffered data to send
	NVIC_DISABLE_IRQ(hardware().irq);
	LPUART0_CTRL = 0;

	// Switch both RX and TX pins to GPIO pins
	volatile uint32_t *reg;
	reg = portConfigRegister(hardware().rx_pin[rx_pin_index_]);
	*reg = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);

	reg = portConfigRegister(hardware().tx_pin[tx_pin_index_]);
	*reg = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);

	rx_buffer_head_ = 0;
	rx_buffer_tail_ = 0;
	if (rts_pin_) rts_deassert();
}


bool HardwareSerial_LPUART::attachCts(uint8_t pin)  {
	if (!(SIM_SCGC2 & SIM_SCGC2_LPUART0)) return 0;
	if (pin == 56) {
		CORE_PIN56_CONFIG = PORT_PCR_MUX(5) | PORT_PCR_PE; // weak pulldown
	} else {
		UART5_MODEM &= ~UART_MODEM_TXCTSE;
		return 0;
	}
	UART5_MODEM |= UART_MODEM_TXCTSE;
	return 1;
}

size_t HardwareSerial_LPUART::write(uint8_t c) {
	uint32_t head, n;

	if (!(hardware().clock_gate_register & hardware().clock_gate_mask)) return 0;
	if (transmit_pin_) transmit_assert();
	head = tx_buffer_head_;
	if (++head >= tx_buffer_total_size_) head = 0;
	while (tx_buffer_tail_ == head) {
		int priority = nvic_execution_priority();
		if (priority <= hardware().irq_priority) {
			if ((LPUART0_STAT & LPUART_STAT_TDRE)) {
				uint32_t tail = tx_buffer_tail_;
				if (++tail >= tx_buffer_total_size_) tail = 0;
				if (tail < tx_buffer_size_) {
					n = tx_buffer_[tail];
				} else {
					n = tx_buffer_storage_[tail-tx_buffer_size_];
				}
				LPUART0_DATA = n;
				tx_buffer_tail_ = tail;
			}
		} else if (priority >= 256) {
			yield(); // wait
		}
	}
	if (head < tx_buffer_size_) {
		tx_buffer_[head] = c;
	} else {
		tx_buffer_storage_[head - tx_buffer_size_] = c;
	}
	transmitting_ = 1;
	tx_buffer_head_ = head;
	BITBAND_SET_BIT(LPUART0_CTRL, TIE_BIT);
	return 1;
}

void HardwareSerial_LPUART::status_isr_lpuart() {
	uint32_t head, tail, n;
	uint32_t c;

	if (LPUART0_STAT & LPUART_STAT_RDRF) {
//		if (use9Bits && (UART5_C3 & 0x80)) {
//			n = UART5_D | 0x100;
//		} else {
//			n = UART5_D;
//		}
		n = LPUART0_DATA & 0x3ff;	// use only the 10 data bits
		head = rx_buffer_head_ + 1;
		if (head >= rx_buffer_total_size_) head = 0;
		if (head != rx_buffer_tail_) {
			if (head < rx_buffer_size_) {
				rx_buffer_[head] = n;
			} else {
				rx_buffer_storage_[head-rx_buffer_size_] = n;
			}
			rx_buffer_head_ = head;
		}
		if (rts_pin_) {
			uint32_t avail;
			tail = tx_buffer_tail_;
			if (head >= tail) avail = head - tail;
			else avail = rx_buffer_total_size_ + head - tail;
			if (avail >= rts_high_watermark_) rts_deassert();
		}
	}
	c = LPUART0_CTRL;
	if ((c & LPUART_CTRL_TIE) && (LPUART0_STAT & LPUART_STAT_TDRE)) {
		head = tx_buffer_head_;
		tail = tx_buffer_tail_;
		if (head == tail) {
			BITBAND_CLR_BIT(LPUART0_CTRL, TIE_BIT);
			BITBAND_SET_BIT(LPUART0_CTRL, TCIE_BIT);
			//LPUART0_CTRL &= ~LPUART_CTRL_TIE; 
  			//LPUART0_CTRL |= LPUART_CTRL_TCIE; // Actually wondering if we can just leave this one on...
		} else {
			if (++tail >= SERIAL6_TX_BUFFER_SIZE) tail = 0;
			if (tail < tx_buffer_size_) {
				n = tx_buffer_[tail];
			} else {
				n = tx_buffer_storage_[tail-tx_buffer_size_];
			}
			//if (use9Bits) UART5_C3 = (UART5_C3 & ~0x40) | ((n & 0x100) >> 2);
			LPUART0_DATA = n;
			tx_buffer_tail_ = tail;
		}
	}
	if ((c & LPUART_CTRL_TCIE) && (LPUART0_STAT & LPUART_STAT_TC)) {
		transmitting_ = 0;
		if (transmit_pin_) transmit_deassert();
		BITBAND_CLR_BIT(LPUART0_CTRL, TCIE_BIT);
		// LPUART0_CTRL &= ~LPUART_CTRL_TCIE; // Actually wondering if we can just leave this one on...
	}

}
#endif

//==========================================================
// Define objects
//==========================================================
extern "C" {
	extern void uart0_status_isr(void);
	extern void uart1_status_isr(void);
	extern void uart2_status_isr(void);
	extern void uart3_status_isr(void);
	extern void uart4_status_isr(void);
	extern void uart5_status_isr(void);
}

#if defined(KINETISK)
//==========================================================
// T3.x objects
//==========================================================

// Serial1
static volatile BUFTYPE tx_buffer1[SERIAL1_TX_BUFFER_SIZE];
static volatile BUFTYPE rx_buffer1[SERIAL1_RX_BUFFER_SIZE];


const HardwareSerial::HardwareSerial_Hardware_t HardwareSerial::serial1_hardware = {
		SIM_SCGC4, SIM_SCGC4_UART0, IRQ_UART0_STATUS, IRQ_PRIORITY, 38, 24, 
#ifdef HAS_KINETISK_UART0_FIFO
	1, UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_ILIE,
#else
	0, UART_C2_TE | UART_C2_RE | UART_C2_RIE,
#endif
	F_CPU,

#if defined(__MK20DX128__) || defined(__MK20DX256__)
	//  RX Pins
	0, 21,
	3, 3,
	// TX Pins
	1, 5, 
	3, 3,
	// CTS Pins
	18, 20,
	3, 3

#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
	0, 21, 27,
	3, 3, 3,
	1, 5, 26,
 	3, 3, 3,		
	18, 20,
	3, 3
#endif
};


HardwareSerial Serial1((uintptr_t)&KINETISK_UART0, (uintptr_t)&HardwareSerial::serial1_hardware, tx_buffer1, SERIAL1_TX_BUFFER_SIZE, 
	rx_buffer1,  SERIAL1_RX_BUFFER_SIZE);
void uart0_status_isr(void) {

#ifdef HAS_KINETISK_UART0_FIFO
	Serial1.status_isr_fifo();
#else
	Serial1.status_isr();
#endif
}

void serialEvent1() __attribute__((weak));
void serialEvent1() {}

// Serial2

static volatile BUFTYPE tx_buffer2[SERIAL2_TX_BUFFER_SIZE];
static volatile BUFTYPE rx_buffer2[SERIAL2_RX_BUFFER_SIZE];

const HardwareSerial::HardwareSerial_Hardware_t HardwareSerial::serial2_hardware = {
		SIM_SCGC4, SIM_SCGC4_UART1, IRQ_UART1_STATUS, IRQ_PRIORITY, 38, 24, 
#ifdef HAS_KINETISK_UART1_FIFO
	1, UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_ILIE,
#else
	0, UART_C2_TE | UART_C2_RE | UART_C2_RIE,
#endif
	F_CPU,

#if defined(__MK20DX128__) || defined(__MK20DX256__)
	//  RX Pins
	9, 26,
	3, 3,
	// TX Pins
	10, 31, 
	3, 3,
	// CTS Pins
	23, 255,
	3, 0

#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
	9, 59, 255,
	3, 3, 0,
	10, 58, 255,
 	3, 3, 0,		
	23, 60,
	3, 3
#endif
};

HardwareSerial Serial2((uintptr_t)&KINETISK_UART1, (uintptr_t)&HardwareSerial::serial2_hardware, tx_buffer2, SERIAL2_TX_BUFFER_SIZE, 
	rx_buffer2,  SERIAL2_RX_BUFFER_SIZE);

void uart1_status_isr(void) {
#ifdef HAS_KINETISK_UART1_FIFO
	Serial2.status_isr_fifo();
#else
	Serial2.status_isr();
#endif
}

void serialEvent2() __attribute__((weak));
void serialEvent2() {}

// Serial 3

static volatile BUFTYPE tx_buffer3[SERIAL3_TX_BUFFER_SIZE];
static volatile BUFTYPE rx_buffer3[SERIAL3_RX_BUFFER_SIZE];

const HardwareSerial::HardwareSerial_Hardware_t HardwareSerial::serial3_hardware = {
		SIM_SCGC4, SIM_SCGC4_UART2, IRQ_UART2_STATUS, IRQ_PRIORITY, 38, 24, 0,
	UART_C2_TE | UART_C2_RE | UART_C2_RIE,
	F_BUS,

#if defined(__MK20DX128__) || defined(__MK20DX256__)
	//  RX Pins
	7, 255,
	3, 0, 
	// TX Pins
	8, 255, 
	3, 0,
	// CTS Pins
	14, 255,
	3, 0

#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
	7, 255, 255,
	3, 0, 0,
	8, 255, 255,
 	3, 0, 0,		
	14, 255,
	3, 0
#endif
};

HardwareSerial Serial3((uintptr_t)&KINETISK_UART2, (uintptr_t)&HardwareSerial::serial3_hardware, tx_buffer3, SERIAL3_TX_BUFFER_SIZE, 
	rx_buffer3,  SERIAL3_RX_BUFFER_SIZE );

void uart2_status_isr(void) {
	Serial3.status_isr();
}

void serialEvent3() __attribute__((weak));
void serialEvent3() {}

// Serial4

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
static volatile BUFTYPE tx_buffer4[SERIAL4_TX_BUFFER_SIZE];
static volatile BUFTYPE rx_buffer4[SERIAL4_RX_BUFFER_SIZE];

const HardwareSerial::HardwareSerial_Hardware_t HardwareSerial::serial4_hardware = {
		SIM_SCGC4, SIM_SCGC4_UART3, IRQ_UART3_STATUS, IRQ_PRIORITY, 38, 24, 0,
	UART_C2_TE | UART_C2_RE | UART_C2_RIE,
	F_BUS,

	31, 63, 255,
	3, 3, 0,
	32, 62, 255,
 	3, 3, 0,		
	255, 255,
	0, 0
};

HardwareSerial Serial4((uintptr_t)&KINETISK_UART3, (uintptr_t)&HardwareSerial::serial4_hardware, tx_buffer4, SERIAL4_TX_BUFFER_SIZE, 
	rx_buffer4,  SERIAL4_RX_BUFFER_SIZE);

void uart3_status_isr(void) {
	Serial4.status_isr();
}

void serialEvent4() __attribute__((weak));
void serialEvent4() {}

// Serial5

static volatile BUFTYPE tx_buffer5[SERIAL5_TX_BUFFER_SIZE];
static volatile BUFTYPE rx_buffer5[SERIAL5_RX_BUFFER_SIZE];

const HardwareSerial::HardwareSerial_Hardware_t HardwareSerial::serial5_hardware = {
		SIM_SCGC1, SIM_SCGC1_UART4, IRQ_UART4_STATUS, IRQ_PRIORITY, 38, 24, 0,
	UART_C2_TE | UART_C2_RE | UART_C2_RIE,
	F_BUS,

	34, 255, 255,
	3, 0, 0,
	33, 255, 255,
 	3, 0, 0,		
	24, 255,
	3, 0
};

HardwareSerial Serial5((uintptr_t)&KINETISK_UART4, (uintptr_t)&HardwareSerial::serial5_hardware, tx_buffer5, SERIAL5_TX_BUFFER_SIZE, 
	rx_buffer5,  SERIAL5_RX_BUFFER_SIZE );

void uart4_status_isr(void) {
	Serial5.status_isr();
}

void serialEvent5() __attribute__((weak));
void serialEvent5() {}

#endif

// Serial6
// Warning - different for T3.6 (LPUART)
#if defined(__MK66FX1M0__)
static volatile BUFTYPE tx_buffer6[SERIAL6_TX_BUFFER_SIZE];
static volatile BUFTYPE rx_buffer6[SERIAL6_RX_BUFFER_SIZE];

const HardwareSerial::HardwareSerial_Hardware_t HardwareSerial::serial6_hardware = {
	SIM_SCGC2, SIM_SCGC2_LPUART0, IRQ_LPUART0, IRQ_PRIORITY, 38, 24, 0,
	0,
	0,

	47, 255, 255,
	5, 0, 0,
	48, 255, 255,
 	5, 0, 0,		
	56, 255,
	3, 0
};

HardwareSerial_LPUART Serial6((uintptr_t)&KINETISK_UART5, (uintptr_t)&HardwareSerial::serial6_hardware, tx_buffer6, SERIAL6_TX_BUFFER_SIZE, 
	rx_buffer6,  SERIAL6_RX_BUFFER_SIZE );

void lpuart0_status_isr(void) {
	Serial6.status_isr_lpuart();
}
void serialEvent6() __attribute__((weak));
void serialEvent6() {}

#elif defined(__MK64FX512__)
static volatile BUFTYPE tx_buffer6[SERIAL6_TX_BUFFER_SIZE];
static volatile BUFTYPE rx_buffer6[SERIAL6_RX_BUFFER_SIZE];

const HardwareSerial::HardwareSerial_Hardware_t HardwareSerial::serial6_hardware = {
	SIM_SCGC1, SIM_SCGC1_UART5, IRQ_UART5_STATUS, IRQ_PRIORITY, 38, 24, 0,
	UART_C2_TE | UART_C2_RE | UART_C2_RIE,
	F_BUS,

	47, 255, 255,
	3, 0, 0,
	48, 255, 255,
 	3, 0, 0,		
	56, 255,
	3, 0
};

HardwareSerial Serial6((uintptr_t)&KINETISK_UART5, (uintptr_t)&HardwareSerial::serial6_hardware, tx_buffer6, SERIAL6_TX_BUFFER_SIZE, 
	rx_buffer6,  SERIAL6_RX_BUFFER_SIZE);

void uart5_status_isr(void) {
	Serial6.status_isr();
}
void serialEvent6() __attribute__((weak));
void serialEvent6() {}


#endif

//==========================================================
// Teensy LC objects
//==========================================================
#elif defined(KINETISL)
// Serial1
static volatile BUFTYPE tx_buffer1[SERIAL1_TX_BUFFER_SIZE];
static volatile BUFTYPE rx_buffer1[SERIAL1_RX_BUFFER_SIZE];

const HardwareSerial::HardwareSerial_Hardware_t HardwareSerial::serial1_hardware = {
		SIM_SCGC4, SIM_SCGC4_UART0, IRQ_UART0_STATUS, IRQ_PRIORITY, 38, 24, 1,
	UART_C2_TE | UART_C2_RE | UART_C2_RIE,
	#if F_CPU <= 2000000
	(F_PLL / 16 ),
	#elif F_CPU <= 16000000
	(F_PLL / (F_PLL / 1000000)),
	#else
	(F_PLL / 2 / 16),
	#endif
	0, 21, 3, 25,
	3, 3, 2, 4,
	1, 5, 4, 24,
 	3, 3, 2, 4		
};


HardwareSerial Serial1((uintptr_t)&KINETISK_UART0, (uintptr_t)&HardwareSerial::serial1_hardware, tx_buffer1, SERIAL1_TX_BUFFER_SIZE, 
	rx_buffer1,  SERIAL1_RX_BUFFER_SIZE);
void uart0_status_isr(void) {
	Serial1.status_isr();
}

void serialEvent1() __attribute__((weak));
void serialEvent1() {}

// Serial2

static volatile BUFTYPE tx_buffer2[SERIAL2_TX_BUFFER_SIZE];
static volatile BUFTYPE rx_buffer2[SERIAL2_RX_BUFFER_SIZE];

const HardwareSerial::HardwareSerial_Hardware_t HardwareSerial::serial2_hardware = {
		SIM_SCGC4, SIM_SCGC4_UART1, IRQ_UART1_STATUS, IRQ_PRIORITY, 38, 24, 1,
	UART_C2_TE | UART_C2_RE | UART_C2_RIE,
	(F_BUS / 16),
	9, 255, 255, 255,
	3, 0, 0, 0, 
	10, 255, 255, 255,
 	3, 0, 0, 0 		
};

HardwareSerial Serial2((uintptr_t)&KINETISK_UART1, (uintptr_t)&HardwareSerial::serial2_hardware, tx_buffer2, SERIAL2_TX_BUFFER_SIZE, 
	rx_buffer2,  SERIAL2_RX_BUFFER_SIZE);

void uart1_status_isr(void) {
	Serial2.status_isr();
}

void serialEvent2() __attribute__((weak));
void serialEvent2() {}

// Serial 3

static volatile BUFTYPE tx_buffer3[SERIAL3_TX_BUFFER_SIZE];
static volatile BUFTYPE rx_buffer3[SERIAL3_RX_BUFFER_SIZE];

const HardwareSerial::HardwareSerial_Hardware_t HardwareSerial::serial3_hardware = {
		SIM_SCGC4, SIM_SCGC4_UART2, IRQ_UART2_STATUS, IRQ_PRIORITY, 38, 24, 0,
	UART_C2_TE | UART_C2_RE | UART_C2_RIE,
	(F_BUS / 16),

	7, 6, 255, 255, 
	3, 3, 0, 0, 
	8, 20, 255, 255,
 	3, 3, 0, 0		
};

HardwareSerial Serial3((uintptr_t)&KINETISK_UART2, (uintptr_t)&HardwareSerial::serial3_hardware, tx_buffer3, SERIAL3_TX_BUFFER_SIZE, 
	rx_buffer3,  SERIAL3_RX_BUFFER_SIZE );

void uart2_status_isr(void) {
	Serial3.status_isr();
}

void serialEvent3() __attribute__((weak));
void serialEvent3() {}

#endif // Kinetisl

// Warning: Now you pass in the Baud not the divisor...
void serial_begin(uint32_t divisor) {
	Serial1.begin(divisor);
}

void serial_print(const char *p) {
	Serial1.print(p);
}

static void serial_phex1(uint32_t n)
{
	n &= 15;
	if (n < 10) {
		Serial1.write('0' + n);
	} else {
		Serial1.write('A' - 10 + n);
	}
}

void serial_phex(uint32_t n)
{
	serial_phex1(n >> 4);
	serial_phex1(n);
}

void serial_phex16(uint32_t n)
{
	serial_phex(n >> 8);
	serial_phex(n);
}

void serial_phex32(uint32_t n)
{
	serial_phex(n >> 24);
	serial_phex(n >> 16);
	serial_phex(n >> 8);
	serial_phex(n);
}
