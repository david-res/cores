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

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include "kinetis.h"

// uncomment to enable 9 bit formats
//#define SERIAL_9BIT_SUPPORT

#define SERIAL_7E1 0x02
#define SERIAL_7O1 0x03
#define SERIAL_8N1 0x00
#define SERIAL_8N2 0x04
#define SERIAL_8E1 0x06
#define SERIAL_8O1 0x07
#define SERIAL_7E1_RXINV 0x12
#define SERIAL_7O1_RXINV 0x13
#define SERIAL_8N1_RXINV 0x10
#define SERIAL_8N2_RXINV 0x14
#define SERIAL_8E1_RXINV 0x16
#define SERIAL_8O1_RXINV 0x17
#define SERIAL_7E1_TXINV 0x22
#define SERIAL_7O1_TXINV 0x23
#define SERIAL_8N1_TXINV 0x20
#define SERIAL_8N2_TXINV 0x24
#define SERIAL_8E1_TXINV 0x26
#define SERIAL_8O1_TXINV 0x27
#define SERIAL_7E1_RXINV_TXINV 0x32
#define SERIAL_7O1_RXINV_TXINV 0x33
#define SERIAL_8N1_RXINV_TXINV 0x30
#define SERIAL_8N2_RXINV_TXINV 0x34
#define SERIAL_8E1_RXINV_TXINV 0x36
#define SERIAL_8O1_RXINV_TXINV 0x37
#ifdef SERIAL_9BIT_SUPPORT
#define SERIAL_9N1 0x84
#define SERIAL_9E1 0x8E
#define SERIAL_9O1 0x8F
#define SERIAL_9N1_RXINV 0x94
#define SERIAL_9E1_RXINV 0x9E
#define SERIAL_9O1_RXINV 0x9F
#define SERIAL_9N1_TXINV 0xA4
#define SERIAL_9E1_TXINV 0xAE
#define SERIAL_9O1_TXINV 0xAF
#define SERIAL_9N1_RXINV_TXINV 0xB4
#define SERIAL_9E1_RXINV_TXINV 0xBE
#define SERIAL_9O1_RXINV_TXINV 0xBF

// Teensy LC and 3.5 and 3.6 Uarts have 1/2 bit stop setting
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(KINETISL)
#define SERIAL_2STOP_BITS 0x100
#define SERIAL_8E2 (SERIAL_8E1 | SERIAL_2STOP_BITS)
#define SERIAL_8O2 (SERIAL_8O1 | SERIAL_2STOP_BITS)
#define SERIAL_8E2_RXINV (SERIAL_8E1_RXINV | SERIAL_2STOP_BITS)
#define SERIAL_8O2_RXINV (SERIAL_8O1_RXINV | SERIAL_2STOP_BITS)
#define SERIAL_8E2_TXINV (SERIAL_8E1_TXINV | SERIAL_2STOP_BITS)
#define SERIAL_8O2_TXINV (SERIAL_8O1_TXINV | SERIAL_2STOP_BITS)
#define SERIAL_8E2_RXINV_TXINV (SERIAL_8E1_RXINV_TXINV | SERIAL_2STOP_BITS)
#define SERIAL_8O2_RXINV_TXINV (SERIAL_8O1_RXINV_TXINV | SERIAL_2STOP_BITS)
#endif
// bit0: parity, 0=even, 1=odd
// bit1: parity, 0=disable, 1=enable
// bit2: mode, 1=9bit, 0=8bit
// bit3: mode10: 1=10bit, 0=8bit
// bit4: rxinv, 0=normal, 1=inverted
// bit5: txinv, 0=normal, 1=inverted
// bit6: unused
// bit7: actual data goes into 9th bit


#if defined(KINETISK)
#define BAUD2DIV(baud)  (((F_CPU * 2) + ((baud) >> 1)) / (baud))
#define BAUD2DIV2(baud) (((F_CPU * 2) + ((baud) >> 1)) / (baud))
#define BAUD2DIV3(baud) (((F_BUS * 2) + ((baud) >> 1)) / (baud))
#elif defined(KINETISL)

#if F_CPU <= 2000000
#define BAUD2DIV(baud)  (((F_PLL / 16 ) + ((baud) >> 1)) / (baud))
#elif F_CPU <= 16000000
#define BAUD2DIV(baud)  (((F_PLL / (F_PLL / 1000000)) + ((baud) >> 1)) / (baud))
#else
#define BAUD2DIV(baud)  (((F_PLL / 2 / 16) + ((baud) >> 1)) / (baud))
#endif

#define BAUD2DIV2(baud) (((F_BUS / 16) + ((baud) >> 1)) / (baud))
#define BAUD2DIV3(baud) (((F_BUS / 16) + ((baud) >> 1)) / (baud))
#endif

#endif // Temporary - included from HardwareSerial.h

#ifdef SERIAL_9BIT_SUPPORT
#define BUFTYPE uint16_t
#else
#define BUFTYPE uint8_t
#endif

// BUGBUG:: need to fiqure out how many neededed per board. 
#if defined(__MK20DX128__) || defined(__MK20DX256__)
#define CNT_TX_PINS 2
#define CNT_RX_PINS 2
#define CNT_CTS_PINS 2
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define CNT_TX_PINS 3
#define CNT_RX_PINS 3
#define CNT_CTS_PINS 2
#elif defined(KINETISL)
#define CNT_TX_PINS 4
#define CNT_RX_PINS 4
#endif
// C++ interface
//
#ifdef __cplusplus
#include "Stream.h"
class HardwareSerial : public Stream
{
public:
	typedef struct {
		volatile uint32_t &clock_gate_register;
		uint32_t clock_gate_mask;
		IRQ_NUMBER_t irq;
		uint16_t irq_priority;
		uint16_t rts_low_watermark;
		uint16_t rts_high_watermark;
		uint8_t uart_fifo; 
		uint32_t c2_enable;
		uint32_t uart_clock;			// which clock to use
		uint8_t  rx_pin[CNT_RX_PINS];
		uint8_t  rx_mux[CNT_RX_PINS];
		uint8_t  tx_pin[CNT_TX_PINS];
		uint8_t  tx_mux[CNT_TX_PINS];
#ifdef KINETISK
		uint8_t  cts_pin[CNT_CTS_PINS];
		uint8_t cts_mux[CNT_CTS_PINS];
#endif		
	} HardwareSerial_Hardware_t;
	static const HardwareSerial_Hardware_t serial1_hardware;
	static const HardwareSerial_Hardware_t serial2_hardware;
	static const HardwareSerial_Hardware_t serial3_hardware;
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
#endif
	static const HardwareSerial_Hardware_t serial4_hardware;
	static const HardwareSerial_Hardware_t serial5_hardware;
	static const HardwareSerial_Hardware_t serial6_hardware;


public:
	constexpr HardwareSerial(uintptr_t myport, uintptr_t myhardware, 
			volatile BUFTYPE *_tx_buffer, size_t _tx_buffer_size, 
			volatile BUFTYPE *_rx_buffer, size_t _rx_buffer_size)
		: port_addr_(myport), hardware_addr_(myhardware),
		tx_buffer_(_tx_buffer), rx_buffer_(_rx_buffer), tx_buffer_size_(_tx_buffer_size),  rx_buffer_size_(_rx_buffer_size),
		tx_buffer_total_size_(_tx_buffer_size), rx_buffer_total_size_(_rx_buffer_size)
		{}

	virtual void begin(uint32_t baud);
	virtual void begin(uint32_t baud, uint32_t format);
	virtual void end(void);
	void transmitterEnable(uint8_t pin);
	void setRX(uint8_t pin);
	void setTX(uint8_t pin, bool opendrain=false) ;
	bool attachRts(uint8_t pin) ;
	virtual bool attachCts(uint8_t pin) ;
	int available(void);
	int peek(void);
	virtual int read(void);
	void flush(void);
	void clear(void);
	int availableForWrite(void);
	void addStorageForRead(void *buffer, size_t length);
	void addStorageForWrite(void *buffer, size_t length);

	using Print::write;
	virtual size_t write(uint8_t c);
	virtual size_t write(unsigned long n)   { return write((uint8_t)n); }
	virtual size_t write(long n)            { return write((uint8_t)n); }
	virtual size_t write(unsigned int n)    { return write((uint8_t)n); }
	virtual size_t write(int n)             { return write((uint8_t)n); }
	virtual size_t write(const uint8_t *buffer, size_t size);
    virtual size_t write(const char *str);
	virtual size_t write9bit(uint32_t c)	{ return write((uint8_t)c);  }
	operator bool()			{ return true; }

protected: 
#ifdef KINETISK
	KINETISK_UART_t & port() { return *(KINETISK_UART_t *)port_addr_; }
#elif defined(KINETISL)
	KINETISK_UART_t & port() { return *(KINETISK_UART_t *)port_addr_; }
#endif	
	const HardwareSerial_Hardware_t & hardware() { return *(const HardwareSerial_Hardware_t *)hardware_addr_; }

	uintptr_t 			port_addr_;
	uintptr_t 			hardware_addr_;

#ifdef SERIAL_9BIT_SUPPORT
	uint8_t 			use9Bits_ = 0;
#else
	const uint8_t 		use9Bits_ = 0;
#endif

	volatile BUFTYPE 	*tx_buffer_;
	volatile BUFTYPE 	*rx_buffer_;
	volatile BUFTYPE	*rx_buffer_storage_ = nullptr;
	volatile BUFTYPE	*tx_buffer_storage_ = nullptr;
	size_t				tx_buffer_size_;
	size_t				rx_buffer_size_;

	// This is the 
	size_t				tx_buffer_total_size_;
	size_t				rx_buffer_total_size_;


	size_t  			rts_low_watermark_ = 0;
	size_t  			rts_high_watermark_ = 0;

	volatile uint8_t 	transmitting_ = 0;

	uint8_t 			tx_pin_index_ = 0;
	bool 				tx_opendrain_ = 0;
	uint8_t 			rx_pin_index_ = 0;
	uint8_t 			cts_pin_index_ = 0;

	// Will have rx and tx counters 16 bits as the user may 
	// extend the buffers through add buffer calls. 
	// Maybe should be size_t?
	volatile uint16_t 	tx_buffer_head_ = 0;
	volatile uint16_t 	tx_buffer_tail_ = 0;
	volatile uint16_t 	rx_buffer_head_ = 0;
	volatile uint16_t 	rx_buffer_tail_ = 0;

#if defined(KINETISK)
	volatile uint8_t 	*transmit_pin_=NULL;
	volatile uint8_t 	*rts_pin_=NULL;

	inline void transmit_assert() {*transmit_pin_ = 1;}
	inline void transmit_deassert() {*transmit_pin_ = 0;}
  	inline void rts_assert()   {*rts_pin_ = 0; }
  	inline void rts_deassert()  {*rts_pin_ = 1;}
#elif defined(KINETISL)
	volatile uint8_t *		transmit_pin_=NULL;
	uint8_t 				transmit_mask_=0;
	volatile uint8_t 		*rts_pin_=NULL;
	uint8_t 				rts_mask_=0;

	inline void transmit_assert() { *(transmit_pin_+4) = transmit_mask_; }
	inline void transmit_deassert() {*(transmit_pin_+8) = transmit_mask_; }
	inline void rts_assert() {*(rts_pin_+8) = rts_mask_; }
	inline void rts_deassert() {*(rts_pin_+4) = rts_mask_; }
#endif
	friend void uart0_status_isr(void);
	friend void uart1_status_isr(void);
	friend void uart2_status_isr(void);
	friend void uart3_status_isr(void);
	friend void uart4_status_isr(void);
	friend void uart5_status_isr(void);
	inline void status_isr(void);
	#ifdef KINETISK
	inline void status_isr_fifo(void);
	#endif
};

#if defined(__MK66FX1M0__)
// Do sub class for LPUart
class HardwareSerial_LPUART : public HardwareSerial
{
public:
	constexpr HardwareSerial_LPUART(uintptr_t myport, uintptr_t myhardware, 
			volatile BUFTYPE *_tx_buffer, size_t _tx_buffer_size, 
			volatile BUFTYPE *_rx_buffer, size_t _rx_buffer_size) :
		HardwareSerial(myport, myhardware, _tx_buffer, _tx_buffer_size, _rx_buffer, _rx_buffer_size) {}

	virtual void begin(uint32_t baud);
	virtual void begin(uint32_t baud, uint32_t format);
	virtual void end(void);
	virtual bool attachCts(uint8_t pin) ;
	using Print::write;
	virtual size_t write(uint8_t c);

	friend void lpuart0_status_isr(void);
	inline void status_isr_lpuart(void);

};
#endif

extern HardwareSerial Serial1;
extern void serialEvent1(void);

extern HardwareSerial Serial2;
extern void serialEvent2(void);

extern HardwareSerial Serial3;
extern void serialEvent3(void);

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
extern HardwareSerial Serial4;
extern void serialEvent4(void);
extern HardwareSerial Serial5;
extern void serialEvent5(void);
#if defined(__MK64FX512__)
extern HardwareSerial Serial6;
#else
extern HardwareSerial_LPUART Serial6;
#endif
extern void serialEvent6(void);
#endif

#endif

// Define C versions for those functions called in debug of core. 
#ifdef __cplusplus
extern "C" {
#endif
void serial_begin(uint32_t divisor);
void serial_print(const char *p);
void serial_phex(uint32_t n);
void serial_phex16(uint32_t n);
void serial_phex32(uint32_t n);
#ifdef __cplusplus
}
#endif



#endif
