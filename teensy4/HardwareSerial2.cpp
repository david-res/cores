/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2019 PJRC.COM, LLC.
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

#include <Arduino.h>
#include "HardwareSerial.h"

#ifndef SERIAL2_TX_BUFFER_SIZE
#define SERIAL2_TX_BUFFER_SIZE     40 // number of outgoing bytes to buffer
#endif
#ifndef SERIAL2_RX_BUFFER_SIZE
#define SERIAL2_RX_BUFFER_SIZE     64 // number of incoming bytes to buffer
#endif
#define IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest


void IRQHandler_Serial2()
{
	Serial2.IRQHandler();
}

#ifndef SERIAL2_RX_PINS 
#define SERIAL2_CTS_PIN 0xff, 0 
#if !defined(ARDUINO_TEENSY_MICROMOD)
#define SERIAL2_UART_ADDR IMXRT_LPUART4_ADDRESS
#define SERIAL2_LPUART	IRQ_LPUART4, CCM_CCGR1, CCM_CCGR1_LPUART4(CCM_CCGR_ON), XBARA1_OUT_LPUART4_TRG_INPUT
#define SERIAL2_RX_PINS	{{7,2, &IOMUXC_LPUART4_RX_SELECT_INPUT, 2}, {0xff, 0xff, nullptr, 0}}
#define SERIAL2_TX_PINS	{{8,2, &IOMUXC_LPUART4_TX_SELECT_INPUT, 2}, {0xff, 0xff, nullptr, 0}}

#else
#define SERIAL2_UART_ADDR IMXRT_LPUART3_ADDRESS
#define SERIAL2_LPUART   IRQ_LPUART3, CCM_CCGR0, CCM_CCGR0_LPUART3(CCM_CCGR_ON), XBARA1_OUT_LPUART3_TRG_INPUT
#define SERIAL2_RX_PINS  {{16,2, &IOMUXC_LPUART3_RX_SELECT_INPUT, 0}, {0xff, 0xff, nullptr, 0}}
#define SERIAL2_TX_PINS  {{17,2, &IOMUXC_LPUART3_TX_SELECT_INPUT, 0}, {0xff, 0xff, nullptr, 0}}
#endif
#endif


// Serial2
static BUFTYPE tx_buffer2[SERIAL2_TX_BUFFER_SIZE];
static BUFTYPE rx_buffer2[SERIAL2_RX_BUFFER_SIZE];

static HardwareSerialIMXRT::hardware_t Serial2_Hardware = {
	1, 
	&IRQHandler_Serial2, 
	&serialEvent2,
	IRQ_PRIORITY, 38, 24, // IRQ, rts_low_watermark, rts_high_watermark
	// Stuff that can be overwritten easily by variant
	SERIAL2_LPUART, SERIAL2_RX_PINS, SERIAL2_TX_PINS, SERIAL2_CTS_PIN
};
HardwareSerialIMXRT Serial2(SERIAL2_UART_ADDR, &Serial2_Hardware, tx_buffer2,
	SERIAL2_TX_BUFFER_SIZE, rx_buffer2, SERIAL2_RX_BUFFER_SIZE);
