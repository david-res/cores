
#include "HardwareSerial.h"
#include "core_pins.h"
//#include "Arduino.h"
//#include "debug/printf.h"

/*typedef struct {
        const uint32_t VERID;
        const uint32_t PARAM;
        volatile uint32_t GLOBAL;
        volatile uint32_t PINCFG;
        volatile uint32_t BAUD;
        volatile uint32_t STAT;
        volatile uint32_t CTRL;
        volatile uint32_t DATA;
        volatile uint32_t MATCH;
        volatile uint32_t MODIR;
        volatile uint32_t FIFO;
        volatile uint32_t WATER;
} IMXRT_LPUART_t; */

#define UART_CLOCK 24000000

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
#ifndef SERIAL7_TX_BUFFER_SIZE
#define SERIAL7_TX_BUFFER_SIZE     40 // number of outgoing bytes to buffer
#endif
#ifndef SERIAL7_RX_BUFFER_SIZE
#define SERIAL7_RX_BUFFER_SIZE     64 // number of incoming bytes to buffer
#endif
#ifndef SERIAL8_TX_BUFFER_SIZE
#define SERIAL8_TX_BUFFER_SIZE     40 // number of outgoing bytes to buffer
#endif
#ifndef SERIAL8_RX_BUFFER_SIZE
#define SERIAL8_RX_BUFFER_SIZE     64 // number of incoming bytes to buffer
#endif

#define IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest

#define CTRL_ENABLE 		(LPUART_CTRL_TE | LPUART_CTRL_RE | LPUART_CTRL_RIE)
#define CTRL_TX_ACTIVE		(CTRL_ENABLE | LPUART_CTRL_TIE)
#define CTRL_TX_COMPLETING	(CTRL_ENABLE | LPUART_CTRL_TCIE)
#define CTRL_TX_INACTIVE	CTRL_ENABLE 

void HardwareSerial::begin(uint32_t baud, uint8_t format)
{
	//printf("HardwareSerial begin\n");
	float base = (float)UART_CLOCK / (float)baud;
	float besterr = 1e20;
	int bestdiv = 1;
	int bestosr = 4;
	for (int osr=4; osr <= 32; osr++) {
		float div = base / (float)osr;
		int divint = (int)(div + 0.5f);
		if (divint < 1) divint = 1;
		else if (divint > 8191) divint = 8191;
		float err = ((float)divint - div) / div;
		if (err < 0.0f) err = -err;
		if (err <= besterr) {
			besterr = err;
			bestdiv = divint;
			bestosr = osr;
		}
	}
	//printf(" baud %d: osr=%d, div=%d\n", baud, bestosr, bestdiv);
	hardware->ccm_register |= hardware->ccm_value;
	hardware->rx_mux_register = hardware->rx_mux_val;
	hardware->tx_mux_register = hardware->tx_mux_val;
	port->BAUD = LPUART_BAUD_OSR(bestosr - 1) | LPUART_BAUD_SBR(bestdiv);

	// Enable the transmitter, receiver and enable receiver interrupt
	attachInterruptVector(hardware->irq, hardware->irq_handler);
	NVIC_SET_PRIORITY(hardware->irq, IRQ_PRIORITY);	// maybe should put into hardware...
	NVIC_ENABLE_IRQ(hardware->irq);

	port->CTRL = CTRL_TX_INACTIVE;
};

int HardwareSerial::available(void)
{
	return -1;
}

int HardwareSerial::peek(void)
{
	return -1;
}

int HardwareSerial::read(void)
{
	return -1;
}
void HardwareSerial::flush(void)
{
	while (transmitting_) yield(); // wait
}

size_t HardwareSerial::write(uint8_t c)
{
	uint32_t head, n;
	//digitalWrite(3, HIGH);
	//digitalWrite(5, HIGH);
//	if (transmit_pin_) transmit_assert();
	head = tx_buffer_head_;
	if (++head >= tx_buffer_total_size_) head = 0;
	while (tx_buffer_tail_ == head) {
		/* int priority = nvic_execution_priority();
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
		} */
		yield();
	}
	//digitalWrite(5, LOW);
	//Serial.printf("WR %x %d %d %d %x %x\n", c, head, tx_buffer_size_,  tx_buffer_total_size_, (uint32_t)tx_buffer_, (uint32_t)tx_buffer_storage_);
	if (head < tx_buffer_size_) {
		tx_buffer_[head] = c;
	} else {
		tx_buffer_storage_[head - tx_buffer_size_] = c;
	}
	transmitting_ = 1;
	tx_buffer_head_ = head;
	port->CTRL |= LPUART_CTRL_TIE; // (may need to handle this issue)BITBAND_SET_BIT(LPUART0_CTRL, TIE_BIT);
	//digitalWrite(3, LOW);
	return 1;
}

void HardwareSerial::IRQHandler() 
{
	//digitalWrite(4, HIGH);
	uint32_t head, tail, n;
	uint32_t ctrl;

	// See if we are transmitting and room in buffer. 
	ctrl = port->CTRL;
	if ((ctrl & LPUART_CTRL_TIE) && (port->STAT & LPUART_STAT_TDRE))
	{
		head = tx_buffer_head_;
		tail = tx_buffer_tail_;
		if (head == tail) {
			port->CTRL = CTRL_TX_COMPLETING;
		} else {
			if (++tail >= tx_buffer_total_size_) tail = 0;
			if (tail < tx_buffer_size_) {
				n = tx_buffer_[tail];
			} else {
				n = tx_buffer_storage_[tail-tx_buffer_size_];
			}
			//if (use9Bits_) port().C3 = (port().C3 & ~0x40) | ((n & 0x100) >> 2);
			port->DATA = n;
			tx_buffer_tail_ = tail;
		}
	}

	if ((ctrl & LPUART_CTRL_TCIE) && (port->STAT & LPUART_STAT_TDRE))
	{
		transmitting_ = 0;
		//if (transmit_pin_) transmit_deassert();
		port->CTRL = CTRL_TX_INACTIVE;
	}
	//digitalWrite(4, LOW);
}

void IRQHandler_Serial1()
{
	Serial1.IRQHandler();
}

void IRQHandler_Serial2()
{
	Serial2.IRQHandler();
}

void IRQHandler_Serial3()
{
	Serial3.IRQHandler();
}

void IRQHandler_Serial4()
{
	Serial4.IRQHandler();
}

void IRQHandler_Serial5()
{
	Serial5.IRQHandler();
}

void IRQHandler_Serial6()
{
	Serial6.IRQHandler();
}

void IRQHandler_Serial7()
{
	Serial7.IRQHandler();
}

void IRQHandler_Serial8()
{
	Serial8.IRQHandler();
}




// Serial1
static BUFTYPE tx_buffer1[SERIAL1_TX_BUFFER_SIZE];
static BUFTYPE rx_buffer1[SERIAL1_RX_BUFFER_SIZE];

const HardwareSerial::hardware_t UART6_Hardware = {
	IRQ_LPUART6, &IRQHandler_Serial1,
	CCM_CCGR3, CCM_CCGR3_LPUART6(CCM_CCGR_ON),
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03, // pin 0
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02, // pin 1
	2, // page 473
	2, // page 472
};
HardwareSerial Serial1(&IMXRT_LPUART6, &UART6_Hardware, tx_buffer1, SERIAL1_TX_BUFFER_SIZE,
	rx_buffer1,  SERIAL1_RX_BUFFER_SIZE);

// Serial2
static BUFTYPE tx_buffer2[SERIAL2_TX_BUFFER_SIZE];
static BUFTYPE rx_buffer2[SERIAL2_RX_BUFFER_SIZE];

static HardwareSerial::hardware_t UART4_Hardware = {
	IRQ_LPUART4, &IRQHandler_Serial2,
	CCM_CCGR1, CCM_CCGR1_LPUART4(CCM_CCGR_ON),
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01, // pin 6
	IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00, // pin 7
	2, // page 521
	2, // page 520
};
HardwareSerial Serial2(&IMXRT_LPUART4, &UART4_Hardware, tx_buffer2, SERIAL2_TX_BUFFER_SIZE, 
	rx_buffer2,  SERIAL2_RX_BUFFER_SIZE);

// Serial3
static BUFTYPE tx_buffer3[SERIAL3_TX_BUFFER_SIZE];
static BUFTYPE rx_buffer3[SERIAL3_RX_BUFFER_SIZE];
static HardwareSerial::hardware_t UART2_Hardware = {
	IRQ_LPUART2, &IRQHandler_Serial3,
	CCM_CCGR0, CCM_CCGR0_LPUART2(CCM_CCGR_ON),
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03, // pin 15
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02, // pin 14
	2, // page 491
	2, // page 490
};
HardwareSerial Serial3(&IMXRT_LPUART2, &UART2_Hardware,tx_buffer3, SERIAL3_TX_BUFFER_SIZE,
	rx_buffer3,  SERIAL3_RX_BUFFER_SIZE);

// Serial4
static BUFTYPE tx_buffer4[SERIAL4_TX_BUFFER_SIZE];
static BUFTYPE rx_buffer4[SERIAL4_RX_BUFFER_SIZE];
static HardwareSerial::hardware_t UART3_Hardware = {
	IRQ_LPUART3, &IRQHandler_Serial4,
	CCM_CCGR0, CCM_CCGR0_LPUART3(CCM_CCGR_ON),
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07, // pin 16
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06, // pin 17
	2, // page 495
	2, // page 494
};
HardwareSerial Serial4(&IMXRT_LPUART3, &UART3_Hardware, tx_buffer4, SERIAL4_TX_BUFFER_SIZE,
	rx_buffer4,  SERIAL4_RX_BUFFER_SIZE);

// Serial5
static BUFTYPE tx_buffer5[SERIAL5_TX_BUFFER_SIZE];
static BUFTYPE rx_buffer5[SERIAL5_RX_BUFFER_SIZE];
static HardwareSerial::hardware_t UART8_Hardware = {
	IRQ_LPUART8, &IRQHandler_Serial5,
	CCM_CCGR6, CCM_CCGR6_LPUART8(CCM_CCGR_ON),
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11, // pin 21
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10, // pin 20
	2, // page 499
	2, // page 498
};
HardwareSerial Serial5(&IMXRT_LPUART8, &UART8_Hardware, tx_buffer5, SERIAL5_TX_BUFFER_SIZE,
	rx_buffer5,  SERIAL5_RX_BUFFER_SIZE);

// Serial6
static BUFTYPE tx_buffer6[SERIAL6_TX_BUFFER_SIZE];
static BUFTYPE rx_buffer6[SERIAL6_RX_BUFFER_SIZE];
static HardwareSerial::hardware_t UART1_Hardware = {
	IRQ_LPUART1, &IRQHandler_Serial6,
	CCM_CCGR5, CCM_CCGR5_LPUART1(CCM_CCGR_ON),
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_13, // pin 25
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_12, // pin 24
	2, // page 486
	2, // page 485
};
HardwareSerial Serial6(&IMXRT_LPUART1, &UART1_Hardware, tx_buffer6, SERIAL6_TX_BUFFER_SIZE,
	rx_buffer6,  SERIAL6_RX_BUFFER_SIZE);

// Serial7
static BUFTYPE tx_buffer7[SERIAL7_TX_BUFFER_SIZE];
static BUFTYPE rx_buffer7[SERIAL7_RX_BUFFER_SIZE];
__attribute__((section(".progmem")))
static HardwareSerial::hardware_t UART7_Hardware = {
	IRQ_LPUART7, &IRQHandler_Serial7,
	CCM_CCGR5, CCM_CCGR5_LPUART7(CCM_CCGR_ON),
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_32, // pin 28
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_31, // pin 29
	2, // page 458
	2, // page 457
};
HardwareSerial Serial7(&IMXRT_LPUART7, &UART7_Hardware, tx_buffer7, SERIAL7_TX_BUFFER_SIZE,
	rx_buffer7,  SERIAL7_RX_BUFFER_SIZE);

// Serial8
static BUFTYPE tx_buffer8[SERIAL8_TX_BUFFER_SIZE];
static BUFTYPE rx_buffer8[SERIAL8_RX_BUFFER_SIZE];
__attribute__((section(".progmem")))
static HardwareSerial::hardware_t UART5_Hardware = {
	IRQ_LPUART5, &IRQHandler_Serial8,
	CCM_CCGR3, CCM_CCGR3_LPUART5(CCM_CCGR_ON),
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_24, // pin 30
	IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_23, // pin 31
	2, // page 450
	2, // page 449
};
HardwareSerial Serial8(&IMXRT_LPUART5, &UART5_Hardware, tx_buffer8, SERIAL8_TX_BUFFER_SIZE,
	rx_buffer8,  SERIAL8_RX_BUFFER_SIZE);




