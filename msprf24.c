/* msprf24.c
 * MSP430 library for interfacing with the nRF24L01+ RF transceiver by
 * Nordic Semiconductor.
 *
 * Serial interfaces supported:
 * 1. USI - developed on MSP430G2231
 * 2. USCI_A - developed on MSP430G2553
 * 3. USCI_B - developed on MSP430G2553
 * 4. USCI_A F5xxx - developed on MSP430F5172
 * 5. USCI_B F5xxx - developed on MSP430F5172
 *
 * MSP430-specific code inspired/derived from dkedr's nrf24 library posted on the 43oh forums:
 * http://www.43oh.com/forum/viewtopic.php?f=10&t=2572
 *
 *
 * Copyright (c) 2012, Eric Brundick <spirilis@linux.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any purpose
 * with or without fee is hereby granted, provided that the above copyright notice
 * and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT,
 * OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <msp430.h>
#include "msprf24.h"
#include "msp430_spi.h"
#include "nRF24L01.h"
#include "nrf_userconfig.h"
/* ^ Provides nrfCSNport, nrfCSNportout, nrfCSNpin,
     nrfCEport, nrfCEportout, nrfCEpin,
     nrfIRQport, nrfIRQpin
     Also specify # clock cycles for 5ms, 10us and 130us sleeps.
 */
/* Private library variables */
uint8_t rf_feature;  // Used to track which features have been enabled

/* CE (Chip Enable/RF transceiver activate signal) and CSN (SPI chip-select) operations. */
#define CSN_EN nrfCSNportout &= ~nrfCSNpin
#define CSN_DIS nrfCSNportout |= nrfCSNpin
#define CE_EN nrfCEportout |= nrfCEpin
#define CE_DIS nrfCEportout &= ~nrfCEpin



/* SPI drivers now supplied by msp430_spi.c */




/* Basic I/O to the device. */
uint8_t r_reg(uint8_t addr)
{
	uint16_t i;

	CSN_EN;
	i = spi_transfer16(RF24_NOP | ((addr & RF24_REGISTER_MASK) << 8));
	rf_status = (uint8_t) ((i & 0xFF00) >> 8);
	CSN_DIS;
	return (uint8_t) (i & 0x00FF);
}

void w_reg(uint8_t addr, uint8_t data)
{
	uint16_t i;
	CSN_EN;
	i = spi_transfer16( (data & 0x00FF) | (((addr & RF24_REGISTER_MASK) | RF24_W_REGISTER) << 8) );
	rf_status = (uint8_t) ((i & 0xFF00) >> 8);
	CSN_DIS;
}

void w_tx_addr(uint8_t *addr)
{
	int i;

	CSN_EN;
	rf_status = spi_transfer(RF24_TX_ADDR | RF24_W_REGISTER);
	for (i=rf_addr_width-1; i>=0; i--) {
		spi_transfer(addr[i]);
	}
	CSN_DIS;
}

void w_rx_addr(uint8_t pipe, uint8_t *addr)
{
	int i;

	if (pipe > 5)
		return;  // Only 6 pipes available
	CSN_EN;
	rf_status = spi_transfer((RF24_RX_ADDR_P0 + pipe) | RF24_W_REGISTER);
	if (pipe > 1) {  // Pipes 2-5 differ from pipe1's addr only in the LSB.
		spi_transfer(addr[rf_addr_width-1]);
	} else {
		for (i=rf_addr_width-1; i>=0; i--) {
			spi_transfer(addr[i]);
		}
	}
	CSN_DIS;
}

void w_tx_payload(uint8_t len, uint8_t *data)
{
	uint16_t i=0;
	CSN_EN;
	if (len % 2) {  // Odd payload size?  Make it even by stuffing the command in a 16-bit xfer
		// Borrowing 'i' to extract STATUS...
		i = spi_transfer16( (RF24_W_TX_PAYLOAD << 8) | (0x00FF & data[0]) );
		rf_status = (i & 0xFF00) >> 8;
		i = 1;
	} else {
		rf_status = spi_transfer(RF24_W_TX_PAYLOAD);
	}
	for (; i < len; i+=2) {
		// SPI transfers MSB first
		spi_transfer16( (data[i] << 8) | (0x00FF & data[i+1]) );
	}
	CSN_DIS;
}

void w_tx_payload_noack(uint8_t len, uint8_t *data)
{
	uint16_t i=0;

	if ( !(rf_feature & RF24_EN_DYN_ACK) )  // DYN ACK must be enabled to allow NOACK packets
		return;
	CSN_EN;
	if (len % 2) {
		// Borrowing 'i' to extract STATUS...
		i = spi_transfer16( (RF24_W_TX_PAYLOAD_NOACK << 8) | (0x00FF & data[0]) );
		rf_status = (i & 0xFF00) >> 8;
		i = 1;
	} else {
		rf_status = spi_transfer(RF24_W_TX_PAYLOAD_NOACK);
	}
	for (; i < len; i+=2) {
		// SPI transfers MSB first
		spi_transfer16( (data[i] << 8) | (0x00FF & data[i+1]) );
	}
	CSN_DIS;
}

uint8_t r_rx_peek_payload_size()
{
	uint16_t i;

	CSN_EN;
	i = spi_transfer16(RF24_NOP | (RF24_R_RX_PL_WID << 8));
	rf_status = (uint8_t) ((i & 0xFF00) >> 8);
	CSN_DIS;
	return (uint8_t) (i & 0x00FF);
}

uint8_t r_rx_payload(uint8_t len, uint8_t *data)
{
	uint16_t i=0,j;
	CSN_EN;
	if (len % 2) {
		// Borrowing 'i' to extract STATUS...
		i = spi_transfer16((RF24_R_RX_PAYLOAD << 8) | RF24_NOP);
		rf_status = (i & 0xFF00) >> 8;
		data[0] = i & 0x00FF;
		i = 1;
	} else {
		rf_status = spi_transfer(RF24_R_RX_PAYLOAD);
	}
	for (; i < len; i+=2) {
		j = spi_transfer16(0xFFFF);
		// SPI transfers MSB first
		data[i] = (j & 0xFF00) >> 8;
		data[i+1] = (j & 0x00FF);
	}
	CSN_DIS;
	// The RX pipe this data belongs to is stored in STATUS
	return ((rf_status & 0x0E) >> 1);
}

void flush_tx()
{
	CSN_EN;
	rf_status = spi_transfer(RF24_FLUSH_TX);
	CSN_DIS;
}

void flush_rx()
{
	CSN_EN;
	rf_status = spi_transfer(RF24_FLUSH_RX);
	CSN_DIS;
}

void tx_reuse_lastpayload()
{
	CSN_EN;
	rf_status = spi_transfer(RF24_REUSE_TX_PL);
	CSN_DIS;
}

void pulse_ce()
{
	CE_EN;
	__delay_cycles(DELAY_CYCLES_15US);
	CE_DIS;
}

/* Used to manually ACK with a payload.  Must have RF24_EN_ACK_PAY enabled; this is not enabled by default
 * with msprf24_init() FYI.
 * When RF24_EN_ACK_PAY is enabled on the PTX side, ALL transmissions must be manually ACK'd by the receiver this way.
 * The receiver (PRX) side needs to have RF24_EN_ACK_PAY enabled too, or else it will automatically ACK with a zero-byte packet.
 *
 * If you have this enabled on the PTX but not the PRX, the transmission will go through and the PRX will receive/notify about
 * the RX payload, but the PTX will ignore the zero-byte autoack from the PRX and perform its retransmit sequence, erroring
 * out with MAX_RT (RF24_IRQ_TXFAILED) after (RF24_SETUP_RETR >> RF24_ARC) retransmissions.
 * When this occurs, the PRX will still only notify its microcontroller of the payload once (the PID field in the packet uniquely
 * identifies it so the PRX knows it's the same packet being retransmitted) but it's obviously wasting on-air time (and power).
 */
void w_ack_payload(uint8_t pipe, uint8_t len, uint8_t *data)
{
	uint16_t i=0;
	CSN_EN;

	if (pipe > 5)
		return;
	if ( !(rf_feature & RF24_EN_ACK_PAY) )  // ACK payloads must be enabled...
		return;

	if (len % 2) {
		// Borrowing 'i' to extract STATUS...
		i = spi_transfer16( ((RF24_W_ACK_PAYLOAD | pipe) << 8) | (0x00FF & data[0]) );
		rf_status = (i & 0xFF00) >> 8;
		i = 1;
	} else {
		rf_status = spi_transfer(RF24_W_ACK_PAYLOAD | pipe);
	}
	for (; i < len; i+=2) {
		// SPI transfers MSB first
		spi_transfer16( (data[i] << 8) | (0x00FF & data[i+1]) );
	}
	CSN_DIS;
}






/* Configuration parameters used to set-up the RF configuration */
uint8_t rf_crc;
uint8_t rf_addr_width;
uint8_t rf_speed_power;
uint8_t rf_channel;
/* Status variable updated every time SPI I/O is performed */
uint8_t rf_status;
/* IRQ state is stored in here after msprf24_get_irq_reason(), RF24_IRQ_FLAGGED raised during
 * the IRQ port ISR--user application issuing LPMx sleep or polling should watch for this to
 * determine if the wakeup reason was due to nRF24 IRQ.
 */
volatile uint8_t rf_irq;





/* Library functions */
void msprf24_init()
{
	// Setup SPI
	spi_init();
	_EINT();  // Enable interrupts (set GIE in SR)

	// Setup IRQ
	#if nrfIRQport == 1
		P1DIR &= ~nrfIRQpin;  // IRQ line is input
		P1OUT |= nrfIRQpin;   // Pull-up resistor enabled
		P1REN |= nrfIRQpin;
		P1IES |= nrfIRQpin;   // Trigger on falling-edge
		P1IFG &= ~nrfIRQpin;  // Clear any outstanding IRQ
		P1IE |= nrfIRQpin;    // Enable IRQ interrupt
	#elif nrfIRQport == 2
		P2DIR &= ~nrfIRQpin;  // IRQ line is input
		P2OUT |= nrfIRQpin;   // Pull-up resistor enabled
		P2REN |= nrfIRQpin;
		P2IES |= nrfIRQpin;   // Trigger on falling-edge
		P2IFG &= ~nrfIRQpin;  // Clear any outstanding IRQ
		P2IE |= nrfIRQpin;    // Enable IRQ interrupt
	#elif nrfIRQport == 3
		P3DIR &= ~nrfIRQpin;  // IRQ line is input
		P3OUT |= nrfIRQpin;   // Pull-up resistor enabled
		P3REN |= nrfIRQpin;
		P3IES |= nrfIRQpin;   // Trigger on falling-edge
		P3IFG &= ~nrfIRQpin;  // Clear any outstanding IRQ
		P3IE |= nrfIRQpin;    // Enable IRQ interrupt
	#elif nrfIRQport == 4
		P4DIR &= ~nrfIRQpin;  // IRQ line is input
		P4OUT |= nrfIRQpin;   // Pull-up resistor enabled
		P4REN |= nrfIRQpin;
		P4IES |= nrfIRQpin;   // Trigger on falling-edge
		P4IFG &= ~nrfIRQpin;  // Clear any outstanding IRQ
		P4IE |= nrfIRQpin;    // Enable IRQ interrupt
	#elif nrfIRQport == 5
		P5DIR &= ~nrfIRQpin;  // IRQ line is input
		P5OUT |= nrfIRQpin;   // Pull-up resistor enabled
		P5REN |= nrfIRQpin;
		P5IES |= nrfIRQpin;   // Trigger on falling-edge
		P5IFG &= ~nrfIRQpin;  // Clear any outstanding IRQ
		P5IE |= nrfIRQpin;    // Enable IRQ interrupt
	#elif nrfIRQport == 6
		P6DIR &= ~nrfIRQpin;  // IRQ line is input
		P6OUT |= nrfIRQpin;   // Pull-up resistor enabled
		P6REN |= nrfIRQpin;
		P6IES |= nrfIRQpin;   // Trigger on falling-edge
		P6IFG &= ~nrfIRQpin;  // Clear any outstanding IRQ
		P6IE |= nrfIRQpin;    // Enable IRQ interrupt
	#elif nrfIRQport == 7
		P7DIR &= ~nrfIRQpin;  // IRQ line is input
		P7OUT |= nrfIRQpin;   // Pull-up resistor enabled
		P7REN |= nrfIRQpin;
		P7IES |= nrfIRQpin;   // Trigger on falling-edge
		P7IFG &= ~nrfIRQpin;  // Clear any outstanding IRQ
		P7IE |= nrfIRQpin;    // Enable IRQ interrupt
	#elif nrfIRQport == 8
		P8DIR &= ~nrfIRQpin;  // IRQ line is input
		P8OUT |= nrfIRQpin;   // Pull-up resistor enabled
		P8REN |= nrfIRQpin;
		P8IES |= nrfIRQpin;   // Trigger on falling-edge
		P8IFG &= ~nrfIRQpin;  // Clear any outstanding IRQ
		P8IE |= nrfIRQpin;    // Enable IRQ interrupt
	#elif nrfIRQport == 9
		P9DIR &= ~nrfIRQpin;  // IRQ line is input
		P9OUT |= nrfIRQpin;   // Pull-up resistor enabled
		P9REN |= nrfIRQpin;
		P9IES |= nrfIRQpin;   // Trigger on falling-edge
		P9IFG &= ~nrfIRQpin;  // Clear any outstanding IRQ
		P9IE |= nrfIRQpin;    // Enable IRQ interrupt
	#elif nrfIRQport == 10
		P10DIR &= ~nrfIRQpin;  // IRQ line is input
		P10OUT |= nrfIRQpin;   // Pull-up resistor enabled
		P10REN |= nrfIRQpin;
		P10IES |= nrfIRQpin;   // Trigger on falling-edge
		P10IFG &= ~nrfIRQpin;  // Clear any outstanding IRQ
		P10IE |= nrfIRQpin;    // Enable IRQ interrupt
	#endif

	// Setup CSN/CE ports
	#if nrfCSNport == 1
		P1DIR |= nrfCSNpin;
	#elif nrfCSNport == 2
		P2DIR |= nrfCSNpin;
	#elif nrfCSNport == 3
		P3DIR |= nrfCSNpin;
	#elif nrfCSNport == 4
		P4DIR |= nrfCSNpin;
	#elif nrfCSNport == 5
		P5DIR |= nrfCSNpin;
	#elif nrfCSNport == 6
		P6DIR |= nrfCSNpin;
	#elif nrfCSNport == 7
		P7DIR |= nrfCSNpin;
	#elif nrfCSNport == 8
		P8DIR |= nrfCSNpin;
	#elif nrfCSNport == 9
		P9DIR |= nrfCSNpin;
	#elif nrfCSNport == 10
		P10DIR |= nrfCSNpin;
	#elif nrfCSNport == J
		PJDIR |= nrfCSNpin;
	#endif
	CSN_DIS;

	#if nrfCEport == 1
		P1DIR |= nrfCEpin;
	#elif nrfCEport == 2
		P2DIR |= nrfCEpin;
	#elif nrfCEport == 3
		P3DIR |= nrfCEpin;
	#elif nrfCEport == 4
		P4DIR |= nrfCEpin;
	#elif nrfCEport == 5
		P5DIR |= nrfCEpin;
	#elif nrfCEport == 6
		P6DIR |= nrfCEpin;
	#elif nrfCEport == 7
		P7DIR |= nrfCEpin;
	#elif nrfCEport == 8
		P8DIR |= nrfCEpin;
	#elif nrfCEport == 9
		P9DIR |= nrfCEpin;
	#elif nrfCEport == 10
		P10DIR |= nrfCEpin;
	#elif nrfCEport == J
		PJDIR |= nrfCEpin;
	#endif
	CE_DIS;

	/* Straw-man spi_transfer with no Chip Select lines enabled; this is to workaround errata bug USI5
	 * on the MSP430G2452 and related (see http://www.ti.com/lit/er/slaz072/slaz072.pdf)
	 * Shouldn't hurt anything since we expect no CS lines enabled by the user during this function's execution.
	 */
	spi_transfer(RF24_NOP);

	// Wait 100ms for RF transceiver to initialize.
	uint8_t c = 20;
	for (; c; c--) {
		__delay_cycles(DELAY_CYCLES_5MS);
	}

	// Configure RF transceiver with current value of rf_* configuration variables
	msprf24_irq_clear(RF24_IRQ_MASK);  // Forget any outstanding IRQs
	msprf24_close_pipe_all();          /* Start off with no pipes enabled, let the user open as needed.  This also
					    * clears the DYNPD register.
					    */
	msprf24_set_retransmit_delay(2000);  // A default I chose
	msprf24_set_retransmit_count(15);    // A default I chose
	msprf24_set_speed_power();
	msprf24_set_channel();
	msprf24_set_address_width();
	rf_feature = 0x00;  // Initialize this so we're starting from a clean slate
	msprf24_enable_feature(RF24_EN_DPL);      // Dynamic payload size capability (set with msprf24_set_pipe_packetsize(x, 0))
	msprf24_enable_feature(RF24_EN_DYN_ACK);  // Ability to use w_tx_payload_noack()

	msprf24_powerdown();
	flush_tx();
	flush_rx();
}

void msprf24_enable_feature(uint8_t feature)
{
	if ( (rf_feature & feature) != feature ) {
		rf_feature |= feature;
		rf_feature &= 0x07;  // Only bits 0, 1, 2 allowed to be set
		w_reg(RF24_FEATURE, rf_feature);
	}
}

void msprf24_disable_feature(uint8_t feature)
{
	if ( (rf_feature & feature) == feature ) {
		rf_feature &= ~feature;
		w_reg(RF24_FEATURE, rf_feature);
	}
}

void msprf24_close_pipe(uint8_t pipeid)
{
	uint8_t rxen, enaa;

	if (pipeid > 5)
		return;

	rxen = r_reg(RF24_EN_RXADDR);
	enaa = r_reg(RF24_EN_AA);

	rxen &= ~(1 << pipeid);
	enaa &= ~(1 << pipeid);

	w_reg(RF24_EN_RXADDR, rxen);
	w_reg(RF24_EN_AA, enaa);
}

void msprf24_close_pipe_all()
{
	w_reg(RF24_EN_RXADDR, 0x00);
	w_reg(RF24_EN_AA, 0x00);
	w_reg(RF24_DYNPD, 0x00);
}

void msprf24_open_pipe(uint8_t pipeid, uint8_t autoack)
{
	uint8_t rxen, enaa;

	if (pipeid > 5)
		return;

	rxen = r_reg(RF24_EN_RXADDR);
	enaa = r_reg(RF24_EN_AA);

	if (autoack)
		enaa |= (1 << pipeid);
	else
		enaa &= ~(1 << pipeid);
	rxen |= (1 << pipeid);
	w_reg(RF24_EN_RXADDR, rxen);
	w_reg(RF24_EN_AA, enaa);
}

uint8_t msprf24_pipe_isopen(uint8_t pipeid)
{
	uint8_t rxen;

	if (pipeid > 5)
		return 0;

	rxen = r_reg(RF24_EN_RXADDR);

	return ( (1<<pipeid) == (rxen & (1<<pipeid)) );
}

void msprf24_set_pipe_packetsize(uint8_t pipe, uint8_t size)
{
	uint8_t dynpdcfg;

	if (pipe > 5)
		return;

	dynpdcfg = r_reg(RF24_DYNPD);
	if (size < 1) {
		if ( !(rf_feature & RF24_EN_DPL) )  // Cannot set dynamic payload if EN_DPL is disabled.
			return;
		if (!( (1<<pipe) & dynpdcfg )) {
			// DYNPD not enabled for this pipe, enable it
			dynpdcfg |= 1 << pipe;
		}
	} else {
		dynpdcfg &= ~(1 << pipe);  // Ensure DynPD is disabled for this pipe
		if (size > 32)
			size = 32;
		w_reg(RF24_RX_PW_P0 + pipe, size);
	}
	w_reg(RF24_DYNPD, dynpdcfg);
}

void msprf24_set_retransmit_delay(uint16_t us)
{
	uint8_t c;

	// using 'c' to evaluate current RF speed
	c = rf_speed_power & RF24_SPEED_MASK;
	if (us > 4000)
		us = 4000;
	if (us < 1500 && c == RF24_SPEED_250KBPS)
		us = 1500;
	if (us < 500)
		us = 500;

	// using 'c' to save current value of ARC (auto-retrans-count) since we're not changing that here
	c = r_reg(RF24_SETUP_RETR) & 0x0F;
	us = (us-250) / 250;
	us <<= 4;
	w_reg(RF24_SETUP_RETR, c | (us & 0xF0));
}

void msprf24_set_retransmit_count(uint8_t count)
{
	uint8_t c;

	c = r_reg(RF24_SETUP_RETR) & 0xF0;
	w_reg(RF24_SETUP_RETR, c | (count & 0x0F));
}

uint8_t msprf24_get_last_retransmits()
{
	return r_reg(RF24_OBSERVE_TX) & 0x0F;
}

uint8_t msprf24_get_lostpackets()
{
	return (r_reg(RF24_OBSERVE_TX) >> 4) & 0x0F;
}

inline uint8_t _msprf24_crc_mask()
{
	return (rf_crc & 0x0C);
}

inline uint8_t _msprf24_irq_mask()
{
	return ~(RF24_MASK_RX_DR | RF24_MASK_TX_DS | RF24_MASK_MAX_RT);
}

uint8_t msprf24_is_alive()
{
	uint8_t aw;

	aw = r_reg(RF24_SETUP_AW);
	return((aw & 0xFC) == 0x00 && (aw & 0x03) != 0x00);
}

uint8_t msprf24_set_config(uint8_t cfgval)
{
	uint8_t previous_config;

	previous_config = r_reg(RF24_CONFIG);
	w_reg(RF24_CONFIG, (_msprf24_crc_mask() | cfgval) & _msprf24_irq_mask());
	return previous_config;
}

void msprf24_set_speed_power()
{
	if ( (rf_speed_power & RF24_SPEED_MASK) == RF24_SPEED_MASK )  // Speed setting RF_DR_LOW=1, RF_DR_HIGH=1 is reserved, clamp it to minimum
		rf_speed_power = (rf_speed_power & ~RF24_SPEED_MASK) | RF24_SPEED_MIN;
	w_reg(RF24_RF_SETUP, (rf_speed_power & 0x2F));
}

void msprf24_set_channel()
{
	if (rf_channel > 125)
		rf_channel = 0;
	w_reg(RF24_RF_CH, (rf_channel & 0x7F));
}

void msprf24_set_address_width()
{
	if (rf_addr_width < 3 || rf_addr_width > 5)
		return;
	w_reg(RF24_SETUP_AW, ((rf_addr_width-2) & 0x03));
}

uint8_t msprf24_current_state()
{
	uint8_t config;

	if (!msprf24_is_alive())               // Can't read/detect a valid value from SETUP_AW? (typically SPI or device fault)
		return RF24_STATE_NOTPRESENT;
	config = r_reg(RF24_CONFIG);
	if ( (config & RF24_PWR_UP) == 0x00 )  // PWR_UP=0?
		return RF24_STATE_POWERDOWN;
	if ( !(nrfCEportout & nrfCEpin) )      // PWR_UP=1 && CE=0?
		return RF24_STATE_STANDBY_I;
	if ( !(config & RF24_PRIM_RX) ) {      // PWR_UP=1 && CE=1 && PRIM_RX=0?
		if ( (r_reg(RF24_FIFO_STATUS) & RF24_TX_EMPTY) )  // TX FIFO empty?
			return RF24_STATE_STANDBY_II;
		return RF24_STATE_PTX; // If TX FIFO is not empty, we are in PTX (active transmit) mode.
	}
	if ( r_reg(RF24_RF_SETUP) & 0x90 )     // Testing CONT_WAVE or PLL_LOCK?
		return RF24_STATE_TEST;
	return RF24_STATE_PRX;                 // PWR_UP=1, PRIM_RX=1, CE=1 -- Must be PRX
}

// Power down device, 0.9uA power draw
void msprf24_powerdown()
{
	CE_DIS;
	w_reg(RF24_STATUS, RF24_IRQ_MASK); // Clear all IRQs so the IRQ line isn't draining power during powerdown mode
	msprf24_set_config(0);  // PWR_UP=0
}

// Enable Standby-I, 26uA power draw
void msprf24_standby()
{
	uint8_t state = msprf24_current_state();
	if (state == RF24_STATE_NOTPRESENT || state == RF24_STATE_STANDBY_I)
		return;
	CE_DIS;
	msprf24_set_config(RF24_PWR_UP);  // PWR_UP=1, PRIM_RX=0
	if (state == RF24_STATE_POWERDOWN) {  // If we're powering up from deep powerdown...
		//CE_EN;  // This is a workaround for SI24R1 chips, though it seems to screw things up so disabled for now til I can obtain an SI24R1 for testing.
		__delay_cycles(DELAY_CYCLES_5MS);  // Then wait 5ms for the crystal oscillator to spin up.
		//CE_DIS;
	}
}

// Enable PRX mode
void msprf24_activate_rx()
{
	msprf24_standby();
	// Purge any existing RX FIFO or RX interrupts
	flush_rx();
	w_reg(RF24_STATUS, RF24_RX_DR);

	// Enable PRIM_RX
	msprf24_set_config(RF24_PWR_UP | RF24_PRIM_RX);
	CE_EN;
	// 130uS required for PLL lock to stabilize, app can go do other things and wait
	// for incoming I/O.
}

// Enable Standby-II / PTX mode
/* Standby-II is enabled if the TX FIFO is empty, otherwise the chip enters PTX
 *     mode to send the TX FIFO buffer contents until it's all done, at which point
 *     the chip falls back to Standby-II again.
 */
void msprf24_activate_tx()
{
	msprf24_standby();
	// Cancel any outstanding TX interrupt
	w_reg(RF24_STATUS, RF24_TX_DS|RF24_MAX_RT);

	// Pulse CE for 10us to activate PTX
	pulse_ce();
}

/* Evaluate state of TX, RX FIFOs
 * Compare this with RF24_QUEUE_* #define's from msprf24.h
 */
uint8_t msprf24_queue_state()
{
	return r_reg(RF24_FIFO_STATUS);
}

/* Scan current channel for activity, produce an 8-bit integer indicating % of time
 * spent with RPD=1 (valid RF activity present) for a 133ms period.
 */
uint8_t msprf24_scan()
{
	int testcount = 1023;
	uint16_t rpdcount = 0;
	uint8_t last_state;

	last_state = msprf24_current_state();
	if (last_state != RF24_STATE_PRX)
		msprf24_activate_rx();
	for (; testcount > 0; testcount--) {
		if (r_reg(RF24_RPD))
			rpdcount++;
		__delay_cycles(DELAY_CYCLES_130US);
		flush_rx();
		w_reg(RF24_STATUS, RF24_RX_DR);  /* Flush any RX FIFO contents or RX IRQs that
						  * may have generated as a result of having PRX active.
						  */
	}
	if (last_state != RF24_STATE_PRX)
		msprf24_standby();  // If we weren't in RX mode before, leave it in Standby-I.
	return( (uint8_t) (rpdcount/4) );
}

// Check if there is pending RX fifo data
uint8_t msprf24_rx_pending()
{
	CSN_EN;
	rf_status = spi_transfer(RF24_NOP);
	CSN_DIS;

	if ((rf_status & 0x0E) < 0x0E)
		return 1;
	return 0;
}

// Get IRQ flag status
uint8_t msprf24_get_irq_reason()
{
	uint8_t rf_irq_old = rf_irq;

	//rf_irq &= ~RF24_IRQ_FLAGGED;  -- Removing in lieu of having this check determined at irq_clear() time
	CSN_EN;
	rf_status = spi_transfer(RF24_NOP);
	CSN_DIS;
	rf_irq = (rf_status & RF24_IRQ_MASK) | rf_irq_old;
	return rf_irq;
}

/* Clear IRQ flags */
void msprf24_irq_clear(uint8_t irqflag)
{
	uint8_t fifostat;

	rf_irq = 0x00;  // Clear IRQs; afterward analyze RX FIFO to see if we should re-set RX IRQ flag.
	CSN_EN;
	rf_status = spi_transfer(RF24_STATUS | RF24_W_REGISTER);
	spi_transfer(irqflag);
	CSN_DIS;

	// Per datasheet procedure, check FIFO_STATUS to see if there's more RX FIFO data to process.
	if (irqflag & RF24_IRQ_RX) {
		CSN_EN;
		rf_status = spi_transfer(RF24_FIFO_STATUS | RF24_R_REGISTER);
		fifostat = spi_transfer(RF24_NOP);
		CSN_DIS;
		if ( !(fifostat & RF24_RX_EMPTY) )
			rf_irq |= RF24_IRQ_RX | RF24_IRQ_FLAGGED;  // Signal to user that there is remaining data, even if it's not "new"
	}
}










/*      -       -       Interrupt vectors       -       -       */

// RF transceiver IRQ handling
#if   nrfIRQport == 2
  #ifdef __GNUC__
  __attribute__((interrupt(PORT2_VECTOR)))
  void P2_IRQ (void) {
  #else
  #pragma vector = PORT2_VECTOR
  __interrupt void P2_IRQ (void) {
  #endif
	if(P2IFG & nrfIRQpin) {
		__bic_SR_register_on_exit(LPM4_bits);    // Wake up
		rf_irq |= RF24_IRQ_FLAGGED;
		P2IFG &= ~nrfIRQpin;   // Clear interrupt flag
	}
}

#elif nrfIRQport == 1
  #ifdef __GNUC__
  __attribute__((interrupt(PORT1_VECTOR)))
  void P1_IRQ (void) {
  #else
  #pragma vector = PORT1_VECTOR
  __interrupt void P1_IRQ (void) {
  #endif
	if(P1IFG & nrfIRQpin) {
		__bic_SR_register_on_exit(LPM4_bits);
		rf_irq |= RF24_IRQ_FLAGGED;
		P1IFG &= ~nrfIRQpin;
	}
}

#elif nrfIRQport == 3 && defined(P3IV_)
  #ifdef __GNUC__
  __attribute__((interrupt(PORT3_VECTOR)))
  void P3_IRQ (void) {
  #else
  #pragma vector = PORT3_VECTOR
  __interrupt void P3_IRQ (void) {
  #endif
	if (P3IFG & nrfIRQpin) {
		__bic_SR_register_on_exit(LPM4_bits);
		rf_irq |= RF24_IRQ_FLAGGED;
		P3IFG &= ~nrfIRQpin;
	}
}

#elif nrfIRQport == 4 && defined(P4IV_)
  #ifdef __GNUC__
  __attribute__((interrupt(PORT4_VECTOR)))
  void P4_IRQ (void) {
  #else
  #pragma vector = PORT4_VECTOR
  __interrupt void P4_IRQ (void) {
  #endif
	if (P4IFG & nrfIRQpin) {
		__bic_SR_register_on_exit(LPM4_bits);
		rf_irq |= RF24_IRQ_FLAGGED;
		P4IFG &= ~nrfIRQpin;
	}
}

#elif nrfIRQport == 5 && defined(P5IV_)
  #ifdef __GNUC__
  __attribute__((interrupt(PORT5_VECTOR)))
  void P5_IRQ (void) {
  #else
  #pragma vector = PORT5_VECTOR
  __interrupt void P5_IRQ (void) {
  #endif
	if (P5IFG & nrfIRQpin) {
		__bic_SR_register_on_exit(LPM4_bits);
		rf_irq |= RF24_IRQ_FLAGGED;
		P5IFG &= ~nrfIRQpin;
	}
}

#elif nrfIRQport == 6 && defined(P6IV_)
  #ifdef __GNUC__
  __attribute__((interrupt(PORT6_VECTOR)))
  void P6_IRQ (void) {
  #else
  #pragma vector = PORT6_VECTOR
  __interrupt void P6_IRQ (void) {
  #endif
	if (P6IFG & nrfIRQpin) {
		__bic_SR_register_on_exit(LPM4_bits);
		rf_irq |= RF24_IRQ_FLAGGED;
		P6IFG &= ~nrfIRQpin;
	}
}

#elif nrfIRQport == 7 && defined(P7IV_)
  #ifdef __GNUC__
  __attribute__((interrupt(PORT7_VECTOR)))
  void P7_IRQ (void) {
  #else
  #pragma vector = PORT7_VECTOR
  __interrupt void P7_IRQ (void) {
  #endif
	if (P7IFG & nrfIRQpin) {
		__bic_SR_register_on_exit(LPM4_bits);
		rf_irq |= RF24_IRQ_FLAGGED;
		P7IFG &= ~nrfIRQpin;
	}
}

#elif nrfIRQport == 8 && defined(P8IV_)
  #ifdef __GNUC__
  __attribute__((interrupt(PORT8_VECTOR)))
  void P8_IRQ (void) {
  #else
  #pragma vector = PORT8_VECTOR
  __interrupt void P8_IRQ (void) {
  #endif
	if (P8IFG & nrfIRQpin) {
		__bic_SR_register_on_exit(LPM4_bits);
		rf_irq |= RF24_IRQ_FLAGGED;
		P8IFG &= ~nrfIRQpin;
	}
}

#elif nrfIRQport == 9 && defined(P9IV_)
  #ifdef __GNUC__
  __attribute__((interrupt(PORT9_VECTOR)))
  void P9_IRQ (void) {
  #else
  #pragma vector = PORT9_VECTOR
  __interrupt void P9_IRQ (void) {
  #endif
	if (P9IFG & nrfIRQpin) {
		__bic_SR_register_on_exit(LPM4_bits);
		rf_irq |= RF24_IRQ_FLAGGED;
		P9IFG &= ~nrfIRQpin;
	}
}

#elif nrfIRQport == 10 && defined(P10IV_)
  #ifdef __GNUC__
  __attribute__((interrupt(PORT10_VECTOR)))
  void P10_IRQ (void) {
  #else
  #pragma vector = PORT10_VECTOR
  __interrupt void P10_IRQ (void) {
  #endif
	if (P10IFG & nrfIRQpin) {
		__bic_SR_register_on_exit(LPM4_bits);
		rf_irq |= RF24_IRQ_FLAGGED;
		P10IFG &= ~nrfIRQpin;
	}
}
#endif

