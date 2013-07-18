/* msprf24.h
 * MSP430 library for interfacing with the nRF24L01+ RF transceiver by
 * Nordic Semiconductor.
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

#ifndef _MSPRF24_H_
#define _MSPRF24_H_

#include "nRF24L01.h"

/* Configuration variables used to tune RF settings during initialization and for
 * runtime reconfiguration.  You should define all 4 of these before running msprf24_init();
 */
extern unsigned char rf_crc;
extern unsigned char rf_addr_width;
extern unsigned char rf_speed_power;
extern unsigned char rf_channel;

/* Status variable updated every time SPI I/O is performed */
extern unsigned char rf_status;
/* Test this against RF24_IRQ_FLAGGED to see if the nRF24's IRQ was raised; it also
 * holds the last recorded IRQ status from msprf24_irq_get_reason();
 */
extern volatile unsigned char rf_irq;

/* RF speed settings -- nRF24L01+ compliant, older nRF24L01 does not have 2Mbps. */
#define RF24_SPEED_250KBPS  0x20
#define RF24_SPEED_1MBPS    0x00
#define RF24_SPEED_2MBPS    0x08
#define RF24_SPEED_MAX      RF24_SPEED_2MBPS
#define RF24_SPEED_MIN      RF24_SPEED_250KBPS
#define RF24_SPEED_MASK     0x28

/* RF transmit power settings */
#define RF24_POWER_0DBM        0x06
#define RF24_POWER_MINUS6DBM   0x04
#define RF24_POWER_MINUS12DBM  0x02
#define RF24_POWER_MINUS18DBM  0x00
#define RF24_POWER_MAX         RF24_POWER_0DBM
#define RF24_POWER_MIN         RF24_POWER_MINUS18DBM
#define RF24_POWER_MASK        0x06

/* Available states for the transceiver's state machine */
#define RF24_STATE_NOTPRESENT  0x00
#define RF24_STATE_POWERDOWN   0x01
#define RF24_STATE_STANDBY_I   0x02
#define RF24_STATE_STANDBY_II  0x03
#define RF24_STATE_PTX         0x04
#define RF24_STATE_PRX         0x05
#define RF24_STATE_TEST        0x06

/* IRQ "reasons" that can be tested. */
#define RF24_IRQ_TXFAILED      0x10
#define RF24_IRQ_TX            0x20
#define RF24_IRQ_RX            0x40
#define RF24_IRQ_MASK          0x70
// Bit 7 used to signify that the app should check IRQ status, without
// wasting time in the interrupt vector trying to do so itself.
#define RF24_IRQ_FLAGGED       0x80

/* Queue FIFO states that can be tested. */
#define RF24_QUEUE_TXFULL      RF24_FIFO_FULL
#define RF24_QUEUE_TXEMPTY     RF24_TX_EMPTY
#define RF24_QUEUE_RXFULL      RF24_RX_FULL
#define RF24_QUEUE_RXEMPTY     RF24_RX_EMPTY


/* FUNCTIONS! */

// SPI driver needs to provide these
void spi_init();
char spi_transfer(char);  // SPI xfer 1 byte
int spi_transfer16(int);  // SPI xfer 2 bytes
int spi_transfer9(int);   // SPI xfer 9 bits (courtesy for driving LCD screens)

// Register & FIFO I/O
unsigned char r_reg(unsigned char addr);
void w_reg(unsigned char addr, char data);
void w_tx_addr(char *addr);             // Configure TX address to send next packet
void w_rx_addr(unsigned char pipe, char *addr);  // Configure RX address of "rf_addr_width" size into the specified pipe
void w_tx_payload(unsigned char len, char *data);
void w_tx_payload_noack(unsigned char len, char *data);  /* Only used in auto-ack mode with RF24_EN_DYN_ACK enabled;
						 * send this packet with no auto-ack.
						 */
unsigned char r_rx_peek_payload_size();  // Peek size of incoming RX payload
unsigned char r_rx_payload(unsigned char len, char *data);
void flush_tx();
void flush_rx();
void tx_reuse_lastpayload();   /* Enable retransmitting contents of TX FIFO endlessly until flush_tx() or the FIFO contents are replaced.
				* Actual retransmits don't occur until CE pin is strobed using pulse_ce();
				*/
void pulse_ce();  // Pulse CE pin to activate retransmission of TX FIFO contents after tx_reuse_lastpayload();
void w_ack_payload(unsigned char pipe, unsigned char len, char *data);  // Used when RF24_EN_ACK_PAY is enabled to manually ACK a received packet



// Initialization and configuration
void msprf24_init();  /* Set the various configuration variables before running this.
		       * It will populate the channel/speed/power/default features/etc. values
		       */
void msprf24_close_pipe(unsigned char pipeid);       // Disable specified RX pipe
void msprf24_close_pipe_all();                       // Disable all RX pipes (used during initialization)
void msprf24_open_pipe(unsigned char pipeid, unsigned char autoack); // Enable specified RX pipe, optionally turn auto-ack (Enhanced ShockBurst) on
unsigned char msprf24_pipe_isopen(unsigned char pipeid); // Check if specified RX pipe is active
void msprf24_set_pipe_packetsize(unsigned char pipe, unsigned char size);  // Set static length of pipe's RX payloads (1-32), size=0 enables DynPD.
void msprf24_set_retransmit_delay(int us);           // 500-4000uS range, clamped by RF speed
void msprf24_set_retransmit_count(unsigned char count);       // 0-15 retransmits before MAX_RT (RF24_IRQ_TXFAILED) IRQ raised
unsigned char msprf24_get_last_retransmits();        // # times a packet was retransmitted during last TX attempt
unsigned char msprf24_get_lostpackets();      /* # of packets lost since last time the Channel was set.
	                                       * Running msprf24_set_channel() without modifying rf_channel will reset this counter.
                                               */
unsigned char msprf24_is_alive();                    // Hello World, test if chip is present and/or SPI is working.
unsigned char msprf24_set_config(unsigned char cfgval);
void msprf24_set_speed_power();                      // Commit RF speed & TX power from rf_speed_power variable.
void msprf24_set_channel();                          // Commit RF channel setting from rf_channel variable.
void msprf24_set_address_width();                    // Commit Enhanced ShockBurst Address Width from rf_addr_width variable.
void msprf24_enable_feature(unsigned char feature);    /* Enable specified feature (RF24_EN_* from nRF24L01.h, except RF24_EN_CRC) */
void msprf24_disable_feature(unsigned char feature);   /* Disable specified feature                                                */

// Change chip state and activate I/O
unsigned char msprf24_current_state();    // Get current state of the nRF24L01+ chip, test with RF24_STATE_* #define's
void msprf24_powerdown();                 // Enter Power-Down mode (0.9uA power draw)
void msprf24_standby();                   // Enter Standby-I mode (26uA power draw)
void msprf24_activate_rx();               // Enable PRX mode (~12-14mA power draw)
void msprf24_activate_tx();               // Enable Standby-II or PTX mode; TX FIFO contents will be sent over the air (~320uA STBY2, 7-11mA PTX)
unsigned char msprf24_queue_state();      // Read FIFO_STATUS register; user should compare return value with RF24_QUEUE_* #define's
unsigned char msprf24_scan();             // Scan current channel for RPD (looks for any signals > -64dBm)

// IRQ handling
unsigned char msprf24_rx_pending();		   /* Query STATUS register to determine if RX FIFO data is available for reading. */
unsigned char msprf24_get_irq_reason();            /* Query STATUS register for the IRQ flags, test with RF24_IRQ_* #define's
						    * Result is stored in rf_irq (note- RF24_IRQ_FLAGGED is not automatically cleared by this
						    * function, that's the user's responsibility.)
						    */
void msprf24_irq_clear(unsigned char irqflag);     /* Clear specified Interrupt Flags (RF24_IRQ_* #define's) from the transceiver.
		 				    * Required to allow further transmissions to continue.
						    */

#endif
