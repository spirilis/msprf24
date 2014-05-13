/* nrf_userconfig.h
 * User configuration of nRF24L01+ connectivity parameters, e.g.
 * IRQ, CSN, CE pin assignments, Serial SPI driver type
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

#ifndef _NRF_USERCONFIG_H
#define _NRF_USERCONFIG_H

/* CPU clock cycles for the specified amounts of time--accurate minimum delays
 * required for reliable operation of the nRF24L01+'s state machine.
 */
/* Settings for 1MHz MCLK.
#define DELAY_CYCLES_5MS       5000
#define DELAY_CYCLES_130US     130
#define DELAY_CYCLES_15US      15
 */

/* Settings for 8MHz MCLK.
#define DELAY_CYCLES_5MS       40000
#define DELAY_CYCLES_130US     1040
#define DELAY_CYCLES_15US      120
 */

/* Settings for 16MHz MCLK */
#define DELAY_CYCLES_5MS       80000
#define DELAY_CYCLES_130US     2080
#define DELAY_CYCLES_15US      240

/* Settings for 24MHz MCLK.
#define DELAY_CYCLES_5MS       120000
#define DELAY_CYCLES_130US     3120
#define DELAY_CYCLES_15US      360
 */

/* SPI port--Select which USCI port we're using.
 * Applies only to USCI devices.  USI users can keep these
 * commented out.
 */
//#define SPI_DRIVER_USCI_A 1
#define SPI_DRIVER_USCI_B 1


/* Operational pins -- IRQ, CE, CSN (SPI chip-select)
 */

/* IRQ */
#define nrfIRQport 2
#define nrfIRQpin BIT2

/* CSN SPI chip-select */
#define nrfCSNport 2
#define nrfCSNportout P2OUT
#define nrfCSNpin BIT1

/* CE Chip-Enable (used to put RF transceiver on-air for RX or TX) */
#define nrfCEport 2
#define nrfCEportout P2OUT
#define nrfCEpin BIT0

#endif
