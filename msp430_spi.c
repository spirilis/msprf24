/* msp430_spi.c
 * Library for performing SPI I/O on a wide range of MSP430 chips.
 *
 * Serial interfaces supported:
 * 1. USI - developed on MSP430G2231
 * 2. USCI_A - developed on MSP430G2553
 * 3. USCI_B - developed on MSP430G2553
 * 4. USCI_A F5xxx - developed on MSP430F5172, added F5529
 * 5. USCI_B F5xxx - developed on MSP430F5172, added F5529
 *
 * Copyright (c) 2013, Eric Brundick <spirilis@linux.com>
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
#include "msp430_spi.h"
#include "nrf_userconfig.h"


#ifdef __MSP430_HAS_USI__
void spi_init()
{
	/* USI SPI setup */
	USICTL0 |= USISWRST;
	USICTL1 = USICKPH;                // USICKPH=1 means sampling is done on the leading edge of the clock
	USICKCTL = USISSEL_2 | USIDIV_0;  // Clock source = SMCLK/1
	USICTL0 = USIPE7 | USIPE6 | USIPE5 | USIMST | USIOE;
	USISR = 0x0000;
}

uint8_t spi_transfer(uint8_t inb)
{
	USISRL = inb;
	USICNT = 8;            // Start SPI transfer
	while ( !(USICTL1 & USIIFG) )
		;
	return USISRL;
}

/* What wonderful toys TI gives us!  A 16-bit SPI function. */
uint16_t spi_transfer16(uint16_t inw)
{
	USISR = inw;
	USICNT = 16 | USI16B;  // Start 16-bit SPI transfer
	while ( !(USICTL1 & USIIFG) )
		;
	return USISR;
}

/* Not used by msprf24, but added for courtesy (LCD display support).  9-bit SPI. */
uint16_t spi_transfer9(uint16_t inw)
{
	USISR = inw;
	USICNT = 9 | USI16B;  // Start 9-bit SPI transfer
	while ( !(USICTL1 & USIIFG) )
		;
	return USISR;
}
#endif

/* USCI 16-bit transfer functions rely on the Little-Endian architecture and use
 * an internal uint8_t * pointer to manipulate the individual 8-bit segments of a
 * 16-bit integer.
 */

// USCI for F2xxx and G2xx3 devices
#if defined(__MSP430_HAS_USCI__) && defined(SPI_DRIVER_USCI_A) && !defined(__MSP430_HAS_TB3__)
void spi_init()
{
	/* Configure ports on MSP430 device for USCI_A */
	P1SEL |= BIT1 | BIT2 | BIT4;
	P1SEL2 |= BIT1 | BIT2 | BIT4;

	/* USCI-A specific SPI setup */
	UCA0CTL1 |= UCSWRST;
	UCA0MCTL = 0x00;  // Clearing modulation control per TI user's guide recommendation
	UCA0CTL0 = UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC;  // SPI mode 0, master
	UCA0BR0 = 0x01;  // SPI clocked at same speed as SMCLK
	UCA0BR1 = 0x00;
	UCA0CTL1 = UCSSEL_2;  // Clock = SMCLK, clear UCSWRST and enables USCI_A module.
}

uint8_t spi_transfer(uint8_t inb)
{
	UCA0TXBUF = inb;
	while ( !(IFG2 & UCA0RXIFG) )  // Wait for RXIFG indicating remote byte received via SOMI
		;
	return UCA0RXBUF;
}

uint16_t spi_transfer16(uint16_t inw)
{
	uint16_t retw;
	uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;

	UCA0TXBUF = inw8[1];
	while ( !(IFG2 & UCA0RXIFG) )
		;
	retw8[1] = UCA0RXBUF;
	UCA0TXBUF = inw8[0];
	while ( !(IFG2 & UCA0RXIFG) )
		;
	retw8[0] = UCA0RXBUF;
	return retw;
}

uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p1dir_save, p1out_save, p1ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p1ren_save = P1REN; p1out_save = P1OUT; p1dir_save = P1DIR;
	P1REN &= ~(BIT1 | BIT2 | BIT4);
	P1OUT &= ~(BIT1 | BIT2 | BIT4);
	P1DIR = (P1DIR & ~(BIT1 | BIT2 | BIT4)) | BIT2 | BIT4;
	P1SEL &= ~(BIT1 | BIT2 | BIT4);
	P1SEL2 &= ~(BIT1 | BIT2 | BIT4);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P1OUT |= BIT2;
	P1OUT |= BIT4;
	if (P1IN & BIT1)
		retw |= 0x0100;
	P1OUT &= ~BIT4;

	// Restore port states and continue with 8-bit SPI
	P1SEL |= BIT1 | BIT2 | BIT4;
	P1SEL2 |= BIT1 | BIT2 | BIT4;
	P1DIR = p1dir_save;
	P1OUT = p1out_save;
	P1REN = p1ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif

#if defined(__MSP430_HAS_USCI__) && defined(SPI_DRIVER_USCI_B) && !defined(__MSP430_HAS_TB3__)
void spi_init()
{
	/* Configure ports on MSP430 device for USCI_B */
	P1SEL |= BIT5 | BIT6 | BIT7;
	P1SEL2 |= BIT5 | BIT6 | BIT7;

	/* USCI-B specific SPI setup */
	UCB0CTL1 |= UCSWRST;
	UCB0CTL0 = UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC;  // SPI mode 0, master
	UCB0BR0 = 0x01;  // SPI clocked at same speed as SMCLK
	UCB0BR1 = 0x00;
	UCB0CTL1 = UCSSEL_2;  // Clock = SMCLK, clear UCSWRST and enables USCI_B module.
}

uint8_t spi_transfer(uint8_t inb)
{
	UCB0TXBUF = inb;
	while ( !(IFG2 & UCB0RXIFG) )  // Wait for RXIFG indicating remote byte received via SOMI
		;
	return UCB0RXBUF;
}

uint16_t spi_transfer16(uint16_t inw)
{
	uint16_t retw;
	uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;

	UCB0TXBUF = inw8[1];
	while ( !(IFG2 & UCB0RXIFG) )
		;
	retw8[1] = UCB0RXBUF;
	UCB0TXBUF = inw8[0];
	while ( !(IFG2 & UCB0RXIFG) )
		;
	retw8[0] = UCB0RXBUF;
	return retw;
}

uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p1dir_save, p1out_save, p1ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p1ren_save = P1REN; p1out_save = P1OUT; p1dir_save = P1DIR;
	P1REN &= ~(BIT5 | BIT6 | BIT7);
	P1OUT &= ~(BIT5 | BIT6 | BIT7);
	P1DIR = (P1DIR & ~(BIT5 | BIT6 | BIT7)) | BIT5 | BIT7;
	P1SEL &= ~(BIT5 | BIT6 | BIT7);
	P1SEL2 &= ~(BIT5 | BIT6 | BIT7);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P1OUT |= BIT7;
	P1OUT |= BIT5;
	if (P1IN & BIT6)
		retw |= 0x0100;
	P1OUT &= ~BIT5;

	// Restore port states and continue with 8-bit SPI
	P1SEL |= BIT5 | BIT6 | BIT7;
	P1SEL2 |= BIT5 | BIT6 | BIT7;
	P1DIR = p1dir_save;
	P1OUT = p1out_save;
	P1REN = p1ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif

// USCI for G2xx4/G2xx5 devices
#if defined(__MSP430_HAS_USCI__) && defined(SPI_DRIVER_USCI_A) && defined(__MSP430_HAS_TB3__)
void spi_init()
{
	/* Configure ports on MSP430 device for USCI_A */
	P3SEL |= BIT0 | BIT4 | BIT5;
	P3SEL2 &= ~(BIT0 | BIT4 | BIT5);

	/* USCI-A specific SPI setup */
	UCA0CTL1 |= UCSWRST;
	UCA0MCTL = 0x00;  // Clearing modulation control per TI user's guide recommendation
	UCA0CTL0 = UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC;  // SPI mode 0, master
	UCA0BR0 = 0x01;  // SPI clocked at same speed as SMCLK
	UCA0BR1 = 0x00;
	UCA0CTL1 = UCSSEL_2;  // Clock = SMCLK, clear UCSWRST and enables USCI_A module.
}

uint8_t spi_transfer(uint8_t inb)
{
	UCA0TXBUF = inb;
	while ( !(IFG2 & UCA0RXIFG) )  // Wait for RXIFG indicating remote byte received via SOMI
		;
	return UCA0RXBUF;
}

uint16_t spi_transfer16(uint16_t inw)
{
	uint16_t retw;
	uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;

	UCA0TXBUF = inw8[1];
	while ( !(IFG2 & UCA0RXIFG) )
		;
	retw8[1] = UCA0RXBUF;
	UCA0TXBUF = inw8[0];
	while ( !(IFG2 & UCA0RXIFG) )
		;
	retw8[0] = UCA0RXBUF;
	return retw;
}

uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p3dir_save, p3out_save, p3ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p3ren_save = P3REN; p3out_save = P3OUT; p3dir_save = P3DIR;
	P3REN &= ~(BIT0 | BIT4 | BIT5);
	P3OUT &= ~(BIT0 | BIT4 | BIT5);
	P3DIR = (P3DIR & ~(BIT0 | BIT4 | BIT5)) | BIT0 | BIT4;
	P3SEL &= ~(BIT0 | BIT4 | BIT5);
	P3SEL2 &= ~(BIT0 | BIT4 | BIT5);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P3OUT |= BIT4;
	P3OUT |= BIT0;
	if (P3IN & BIT5)
		retw |= 0x0100;
	P3OUT &= ~BIT0;

	// Restore port states and continue with 8-bit SPI
	P3SEL |= BIT0 | BIT4 | BIT5;
	P3DIR = p3dir_save;
	P3OUT = p3out_save;
	P3REN = p3ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif

#if defined(__MSP430_HAS_USCI__) && defined(SPI_DRIVER_USCI_B) && defined(__MSP430_HAS_TB3__)
void spi_init()
{
	/* Configure ports on MSP430 device for USCI_B */
	P3SEL |= BIT1 | BIT2 | BIT3;
	P3SEL2 &= ~(BIT1 | BIT2 | BIT3);

	/* USCI-B specific SPI setup */
	UCB0CTL1 |= UCSWRST;
	UCB0CTL0 = UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC;  // SPI mode 0, master
	UCB0BR0 = 0x01;  // SPI clocked at same speed as SMCLK
	UCB0BR1 = 0x00;
	UCB0CTL1 = UCSSEL_2;  // Clock = SMCLK, clear UCSWRST and enables USCI_B module.
}

uint8_t spi_transfer(uint8_t inb)
{
	UCB0TXBUF = inb;
	while ( !(IFG2 & UCB0RXIFG) )  // Wait for RXIFG indicating remote byte received via SOMI
		;
	return UCB0RXBUF;
}

uint16_t spi_transfer16(uint16_t inw)
{
	uint16_t retw;
	uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;

	UCB0TXBUF = inw8[1];
	while ( !(IFG2 & UCB0RXIFG) )
		;
	retw8[1] = UCB0RXBUF;
	UCB0TXBUF = inw8[0];
	while ( !(IFG2 & UCB0RXIFG) )
		;
	retw8[0] = UCB0RXBUF;
	return retw;
}

uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p3dir_save, p3out_save, p3ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p3ren_save = P3REN; p3out_save = P3OUT; p3dir_save = P3DIR;
	P3REN &= ~(BIT1 | BIT2 | BIT3);
	P3OUT &= ~(BIT1 | BIT2 | BIT3);
	P3DIR = (P3DIR & ~(BIT1 | BIT2 | BIT3)) | BIT1 | BIT3;
	P3SEL &= ~(BIT1 | BIT2 | BIT3);
	P3SEL2 &= ~(BIT1 | BIT2 | BIT3);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P3OUT |= BIT1;
	P3OUT |= BIT3;
	if (P3IN & BIT2)
		retw |= 0x0100;
	P3OUT &= ~BIT3;

	// Restore port states and continue with 8-bit SPI
	P3SEL |= BIT1 | BIT2 | BIT3;
	P3DIR = p3dir_save;
	P3OUT = p3out_save;
	P3REN = p3ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif

// USCI for F5xxx/6xxx devices--F5172 specific P1SEL settings
#if defined(__MSP430_HAS_USCI_A0__) && defined(SPI_DRIVER_USCI_A)
void spi_init()
{
	/* Configure ports on MSP430 device for USCI_A */
	#ifdef __MSP430F5172
	P1SEL |= BIT0 | BIT1 | BIT2;
	#endif
	#ifdef __MSP430F5529
	P3SEL |= BIT3 | BIT4;
	P2SEL |= BIT7;
	#endif

	/* USCI-A specific SPI setup */
	UCA0CTL1 |= UCSWRST;
	UCA0MCTL = 0x00;  // Clearing modulation control per TI user's guide recommendation
	UCA0CTL0 = UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC;  // SPI mode 0, master
	UCA0BR0 = 0x01;  // SPI clocked at same speed as SMCLK
	UCA0BR1 = 0x00;
	UCA0CTL1 = UCSSEL_2;  // Clock = SMCLK, clear UCSWRST and enables USCI_A module.
}

uint8_t spi_transfer(uint8_t inb)
{
	UCA0TXBUF = inb;
	while ( !(UCA0IFG & UCRXIFG) )  // Wait for RXIFG indicating remote byte received via SOMI
		;
	return UCA0RXBUF;
}

uint16_t spi_transfer16(uint16_t inw)
{
	uint16_t retw;
	uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;

	UCA0TXBUF = inw8[1];
	while ( !(UCA0IFG & UCRXIFG) )
		;
	retw8[1] = UCA0RXBUF;
	UCA0TXBUF = inw8[0];
	while ( !(UCA0IFG & UCRXIFG) )
		;
	retw8[0] = UCA0RXBUF;
	return retw;
}

#ifdef __MS430F5172
uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p1dir_save, p1out_save, p1ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p1ren_save = P1REN; p1out_save = P1OUT; p1dir_save = P1DIR;
	P1REN &= ~(BIT0 | BIT1 | BIT2);
	P1OUT &= ~(BIT0 | BIT1 | BIT2);
	P1DIR = (P1DIR & ~(BIT0 | BIT1 | BIT2)) | BIT0 | BIT1;
	P1SEL &= ~(BIT0 | BIT1 | BIT2);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P1OUT |= BIT1;
	P1OUT |= BIT0;
	if (P1IN & BIT2)
		retw |= 0x0100;
	P1OUT &= ~BIT0;

	// Restore port states and continue with 8-bit SPI
	P1SEL |= BIT0 | BIT1 | BIT2;
	P1DIR = p1dir_save;
	P1OUT = p1out_save;
	P1REN = p1ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif

#ifdef __MS430F5529
uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p3dir_save, p3out_save, p3ren_save;
	uint8_t p2dir_save, p2out_save, p2ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p3ren_save = P3REN; p3out_save = P3OUT; p3dir_save = P3DIR;
	p2ren_save = P2REN; p2out_save = P2OUT; p2dir_save = P2DIR;
	P3REN &= ~(BIT3 | BIT4); P2REN &= ~BIT7;
	P3OUT &= ~(BIT3 | BIT4); P2OUT &= ~BIT7;
	P3DIR = (P3DIR | ~(BIT3 | BIT4)) | BIT3; P2DIR |= BIT7;
	P3SEL &= ~(BIT3 | BIT4); P2SEL &= ~BIT7;

	// Perform single-bit transfer
	if (inw & 0x0100)
		P3OUT |= BIT3;
	P2OUT |= BIT7;
	if (P3IN & BIT4)
		retw |= 0x0100;
	P2OUT &= ~BIT7;

	// Restore port states and continue with 8-bit SPI
	P3SEL |= BIT3 | BIT4; P2SEL |= BIT7;
	P3DIR = p3dir_save; P2DIR = p2dir_save;
	P3OUT = p3out_save; P2OUT = p2out_save;
	P3REN = p3ren_save; P2REN = p2ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif

#endif

#if defined(__MSP430_HAS_USCI_B0__) && defined(SPI_DRIVER_USCI_B)
void spi_init()
{
	/* Configure ports on MSP430 device for USCI_B */
	#ifdef __MSP430F5172
	P1SEL |= BIT3 | BIT4 | BIT5;
	#endif
	#ifdef __MSP430F5529
	P3SEL |= BIT0 | BIT1 | BIT2;
	#endif

	/* USCI-B specific SPI setup */
	UCB0CTL1 |= UCSWRST;
	UCB0CTL0 = UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC;  // SPI mode 0, master
	UCB0BR0 = 0x01;  // SPI clocked at same speed as SMCLK
	UCB0BR1 = 0x00;
	UCB0CTL1 = UCSSEL_2;  // Clock = SMCLK, clear UCSWRST and enables USCI_B module.
}

uint8_t spi_transfer(uint8_t inb)
{
	UCB0TXBUF = inb;
	while ( !(UCB0IFG & UCRXIFG) )  // Wait for RXIFG indicating remote byte received via SOMI
		;
	return UCB0RXBUF;
}

uint16_t spi_transfer16(uint16_t inw)
{
	uint16_t retw;
	uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;

	UCB0TXBUF = inw8[1];
	while ( !(UCB0IFG & UCRXIFG) )
		;
	retw8[1] = UCB0RXBUF;
	UCB0TXBUF = inw8[0];
	while ( !(UCB0IFG & UCRXIFG) )
		;
	retw8[0] = UCB0RXBUF;
	return retw;
}

#ifdef __MSP430F5172
uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p1dir_save, p1out_save, p1ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p1ren_save = P1REN; p1out_save = P1OUT; p1dir_save = P1DIR;
	P1REN &= ~(BIT3 | BIT4 | BIT5);
	P1OUT &= ~(BIT3 | BIT4 | BIT5);
	P1DIR = (P1DIR & ~(BIT3 | BIT4 | BIT5)) | BIT3 | BIT4;
	P1SEL &= ~(BIT3 | BIT4 | BIT5);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P1OUT |= BIT4;
	P1OUT |= BIT3;
	if (P1IN & BIT5)
		retw |= 0x0100;
	P1OUT &= ~BIT3;

	// Restore port states and continue with 8-bit SPI
	P1SEL |= BIT3 | BIT4 | BIT5;
	P1DIR = p1dir_save;
	P1OUT = p1out_save;
	P1REN = p1ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif

#ifdef __MSP430F5529
uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p3dir_save, p3out_save, p3ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p3ren_save = P3REN; p3out_save = P3OUT; p3dir_save = P3DIR;
	P3REN &= ~(BIT0 | BIT1 | BIT2);
	P3OUT &= ~(BIT0 | BIT1 | BIT2);
	P3DIR = (P3DIR & ~(BIT0 | BIT1 | BIT2)) | BIT0 | BIT2;
	P3SEL &= ~(BIT0 | BIT1 | BIT2);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P3OUT |= BIT0;
	P3OUT |= BIT2;
	if (P3IN & BIT1)
		retw |= 0x0100;
	P3OUT &= ~BIT2;

	// Restore port states and continue with 8-bit SPI
	P3SEL |= BIT0 | BIT1 | BIT2;
	P3DIR = p3dir_save;
	P3OUT = p3out_save;
	P3REN = p3ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif

#endif

// Wolverine and other FRAM series chips
#if defined(__MSP430_HAS_EUSCI_A0__) && (defined(SPI_DRIVER_USCI_A) || defined(SPI_DRIVER_USCI_A0))
void spi_init()
{
	/* Configure ports on MSP430 device for USCI_A0 */
	#if defined(__MSP430FR5969__)
	P1SEL0 &= ~BIT5;
	P1SEL1 |= BIT5;
	P2SEL0 &= ~(BIT0 | BIT1);
	P2SEL1 |= BIT0 | BIT1;
	#endif
	#if defined(__MSP430FR5739__)
	
	#endif
	#if defined(__MSP430FR4133__)
	P1SEL0 |= BIT0 | BIT1 | BIT2;
	#endif

	#if defined(__MSP430FR2311__)
	P1SEL0 |= BIT5 | BIT6 | BIT7;
	#endif

	/* USCI_A specific SPI setup */
	UCA0CTLW0 |= UCSWRST;
	UCA0CTLW0 = UCCKPH | UCMST | UCMSB | UCSYNC | UCSSEL_2 | UCSWRST;
	UCA0BRW = 0x01;
	UCA0CTLW0 &= ~UCSWRST;
}

uint8_t spi_transfer(uint8_t inb)
{
	UCA0TXBUF = inb;
	while ( !(UCA0IFG & UCRXIFG) )  // Wait for RXIFG indicating remote byte received via SOMI
		;
	return UCA0RXBUF;
}

uint16_t spi_transfer16(uint16_t inw)
{
	uint16_t retw;
	uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;

	UCA0TXBUF = inw8[1];
	while ( !(UCA0IFG & UCRXIFG) )
		;
	retw8[1] = UCA0RXBUF;
	UCA0TXBUF = inw8[0];
	while ( !(UCA0IFG & UCRXIFG) )
		;
	retw8[0] = UCA0RXBUF;
	return retw;
}

#ifdef __MSP430FR5969__
uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p1dir_save, p1out_save, p1ren_save;
	uint8_t p2dir_save, p2out_save, p2ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p1ren_save = P1REN; p1out_save = P1OUT; p1dir_save = P1DIR;
	p2ren_save = P2REN; p2out_save = P2OUT; p2dir_save = P2DIR;

	P1REN &= ~BIT5;
	P2REN &= ~(BIT0 | BIT1);
	P1OUT &= ~BIT5;
	P2OUT &= ~(BIT0 | BIT1);
	P1DIR |= BIT5;
	P2DIR = (P2DIR & ~(BIT0 | BIT1)) | BIT0;

	P1SEL1 &= ~BIT5;
	P2SEL1 &= ~(BIT0 | BIT1);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P2OUT |= BIT0;
	P1OUT |= BIT5;
	if (P2IN & BIT1)
		retw |= 0x0100;
	P1OUT &= ~BIT5;

	// Restore port states and continue with 8-bit SPI
	P1SEL1 |= BIT5;
	P2SEL1 |= BIT0 | BIT1;

	P1DIR = p1dir_save;
	P2DIR = p2dir_save;
	P1OUT = p1out_save;
	P2OUT = p2out_save;
	P1REN = p1ren_save;
	P2REN = p2ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif
#ifdef __MSP430FR4133__
uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p1dir_save, p1out_save, p1ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p1ren_save = P1REN; p1out_save = P1OUT; p1dir_save = P1DIR;

	P1REN &= ~(BIT0 | BIT1 | BIT2);
	P1OUT &= ~(BIT0 | BIT1 | BIT2);
	P1DIR = (P1DIR & ~(BIT0 | BIT1 | BIT2)) | BIT0 | BIT2;
	P1SEL0 &= ~(BIT0 | BIT1 | BIT2);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P1OUT |= BIT0;
	P1OUT |= BIT2;
	if (P1IN & BIT1)
		retw |= 0x0100;
	P1OUT &= ~BIT2;

	// Restore port states and continue with 8-bit SPI
	P1SEL0 |= BIT0 | BIT1 | BIT2;

	P1DIR = p1dir_save;
	P1OUT = p1out_save;
	P1REN = p1ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif
#ifdef __MSP430FR2311__
uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p1dir_save, p1out_save, p1ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p1ren_save = P1REN; p1out_save = P1OUT; p1dir_save = P1DIR;

	P1REN &= ~(BIT5 | BIT6 | BIT7);
	P1OUT &= ~(BIT5 | BIT6 | BIT7);
	P1DIR = (P1DIR & ~(BIT5 | BIT6 | BIT7)) | BIT5 | BIT7;
	P1SEL0 &= ~(BIT5 | BIT6 | BIT7);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P1OUT |= BIT7;
	P1OUT |= BIT5;
	if (P1IN & BIT6)
		retw |= 0x0100;
	P1OUT &= ~BIT5;

	// Restore port states and continue with 8-bit SPI
	P1SEL0 |= BIT5 | BIT6 | BIT7;

	P1DIR = p1dir_save;
	P1OUT = p1out_save;
	P1REN = p1ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif

#endif

#if defined(__MSP430_HAS_EUSCI_A1__) && defined(SPI_DRIVER_USCI_A1)
void spi_init()
{
	/* Configure ports on MSP430 device for USCI_A1 */
	#if defined(__MSP430FR5969__)
	P2SEL0 &= ~(BIT4 | BIT5 | BIT6);
	P2SEL1 |= BIT4 | BIT5 | BIT6;
	#endif
	#if defined(__MSP430FR5739__)
	
	#endif

	/* USCI_A specific SPI setup */
	UCA1CTLW0 |= UCSWRST;
	UCA1CTLW0 = UCCKPH | UCMST | UCMSB | UCSYNC | UCSSEL_2 | UCSWRST;
	UCA1BRW = 0x01;
	UCA1CTLW0 &= ~UCSWRST;
}

uint8_t spi_transfer(uint8_t inb)
{
	UCA1TXBUF = inb;
	while ( !(UCA1IFG & UCRXIFG) )  // Wait for RXIFG indicating remote byte received via SOMI
		;
	return UCA1RXBUF;
}

uint16_t spi_transfer16(uint16_t inw)
{
	uint16_t retw;
	uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;

	UCA1TXBUF = inw8[1];
	while ( !(UCA1IFG & UCRXIFG) )
		;
	retw8[1] = UCA1RXBUF;
	UCA1TXBUF = inw8[0];
	while ( !(UCA1IFG & UCRXIFG) )
		;
	retw8[0] = UCA1RXBUF;
	return retw;
}

#ifdef __MSP430FR5969__
uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p2dir_save, p2out_save, p2ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p2ren_save = P2REN; p2out_save = P2OUT; p2dir_save = P2DIR;

	P2REN &= ~(BIT4 | BIT5 | BIT6);
	P2OUT &= ~(BIT4 | BIT5 | BIT6);
	P2DIR = (P2DIR & ~(BIT4 | BIT5 | BIT6)) | BIT4 | BIT5;

	P2SEL1 &= ~(BIT0 | BIT1);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P2OUT |= BIT5;
	P2OUT |= BIT4;
	if (P2IN & BIT6)
		retw |= 0x0100;
	P2OUT &= ~BIT4;

	// Restore port states and continue with 8-bit SPI
	P2SEL1 |= BIT0 | BIT1;

	P2DIR = p2dir_save;
	P2OUT = p2out_save;
	P2REN = p2ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif
#endif

#if defined(__MSP430_HAS_EUSCI_B0__) && (defined(SPI_DRIVER_USCI_B) || defined(SPI_DRIVER_USCI_B0))
void spi_init()
{
	/* Configure ports on MSP430 device for USCI_B0 */
	#if defined(__MSP430FR5969__)
	P1SEL0 &= ~(BIT6 | BIT7);
	P1SEL1 |= BIT6 | BIT7;
	P2SEL0 &= ~BIT2;
	P2SEL1 |= BIT2;
	#endif
	#if defined(__MSP430FR5739__)
	
	#endif
	#if defined(__MSP430FR4133__)
	P5SEL0 |= BIT1 | BIT2 | BIT3;
	#endif

	#if defined(__MSP430FR2311__)
	P2SEL0 |= BIT3 | BIT4 | BIT5;
	#endif

	/* USCI_B specific SPI setup */
	UCB0CTLW0 |= UCSWRST;
	UCB0CTLW0 = UCCKPH | UCMST | UCMSB | UCSYNC | UCSSEL_2 | UCSWRST;
	UCB0BRW = 0x01;
	UCB0CTLW0 &= ~UCSWRST;
}

uint8_t spi_transfer(uint8_t inb)
{
	UCB0TXBUF = inb;
	while ( !(UCB0IFG & UCRXIFG) )  // Wait for RXIFG indicating remote byte received via SOMI
		;
	return UCB0RXBUF;
}

uint16_t spi_transfer16(uint16_t inw)
{
	uint16_t retw;
	uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;

	UCB0TXBUF = inw8[1];
	while ( !(UCB0IFG & UCRXIFG) )
		;
	retw8[1] = UCB0RXBUF;
	UCB0TXBUF = inw8[0];
	while ( !(UCB0IFG & UCRXIFG) )
		;
	retw8[0] = UCB0RXBUF;
	return retw;
}

#ifdef __MSP430FR5969__
uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p1dir_save, p1out_save, p1ren_save;
	uint8_t p2dir_save, p2out_save, p2ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p1ren_save = P1REN; p1out_save = P1OUT; p1dir_save = P1DIR;
	p2ren_save = P2REN; p2out_save = P2OUT; p2dir_save = P2DIR;

	P1REN &= ~(BIT6 | BIT7);
	P2REN &= ~BIT2;
	P1OUT &= ~(BIT6 | BIT7);
	P2OUT &= ~BIT2;
	P1DIR = (P1DIR & ~(BIT6 | BIT7)) | BIT6;
	P2DIR |= BIT2;

	P1SEL1 &= ~(BIT6 | BIT7);
	P2SEL1 &= ~BIT2;

	// Perform single-bit transfer
	if (inw & 0x0100)
		P1OUT |= BIT6;
	P2OUT |= BIT2;
	if (P1IN & BIT7)
		retw |= 0x0100;
	P2OUT &= ~BIT2;

	// Restore port states and continue with 8-bit SPI
	P1SEL1 |= BIT6 | BIT7;
	P2SEL1 |= BIT2;

	P1DIR = p1dir_save;
	P2DIR = p2dir_save;
	P1OUT = p1out_save;
	P2OUT = p2out_save;
	P1REN = p1ren_save;
	P2REN = p2ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif
#ifdef __MSP430FR4133__
uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p5dir_save, p5out_save, p5ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p5ren_save = P5REN; p5out_save = P5OUT; p5dir_save = P5DIR;

	P5REN &= ~(BIT1 | BIT2 | BIT3);
	P5OUT &= ~(BIT1 | BIT2 | BIT3);
	P5DIR = (P1DIR & ~(BIT1 | BIT2 | BIT3)) | BIT1 | BIT2;
	P5SEL0 &= ~(BIT1 | BIT2 | BIT3);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P5OUT |= BIT2;
	P5OUT |= BIT1;
	if (P5IN & BIT3)
		retw |= 0x0100;
	P5OUT &= ~BIT1;

	// Restore port states and continue with 8-bit SPI
	P5SEL0 |= BIT1 | BIT2 | BIT3;

	P5DIR = p5dir_save;
	P5OUT = p5out_save;
	P5REN = p5ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif
#ifdef __MSP430FR2311__
uint16_t spi_transfer9(uint16_t inw)
{
	uint8_t p2dir_save, p2out_save, p2ren_save;
	uint16_t retw=0;

	/* Reconfigure I/O ports for bitbanging the MSB */
	p2ren_save = P2REN; p2out_save = P2OUT; p2dir_save = P2DIR;

	P2REN &= ~(BIT3 | BIT4 | BIT5);
	P2OUT &= ~(BIT3 | BIT4 | BIT5);
	P2DIR |= BIT3 | BIT5;

	P2SEL0 &= ~(BIT3 | BIT4 | BIT5);

	// Perform single-bit transfer
	if (inw & 0x0100)
		P2OUT |= BIT5;
	P2OUT |= BIT3;
	if (P2IN & BIT4)
		retw |= 0x0100;
	P2OUT &= ~BIT3;

	// Restore port states and continue with 8-bit SPI
	P2SEL0 |= BIT3 | BIT4 | BIT5;

	P2DIR = p2dir_save;
	P2OUT = p2out_save;
	P2REN = p2ren_save;

	retw |= spi_transfer( (uint8_t)(inw & 0x00FF) );
	return retw;
}
#endif


#endif
