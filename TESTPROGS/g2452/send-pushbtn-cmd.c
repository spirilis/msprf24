/* Pushbutton Command Sender
 *
 * Each press of the pushbutton (debounced by a 92ms wait) sends a packet OTA to
 * address 0xAB 0xAC 0xAD 0xAE 0x01
 * with the following byte sequence:
 * +----+----+----+
 * |0xFF|0x01|0x01|
 * +----+----+----+
 *
 */

#include <msp430.h>
#include "msprf24.h"
#include <stdint.h>

volatile uint16_t sleep_counter;
volatile uint8_t pushbutton_int;

const uint8_t dummyaddr[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
const uint8_t destaddr[] = { 0xAB, 0xAC, 0xAD, 0xAE, 0x01 };  // Dest addr for payload

const uint8_t rfpayload[] = { 0xFF, 0x01, 0x01 };  // Data to send OTA

int main()
{
	uint8_t txretry_count=0, txretry_latch=0;

	WDTCTL = WDTPW | WDTHOLD;
	DCOCTL = CALDCO_16MHZ;
	BCSCTL1 = CALBC1_16MHZ;
	BCSCTL2 = DIVS_2;  // SMCLK = 8MHz
	BCSCTL3 = LFXT1S_2;  // ACLK = VLO
	__delay_cycles(8000);     // Wait for VLO settling
	while (BCSCTL3 & LFXT1OF)
		;

	// Initialize P1.3 (pushbutton)
	P1DIR &= ~BIT3;
	P1REN |= BIT3;
	P1OUT |= BIT3;  // Pullup resistor
	P1IES |= BIT3;  // Active LOW
	P1IFG &= ~BIT3;
	P1IE |= BIT3;   // Enable IRQ

	// Initialize WDT timer
	IFG1 &= ~WDTIFG;
	IE1 |= WDTIE;
	sleep_counter = 0;
	pushbutton_int = 0;

	// Initialize nRF24L01+ RF transceiver
	rf_crc = RF24_EN_CRC;
	rf_addr_width = 5;
	rf_speed_power = RF24_SPEED_MIN | RF24_POWER_MAX;
	rf_channel = 8;
	msprf24_init();
	msprf24_open_pipe(0, 1);  // This is only for receiving auto-ACK packets.
	msprf24_set_pipe_packetsize(0, 0);  // Dynamic payload support
        // Note: Pipe#0 is hardcoded in the transceiver hardware as the designated "pipe" for a TX node to receive
        // auto-ACKs.  This does not have to match the pipe# used on the RX side.

	// Main loop
	while (1) {
		// Handle any nRF24 IRQs first
		if (rf_irq & RF24_IRQ_FLAGGED) {
			msprf24_get_irq_reason();

			if (rf_irq & RF24_IRQ_TX) {
				msprf24_irq_clear(RF24_IRQ_TX);
				flush_tx();
				w_rx_addr(0, (uint8_t*)dummyaddr);
				msprf24_powerdown();
			}

			if (rf_irq & RF24_IRQ_TXFAILED) {
				msprf24_irq_clear(RF24_IRQ_TXFAILED);
				if (txretry_count) {
					txretry_count--;
					if (!txretry_latch) {
						txretry_latch = 1;
						tx_reuse_lastpayload();
					}
					pulse_ce();
				} else {
					// Ran out of retries, give up
					flush_tx();
					w_rx_addr(0, (uint8_t*)dummyaddr);
					txretry_latch = 0;
					msprf24_powerdown();
				}
			}
			// No need to handle RX packets since we never enter RX mode
		}

		if (pushbutton_int) {
			// Software debounce (~92ms with VLOCLK)
			sleep_counter = 2;
			WDTCTL = WDT_ADLY_16;
			while (sleep_counter)
				LPM3;
			WDTCTL = WDTPW | WDTHOLD;
			if ( !(P1IN & BIT3) ) {  // P1.3 still active(LOW)?
				// OK, looks like we have a real button press...
				// But first, do we already have a packet in flight:
				if (msprf24_queue_state() & RF24_QUEUE_TXEMPTY) {
					// We don't, so proceed.
					msprf24_standby();
					w_tx_addr((uint8_t*)destaddr);
					w_rx_addr(0, (uint8_t*)destaddr);  // To catch the auto-ACK reply
					w_tx_payload(3, (uint8_t*)rfpayload);
					msprf24_activate_tx();
					txretry_count = 20;  // Try damned hard to send this, retrying up to 20 * ARC times (ARC=15 by default)
				}
				// If not, ignore this press.
			}
			pushbutton_int = 0;
			P1IFG &= ~BIT3;  // Clear any pending IRQs that may have fired during our debounce
			P1IE |= BIT3;
		}

		LPM3;
	}
}

// WDT overflow/timer
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	IFG1 &= ~WDTIFG;
	if (sleep_counter)
	        sleep_counter--;
	else
	        __bic_SR_register_on_exit(LPM3_bits);
}

// PORT1 interrupt ISR
#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void)
{
        if (P1IFG & BIT3) {
                P1IFG &= ~BIT3;
		P1IE &= ~BIT3;
		pushbutton_int = 1;
                __bic_SR_register_on_exit(LPM3_bits);
        }
}
