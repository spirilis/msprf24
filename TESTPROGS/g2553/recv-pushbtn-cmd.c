/* recv-pushbtn-cmd
 * Receive pushbutton-press commands from a TX device (see g2452/send-pushbtn-cmd.c)
 * and blink the red LED to match.
 */

#include <msp430.h>
#include "msprf24.h"
#include <stdint.h>
#include <string.h>

const uint8_t ouraddr[] = { 0xAB, 0xAC, 0xAD, 0xAE, 0x01 };  // Our RX address
const uint8_t dummyaddr[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };

const uint8_t rfpayload[] = { 0xFF, 0x01, 0x01 };  // Data we expect to see indicating a valid pushbutton press

int main()
{
	uint8_t pktlen, pipeid;
	uint8_t rfbuf[32];

	WDTCTL = WDTPW | WDTHOLD;
	DCOCTL = CALDCO_16MHZ;
	BCSCTL1 = CALBC1_16MHZ;
	BCSCTL2 = DIVS_2;  // SMCLK = 8MHz

	// Initialize red LED
	P1DIR |= BIT6;
	P1OUT &= ~BIT6;

	// Initialize nRF24L01+ RF transceiver
	rf_crc = RF24_EN_CRC;
	rf_addr_width = 5;
	rf_speed_power = RF24_SPEED_MIN | RF24_POWER_MAX;
	rf_channel = 8;
	msprf24_init();
	msprf24_open_pipe(1, 1);  // Receive pipe#1 (could use #0 too, as we don't do any TX...)
	msprf24_set_pipe_packetsize(1, 0);  // Dynamic payload support

	w_rx_addr(1, (uint8_t*)ouraddr);
	msprf24_activate_rx();  // Start listening
	// Main loop
	while (1) {
		// Handle incoming nRF24 IRQs
		if (rf_irq & RF24_IRQ_FLAGGED) {
			msprf24_get_irq_reason();

			if (rf_irq & RF24_IRQ_RX || msprf24_rx_pending()) {
				pktlen = r_rx_peek_payload_size();
				if (!pktlen || pktlen > 32) {  /* Erroneous >32byte packets need to be flushed right away
							        * I have actually seen these occur, even with CRC enabled, and it 
							        * logjams the FIFOs because r_rx_payload() can't read them...
							        */
					flush_rx();
					msprf24_irq_clear(RF24_IRQ_RX);
				} else {
					pipeid = r_rx_payload(pktlen, rfbuf);
					msprf24_irq_clear(RF24_IRQ_RX);
					if (pipeid == 1) {  // Only paying attention to pipe#1 (this should always eval true)
						if (!memcmp(rfbuf, rfpayload, 3))  // Valid pushbutton packet
							P1OUT ^= BIT6;             // Toggle red LED
					}
				}
			}
		}

		LPM4;
	}
}
