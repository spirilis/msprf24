#include <msp430.h>
#include "msprf24.h"
#include "nrf_userconfig.h"

volatile unsigned int user;

int main()
{
	uint8_t addr[5];
	uint8_t buf[32];

	WDTCTL = WDTHOLD | WDTPW;
	DCOCTL = CALDCO_1MHZ;
	BCSCTL1 = CALBC1_1MHZ;
	BCSCTL2 = DIVS_0;  // SMCLK = DCOCLK/1
	// SPI (USI) uses SMCLK, prefer SMCLK=DCO (no clock division)
	user = 0xFE;

	/* Initial values for nRF24L01+ library config variables */
	rf_crc = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
	rf_addr_width      = 5;
	rf_speed_power     = RF24_SPEED_MIN | RF24_POWER_MIN;
	rf_channel         = 119;
	msprf24_init();
	msprf24_set_pipe_packetsize(0, 32);
	msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst

	// Set our RX address
	addr[0] = 0xDE;
	addr[1] = 0xAD;
	addr[2] = 0xBE;
	addr[3] = 0xEF;
	addr[4] = 0x00;
	w_rx_addr(0, addr);

	// Receive mode
	if (!(RF24_QUEUE_RXEMPTY & msprf24_queue_state())) {
		flush_rx();
	}
	msprf24_activate_rx();
	LPM4;

	while (1) {
		if (rf_irq & RF24_IRQ_FLAGGED) {
			msprf24_get_irq_reason();
		}
		if (rf_irq & RF24_IRQ_RX || msprf24_rx_pending()) {
			r_rx_payload(32, buf);
			msprf24_irq_clear(RF24_IRQ_RX);
			user = data[0];
		} else {
			user = 0xFF;
		}
		LPM4;
	}
	return 0;
}
