#include <msp430.h>
#include "msprf24.h"
#include "nrf_userconfig.h"

void test_msprf24_init();

volatile unsigned int user;
volatile uint8_t dat;

int main()
{
	uint8_t addr[4];
	uint8_t buf[32];

	WDTCTL = WDTHOLD | WDTPW;
	DCOCTL = CALDCO_16MHZ;
	BCSCTL1 = CALBC1_16MHZ;
	BCSCTL2 = DIVS_1;  // SMCLK = DCOCLK/2
	// SPI (USCI) uses SMCLK, prefer SMCLK < 10MHz (SPI speed limit for nRF24 = 10MHz)
	user = 0xFE;

	/* Initial values for nRF24L01+ library config variables */
	rf_crc = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
	rf_addr_width      = 4;
	rf_speed_power     = RF24_SPEED_MIN | RF24_POWER_MIN;
	rf_channel	   = 119;
	test_msprf24_init();  // Init SPI, output ports only
	msprf24_set_config(RF24_PWR_UP);
	__delay_cycles(DELAY_CYCLES_5MS);
	dat = r_reg(RF24_CONFIG);

	_DINT();
	LPM4;
	return(0);
}

void test_msprf24_init()
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
	#endif

	// Setup CSN/CE ports
	#if nrfCSNport == 1
		P1DIR |= nrfCSNpin;
	#elif nrfCSNport == 2
		P2DIR |= nrfCSNpin;
	#elif nrfCSNport == 3
		P3DIR |= nrfCSNpin;
	#endif
	CSN_DIS;

	#if nrfCEport == 1
		P1DIR |= nrfCEpin;
	#elif nrfCEport == 2
		P2DIR |= nrfCEpin;
	#elif nrfCEport == 3
		P3DIR |= nrfCEpin;
	#endif
	CE_DIS;
}
