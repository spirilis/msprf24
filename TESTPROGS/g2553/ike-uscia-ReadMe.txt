Example uses 2 msp430 LaunchPads with 2 MSP430G2553 MCUs. 
Remove Rx and Tx jumpers form J3 at upper right section of LP.
Leave LED jumpers on J5 at lower left section of LP.
You will need this lib at address https://github.com/spirilis/msprf24/

Example uses RF24_SPI_DRIVER_USCI_A (edit nrf_userconfig.h if needed), because we need green LED at P1.6.

nrf24l01 Vcc - LP pin  1 VCC.
nrf24l01 GND - LP pin 20 GND.
nrf24l01 MI  - LP pin  3 P1.1 USCI_A0 SPI mode: slave data out/master in.
nrf24l01 MO  - LP pin  4 P1.2 USCI_A0 SPI mode: slave data in/master out.
nrf24l01 SCK - LP pin  6 P1.4 USCI_A0 clock input/output.       
nrf24l01 CE  - LP pin  8 P2.0 General-purpose digital I/O pin.
nrf24l01 CSN - LP pin  9 P2.1 General-purpose digital I/O pin.
nrf24l01 IRQ - LP pin 10 P2.2 General-purpose digital I/O pin.

