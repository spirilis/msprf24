/* Pkt - Logical Link Control layer for nRF24L01+ radio communications
 *
 * Based on packet_processor.c concept code used in Grill Monitor and other msprf24-based
 * applications.
 *
 *
 *
 * Copyright (c) 2015, Eric Brundick <spirilis@linux.com>
 * Adapted from the Energia/Arduino lib of the same name by Eric Brundick (2014)
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


#ifndef PKT_H
#define PKT_H

#include <msp430.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

typedef uint8_t boolean;

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

#define PKT_MAX_PKTLEN 16
#define PKT_DEFAULT_QUEUE_DEPTH 3
#define PKT_DEFAULT_PROGID_REGISTRATIONS 4

typedef void(*PKT_CALLBACK)(const uint8_t progID, const int len, const void *buffer);

typedef struct {
	uint8_t progID;
	uint8_t rfaddr[5];
	uint8_t pktlen;
	uint8_t buffer[PKT_MAX_PKTLEN];
} PktTXQueue;

typedef struct {
	uint8_t progID;
	PKT_CALLBACK callback;
} PktRXprogram;

class Pkt {
	private:
		PktTXQueue **txQueue;
		void *txQueueArrayBacking;
		unsigned int txQueueDepth;

		PktRXprogram **progRegs;
		unsigned int progRegMaxCount;
		void *progRegsArrayBacking;

		boolean do_deepsleep_after_tx;
		PKT_CALLBACK unknownProgramCallback;
		PKT_CALLBACK defaultCallback;

	public:
		Pkt();
		void setTXqueueDepth(unsigned int queuedepth);
		void setMaxPrograms(unsigned int maxprogs);
		void begin();
		void end();

		// TX methods
		boolean send(const uint8_t progID, const uint8_t *rfaddr, const int len, const void *buffer);
		void flush();
		void setModeTXonly(boolean yesno);

		// RX methods
		boolean available();
		void loop();

		boolean attachAllPrograms(PKT_CALLBACK callback);
		boolean attachProgram(const uint8_t progID, PKT_CALLBACK callback);
		boolean detachAllPrograms();
		boolean detachProgram(const uint8_t progID);

		boolean attachUnknownProgram(PKT_CALLBACK callback);
		boolean detachUnknownProgram();
};


#endif /* PKT_H */
