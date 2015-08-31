Pkt is a C++ library originally developed for Energia/Enrf24 which I have adapted to use
msprf24 so that it may be used in non-Arduino related applications.

This does require that you use the C++ compiler for your whole project instead of
straight C, however the rest of your applicaton certainly may be coded in C.

Pkt provides an abstraction layer on top of the nRF24L01+ which interprets packets TX or
RX in terms of sub-packets delimited by an 8-bit "program ID" (0xFF is reserved)
followed by an 8-bit length identifier.  Multiple "packets" can be stuffed inside a
single <=32 byte nRF24 payload.

On the TX side, you can use Pkt's send() followed by flush() functions to pack some data
into its buffers and then transmit them out.  These packets can be assigned to different
nRF24L01+ RF addresses, and it will work out how it wants to pack subsequent packets
intended for the same RF address if appropriate.

Multiple send()'s may be run ending in a single flush(); the default "TX queue depth"
is 3, i.e. three send()'s can be run before it kicks back false and silently discards
your next send().  The payload within each packet can't exceed 16 bytes by default.
16 bytes would be 18 in the frame (add program ID + length bytes), so that doesn't
quite fit 2 packets per nRF24 frame.

On the RX side, you will set up the pipes using msprf24 and the RX address with msprf24's
w_rx_addr - keep in mind the Pkt interface is only designed to know about pipe#1 for RX,
since it's derived from Enrf24 which only acknowledged the presence of one RX pipe.
It should in theory work with multiple RX pipes though, but it won't be able to
discriminate how it handles packets coming from pipe #1 vs pipe #2, 3, 4, etc. beyond
the fact that it will interpret each frame's contents in terms of its "program ID" paradigm.

For RX, the library is designed for the calling firmware to assign "callback functions"
to each program ID.  When an RX packet is detected by the user's firmware, the user's
firmware should call Pkt's loop() function to "handle" the incoming RX request; loop()
is where the payload is read and callback functions are dispatched.

A typical while() loop in a C application may look like this:

Pkt radio;

int main() {
....blahblah...
    while(1) {
        if (rf_irq & RF24_IRQ_FLAGGED) {
	    if (radio.available()) {
                radio.loop();  // Handle the RX and clear IRQs as necessary
            }
        }
 
        if (!(rf_irq & RF24_IRQ_FLAGGED))
            LPM4;
    }
    return 0;
}


A few items of trivia to note:

1. This library uses malloc().  It might go away.  Probably would be appropriate
   to have the library stick to #define's defaults instead of letting it be
   dynamic; the dynamic TX queue depth and program ID callback registry was intended
   to make it intuitive and configurable for Energia/Arduino users.

2. There is a function setModeTXonly(true/false) which will tailor Pkt's behavior
   during TX.  If this is false, the Pkt library will automatically issue
   msprf24_activate_rx() after TX is done.  Otherwise, when "true", the Pkt library
   will automatically issue msprf24_powerdown() after TX is done.

3. The callback function prototype is:
typedef void(*PKT_CALLBACK)(const uint8_t progID, const int len, const void *buffer);
   so all callback functions expect to return void, but take a const uint8_t,
   const int, and const void *

4. The function prototypes for all the callback attach/detach functions is:

        boolean attachAllPrograms(PKT_CALLBACK callback);
        boolean attachProgram(const uint8_t progID, PKT_CALLBACK callback);
        boolean detachAllPrograms();
        boolean detachProgram(const uint8_t progID);

        boolean attachUnknownProgram(PKT_CALLBACK callback);
        boolean detachUnknownProgram();

    This should be self-explanatory.  If you do not issue attachUnknownProgram
    or attachAllPrograms but you receive a packet or a subset of a received frame
    happens to include a program ID that you do not have a callback for, this data is
    silently discarded by the library.

5. Default # of individual program IDs that can be registered with a callback is 4.
   This can be adjusted using setMaxPrograms() - note this call will induce a free()
   followed by malloc() to allocate a new array of program ID + callback function
   pointer structures.
   ** Note: If I do decide to extricate malloc/free, this function along with the
            setTXqueueDepth function will disappear.

6. During active TX mode, i.e. within the library's flush() function, the library
   will enter LPM0 while waiting for the nRF24 IRQ to fire and rf_irq to get updated.

7. If AutoACK is enabled, and TX fails, it does so silently; this library will not
   try to re-submit the TX that failed and it has no way of notifying you that it
   failed.  Might have to think this one through deeper if it's important enough.
