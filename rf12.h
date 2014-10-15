// 8/8/2013 changed packet structure
// 14/12/2013 version for Tx2 pcb


// #include <stdint.h>

// Data type definitions change
//   to PIC16 using CC5X compiler

//#define bit int1
#define uint8_t uns8
#define int8_t int8
#define long_t int32
#define uint16_t uns16
#define uint32_t uns32


/// RFM12B driver definitions

#ifdef COMPAT
#define NETWORK_ID 			0xD4	// retains compatibility with older RF12 module
#else
#define NETWORK_ID 			0xC3		// unique to this network
#endif
#define RF12_SND_ACK		0x80
#define RF12_RQ_ACK			0x40
#define RF12_PKTLEN_MASK	0x1F

/// Shorthand for RFM12B group byte in rf12_buf.
#define rf12_grp        rf12_buf[0]
/// Shorthand for RFM12B header byte in rf12_buf.
#define rf12_hdr1        rf12_buf[1]
#define rf12_hdr2        rf12_buf[2]
/// Shorthand for RFM12B length byte in rf12_buf.
#define rf12_len        rf12_buf[3]
/// Shorthand for first RFM12B data byte in rf12_buf.
#define rf12_data       (rf12_buf + 4)

#define RF12_DESTID		rf12_hdr1
#define RF12_SOURCEID	rf12_hdr2
#define RF12_ACKFLAGS	rf12_len

/// RFM12B Maximum message size in bytes.
// #define RF12_MAXDATA    63
#define RF12_MAXDATA    4   // 4-bytes packet

#define RF_MAX   (RF12_MAXDATA + 6)

#define RETRIES     6               // stop retrying after 6 times

// RF12 command codes
#define RF_RECEIVER_ON  0x8289      // er and ex only
#define RF_XMITTER_ON   0x8229      // et and ex only
#define RF_IDLE_MODE    0x8209      // ex only
#define RF_SLEEP_MODE   0x8201      // everything off
#define RF_WAKEUP_MODE  0x8207
#define RF_TXREG_WRITE  0xB800
#define RF_RX_FIFO_READ 0xB000
#define RF_WAKEUP_TIMER 0xE000
#define RF_FIFO_CLEAR 	0x8098
#define RF_FIFO_ENABLE 	0x80D8

// RF12 status bits
#define RF_RGIT_FFIT    0x8000
#define RF_LBD_BIT      0x0400
#define RF_RSSI_BIT     0x0100

// transceiver states, these determine what to do with each interrupt
enum {
    TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE,
    TXRECV,
    TXPRE1, TXPRE2, TXPRE3, TXSYN1, TXSYN2,
};

// Macro for port bit outputs
#define output_low(p,N) p &= ~(1<<N); PORTC = portc

#define output_high(p,N) p |= (1<<N); PORTC = portc


// Macro to simplify detecting a request for an ACK.
#define RF12_WANTS_ACK ((RF12_ACKFLAGS & RF12_RQ_ACK) && !(RF12_ACKFLAGS & RF12_SND_ACK))

// Macro to simplify detecting an ACK response
#define RF12_GOT_ACK ((RF12_ACKFLAGS & RF12_SND_ACK) && !(RF12_ACKFLAGS & RF12_RQ_ACK))

extern void rf12_interrupt(void);


