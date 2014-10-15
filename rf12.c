// Alpha module transceiver driver for PIC
// Adapted from JeeLib for Arduino by I.B. 22/07/2013

// rf12.c adapted for use with ccs compiler
//  8/08/2013 changed packet structure
// 13/08/2013 added sleep mode
// 16/09/2014 increased power to 0dB

#include <rf12_hardware.h>

// function prototypes
uint16_t _crc16_update(uint16_t crc, uint8_t a);

// packet data buffers used
static uint8_t send[RF12_MAXDATA];  // send data buffer
static uint8_t rcve[RF12_MAXDATA];  // received data buffer
static uint8_t nodeid;              // address of this node

volatile uint8_t rf12_buf[RF_MAX];  // recv/xmit buf, including hdr & crc bytes
volatile uint8_t rxfill;     		// number of data bytes in rf12_buf
volatile int8_t rxstate;     		// current transceiver state
volatile uint16_t rf12_crc;         // running crc value

//**********************************************************************
extern uint8_t portc;		// pre-declaration

uint8_t *memcpy(uint8_t *dst,uint8_t *src,uns8 len)
{
	uns8 i,j;
	uint8_t *d = dst;
	uint8_t *s = src;
	for (i=0; i<len; ++i)
	{
		j = s[i];
		d[i] = j;
	}
	return (dst);
}

uint16_t spi_xfer (uint16_t word)
{
char j;
uns16 result =0;

	output_low (SCK);
	output_low (nSEL);		// Enable
	for (j=0;j<16;j++)
	{
		result <<= 1;
		if (word & 0x8000) {
			output_high (SDO);
		}
		else {
			output_low (SDO);
		}
		output_high (SCK);
		if (SDI)
			result |= 1;
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		output_low (SCK);
		word <<= 1;
	}
	output_high (nSEL);		// Disable
	output_low (SCK);
	return (result);
}

void rf12_spiInit () {
	;
}

uint16_t rf12_xferSlow (uint16_t cmd)
{
uint16_t reply;

   // Exchange 16 bits
   reply = spi_xfer(cmd);

   return reply;
}

#pragma sharedAllocation

static void rf12_xfer (uint16_t cmd) {

  // send 16 bits
  spi_xfer(cmd);

}


// This call provides direct access to the RFM12B registers. If you're careful
// to avoid configuring the wireless module in a way which stops the driver
// from functioning, this can be used to adjust frequencies, power levels,
// RSSI threshold, etc. See the RFM12B wireless module documentation.
//
// This call will briefly disable interrupts to avoid clashes on the SPI bus.
//
// Returns the 16-bit value returned by SPI when used with the
// "0x0000" status poll command.

uint16_t rf12_control(uint16_t cmd)
{
uint16_t r;
	// disable interrupt
	GIE = 0;
	r = rf12_xferSlow(cmd);
	// enable interrupt
	GIE = 1;
	return r;
}


//**********************************************************************

void rf12_interrupt(void)
{
int8_t len;

		len = (int8_t) (rf12_len & RF12_PKTLEN_MASK);
		if (rxstate == TXRECV)
		{
			// Receive a byte

			uint8_t in = (uint8_t) (rf12_xferSlow(RF_RX_FIFO_READ) & 0x00FF);

			if (rxfill == 0)
				rf12_buf[rxfill++] = NETWORK_ID;

			rf12_buf[rxfill++] = in;
			rf12_crc = _crc16_update(rf12_crc, in);

			if (rxfill >= len + 6 || rxfill >= RF_MAX)
				rf12_xfer(RF_IDLE_MODE);

		}
		else
		{
			// Transmit a byte

			uint8_t out;

			if (rxstate < 0)
			{
				// here sending ID, length or data packet
				uint8_t pos = 4 + len + rxstate;
				++rxstate;
				out = rf12_buf[pos];
				rf12_crc = _crc16_update(rf12_crc, out);

			}
			else
			{
				// here sending the preamble, synchron or CRC value
				switch (rxstate)
				{
					case TXSYN1: out = 0x2D; break;
	                case TXSYN2: out = NETWORK_ID; rxstate = 0-(4 + len); break;
					case TXCRC1: out = (uint8_t)(rf12_crc & 0x00FF); break;
					case TXCRC2: out = (uint8_t)(rf12_crc >> 8); break;
					case TXDONE: rf12_xfer(RF_IDLE_MODE); // fall through
					default:     out = 0xAA;
				}
				++rxstate;
			}
			rf12_xfer(RF_TXREG_WRITE + out);

		}
}

//**********************************************************************


static void rf12_recvStart ()
{
    rxfill = rf12_len = 0;
    rf12_crc = 0xFFFF;
    rxstate = TXRECV;
    rf12_xfer(RF_FIFO_CLEAR);
    rf12_xfer(RF_FIFO_ENABLE);		// Reset synchron recognition
    rf12_xfer(RF_RECEIVER_ON);
}


// Call this periodically to keep the RF driver state machine going
// Note that even if you only want to transmit packets, you still have to call
// rf12_recvDone() periodically, because it keeps the RFM12B logic going. If
// you don't, rf12_canSend() will never return true.

uint8_t rf12_recvDone ()
{
uint8_t len= rf12_len & RF12_PKTLEN_MASK;
    if (rxstate == TXRECV && ((rxfill >= (len + 6)) || (rxfill >= RF_MAX)))
    {
        rxstate = TXIDLE;
        if (len > RF12_MAXDATA)
            rf12_crc = 1; // force bad crc if packet length is invalid
        if (RF12_DESTID == 0 || RF12_DESTID == nodeid)
        {
			if (len)
	    		memcpy(rcve, rf12_data, RF12_MAXDATA);
			return 1; // it's broadcast or addressed to this node
		}
    }
    if (rxstate == TXIDLE)
        rf12_recvStart();
    return 0;
}

//**********************************************************************

// Call this when you have some data to send. If it returns true, then you can
// use rf12_sendStart() to start the transmission. Else you need to wait and
// retry this call at a later moment.

uint8_t rf12_canSend ()
{
uint16_t rf12_status;
    if (rxstate == TXRECV && rxfill == 0)
    {
     	rf12_status = rf12_control(0x0000);
      	if ((rf12_status & RF_RSSI_BIT) == 0)
      	{
           rf12_control(RF_IDLE_MODE); // stop receiver
           rxstate = TXIDLE;
           return 1;
       	}
   	}
    return 0;
}

// Start transmission, with ACK and CTL flags prepared in len
void rf12_sendStart (uint8_t toNodeID, uint8_t len)
{
    rf12_hdr1 = toNodeID;
    rf12_hdr2 = nodeid;
    rf12_len = len;

    rf12_crc = 0xFFFF;
    rxstate = TXPRE1;
    rf12_xfer(RF_XMITTER_ON); // bytes will be fed via interrupts
}


// Switch to transmission mode and send a packet, with ACK and CTL flags prepared in len.
void rf12_sendStart_pack (uint8_t toNodeID, uint8_t* ptr, uint8_t len)
{
    memcpy(rf12_data, ptr, len & RF12_PKTLEN_MASK);
    rf12_sendStart(toNodeID, len);
}

// Reply immediately to a transmission requiring acknowledge.  Zero length packet.
void rf12_sendACK (uint8_t id)
{
    nodeid = id;
	while (!rf12_canSend ())
	{
		clrwdt();
    	rf12_recvDone(); // keep the driver state machine going, ignore incoming
	}
	rf12_sendStart(RF12_SOURCEID,RF12_SND_ACK);
}

// Send the data packet in send[], requiring acknowledge
void rf12_send_pack (uint8_t toNodeID, uint8_t id)
{
    nodeid = id;
	while (!rf12_canSend ())
	{
		clrwdt();
    	rf12_recvDone(); // keep the driver state machine going, ignore incoming
	}
	rf12_sendStart_pack(toNodeID, send, RF12_MAXDATA | RF12_RQ_ACK);

}


//**********************************************************************
// Enter a low-power mode.
// Parameter n=0:  sleep indefinitely
// n=-1: terminate sleep mode, wake up
// n=1 - 127: sleep, but RF module will create an interrupt 32*n ms later

void rf12_sleep (int8 n) {
    if (n < 0)
        rf12_control(RF_IDLE_MODE);
    else {
        rf12_control(RF_WAKEUP_TIMER | 0x0500 | n);
        rf12_control(RF_SLEEP_MODE);
        if (n > 0)
            rf12_control(RF_WAKEUP_MODE);
    }
    rxstate = TXIDLE;
}

//**********************************************************************


// Initialise, setting the node id

void rf12_initialize (uint8_t id)
{
uint16_t j,k;

    nodeid = id;
    rf12_spiInit();
    rf12_xfer(0x0000); // initial SPI transfer added to avoid power-up problem

    rf12_xfer(RF_SLEEP_MODE); // DC (disable clk pin), enable lbd
    rf12_xfer(0x94C4); // VDI - disables INT input that could cause hang-up
    rf12_xfer(RF_TXREG_WRITE); // in case we're still in OOK mode
    // wait until RFM12B is out of power-up reset, this takes several *seconds*
    for (j=0;j<300;++j)
    {
		for (k=0;k<100;++k)
			nop();
		rf12_xfer(0x0000);
		clrwdt();
		if (nIRQ)
			break;
	}
    if (!(nIRQ))
        return;

    rf12_xfer(RF_FIFO_ENABLE); // Enable FIFO, 433 MHz band
    rf12_xfer(0x8201); // disable clock out
    rf12_xfer(0xA636); // 433.975MHz
//    rf12_xfer(0xA618); // 433.900MHz
    rf12_xfer(0xC623); // approx 47 Kbps, i.e. 10000/29/(1+6) Kbps
//    rf12_xfer(0x94C1); // VDI,FAST,67kHz,0dBm,-91dBm
//    rf12_xfer(0x94C4); // VDI,FAST,67kHz,0dBm,-79dBm
    rf12_xfer(0x96C4); // VDI,SLOW,67kHz,0dBm,-79dBm
    rf12_xfer(0xC2EC); // AL,ml,DIG,DQD4
    rf12_xfer(0xCA83); // FIFO8,2-SYNC,!ff,DR
    rf12_xfer(0xCE00 | NETWORK_ID); // synchron pattern

    rf12_xfer(0xC483); // @PWR,NO RSTRIC,!st,!fi,OE,EN
//    rf12_xfer(0x9822); // !mp,45kHz,-5dB OUT
    rf12_xfer(0x9820); // !mp,45kHz,0dB OUT
    rf12_xfer(0xCC77); // PLL setting
    rf12_xfer(0xE000); // NOT USE
    rf12_xfer(0xC800); // NOT USE
    rf12_xfer(0xC0C0); // 5MHz,2.25V low bat
    rf12_len = 0;
    rxstate = TXIDLE;
}

//**********************************************************************
// CRC-16-ANSI, uses polynomial 0xA001, reversed bit ordering
uint16_t _crc16_update(uint16_t crc, uint8_t a)
    {
        int i;

        crc ^= a;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 1)
			{
                crc = (crc >> 1);
                crc ^= 0xA001;
			}
            else
                crc = (crc >> 1);
        }

        return crc;
    }
