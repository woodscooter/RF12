// Software for Tx2 pcb and RXRly2 pcb, using RF12B rf module

#include <rf12_hardware.h>

#include <rf12.h>
//**********************************************************************

// #define UNIT_ID	11
// #define UNIT_ID	22
// #define UNIT_ID	33
#define UNIT_ID	25	// room light

#define DEST_ID	42
#define SEC_DEST_ID	42

// Format of transmitter message, reporting button press
//	DestID, device, action_code, parameter, unused_byte

//  Button 1
const uint8_t format1[] = {DEST_ID, 1, 7, 0xAA, 0xAA};
//  Button 2
const uint8_t format2[] = {SEC_DEST_ID, 2, 7, 0xAA, 0xAA};

// Format of transmitter message, operating receiver remotely
//	DestID, device, action_code, parameter, unused_byte

// #define DIRECT_ACT	// uncomment to use this format
//  Button 1
const uint8_t format3[] = {DEST_ID, 5, 6, 0xAA, 0xAA};
//  Button 2
const uint8_t format4[] = {SEC_DEST_ID, 6, 3, 0xAA, 0xAA};

// action_code can be:
// 0 OFF
// 1 ON
// 2 (Reserved for fade)
// 3 ON for N seconds, then OFF
// 4 ON for N minutes, then OFF
// 5 Fast chime, N times
// 6 Slow chime, N times
// 7 No action

// Parameter byte gives N (0-255)

#define OUT1	portc.4		// output from PIC
#define OUT2	portc.5		// output from PIC
#define IN1		PORTA.5		// input to PIC
#define IN2		PORTA.4		// input to PIC
#define IN3		PORTA.1		// input to PIC
#define IN4		PORTA.0		// input to PIC

//**********************************************************************
// #pragma config = 0x38FC
#pragma config = 0x38F4		// WDT controlled by SWDTEN bit in WDTCON sfr


// function prototypes
void extend (void);


// defs for debounce function

#define SWSTATE				6		// bit positions for debounce control byte
#define NEWCLOSE			5
#define NEWOPEN				4

// #define AUTO_TIMEOUT		(uint16_t) 10 	// time in seconds for auto shut-off
#define AUTO_TIMEOUT		(uint16_t) 0 	// set to 0 for always-on
uint8_t	AWAKE;

bit T0_flag;      		// set in interrupt, cleared in main, for .256us timing
bit ten_ms_flag;		// for control of IO
bit fifty_ms_flag;		// for repeat attempts
static uint8_t destid;	// who we are transmitting to
static uint8_t txrqflag;	// bits for transmit requests
static uint8_t txflag;	// bits for actual transmission
static uint8_t rxflag;	// set for new received data
static uint8_t retry;					// timing for retries
static uint8_t running_count, tens,
		fiftyms, onesec, onemin;    // timer cycles in 0.25ms, 10ms, 50ms, 1s, 1m
static uint8_t
dev,					// Defines output being switched
tenms,       			// For functional timing
seconds,     			// For functional timing
minutes,         		// For functional timing
reload,					// For chimes	number of 10ms between chimes
offtime,     			// For chimes	timer between chimes
repeat;      			// For chimes	number of chimes

static uint8_t tryPending;          	// remaining number of retries
static uint8_t porta, portc;		// shadow for outputs
static uint8_t ctrl_1, ctrl_2;		// control bytes for button inputs
static uint8_t ctrl_3, ctrl_4;		// control bytes for switch inputs

static uint16_t auto_sw_off;			// timer for auto switch-off in seconds

#define IN1_ON	txrqflag.0
#define IN2_ON	txrqflag.1
#define IN1_OFF	txrqflag.2
#define IN2_OFF	txrqflag.3

//**********************************************************************

//---------- INTERRUPT ------------------
#include "int16CXX.h"
#pragma interruptSaveCheck  w  // warning only
#pragma origin 4	// start address of interrupt routine

interrupt serverX(void)
{
	int_save_registers;

	if (T0IF)
	{
		 // service here every 1ms
		 // at 8MHz, Fosc/4 is 0.5us, TMR0 prescaled by 8, count every 4us
		TMR0 = 8;	// 2 cy for latency, interrupt after 250 cycles
		T0IF =0;
		T0_flag =1;
	}	// end T0 service

	if (INTF)
	{
		char sv_FSR = FSR;
		rf12_interrupt();
		FSR = sv_FSR;
		INTF = 0;
	}

	int_restore_registers
}		// end interrupt

//**********************************************************************

#include <rf12.c>

//**********************************************************************
// Put up-to-date data into the send packet array
void load_packet(void)
{

const uint8_t *ptr;
#ifdef DIRECT_ACT

	ptr = format3;
	if (txflag & 0x0A) 	// if it's IN2 opened or closed
		ptr = format4;
#else
	ptr = format1;
	if (txflag & 0x0A) 	// if it's IN2 opened or closed
		ptr = format2;
#endif

	destid = *ptr;
	++ptr;
	send[0] = *ptr;		//
	++ptr;
	send[1] = *ptr;		//
	++ptr;
	send[2] = *ptr;		//
	++ptr;
	send[3] = *ptr;		//
}

//**********************************************************************

// This is the whole of the interface to the RF12 module.
// Called often from the "main while (1)" loop.

void rf_check(void)
{
		// First, check to see whether any packet has been received
      if (rf12_recvDone())
      {
		  // conditional on CRC check being zero
		  	if (rf12_crc == 0)
		  	{
				// received data is now in array rcve
				if (RF12_DESTID == 0)
				{
					// Broadcast data, could do something with it
	  			    extend();
					nop();
				}
				else if (rf12_len & RF12_PKTLEN_MASK)	// if data length not zero
				{
					extend();
					rxflag = 1;			// flag that new data has been received
				}


				// Received acknowledge for last transmission
				if (RF12_GOT_ACK)
				{
					txflag =0;			// cancel the transmit flag
					retry = 0;
					tryPending = 0;	// Stop the repeat sends
				}

				// Send an acknowledge if requested
				if(RF12_WANTS_ACK)
				{
					rf12_sendACK(UNIT_ID);	// Can get blocked here
					nop();
				}
			}
	   }

	// Then every 50ms, see if it's time to send a transmission
	  if (fifty_ms_flag)
	  {
		  fifty_ms_flag =0;

		  // If there's something outstanding
		  if ((txflag == 0) && (txrqflag & 0x0F))
		  {
			  // copy one bit of txrqflag into txflag
			  uint8_t i, f = txrqflag;
			  for (i=0;i<4;++i)
			  {
				  if (f & 1)
				  {
					  // set the same bit in txflag
				  	txflag = 1<<i;
				  	  // clear that bit in txrqflag
				  	txrqflag &= ~(txflag);

				  	  // flag for transmission
				  	tryPending = RETRIES;
				  	retry =0;

				  	  // leave other bits in txrqflag for another time
				  	break;
				  }
				  f >>= 1;
			  }
		  }

			// retry controls the delay between retries
		  if ((tryPending) && (retry))
		  	retry--;

		  	// whenever retry is zero, we can send a packet
		  else if (tryPending)
		  {
 			    extend();
			    load_packet();		// fill the send[] array
				rf12_send_pack (destid,UNIT_ID);  // send, requesting ack
				retry = running_count & 0x03;		// random number 0..3
				retry += 5;		// minimum pause is 250ms

				// here, the number of attempts has been exhausted
				if (--tryPending == 0)
				{
				txflag =0;			// cancel the transmit flag
				retry = 0;
				}
		  }
	  }
}

//**********************************************************************
// Functions for switching to low power after a period of non-use.

void extend (void)
{
	auto_sw_off = AUTO_TIMEOUT;
	AWAKE =1;
}

void switch_off (void)
{
	if (AUTO_TIMEOUT != 0)
		AWAKE = 0;
}

//**********************************************************************
// Debounce function, sets flags to reveal changes of state
// Returns 1 if new closure, otherwise 0

char debounce (bit sample, uint8_t *control_byte, uint8_t max)
{
uint8_t temp = *control_byte & 0x0F;
	if (sample)
	{
		if (temp == max)
			return (0);		// Already at maximum
		++*control_byte;
		++temp;
		if (temp != max)
			return (0);		// Not yet at maximum
		// New switch closure, debounced
		*control_byte |= 1<<SWSTATE;
		*control_byte |= 1<<NEWCLOSE;
		return (1);
	}
	else
	{
		if (temp == 0)
			return (0);		// Already at minimum
		--*control_byte;
		--temp;
		if (temp)
			return (0);		// Counting down
		*control_byte &= ~(1<<SWSTATE);
		*control_byte |= 1<<NEWOPEN;
		return (0);
	}
}



//**********************************************************************
// Device switching on or off
// operates on bits of global "dev", so that single or multiple
// outputs can be addressed at one time
void device(uint8_t state)
{
	if (state)
	{
		// switch output on
		if (dev & 1)
			OUT1 = 1;
		if (dev & 2)
			OUT2 = 1;
	}
	else
	{
		// switch output off
		if (dev & 1)
			OUT1 = 0;
		if (dev & 2)
			OUT2 = 0;
	}
	PORTC = portc;
}



//**********************************************************************
//	Interaction with the outside world
void IOprocess(void)
{
uint8_t temp;

	if (ten_ms_flag)
	{
		// every 10ms
		ten_ms_flag =0;

			// Switch inputs
		debounce(!IN1,&ctrl_1,10);
		debounce(!IN2,&ctrl_2,10);
		debounce(!IN4,&ctrl_4,10);
		if (ctrl_1.NEWCLOSE)
		{
			IN1_ON =1;
			ctrl_1.NEWCLOSE =0;
		}
		if (ctrl_2.NEWCLOSE)
		{
			IN2_ON =1;
			ctrl_2.NEWCLOSE =0;
		}
/*
		if (ctrl_1.NEWOPEN)
		{
			IN1_OFF =1;
			ctrl_1.NEWOPEN =0;
		}
		if (ctrl_2.NEWOPEN)
		{
			IN2_OFF =1;
			ctrl_2.NEWOPEN =0;
		}
*/
		// SW1 toggles OP1

		if (ctrl_4.NEWCLOSE)
		{
			OUT1 ^= 1;
			PORTC = portc;
			ctrl_4.NEWCLOSE =0;
		}

			// Receiver outputs
		if (rxflag == 1)
		{
			dev = rcve[0];
			switch (rcve[1] & 7)
			{
				case 0:	device(0);
						break;
				case 1:	device(1);
						break;
				case 3:	device(1);
						seconds = rcve[2];
						break;
				case 4: device(1);
						minutes = rcve[2];
						break;
				case 5:	reload = 50;		// Fast chime
						offtime = 150;		// Pause 1.5s before first chime
						repeat = rcve[2];
						break;
				case 6:	reload = 200;		// Slow chime
						offtime = 150;		// Pause 1.5s before first chime
						repeat = rcve[2];
						break;
			}	// end switch
			rxflag = 0;

		}	// end if rxflag

	}
}

//**********************************************************************
// Non-blocking timers

void timing(void)
{
	if (T0_flag)
   	{
      	// here every 1 ms
      	T0_flag =0;
      	++running_count;


      	if (++tens >= 10)
      	{
		  	// here every 10ms
		  	tens =0;
	      	clrwdt();
	      	ten_ms_flag =1;

			// Chime timing
			if (offtime)
				if (--offtime == 0)
				{
					if (--repeat == 0)
						reload =0;
					offtime = reload;
					tenms = 20;
					device(1);
				}

			// Chime switch-off
			if (tenms)
				if (--tenms == 0)
				{
					device(0);
				}

			if (++fiftyms >= 5)
			{
				// Here once every 50ms
				fiftyms = 0;
		      	fifty_ms_flag =1;

	PORTA.0 ^= 1;	//!!
	/*
	dev = 3;
	if (PORTA.0)
		device(1);
	else
		device(0);
		*/
				if (++onesec >= 20)
				{
					// Here once every second
					onesec = 0;

					// one-shot switch-off
					if (seconds)
					{
						extend();
						if (--seconds == 0)
						{
							device(0);
						}
					}

					if (++onemin >= 60)
					{
						// Here once a minute
						onemin = 0;

						// one-shot switch-off
						if (minutes)
						{
							extend();
							if (--minutes == 0)
							{
								device(0);
							}
						}
					}

					if (auto_sw_off)
						--auto_sw_off;
					if (auto_sw_off ==0)
						switch_off();
				}
			}
	  	}
   	}
}

//**********************************************************************

void main()
{
COLD_RESET:
	clearRAM(); // set all RAM to 0

	// set registers
	OSCCON = 0x78;		//	8MHz, int osc
	OPTION_REG = 0x12;	//	Pullups, INT on neg edge, TMR0 is Fosc/4 Prescaled 1:8
	TRISA = 0x3F;		//	Bit 0 is an out (ICSP only)
	WPUA = 0x33;		//	Pullup on bits 0,1,4,5
	IOCA = 0;			// 	No interrupt on change, yet
	TRISC = 0x01;		//	All outputs except bit 0
	ANSEL = 0;			//	No analogues
	ADCON0 = 0;
	T1CON = 0;
	T2CON = 0;
	CMCON0 = 0x07;		//	Comparators off
	CMCON1 = 0x02;		//	Don't care
	WDTCON = 0x0F;		//	Prescale 1:4096, wdt on
	INTCON = 0;
	VRCON = 0;

WARM_RESET:

	// enable interrupts
	T0IE = 1;
	INTE = 1;
	GIE = 1;

	// if this is a power-on reset, pause here 0.1 second
	if (POR_ == 0) {
		tenms = 10;
		while (tenms)	{
			nop();
			timing();
			nop();
		}
		POR_ = 1;
	}

	rf12_initialize(UNIT_ID);


	if (!(nIRQ))
	goto COLD_RESET;

	extend();		// sets up auto_sw_off
	load_packet();	// maybe not essential


	while(1)
	{
		while (AWAKE)
		{
			timing();
			IOprocess();
			rf_check();
		}	// end while AWAKE

		// Stop watchdog
		clrwdt();
		SWDTEN = 0;

		// set up interrupt-on-change
		T0IE = 0;
		INTE = 0;
		GIE = 0;

		IOCA = 0x30;	// In1 and In2
		porta = PORTA;	// read port
		RAIF =0;		// clear IOC flag
		RAIE =1;		// enable IOC interrupt

		// go into power-down mode
		rf12_sleep (0);	// messes with GIE setting
		GIE = 0;
		sleep();
		nop();

		// wake on interrupt
		PORTC = portc;
		IOCA = 0;	// cancel IOC
		RAIE = 0;
		RAIF = 0;
		SWDTEN = 1;
		T0IF = 0;
		T0IE = 1;
		INTF = 0;
		INTE = 1;
		GIE = 1;
		rf12_sleep (-1);
		extend();
	}
}
