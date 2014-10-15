// rf12_hardware.h

#ifndef RF12_HARDWARE_H
#define RF12_HARDWARE_H
//**********************************************************************
// hardware defs for RF module

#define nSEL   portc, 3		// output from PIC
#define nIRQ   PORTA.2		// input to PIC

#define SDI		PORTC.0		// input to PIC
#define SDO		portc, 1		// output from PIC
#define SCK		portc, 2		// output from PIC
#endif
