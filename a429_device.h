//////////////////////////////////////////////////////////////////////////////////////////////////
// #include "a429_device.h"
/************************************
	a429_device.h
*************************************/
#ifndef	A429_DEVICE_H
#define A429_DEVICE_H


/* Inputs from the ARINC UART */
int a429DR1_pin = 2;
int a429DR2_pin = 3;
int a429TXR_pin = 9;

/* Outputs to the ARINC UART */
int a429SEL_pin = 4;
int a429OE1_pin = 5;
int a429OE2_pin = 6;
int a429LD1_pin = 7;
int a429LD2_pin = 8;
int a429ENTX_pin = 10;
int a429LDCW_pin = 11;
int a429MR_pin = 12;
int a429DBCEN_pin = 13;

// 16 additional pins are used for the data bus.
// I'm using PORTA and PORTC, mega pins 22-37

// Now define several timer values for communicating with the DEI 1016:
// These are in nano seconds.
int Tmr = 200;  //master reset 
int Tpwld = 130;
int Tsdw = 110;
int Tddr = 200;
int Tssel = 20;
int Thsel = 20;
int Tpwoe = 200;
int Toeoe = 30;
int Tdoedr = 200;


/*
  These defines are the bit values that enable/disable settings of the 
  ARINC 429 UART.
*/
#define PAREN     0x10;  // PARity ENable
#define SLFTST    0x00;  // SeLF TeST
#define NOTSLFTST 0x20;  // Not SeLF TeST
#define PARCK     0x10;  // PARity ChecK
#define TXLO      0x20;  // TX (transmit) LOw speed
#define TXHI      0x00;  // TX (transmit) HIgh speed
#define RXLO      0x40;  // RX (receive)  LOw speed
#define RXHI      0x00;  // RX (receive)  HIgh speed

#endif

