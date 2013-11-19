/****************************************************************************
*
*   Copyright (c) 2007 www.societyofrobots.com
*   (please link back if you use this code!)
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation.
*
*   Alternatively, this software may be distributed under the terms of BSD
*   license.
*
*	SoR Utilities v1, March 10th, 2007
*
*   **Modifications made by Team Robotec**
*   **October 20th 2012**
*
****************************************************************************/

//AVR includes
#include <avr/io.h>		    // include I/O definitions (port names, pin names, etc)
#include <avr/interrupt.h>	// include interrupt support
#include <avr/eeprom.h>

//AVRlib includes
#include "global.h"		// include global settings
#include "buffer.h"		// include buffer function library
#include "a2d.h"		// include A/D converter function library

//define port functions; example: PORT_ON( PORTD, 6);
#define PORT_ON( port_letter, number )			port_letter |= (1<<number)
#define PORT_OFF( port_letter, number )			port_letter &= ~(1<<number)
#define PORT_ALL_ON( port_letter, number )		port_letter |= (number)
#define PORT_ALL_OFF( port_letter, number )		port_letter &= ~(number)
#define FLIP_PORT( port_letter, number )		port_letter ^= (1<<number)
#define PORT_IS_ON( port_letter, number )		( port_letter & (1<<number) )
#define PORT_IS_OFF( port_letter, number )		!( port_letter & (1<<number) )



//************CONFIGURE PORTS************
//configure ports for input or output - specific to ATmega8
void configure_ports(void)
	{
	DDRC = 0x00;  //configure all C ports for input
	PORTC = 0x00; //make sure pull-up resistors are turned off
	DDRD = 0xFF;  //configure all D ports to output
	DDRB = 0x00;  //all B ports input except 6,7 (google search '0b11000111 to hex')
	}
//***************************************

//************DELAY FUNCTIONS************
//wait for X amount of cycles (23 cycles is about .992 milliseconds)
//to calculate: 23/.992*(time in milliseconds) = number of cycles
void delay_cycles(unsigned long int cycles)
	{
	while(cycles > 0)
		cycles--;
	}
//***************************************

//*********SIMPLIFIED FUNCTIONS**********
//Turn LED on
void LED_on(void)
	{
	PORT_OFF(PORTD, 4);//turn LED on
	}
//Turn LED off
void LED_off(void)
	{
	PORT_ON(PORTD, 4);//turn LED off
	}
//Control Left Servo
void servo_left(signed long int speed)
	{
	PORT_ON(PORTD, 2);
	delay_cycles(speed);
	PORT_OFF(PORTD, 2);//keep off
	delay_cycles(500);
	}
//Controll right servo
void servo_right(signed long int speed)
	{
	PORT_ON(PORTD, 3);
	delay_cycles(speed);		
	PORT_OFF(PORTD, 3);//keep off
	delay_cycles(500);
	}
//Controll scanning servo
void servo_scan(signed long int speed)
	{
	PORT_ON(PORTD, 1);
	delay_cycles(speed);		
	PORT_OFF(PORTD, 1);//keep off
	delay_cycles(200);
	}
//***************************************
