/********************************************************************************
 Platform: SPARK V
 Experiment: 10B_Serial_Comminucation_ZigBee_wireless
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited By: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 Last Modification: 22nd September 2010
 AVR Studio Version 4.17, Build 666

 Concepts covered: serial communication using zigbee wireless module

 This experiment demonstrates Robot control over serial port via ZigBee wireless 
 comunication module 

 There are two components to the motion control:
 1. Direction control using pins PORTB0 to 	PORTB3
 2. Velocity control by PWM on pins PD4 and PD5.

 In this experiment for the simplicity PD4 and PD5 are kept at logic 1.
 
 Pins for PWM are kept at logic 1.
  
 Connection Details:   L-1---->PB0;		L-2---->PB1;
   					   R-1---->PB2;		R-2---->PB3;
   					   PD4 (OC1B) ----> Logic 1; 	PD5 (OC1A) ----> Logic 1; 

 Serial Communication: PORTD 0 --> RXD UART receive for RS232 serial communication
					   PORTD 1 --> TXD UART transmit for RS232 serial communication

 Make sure that J5 us set towards the back side of the robot 
 i.e. XBee wireless module is connected to the serial port of the microcontroller.
 For more details on the jumper settings refer to the hardware manual.

 Use baud rate as 9600bps.
 
 To control robot use number pad of the keyboard which is located on the right hand
 side of the keyboard. Make sure that NUM lock is on.

 For more detail on hardware connection and theory refer the hardware manual.

 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega16
 	Frequency: 7372800
 	Optimization: -O0 (For more information read section: Selecting proper optimization
	              options below figure 4.22 in the hardware manual)
 *********************************************************************************/

 /********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode


********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
void relay_pin_config()
{
	DDRD = DDRD | 0x80;		//setting PORTD 7 as output.
	PORTD = PORTD & 0x7f;
}



void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

//turning the electronic device on on the basis of control message
void relay_on(void)
{
	unsigned char port_restore = 0;
	 port_restore = PIND;
	 port_restore = port_restore |  0x80;
	 PORTD = port_restore;	
}

//for turning off the relay device on the basis of control siganl
void relay_off(void)
{
	unsigned char port_restore = 0;
	 port_restore = PIND;
	 port_restore = port_restore &  0x7F;
	 PORTD = port_restore;	
}



//Function to initialize ports
void port_init()
{
 relay_pin_config();
}

//UART0 initialisation
// desired baud rate: 9600
// actual: baud rate:9600 (0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSRB = 0x00; //disable while setting baud rate
 UCSRA = 0x00;
 UCSRC = 0x86;
 UBRRL = 0x2F; //set baud rate lo  //67 is for 16MHz 9600 baudrate
 UBRRH = 0x00; //set baud rate hi
 UCSRB = 0x98; 
}

//Function used for setting motor's direction

void init_devices (void)
{
 cli();         //Clears the global interrupts
 port_init();
 uart0_init();
 sei();         //Enables the global interrupts
}

unsigned char status ;

SIGNAL(SIG_UART_RECV) 
{
	
unsigned char receive_data=0;   // used to save Receiving data

 receive_data = UDR;			
 

 if(receive_data == 0x01)        //ASCII value of 8
 {
	UDR = status;                   //forward
 }

 if(receive_data == 0xff)        //ASCII value of 2
 {
	relay_off();                //back
 }

 
}

//Main Function
int main()
{
	status = 0x01;	//setting status of relay device
	init_devices();	
	relay_on();	//turning the electronic devide on
	while(1);
}

