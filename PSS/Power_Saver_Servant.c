/************************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010
 
 This experiment demonstrates the application of a simple line follower robot. The 
 robot follows a white line over a black backround
 
 Concepts covered:  ADC, LCD interfacing, motion control based on sensor data

 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4

 ADC Connection:
 			  ACD CH.	PORT	Sensor
			  0			PF0		Battery Voltage
			  1			PF1		White line sensor 3
			  2			PF2		White line sensor 2
			  3			PF3		White line sensor 1
			  4			PF4		IR Proximity analog sensor 1*****
			  5			PF5		IR Proximity analog sensor 2*****
			  6			PF6		IR Proximity analog sensor 3*****
			  7			PF7		IR Proximity analog sensor 4*****
			  8			PK0		IR Proximity analog sensor 5
			  9			PK1		Sharp IR range sensor 1
			  10		PK2		Sharp IR range sensor 2
			  11		PK3		Sharp IR range sensor 3
			  12		PK4		Sharp IR range sensor 4
			  13		PK5		Sharp IR range sensor 5
			  14		PK6		Servo Pod 1
			  15		PK7		Servo Pod 2

 ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2. 
 	   To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
 
 Motion control Connection:
 			L-1---->PA0;		L-2---->PA1;
   			R-1---->PA2;		R-2---->PA3;
   			PL3 (OC5A) ----> PWM left; 	PL4 (OC5B) ----> PWM right; 
 
 LCD Display interpretation:
 ****************************************************************************
 *LEFT WL SENSOR	CENTER WL SENSOR	RIGHT WL SENSOR		BLANK			*
 *BLANK				BLANK				BLANK				BLANK			*
 ****************************************************************************
 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)

 2. Make sure that you copy the lcd.c file in your folder

 3. Distance calculation is for Sharp GP2D12 (10cm-80cm) IR Range sensor

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



#include <math.h> //included to support power function
#include "lcd.c"
//#include "ADC.c"


unsigned char data; //to store received data from UDR0
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char Front_Sharp_Sensor=0;
unsigned char Front_IR_Sensor=0;
unsigned char W_Threshold = 0x28;
unsigned char TOTAL_ROOMS = 0x03;
unsigned char last_on ;


//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}



//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();		//configuring for LCD printing.
	adc_pin_config();		//configure ADC pins ( for sensor inputs )
	motion_pin_config();	//configuring bot's movement
	pir_pin_config();		//configuring motion-sensor ports
	buzzer_pin_config(); 	//configuring buzzer ports

}



//Function for velocity control


void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	uart0_init();
	timer5_init();
	sei();   //Enables the global interrupts
}



SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable 

	 if (data == 0x01)
	 {
		UDR0 = 0xff;			//if the device is "ON" turning it "OFF"
	 }
	 
		

}





void save_power()
{

	unsigned char room_count = 0x00;
	_delay_ms(200);
	last_on = 0x03;
	while(1)
	{

		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		Front_Sharp_Sensor = ADC_Conversion(11);
		Front_IR_Sensor = ADC_Conversion(6);
		print_WL();
		
	
		while(Front_Sharp_Sensor > 0x82 || Front_IR_Sensor<0xF0)
		{
			stop();
			buzzer_on();
	  		Front_Sharp_Sensor = ADC_Conversion(11);
			Front_IR_Sensor = ADC_Conversion(6);

		}
			
		buzzer_off();

		if (room_count == TOTAL_ROOMS)
		{
			last_on = 0x03;
			while(1)
			{	
				Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
				Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
				Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
				Front_Sharp_Sensor = ADC_Conversion(11);
				while(Front_Sharp_Sensor > 0x82 || Front_IR_Sensor<0xF0)
				{
					stop();
					buzzer_on();
					Front_Sharp_Sensor = ADC_Conversion(11);
					Front_IR_Sensor = ADC_Conversion(6);

				}
			
				buzzer_off();
				
				if(Center_white_line<W_Threshold && Left_white_line>W_Threshold && Right_white_line>W_Threshold )
			{
				
				forward();
				velocity(100,100);
				last_on = 0x02;
			}

			else if(Center_white_line>W_Threshold && Left_white_line>W_Threshold && Right_white_line>W_Threshold)
			{
				if (last_on != 0x03)
				{
				stop();
		//		turn_around_r();
				break;
				}
				else
				{
				right();
				velocity(100,100);
				}
			}

			else if(Center_white_line<W_Threshold && Left_white_line<W_Threshold && Right_white_line>W_Threshold ) 
			{
				
				forward();
				velocity(50,100);
				last_on = 0x02;
			}

			else if(Center_white_line>W_Threshold && Left_white_line<W_Threshold && Right_white_line>W_Threshold ) 
			{
				
				left();
				velocity(100,100);
				last_on = 0x01;
				_delay_ms(120);
			}


			else if(Center_white_line<W_Threshold && Left_white_line>W_Threshold && Right_white_line<W_Threshold ) 
			{
				
				forward();
				velocity(100,50);
				last_on = 0x02;
			}

			else if(Center_white_line>W_Threshold && Left_white_line>W_Threshold && Right_white_line<W_Threshold ) 
			{
				
				right();
				velocity(100,100);
				last_on = 0x03;
			}

			else if(Center_white_line<W_Threshold && Left_white_line<W_Threshold && Right_white_line<W_Threshold ) 
			{
				
				left();
				velocity(100,100);
				last_on = 0x01;
				_delay_ms(120);
			}
			
			}
			break;
		}


		if(Center_white_line<W_Threshold && Left_white_line>W_Threshold && Right_white_line>W_Threshold )
		{
			
			forward();
			velocity(100,100);
			last_on = 0x02;
		}

		else if(Center_white_line>W_Threshold && Left_white_line>W_Threshold && Right_white_line>W_Threshold && last_on != 0x03)
		{
			
		if (last_on != 0x03)
		{
			stop();
			_delay_ms(2000);
			if (!check_motion())
			{
				
				UDR0 = room_count;
				print_sensor(2,1,10);
				if( ADC_Conversion(10) < 20 )
				{
				
				buzzer_on();
				_delay_ms(2000);
				buzzer_off();
				turn_around();
				
				}

			else
			{
				lcd_string("No Lights On");
				turn_around();
			}

			}
			else
			{
				
				lcd_string("Motion Detected");
				turn_around();
			}

			room_count++;
			last_on = 0x03;

			}
			else
			{
				right();
				velocity(100,100);
				_delay_ms(100);
				
			}
		}

		else if(Center_white_line<W_Threshold && Left_white_line<W_Threshold && Right_white_line>W_Threshold ) 
		{
			
			forward();
			velocity(50,100);
			last_on = 0x02;
		}
		//Strong LEft
		else if(Center_white_line>W_Threshold && Left_white_line<W_Threshold && Right_white_line>W_Threshold ) 
		{
			
			left();
			velocity(100,100);
			last_on = 0x01;
			_delay_ms(120);
		}


		else if(Center_white_line<W_Threshold && Left_white_line>W_Threshold && Right_white_line<W_Threshold ) 
		{
			
			forward();
			velocity(100,50);
			last_on = 0x02;
		}

		else if(Center_white_line>W_Threshold && Left_white_line>W_Threshold && Right_white_line<W_Threshold ) 
		{
			
			right();
			velocity(100,100);
			last_on = 0x03;
	
		}

		else if(Center_white_line<W_Threshold && Left_white_line<W_Threshold && Right_white_line<W_Threshold ) 
		{
			
			left();
			velocity(120,120);
			last_on = 0x01;
			_delay_ms(120);
		}
		
		

		

	}
}

void sensor()
{
	unsigned char flag;
	 while(1)
	 {
		flag = PINB;
		flag = flag & 0x20;
		lcd_print (1,1,flag ,3);
	 }

}

void print_WL()
{
	
		print_sensor(1,1,3);
		print_sensor(1,5,2);
		print_sensor(1,9,1);

	
}



//Main Function
int main()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	save_power();
//	print_WL();
}
