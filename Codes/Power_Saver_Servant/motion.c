
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


unsigned char W_Threshold1 = 0x28;	//guidline for whiteline following
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}



//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

//code for buzzer on	
void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

//code for buzzer off
void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}


//set of motion functions
void forward (void) 
{
  motion_set (0x06);
}

void stop (void)
{
  motion_set (0x00);
}

void left (void)
{
  motion_set (0x05);
}

void right (void)
{
  motion_set (0x0A);
}

//turn around code for bot
void turn_around(void)
{
	
	left();
	velocity(100,100);
	while(ADC_Conversion(2) > W_Threshold1 )
	{
		
	}
}
//turning around from right
void turn_around_r(void)
{
	
	right();
	velocity(100,100);
	while(ADC_Conversion(2) > W_Threshold1 )
	{
		
	}
}
