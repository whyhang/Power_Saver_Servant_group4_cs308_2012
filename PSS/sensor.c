#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char LIGHT_THRESHOLD = 0x10;

//Function to check whether motion is detected or not.
unsigned char check_motion(void)
{
	int x,p;
	unsigned char flag ;
	p = 0;
	for (x = 0 ; x < 1000 ; x++)
	{
		flag = PINB;
		flag = flag & 0x20;		//reading motion sensor value on PINB-6
		if (flag  != 0 )
		{
		p++;
		
		}
				
	
	}
//	lcd_print(1,1,p, 4);
	if (p > 600) // threshhold to be set later
	{
		return 1;
	}
	else 
	{
		return 0;
	}

}


//Function to check if the lights are "ON" or not.
unsigned char check_light()
{
		//reading light sensor's value on ADC-10
		unsigned char light_sense =  ADC_Conversion(10);

		if (light_sense > LIGHT_THRESHOLD)
		{
			return 0;
		}
		else
		{
			return 1;
		}				

}


void pir_pin_config()
{
	DDRB = DDRB & 0xDF;		//setting OC1A as input to take motion sensor's reading
	

}
