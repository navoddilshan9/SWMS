#ifndef F_CPU

# define F_CPU 16000000UL // clock speed is 16MHz

#endif

#include <avr/io.h>        // AVR header
#include <util/delay.h>    // delay header
#include "lcd.h"
#include <stdio.h>
#include <avr/interrupt.h>
#include <stdlib.h>

//******************LCD PROGRAM STARTS*********************//

#define LCD_DATA PORTB          // port B is selected as LCD data port
#define ctrl PORTD              //  port D is selected as LCD command port
#define en PD7                  // enable signal is connected to port D pin 7
#define rw PD6                  // read/write signal is connected to port D pin 6
#define rs PD5                  // register select signal is connected to port D pin 5



//***************** PUSH BUTTON PINS*********************//
#define  CURRENT_RATE_PUSH 1    // for current water rate
#define  TOTAL_WATER_PUSH 0    // for total water 
#define  WATER_CONTROL_PUSH 2    // for  water  control switch
#define  BATTERY_STATUS 3   // for  water  control switch


//***************** VARIABLES *********************//
float time_clock=1;      //Count the time
float pulses=0;          //Store digital pulses from the sensor
volatile uint8_t tot_overflow;   //To count the time



//***************** FUNCTION PROTOTYPES*********************//
void LCD_cmd(unsigned char cmd);
void init_LCD(void);
void timer0_int(void);
void LCD_write(unsigned char data);
void display_tot(void);
int clk(void);




int main(void)

{

	DDRB=0xFF;              // set LCD data port as output
	DDRD=0xE0;              // set LCD signals (RS, RW, E) as out put
 	DDRC &=~(1<<CURRENT_RATE_PUSH);     // set PD1 port as input
	DDRC &=~(1<<TOTAL_WATER_PUSH);     // set PD0 port as input
	DDRC &=~(1<<WATER_CONTROL_PUSH);     // set PD0 port as input
	DDRC &=~(1<<BATTERY_STATUS);     // set PD0 port as input
	DDRD &=~(1<<2);       //Define the INT0 as input pin
	DDRD |=(1<<3);       //Define the LED for water valve
/*	DDRA &=~(1<<0);        //Define the analog input pin*/
	 DDRA = 0x00;  //define the input data for voltage sensor
	init_LCD();             // initialize LCD
	LCD_cmd(0x0C);          // display on, cursor off
	timer0_int();


	MCUCR |= 1<<ISC01;    //interrupt fire on falling edge
	GICR |= 1<< INT0;     //Enable the external interrupt source in general interrupt control register
	sei();              	//enable global interrupts


    ADMUX |=(1<<REFS0);   //Define AVCC with external capacitor at AREF pin

    ADCSRA |=(1<<ADEN)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);//Enable ADC mode and set the prescaler to 128

	welocome_display();
	while(1){
	
		clk();
	/*	timeout_display();*/
		if (PINC & (1<<CURRENT_RATE_PUSH))
		{
			LCD_cmd(0x01);          // make clear LCD
			LCD_Write_String("please wait");
			_delay_ms(100);
			display_rate();     //calling function to display current rate
			LCD_cmd(0x01);          // make clear LCD

		}else if (PINC & (1<<TOTAL_WATER_PUSH))
		{
			LCD_cmd(0x01);          // make clear LCD
			LCD_Write_String("please wait");
			_delay_ms(100);	
			display_tot();    //calling function to display current rate
			LCD_cmd(0x01);          // make clear LCD
		}else if (PINC & (1<<WATER_CONTROL_PUSH))
		{
			LCD_cmd(0x01);          // make clear LCD
			LCD_Write_String("please wait");
			_delay_ms(100);
			water_control();    		
			LCD_cmd(0x01);          // make clear LCD
		}else if (PINC & (1<<BATTERY_STATUS))
		{
			LCD_cmd(0x01);          // make clear LCD
			LCD_Write_String("please wait");
			_delay_ms(200);
			ReadVoltage();    //calling function to display current rate
			LCD_cmd(0x01);          // make clear LCD
		}


		
	}

	return 0;

}

//***************** FUNCTIONS*********************//

int clk(){
	if (tot_overflow>=24)
	{
		if(TCNT0>=106){
			TCNT0=0;
			tot_overflow=0;
			time_clock++;
			return 1;
		}
	}
	return 0;
}

void timer0_int(){
	TCCR0 |= 1<<CS02;            //set the prescaler to 256
	TCNT0=0;                     //reset the timer
	TIMSK |= 1<<TOIE0;           //Enable the timer overflow interrupt
	sei();                       //enable global interrupts

}



ISR(TIMER0_OVF_vect){
	tot_overflow++;
}

ISR(INT0_vect){
	pulses++;
}



//******************Push button functions*********************//


void display_rate(){
	LCD_cmd(0x01);          // make clear LCD
	float temp_pulse=pulses;
	char out_str[30] = {0};
		
	float l_minutes=(temp_pulse/time_clock)/7.5;  //calculation
	
	sprintf(out_str, " %.2lf\r\n", l_minutes);
	LCD_Write_String("Water FLow Rate:");
	LCD_cmd(0xC0);          // move cursor to the start of 2nd line
	LCD_Write_String(out_str);
	LCD_Write_String("L/M");
	_delay_ms(100);
	LCD_cmd(0x01);          // make display ON, cursor ON
	_delay_ms(200);          //display the liter/hour flow rate

}

//******************Welcome displays*********************//

void welocome_display(){
	LCD_cmd(0x01);  
	LCD_Write_String("  WELCOME TO ");
	LCD_cmd(0xC0);          // move cursor to the start of 2nd line
	LCD_Write_String("     -- S.W.M.S");
	
}


//******************Total water *********************//
void display_tot(){
	LCD_cmd(0x01);          // make clear LCD	
	float total_water=(pulses)/7.5;  //calculation

	
	char out_str[30] = {0};
	sprintf(out_str, " %.2lf\r\n", total_water);
	LCD_Write_String("Water units : ");
	//itoa(out_str,show_a,10);
	LCD_cmd(0xC0);          // move cursor to the start of 2nd line
	LCD_Write_String(out_str);
	LCD_Write_String("L");
	_delay_ms(100);
	LCD_cmd(0x01);          // make display ON, cursor ON
	_delay_ms(1);

}
// void display_rate(){
// 	if((int)time_clock%2==0){
// 		float temp_pulse=pulses;
//
// 		float l_minutes=(temp_pulse/time_clock)/7.5;  //calculation
// 		display(l_minutes);
// 		_delay_ms(100);          //display the liter/hour flow rate
// 	}
// }


//******************Water control valve*********************//
void water_control(){
   PORTD ^=(1<<3);
   	LCD_cmd(0x01);          // make clear LCD	
   LCD_Write_String("Water valve switched.");
   _delay_ms(100);
}



//****************** battery votage reader*********************//
void ReadVoltage(){

	LCD_cmd(0x01);          // make clear LCD	
	uint8_t vOUT = 0.0;
	uint8_t vIN = 0.0;
	uint8_t R1 = 10000.0;
	uint8_t R2 = 2000.0;
	uint8_t value = 0;
	char voltageshow [7];
	
	ADCSRA |=(1<<ADSC);                  //start the conversion
	
	while(ADIF==0);                      //Wait for to ADC is completing
	value = ADCH | ADCL<<8;              //read the result
	vOUT = (value * 5.0) / 1024.0;       //calculation the  voltage relative to 5V
	vIN = vOUT / (R2/(R1+R2));          //calculation the actual voltage
	
	LCD_Write_String ("VOLTAGE=");
	sprintf(voltageshow,"%.6lf",vIN);     //Store the vIN value to the
	LCD_Write_String(voltageshow);          //Display the value
	LCD_Write_String("V   ");
	_delay_ms(200);
	
	if (vIN<=1)
	{
		LCD_cmd(0xC0);          // move cursor to the start of 2nd line
		LCD_Write_String("Battery low!!!");    
	}

}

void timeout_display(){
	for (int i=0;i<10;i++)
	{
		LCD_Write_String ("|");
		_delay_ms(100);
	}
}