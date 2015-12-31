/*
 * lightsOnAlarm.c
 *
 * Created: 31.12.2015 10:56:01
 *  Author: mira
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

//  sensor port query and trigger   PB1

void initSensorPort()
{
	MCUCR  &=  ~(1<< PUD);   // enable pullup
	DDRB  &=   ~(1<< PB1);   //  PB1 as input
	PORTB  |=  (1<< PB1 );   //  set pullup high 
	
	GIMSK  |=  (1<<PCIE) ;   // enable interrupt on port B
	PCMSK  |=  (1<<PCINT1);	 // enable interrupt on PINB1
}

int8_t areBothDoorsClosed()
{
	int8_t res;
	res =  (PINB & (1<< PINB1));
	return res;
}

int8_t isAnyDoorOpen()
{
	int8_t res;
	res = ! areBothDoorsClosed();
	return res;
}

ISR (PCINT0_vect)
{
	
}

//  buzzer pwm   output   approx.  4kHz   PB0

void initBuzzer()
{
	
}


//   relais   output    PB4
   

void initHW()
{
	sei();
	initSensorPort();
	
	sei();
}



#include <avr/io.h>

int main(void)
{
    while(1)
    {
        //TODO:: Please write your application code 
    }
}