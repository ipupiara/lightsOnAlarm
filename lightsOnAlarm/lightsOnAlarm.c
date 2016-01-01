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

void doorSensorTrigger();

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
	doorSensorTrigger();
}

//  buzzer pwm   output   approx.  4kHz   PB0


void startBuzzer()
{
	TCCR0B  |=  ((1 << CS01) | (1 << CS00)) ;      // set prescaler to 64
}

void stopBuzzer()
{
	TCCR0B  &=  ~((1 << CS02) |  (1 <<  CS01 ) | (1 << CS00) )   ;	//  clear any prescaler
}

void initBuzzer()
{
	// buzzer using pwm on timer 2
	TCCR0A = ((1 << WGM00) |  (1 <<  WGM01)  |  (1 << COM0A1))  ;
	TCCR0B = (1  << WGM02)  ;
	OCR0A =  0x22;      // top value
	OCR0B =  0x11;      //  square wave of approx 5k  at prescaler  64
	DDRB  |=  (1 <<  DDB1 );
}

/////////////////////////    end  buzzer code  ////////////////////////////


//   relais   output    PB4

void initRelais()
{
	PORTB &= ~(1<< PB4);
	DDRB |=  (1 <<  DDB4);
}

void setRelaisOn()
{
	PORTB |= (1 << PB4);
}   

void setRelaisOff()
{
	PORTB &= ~(1<< PB4);
}

void initHW()
{
	sei();
	initSensorPort();
	initBuzzer();
	initRelais();
	sei();
}


void doorSensorTrigger()
{
	
}

int main(void)
{
	initHW();
    while(1)
    {
        //TODO:: Please write your application code 
    }
}