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

// hardware events 
// interrupt variables

int8_t doorOpened;
int8_t doorClosed;
int8_t sec3Tick;

//  sec3 Timer  timer 1

void setDoorVariables();

void initTimer1()
{
	TCCR1 = (1 << CTC1) ;    // ctc mode with ocr1c, timer stopped 
	OCR1C = 162;             // will give approx 3 ctc events per second
	TCNT1 = 0x00;
	TIMSK |= (1<< TOIE1);
	PLLCSR &=  ~( 1 << PCKE);  // use system clock as clock source for timer1 
	sec3Tick = 0;
}

ISR(TIMER1_OVF_vect)
{
	sec3Tick = 1;
	setDoorVariables();
}

void startTimer1()
{
	TCCR1 |=  ((1 << CS13) | (1 << CS12)  | (1 << CS11)  | (1 << CS10)) ; // start prescaler 16384
}

void stopTimer1()
{
	TCCR1 &=  ~((1 << CS13) | (1 << CS12)  | (1 << CS11)  | (1 << CS10))  ; // clear any prescaler bit
}

//  sensor port query and trigger   PB1


void initSensorPort()
{
	MCUCR  &=  ~(1<< PUD);   // enable pullup
	DDRB  &=   ~(1<< PB1);   //  PB1 as input
	PORTB  |=  (1<< PB1 );   //  set pullup high 
	
	GIMSK  |=  (1<<PCIE) ;   // enable interrupt on port B
	PCMSK  |=  (1<<PCINT1);	 // enable interrupt on PINB1
	doorClosed = 0;
	doorOpened = 0;
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

void setDoorVariables()
{
	if (isAnyDoorOpen()) {
		doorOpened = 1;
	} else if (areBothDoorsClosed()) {
		doorClosed = 1;
	}  
}

ISR (PCINT0_vect)
{
	setDoorVariables();
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
	initTimer1();
//	startTimer1();
	sei();
}



////////////////////

// software events

enum eEventTypes
{
	evNone = 0,
	evDoorsOpen,
	evDoorsClosed,
	evSeconds3Tick
};

enum stateTypes
{
	alarmState = 1,
	idleState	
};


typedef struct {
	int8_t currentState;    
	int8_t alarmSequence;
	int16_t delayCounter;
} alarmStateDataType;

alarmStateDataType currentAlarmStateData;

void setIdleAlarmStateData()
{
	currentAlarmStateData.currentState = idleState;
	currentAlarmStateData.alarmSequence = 0;
	currentAlarmStateData.delayCounter = 0;
}

void setAlarmStateData()
{
	currentAlarmStateData.currentState = alarmState;
	currentAlarmStateData.alarmSequence = 1;
	currentAlarmStateData.delayCounter = 2;
}


int8_t  isInAlarmState()
{
	int8_t res = 0;
	if (currentAlarmStateData.currentState == alarmState) {
		res = 1;
	}
	return res;
}


int8_t getNextEvent()
{
	int8_t res = 0; 
	if  (doorClosed) && (!isInAlarmState()) )){
		res = evDoorsClosed;
	} else if ((doorOpened) && (isInAlarmState())) {
		res = evDoorsOpen;
	}  else if (sec3Tick) {
		res = evSeconds3Tick;
	}  
	doorClosed = 0;
	doorOpened = 0;
	sec3Tick = 0;
	return res;
}

void handleEvent(int8_t ev)
{
	
}


int main(void)
{
	int8_t evnt;
	initHW();
	setIdleAlarmStateData();
    while(1)
    {
		evnt = getNextEvent();
		handleEvent(evnt);
		// any sleep or idle mode .... ? maybe possible and senseful ?   
    }
}