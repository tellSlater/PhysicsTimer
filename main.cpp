/*
 *	Physics Timer
 *
 *	Author: WindFish
 *
 *	Using an 8bit shift register to drive a 4 digit 7 segment display.
 *	Based on an external 16MHz crystal and the timer 0 OVF interrupt
 *	creating a precise timer to be used in physics experiments.
 *	The timer starts with a sensor0 trigger and stops with a sensor1 trigger.
 *
 *	chip ATTiny2313a
 *	
 *	PINS:
 *	PINA0 - External crystal
 *	PINA1 - External crystal
 *	PINB0 - power to digit 0
 *	PINB1 - power to digit 1
 *	PINB2 - power to digit 2
 *	PINB3 - power to digit 3
 *	PINB4 - Shift register data in
 *	PINB5 - Shift register latch
 *	PINB6 - Shift register clock
 *	PIND0 - Reading sensor 0 (triggered when low)
 *	PIND1 - Reading sensor 1 (triggered when low)
 *	PIND2 - To active beeper +
 *	PIND3 - Push button input
 *	PIND5 - Toggled on each timer0 overflow interrupt (can be used to confirm correct timer timing - toggling should be every 256us)
 */
 

#define F_CPU   16000000
#define BUAD    9600
#define BRC     ((F_CPU/16/BUAD) - 1)

#define PINBEEPER PIND2


#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

const uint8_t PROGMEM displayBitmap[] = { 0b0111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111 };	//0 to 9
uint16_t timerDisplay = 0;			//For timing display functions

volatile uint8_t timerState = 0;	//0 SET, 1 RUNNING, 2 STOPPED
volatile uint8_t timerSecs = 0;		//Seconds of timer
volatile uint16_t timerMsecs = 0;	//Milliseconds of timer
uint16_t timerusecs = 0;			//Microseconds of timer for better precision

bool buttonState = 0;				//The state of the button (1 for pushed - 0 for open)
volatile uint8_t timerButton = 0;	//Timer used to debounce the button

uint8_t sensor0buffer = 0;			//Sensor 0 buffer - triggered when >127
bool sensor0 = 0;					//Sensor 0 state
bool sensor0EN = 0;					//Sensor 0 enable
uint8_t sensor1buffer = 0;			//Sensor 1 buffer - triggered when >127
bool sensor1 = 0;					//Sensor 1 state
bool sensor1EN = 0;					//Sensor 1 enable
bool sensor0ArmedPosition = 0;		//If the sensor A is low when the timer is armed, then the timer begins when sensor A goes high and ends when sensor B goes high
bool sensor1ArmedPosition = 0;		//If the sensor A is high when the timer is armed, then the timer begins when sensor A goes low and ends when sensor B goes low									
															
uint8_t digits[] = {0, 0, 0, 0};	//Values of digits of screen

uint16_t timerSleep = 0;			//Timer used for sleeping the device after 60 seconds idle

uint8_t doBEEP = 0;					//When greater than 0, a beep will be executed and then this variable will decrement by 1 until. Used to execute BEEPS

inline void setupUART()
{
	UBRRH = (BRC >> 8);
	UBRRL =  BRC;
	UCSRB = (1 << TXEN);
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0);
}

inline void setrans()
{
	UCSRB = (1 << TXEN);
}

inline void cltrans()
{
	UCSRB = (0 << TXEN);
}

inline void setupPINS()	//Data direction registers and pull up resistor configuration
{
	GIMSK   = 0b00010000;	//Enable pin change 1 interrupts (used in later iteration of the code)
	
	DDRA	= 0b00000000;	//PINA0 sensorA input - PINA1 sensor B input
	PORTA	= 0b00000011;
	DDRB	= 0b01111111;	//PINB4..6 shift register control (4 data, 5 latch, 6 clock) - PINB0..3 digit select
	PORTB	= 0b00000000;
	DDRD	= 0b01000100;	//PIND2 beeper output - PIND3 button input - PIND0 provides vcc for  - PIND6 provides vcc for the shift register and sensors and is low during sleep - PIND5 toggled in interrupt
 	PORTD	= 0b01011000;
}

inline void setupPINSsleep()	//Data direction registers and pull up resistor configuration for sleep
{
	GIMSK   = 0b10000000;	//Enable INT1 interrupt so that the pushbutton can trigger it and wake up the chip
	PCMSK2  = 0b00000000;	//Disable sensors (used in later iteration of the code)
	
	DDRA	= 0b00000000;	//Set all pins to tri state without pull up resistors
	PORTA	= 0b00000000;
	DDRB	= 0b00000000;	
	PORTB	= 0b00000000;
	DDRD	= 0b00000000;	
	PORTD	= 0b00011000;
}

inline void enableSensor(uint8_t x) //Takes 0 or 1 as input and enables sensor 0 or 1
{
	PCMSK2  = 1 << x;
}

inline void disableSensors()		//Disables all sensors
{
	PCMSK2  = 0b00000000;
}

inline void setupTIMERS()
{
	TIMSK  = 0b00000010;   //overflow interrupt
	TCCR0B = 0b00000010;   //clock source = CLK/256 , start PWM
	TCCR0A = 0b00000011;   //PWM

	MCUCR  = 0b01110000;  //Power down mode	
}

inline void SRlatch()					//Shift register latch
{
	PORTB |= 1 << PINB5;
	_delay_us(1);
	PORTB &= ~(1 << PINB5);
	_delay_us(1);
}

inline void SRclock()					//Shift register clock data
{
	PORTB |= 1 << PINB6;
	_delay_us(1);
	PORTB &= ~(1 << PINB6);
	_delay_us(1);
}

inline void SRIN(bool x)				//Shift register input
{
	if (x) PORTB |= 1 << PINB4;
	else PORTB &= ~(1 << PINB4);
	_delay_us(1);
}

void SRwriteByte(uint8_t data) 			//Shift register write byte
{
	for (uint8_t i=0; i<8; ++i)
	{
		SRIN(data & 0x80);
		SRclock();
		data <<= 1; 
	} 
}

inline void digitDisplayed(uint8_t d)		// 8.8.8.8. OFF  -  accepts 0-4 as input and displays the said digit or turns off for 4
{											// 0 1 2 3  4
	PORTB &= 0xf0;
	PORTB |= (1 << d) & 0x0f;
}

inline void beeper(bool x)					//Changes beeper pin to 1 or 0 effectively turning the beeper on and off
{
	if (x) PORTD |= 1 << PINBEEPER;
	else PORTD &= ~(1 << PINBEEPER);
}

inline void BEEP()							//Produces a small BEEP using the beeper
{
	beeper(1);
	_delay_ms(20);
	beeper(0);
}

void introBEEPS()
{
	BEEP();
	_delay_ms(200);
	BEEP();
	_delay_ms(100);
	BEEP();
	_delay_ms(100);
	BEEP();
	_delay_ms(200);
	BEEP();
	_delay_ms(200);
}

void doBEEPS()
{
	if (doBEEP)
	{
		BEEP();
		--doBEEP;
	}
}

inline void timerReset()					//Resets the timer and sets to trigger position
{
	timerSleep = 0;
	timerState = 0;
	timerusecs = 0;
	timerMsecs = 0;
	timerSecs = 0;
}

inline void timerInit()
{
	timerState = 0;
	timerReset();
	enableSensor(0);
	sensor0ArmedPosition = sensor0;
	sensor1ArmedPosition = sensor1;
}

inline void buttonAction()
{
	timerState >= 2? timerState = 0 : ++timerState;
	if (timerState == 0)
	{
		timerReset();
		enableSensor(0);
		sensor0ArmedPosition = sensor0;
		sensor1ArmedPosition = sensor1;
	}
	else if (timerState == 1) enableSensor(1);
	BEEP();
	timerSleep = 0;
}

inline void buttonCheck(bool& bState, volatile uint8_t& timer, volatile const uint8_t* PINX, const uint8_t PINXno, const uint8_t msDebounce = 50) //Software debounced button check
{
	if (((*PINX & (1 << PINXno))) == 0)
	{
		if (!bState && timer > msDebounce)
		{
			bState = 1;
			timer = 0;
			
			//Action for button on here
			buttonAction();
		}
	}
	else
	{
		if (bState && (timer > msDebounce))
		{
			bState = 0;
			timer = 0;
			
			//Action for button off here
		}
	}
}

inline void displayTimer()
{
	static uint8_t digit = 0;
	
	uint8_t toDisplay = ~pgm_read_byte( &displayBitmap[ digits[digit] ] );
	
	if (digit == 0) toDisplay &= ~0x80;
	
	static uint8_t blinkmask = 0xff;
	if (timerState == 0)
	{
		toDisplay |= blinkmask;
		if (timerDisplay >= 790)
		{
			blinkmask ^= 0xff;
			timerDisplay = 0;
		}
	}
	
	if (digit == 2)
	{
		if (PIND & (1 << PIND1)) toDisplay |= 0x80;
		else toDisplay &= ~0x80;
	}
	else if (digit == 3)
	{		
		if (PIND & (1 << PIND0)) toDisplay |= 0x80;
		else toDisplay &= ~0x80;
	}
	
	SRwriteByte(toDisplay);
	digitDisplayed(4);
	SRlatch();
	digitDisplayed(digit);
	
	digit >= 3? digit = 0 : ++digit;
}

inline void calcTimer()
{
	digits[0] = timerSecs % 10;
	digits[1] = timerMsecs / 100;
	digits[2] = (timerMsecs % 100) / 10;
	digits[3] = timerMsecs % 10;
}

inline void readSensors()  //Used for buffered sensor input
{
	if ((PIND & (1<<PIND0)) && (sensor0buffer > 0)) --sensor0buffer;	//Incrementing buffer for sensor0
	else if (!(PIND & (1<<PIND0)) && (sensor0buffer < 4)) ++sensor0buffer;
	
	if (sensor0buffer > 2) sensor0 = true;							//Changing state of sensor0 based on buffer value
	else if (sensor0buffer < 2) sensor0 = false;
	
	if ((PIND & (1<<PIND1)) && (sensor1buffer > 0)) --sensor1buffer;	//Incrementing buffer for sensor1
	else if (!(PIND & (1<<PIND1)) && (sensor1buffer < 4)) ++sensor1buffer;
	
	if (sensor1buffer > 2) sensor1 = true;							//Changing state of sensor1 based on buffer value
	else if (sensor1buffer < 2) sensor1 = false;
	
// 	static bool done = false;				//Debugging the sensor reaction time and setting buffer sizes
// 	if (!done && (!(PIND & (1<<PIND1))))
// 	{
// 		timerState = 1;
// 		done = true;
// 	}
// 	else if (sensor1buffer == 4)
// 	{
// 		timerState = 2;
// 	}
}

inline void sensorAction()  //Used for buffered sensor input
{
	if (timerState == 0)
	{
		if ((sensor0ArmedPosition != sensor0) && sensor0EN)
		{
			timerState = 1;
			enableSensor(1);
			++doBEEP;
			timerSleep = 0;
		}
	}
	else if (timerState == 1)
	{
		if ((sensor1ArmedPosition != sensor1) && sensor1EN)
		{
			if (sensor0ArmedPosition != sensor1)
			{
				timerState = 2;
				disableSensors();
				++doBEEP;
			}
			timerSleep = 0;
			sensor1ArmedPosition = sensor1;
		}
	}
}

int main()
{
	//setupUART();	//Beeper cannot function using UART so we disable transmit with cltrans() - UART is for debuggig purposes
	setupTIMERS();
	setupPINS();
	
	CLKPR = 0x80;	//Enable prescaler change
	CLKPR = 0x01;	//Prescale clock by /2
	 
	sei();

	//cltrans();
	
	introBEEPS();
	
	timerInit();
	
	while(1)
    {
		buttonCheck(buttonState, timerButton, &PIND, PIND3);
		
		calcTimer();
		
		displayTimer();
		
		doBEEPS();
		
		//readSensors(); //Used for buffered sensor input
		
		//sensorAction(); //Used for buffered sensor input
		
		if (timerSleep > 60000)
		{
			setupPINSsleep();
			disableSensors();
			timerSleep = 0;
			sleep_mode();
			setupPINS();
			timerInit();
			introBEEPS();
		}
	}
}

ISR (TIMER0_OVF_vect) //Timer 0 overflow interrupt used for all the timing needs
{
	PORTD ^= 1 << PIND5;
	
	static uint8_t smallTimer = 0;
	++smallTimer;
	
	if (timerState == 1)
	{
		timerusecs += 256;
		timerMsecs += timerusecs/1000;
		timerSecs += timerMsecs/1000;
		timerusecs %= 1000;
		timerMsecs %= 1000;
	}	
	
	if (smallTimer>=4) //milliseconds increment here (4 * 263us)
	{
		smallTimer = 0;
		++timerButton;
		++timerDisplay;
		++timerSleep;
	}
}

ISR (PCINT2_vect) //Pin change interrupt used for reading the sensors
{
	disableSensors();
	
	if (timerState == 0)
	{
		timerState = 1;
		enableSensor(1);
		++doBEEP;
	}
	else if (timerState == 1)
	{
		if (sensor0ArmedPosition != (PIND & (1 << PIND1)))
		{
			timerState = 2;
			disableSensors();
			++doBEEP;			
		}
		else enableSensor(1);
	}
	
	timerSleep = 0;
	GIFR = 1 << PCIF2;
}


//////////////////////////////////////////////////////////////////////////

// FUN - OLD STUFF

// 	digitDisplayed(0);
// 	SRwriteByte(0xff);
// 	digitDisplayed(1);
//
// 	uint8_t cnt = 0;
// 	while(1)
// 	{
// 		SRIN(static_cast<bool>(cnt%3));
// 		SRclock();
// 		_delay_ms(500);
// 		SRlatch();
// 		_delay_ms(500);
// 		++cnt;
// 	}


// OLD PIN CHANGE ISR

// ISR (PCINT1_vect)
// {
// 	disableSensors();
// 	
// 	if (timerState == 0)
// 	{
// 		timerState = 1;
// 		enableSensor(1);
// 		doBEEP = 1;
// 	}
// 	else if (timerState == 1)
// 	{
// 		doBEEP = 1;
// 		if (sensorArmedPosition != (PINA & (1 << PINA1)))
// 		{
// 			timerState = 2;
// 			disableSensors();
// 		}
// 	}
// } 
