
#include "PinChangeInt/PinChangeInt.h"
#import <RC_HAL.h>


static int pinCh1In;
static int pinCh2In;
static int pinCh3In;

uint32_t ulCh1Start;
uint32_t ulCh2Start;
uint32_t ulCh3Start;

volatile uint16_t unCh1InShared;
volatile uint16_t unCh2InShared;
volatile uint16_t unCh3InShared;

volatile uint8_t bUpdateFlagsShared;

void calcCh1();
void calcCh2();
void calcCh3();

bool RC_HAL::instanceFlag = false;
RC_HAL* RC_HAL::single = NULL;

RC_HAL* RC_HAL::getInstance()
{
	if(! instanceFlag)
	{
		single = new RC_HAL();
		instanceFlag = true;
		return single;
	}
	else
	{
		return single;
	}
}


void RC_HAL::init()  {
	
	PCintPort::attachInterrupt(P1IN,  calcCh1 , CHANGE);
	PCintPort::attachInterrupt(P2IN,  calcCh2 , CHANGE);
	PCintPort::attachInterrupt(P3IN,  calcCh3 , CHANGE);
	
	
}

uint16_t RC_HAL::chin(uint8_t ch)  {
	
	
	if (ch>MAX_CHANNEL)
	{
		return 0;
	}
		
		switch (ch){
			case 1:
				return _ch1pwmIn;
				break;
			case 2:
				return _ch2pwmIn;
				break;
			case 3:
				return _ch3pwmIn;
				break;
		}
	
}

void RC_HAL::read_all() {


	static int16_t unCh1In;
	static int16_t unCh2In;
	static int16_t unCh3In;
	
	static uint8_t bUpdateFlags;

	if(bUpdateFlagsShared)
	{
		//noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

		// take a local copy of which channels were updated in case we need to use this in the rest of loop
		bUpdateFlags = bUpdateFlagsShared;
		
		
		//Scambia da varibili share a locali
		if(bUpdateFlags & CH1_FLAG)
		{
			unCh1In = unCh1InShared;
		}
		if(bUpdateFlags & CH2_FLAG)
		{
			unCh2In = unCh2InShared;
			
		}
		if(bUpdateFlags & CH3_FLAG)
		{
			unCh3In = unCh3InShared;
		}
		
		//interrupts();
		
		//Assegna il valore al servo
		if(bUpdateFlags & CH1_FLAG)
		{
			_ch1pwmIn=unCh1In;
			
	//		s.writeMicroseconds(unCh1In);
			
			
		}
		//Assegna il valore al servo
		if(bUpdateFlags & CH2_FLAG)
		{
			_ch2pwmIn=unCh2In;
//			s2.writeMicroseconds(unCh2In);
		
		}
		//Assegna il valore al servo
		if(bUpdateFlags & CH3_FLAG)
		{
			_ch3pwmIn=unCh3In;
	//	    s3.writeMicroseconds(unCh3In);
		}
		
		bUpdateFlags = 0;
	}
	
}


void calcCh1(void)
{
	
	// if the pin is high, its a rising edge of the signal pulse, so lets record its value
	if(digitalRead(P1IN) == HIGH)
	{
		ulCh1Start = micros();
	}
	else
	{
		
		unCh1InShared = (uint16_t)(micros() - ulCh1Start);
		bUpdateFlagsShared |= CH1_FLAG;

	}
}

void calcCh2(void)
{
	
	// if the pin is high, its a rising edge of the signal pulse, so lets record its value
	if(digitalRead(P2IN) == HIGH)
	{
		ulCh2Start = micros();
	}
	else
	{
		
		unCh2InShared = (uint16_t)(micros() - ulCh2Start);
		bUpdateFlagsShared |= CH2_FLAG;

	}
}
void calcCh3(void)
{
	
	// if the pin is high, its a rising edge of the signal pulse, so lets record its value
	if(digitalRead(P3IN) == HIGH)
	{
		ulCh3Start = micros();
	}
	else
	{
		
		unCh3InShared = (uint16_t)(micros() - ulCh3Start);
		bUpdateFlagsShared |= CH3_FLAG;

	}
}

