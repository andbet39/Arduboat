/* 
* RC_HAL.h
*
* Created: 18/07/2014 14:38:56
* Author: andrea.terzani
*/


#ifndef __RC_HAL_H__
#define __RC_HAL_H__
#include <Arduino.h>
#include <Servo/Servo.h>
#include <stdint.h>


#define P1IN 2
#define P2IN 3
#define P3IN 4

#define MAX_CHANNEL 3

#define CH1_FLAG 1
#define CH2_FLAG 2
#define CH3_FLAG 4

class RC_HAL
{

//functions
public:

	    static RC_HAL &shared_instance() {static RC_HAL hal; return hal;}

		
		
		
private:

	static bool instanceFlag;
	static RC_HAL *single;
	
	uint16_t _ch1pwmIn;
	uint16_t _ch2pwmIn;
	uint16_t _ch3pwmIn;
	
	RC_HAL()
	{
		//private constructor
	}
public:
	static RC_HAL* getInstance();

	

	void read_all() ;
	void init() ;

	uint16_t chin(uint8_t ch) ;

	~RC_HAL()
	{
		instanceFlag = false;
	}
		

}; //RC_HAL


#endif //__RC_HAL_H__
