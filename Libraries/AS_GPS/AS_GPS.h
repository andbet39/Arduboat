
#ifndef AS_GPS_h
#define AS_GPS_h

#include <Adafruit_GPS.h>

#include <stdint.h>






class AS_GPS
{
//variables
public:

	void init();
	
	void getSample();

	 int hour;
	int minute;
	int seconds;
	int milliseconds;
	
	int fix;
	int fixquality;
	
	int32_t lat;
	int32_t lon;
	
	uint16_t vel;
	uint16_t alt;
	int satellites;
	
	uint16_t angle;



private:


}; //AS_GPS

#endif //__AS_GPS_H__
