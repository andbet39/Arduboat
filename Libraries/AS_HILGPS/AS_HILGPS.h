
#ifndef AS_HILGPS_h
#define AS_HILGPS_h

#include <stdint.h>


class AS_HILGPS
{
//variables
public:

	void init();
	
	void getSample();
	
	void setHIL(float _lat,float _lon,float _alt);


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


}; //AS_HILGPS

#endif //__AS_HILGPS_H__
