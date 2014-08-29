

#ifndef AS_Nav_h
#define AS_Nav_h

#include "../AS_Mission/AS_Mission.h"
#include "../AS_HILGPS/AS_HILGPS.h"


#include <AS_Math.h>
#include <AS_Common.h>


class AS_Nav {
  
public:

	void init(AS_HILGPS * gps, AS_Mission * mission);
	void update();


	uint16_t nav_bearing;
	uint32_t nav_distance;


private:

	AS_Mission * _mission;
	AS_HILGPS * _gps;



};

#endif
