#include <AS_Nav.h>



void AS_Nav::init(AS_HILGPS * gps, AS_Mission * mission){

	_gps=gps;
	_mission=mission;

}


void AS_Nav::update(){

	if(_mission->started){

		Location actual;
		actual.lat=_gps->lat;
		actual.lng=_gps->lon;

		Location dest;
		dest.lat=_mission->actual_nav_command.latitude;
		dest.lng=_mission->actual_nav_command.longitude;

		nav_bearing = get_bearing_cd(actual,dest);
		nav_distance = get_distance(actual,dest);
	}

}
