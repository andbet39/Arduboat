
#include "AS_HILSensor.h"

//#define USE_SENSOR YES

 

void AS_HILSensor::init(uint8_t updateMillis){

 
	_updateTime = updateMillis;
	_lastUpdateMillis = millis();
	_elapsed=0;

	startupSensor();
}

    void AS_HILSensor::setHIL(float _heading,float _pitch,float _roll){

    	heading=_heading;
    	pitch=_pitch;
    	roll=_roll;
    }


bool AS_HILSensor::getSample(){

	_elapsed=millis()-_lastUpdateMillis;
	
	if(_elapsed>_updateTime ){
		
		if(_connected){
			

		}
		_lastUpdateMillis=millis();
		
		return true;
	}

	return false;
}



void AS_HILSensor::startupSensor(){

	bool problem=false;

	Serial.print("Hil Sensor starting up...\n\r");

			Serial.print("Hil Sensor started...\n\r");
			_connected=true;

	}