
#include "AS_Sensor.h"

//#define USE_SENSOR YES

Adafruit_9DOF                _dof;
Adafruit_LSM303_Accel_Unified _accel;
Adafruit_LSM303_Mag_Unified   _mag ;


void AS_Sensor::init(uint8_t updateMillis){

	_dof   = Adafruit_9DOF();
	_accel = Adafruit_LSM303_Accel_Unified(30301);
	_mag   = Adafruit_LSM303_Mag_Unified(30302);

	_updateTime = updateMillis;
	_lastUpdateMillis = millis();
	_elapsed=0;

	startupSensor();
}



bool AS_Sensor::getSample(){

	_elapsed=millis()-_lastUpdateMillis;
	
	if(_elapsed>_updateTime ){
		
		if(_connected){
			//Se è passato il giusto tempo manda update
			sensors_event_t accel_event;
			sensors_event_t mag_event;
			sensors_vec_t   orientation;

			/* Calculate pitch and roll from the raw accelerometer data */
			_accel.getEvent(&accel_event);
			if (_dof.accelGetOrientation(&accel_event, &orientation))
			{

			}
			_mag.getEvent(&mag_event);
			if (_dof.fusionGetOrientation ( &accel_event, &mag_event, &orientation ))
			{
				heading=orientation.heading;
				pitch=orientation.pitch;
				roll=orientation.roll;

			}

		}
		_lastUpdateMillis=millis();
		
		return true;
	}

	return false;
}



void AS_Sensor::startupSensor(){

	bool problem=false;

	Serial.print("Sensor starting up...\n\r");
	if(!_accel.begin())
	{
		/* There was a problem detecting the LSM303 ... check your connections */
		Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!\n\r"));
		problem=true;
	}
	if(!_mag.begin())
	{
		/* There was a problem detecting the LSM303 ... check your connections */
		Serial.println("Ooops, no LSM303 detected ... Check your wiring!\n\r");
		problem=true;
	}
	if(problem){
		Serial.print("Error initializing sensor\r\n");
		_connected=false;
		}else{
			Serial.print("Sensor started...\n\r");
			_connected=true;
		}

	}