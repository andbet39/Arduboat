#include <FastSerial.h>
#include <stdint.h>
#import <Arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_GPS.h>


#ifndef AS_Sensor_h
#define AS_Sensor_h


class AS_Sensor {
    
public:
   
    
    // initialise scheduler
	void init(uint8_t updateMillis);
    
    bool getSample();
	
	float heading;
	float pitch;
	float roll;
	
    
private:
    

    void startupSensor();
    uint8_t _updateTime;
    uint32_t _lastUpdateMillis;
    uint32_t _elapsed;
	
	bool _connected;

	
    
};

#endif
