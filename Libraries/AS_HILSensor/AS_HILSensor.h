#import <Arduino.h>


#ifndef AS_HILSensor_h
#define AS_HILSensor_h


class AS_HILSensor {
    
public:
   
    
    // initialise scheduler
	void init(uint8_t updateMillis);
    void setHIL(float heading,float pitch,float roll);

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
