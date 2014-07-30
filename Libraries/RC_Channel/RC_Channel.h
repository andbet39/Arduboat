#include <FastSerial.h>

#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>


#ifndef RC_Channel_h
#define RC_Channel_h


class RC_Channel {
 
 public:
  
    Servo servo;
  
	void init(uint8_t pinOut,uint8_t chIn);
	void setPwm(uint16_t pwm);
	void setMinMax(uint16_t min,uint16_t max);
	void writeCurrent();
	uint16_t pwmIn;
	void setOverrideToPwm(int16_t override);
	
	uint16_t getMin();
	uint16_t getMax();
	uint16_t getPwmOut();
	
	uint16_t readRadio();

	uint16_t center();
	void setCenter(uint16_t center);
	void fixCenterPos();
	uint16_t _pwmOut;
private:

	uint16_t timer;
	int angle;
	int dir;
	uint16_t _lastPwm;
	uint8_t _chIn;
	uint8_t _pinOut;
	uint16_t _pwmIn;

	uint16_t _override;
	uint16_t _center;
	uint16_t	_min;
	uint16_t _max;
};

#endif
