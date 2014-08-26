
#include <RC_Channel.h>

#include <RC_HAL.h>

#define SERVO_MAX 1800
#define  SERVO_MIN 1200

static RC_HAL *hal;


void RC_Channel::init(uint8_t pinOut, uint8_t chIn)
{
	
	hal=RC_HAL::getInstance();
	_pinOut=pinOut;
	_chIn=chIn;
	servo.attach(_pinOut);
	
	_min=SERVO_MIN;
	_max=SERVO_MAX;
	_override=0;
	Serial.printf("Servo attahced Channel: %d \n",chIn );
	
}

void RC_Channel::setPwm(uint16_t pwm){
	
	
	if (pwm>_max)
	{
		_max=pwm;
	}
	if (pwm<_min)
	{
		_min=pwm;
	}
	
	if (pwm!=_lastPwm)
	{		
		_lastPwm=pwm;
		_pwmOut=pwm;
	}
	
	//Serial.print(pwm);
	//Serial.print("\n\r");
}

void RC_Channel::writeCurrent(){
	
//	Serial.print(_pwmOut);
	//Serial.print("\n\r");
	servo.writeMicroseconds(_pwmOut);
	
}

uint16_t RC_Channel::center(){
	
	int16_t center = _min+((_max-_min)/2);
	
	return _center;
	
}
uint16_t RC_Channel::getMin(){
	
	return _min;
	
}

uint16_t RC_Channel::getPwmOut(){
	
	return _pwmOut;
	
}
uint16_t RC_Channel::getMax(){
	
	
	return _max;
	
}


void RC_Channel::setCenter(uint16_t center){
	
	_center=center;
}

void RC_Channel::fixCenterPos(){
	_center=pwmIn;
		
}

uint16_t RC_Channel::readRadio(){
	
	pwmIn = hal->chin(_chIn);	
	_pwmIn=pwmIn;
	return pwmIn;
}

void RC_Channel::setMinMax(uint16_t min,uint16_t max){
	
}

void RC_Channel::setOverrideToPwm(int16_t override){
	
	_override=override;
	
	_pwmOut += _override;
	
	
	if (_pwmOut>_max)
	{
		_pwmOut=_max;
	}
	
	if (_pinOut<_min)
	{
		_pwmOut=_min;
	}
	
	if (_max>2000)
	{
		_max=2000;
	}
	
	if (_min<1000)
	{
		_min=1000;
	}
	
	
}