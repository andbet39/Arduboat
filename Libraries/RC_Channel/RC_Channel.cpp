
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
	_pwmOut=_min+((_max-_min)/2);

	Serial.printf("Servo attahced Channel: %d \n",chIn );
	
}

void RC_Channel::setPwm(uint16_t pwm){
	
	
	if (pwm>_max && pwm <2000)
	{
		_max=pwm;
	}
	if (pwm<_min && pwm >1000)
	{
		_min=pwm;
	}
	

	if(pwm<2000 && pwm>1000){	
		_pwmOut=pwm;
		_last_pwm=pwm;
	}





}

void RC_Channel::writeCurrent(){
	
	if (abs(_pwmOut-_last_pwm) > _dead_zone)
	{
		//Serial.printf("Servo OUT :%d \n",_pwmOut);
		servo.writeMicroseconds(_pwmOut);
		_last_pwm=_pwmOut;
	}
	
}

void RC_Channel::setDeadZone(uint16_t dead_zone){
	
		_dead_zone=dead_zone;
	
}

uint16_t RC_Channel::center(){
	
	int16_t center = _min+((_max-_min)/2);
	_center=center;
	return center;

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

float RC_Channel::getControl(){

	uint16_t range=_max-_min;
	float ratio = 2.0/range; 

	return -1+(_pwmOut-_min)*ratio;


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