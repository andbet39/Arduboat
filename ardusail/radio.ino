


void read_radio(){

	RC_HAL *hal=RC_HAL::getInstance();
	hal->read_all();
		
		int pwm =  rudderChannel.readRadio();
		int pwm2 = sailChannel.readRadio();
		int pwm3 = auxChannel.readRadio();
		//Serial.print(pwm);Serial.print(" : ");Serial.print(pwm3);
		//Serial.print("\n\r");

	
	if(current_nav_mode==NAV_MODE_HEADHOLD){
		
			sailChannel.setPwm(pwm2);
			auxChannel.setPwm(pwm3);
	}else{
		
		// SE SIAMO IN MANUAL RIPORTA TUTTI I PWM COME LI LEGGE ( VANNO CMQ SCRITTI )
			
			rudderChannel.setPwm(pwm);
			sailChannel.setPwm(pwm2);
			auxChannel.setPwm(pwm3);
	}
 
	
}

void write_radio(){

	rudderChannel.writeCurrent();
	sailChannel.writeCurrent();
	auxChannel.writeCurrent();
}


