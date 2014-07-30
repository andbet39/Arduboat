

void check_nav_mode(){
	
	if(auxChannel.pwmIn>1500 && current_nav_mode!=NAV_MODE_MANUAL)
	{
		switch_mode(NAV_MODE_MANUAL);
		Serial.print("Switched to MANUAL");
	}
	
	if (auxChannel.pwmIn<1500 && current_nav_mode!=NAV_MODE_HEADHOLD)
	{
		switch_mode(NAV_MODE_HEADHOLD);
			Serial.print("Switched to AUTO");
	}
	
}


void switch_mode(int navmode){
	
	switch (navmode)
	{
		
		case NAV_MODE_MANUAL:
		
		current_nav_mode=NAV_MODE_MANUAL;
		
		break;
		
		
		case NAV_MODE_HEADHOLD:
			Serial.print("Switched to HEAD HOLD");
			nav_bearing = curr_heading;
			current_nav_mode=NAV_MODE_HEADHOLD;
			rudderChannel.fixCenterPos();
			 
			Setpoint=nav_bearing;
			break;
	}	
	
}



