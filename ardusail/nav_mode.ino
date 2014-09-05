

void check_nav_mode(){
	
	if(auxChannel.pwmIn>1500 && current_nav_mode!=NAV_MODE_MANUAL &! overridedMode)
	{
		switch_mode(NAV_MODE_MANUAL);
	}
	
	if (auxChannel.pwmIn<1500 && current_nav_mode!=NAV_MODE_HEADHOLD &! overridedMode)
	{
		switch_mode(NAV_MODE_HEADHOLD);
	}
	
}


void switch_mode(int navmode){
	
	switch (navmode)
	{
		
		case NAV_MODE_MANUAL:
		
		      current_nav_mode=NAV_MODE_MANUAL;
		      Serial.print("Switched to MANUAL\n");

		break;
		
		
		case NAV_MODE_HEADHOLD:
			Serial.print("Switched to HEAD HOLD");
			nav_bearing = curr_heading;
			current_nav_mode=NAV_MODE_HEADHOLD;
			rudderChannel.fixCenterPos();
			headingPID.ResetI();
			Setpoint=nav_bearing;
		break;

                case NAV_MODE_AUTO:
                        Serial.print("\n Switched to MISSION");
                        current_nav_mode=NAV_MODE_AUTO;
                        			
                       headingPID.ResetI();

                        mission.start();
                        
                        
                break;
	}	
	
}



