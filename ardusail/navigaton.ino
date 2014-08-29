


void init_navigation(){
	
	headingPID.SetMode(AUTOMATIC);
}

 void update_nav(){
	


        if(current_nav_mode==NAV_MODE_AUTO){
            navigator.update();
            nav_bearing = navigator.nav_bearing/100;
            nav_distance= navigator.nav_distance;
            
            
            if(nav_distance<WP_RADIUS){
              
              gcs_send_mission_reached();
                mission.reachedCurrent();
              gcs_send_mission_current();
              
            }
        
        }
		
        if(current_nav_mode>0){
		
		int errorHeading = nav_bearing-curr_heading;
		
		if(errorHeading>180 || errorHeading <-180){
			if(errorHeading>180){
				Setpoint=nav_bearing-360;
			}
			if(errorHeading<-180){
				Setpoint=nav_bearing+360;
			}
			
		}else{
			
			Setpoint=nav_bearing;
			
		}
		Input=curr_heading;
		
		headingPID.Compute();
		
		
		int16_t center=rudderChannel.center();
		
		//Serial.print(" OUT: ");Serial.print(Output);Serial.print(" ERR: ");Serial.print(Input-Setpoint);Serial.print(" CUR_HD: ");Serial.print(Input);Serial.print(" TO_HD: ");Serial.print(Setpoint);Serial.print("\n\r");
		rudderChannel.setPwm(center-Output);
	}
	
}


