


void init_navigation(){
	
	headingPID.SetMode(AUTOMATIC);
}

 void update_nav(){
	
	if(current_nav_mode==NAV_MODE_HEADHOLD){
		
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


