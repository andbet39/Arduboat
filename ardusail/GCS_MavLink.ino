
#define 	INT8_MAX   0x7f
#define 	INT8_MIN   (-INT8_MAX - 1)
#define 	UINT8_MAX   (__CONCAT(INT8_MAX, U) * 2U + 1U)
#define 	INT16_MAX   0x7fff
#define 	INT16_MIN   (-INT16_MAX - 1)
#define 	UINT16_MAX   (__CONCAT(INT16_MAX, U) * 2U + 1U)

#define  USE_TELEMETRY true


void gcs_send_position(void){

 	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint32_t timestamp = millis();
	
	mavlink_msg_global_position_int_pack(100,200, &msg,timestamp, gps.lat,gps.lon,gps.alt,0,0,0,0, gps.angle);
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
#if USE_TELEMETRY	
        Serial3.write(buf, len);
#else
	Serial.write(buf, len);
#endif

}

void gcs_send_heartbeat(void){
	
  int system_type = MAV_TYPE_GENERIC;
	int autopilot_type = MAV_AUTOPILOT_GENERIC;
	uint8_t base_mode;
	uint32_t  custom_mode;
	if(current_nav_mode==NAV_MODE_HEADHOLD){
		base_mode = MAV_MODE_GUIDED_ARMED;
		custom_mode = MAV_MODE_GUIDED_ARMED;
	}else{

	        base_mode = MAV_MODE_MANUAL_ARMED;
		custom_mode = MAV_MODE_MANUAL_ARMED;
	}

        if(current_nav_mode==NAV_MODE_AUTO){
        
                base_mode = MAV_MODE_AUTO_ARMED;
		custom_mode = MAV_MODE_AUTO_ARMED;
        
        }
		uint8_t system_status = MAV_STATE_ACTIVE;


	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type,base_mode,custom_mode,system_status);
	
	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes) 
#if USE_TELEMETRY	
        Serial3.write(buf, len);
#else
	Serial.write(buf, len);
#endif
	
}



void gcs_send_attitude(){
	

	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint32_t timestamp = millis();
	float radHeading = toRadians(sensor.heading);
	float radRoll = toRadians(-sensor.roll);
	float radPitch = toRadians(-sensor.pitch);
	
	mavlink_msg_attitude_pack(100,200, &msg, timestamp,  radRoll,  radPitch, radHeading , 0, 0, 0);
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
#if USE_TELEMETRY	
        Serial3.write(buf, len);
#else
	Serial.write(buf, len);
#endif
		
	
}


void gcs_send_navcontroller(){
	

	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint32_t timestamp = millis();
	
	
	mavlink_msg_nav_controller_output_pack(100,200, &msg,  0,  0, curr_heading , nav_bearing, nav_distance, 0,0,0);
	                           
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
#if USE_TELEMETRY	
        Serial3.write(buf, len);
#else
	Serial.write(buf, len);
#endif
	
	
}
void gcs_send_servo_out(){
	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	
	mavlink_msg_servo_output_raw_pack(100,200, &msg,millis(),1,  rudderChannel.getPwmOut(), sailChannel.getPwmOut(),UINT16_MAX ,UINT16_MAX,auxChannel.getPwmOut(),  UINT16_MAX ,UINT16_MAX ,UINT16_MAX);
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
#if USE_TELEMETRY	
        Serial3.write(buf, len);
#else
	Serial.write(buf, len);
#endif
	
}
  

void gcs_send_servo_in(){

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	uint16_t range1= rudderChannel.getMax()-rudderChannel.getMin();
	uint16_t range2= sailChannel.getMax()-sailChannel.getMin();
	uint16_t range3= auxChannel.getMax()-auxChannel.getMin();
	
	
	int16_t ch1Scaled = ((rudderChannel.pwmIn-rudderChannel.getMin())*(20000/range1))-10000;
	int16_t ch2Scaled = ((sailChannel.pwmIn-sailChannel.getMin())*(20000/range2))-10000;
	int16_t ch3Scaled = ((auxChannel.pwmIn-auxChannel.getMin())*(20000/range3))-10000;

	//Serial.print("PWMIN: ");	Serial.print(ch1Scaled);

	mavlink_msg_rc_channels_scaled_pack(100,200, &msg,millis(),1,  ch1Scaled,ch2Scaled,UINT16_MAX ,UINT16_MAX,ch3Scaled,  UINT16_MAX ,UINT16_MAX ,UINT16_MAX,255);
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes)
	
#if USE_TELEMETRY	
        Serial3.write(buf, len);
#else
	Serial.write(buf, len);
#endif

}
void gcs_send_mission_current(){

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
        mavlink_msg_mission_current_pack(100,200, &msg,mission.currentId());
						       
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes)
	
#if USE_TELEMETRY	
        Serial3.write(buf, len);
#else
	Serial.write(buf, len);
#endif

}

void gcs_send_mission_reached(){

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	mavlink_msg_mission_item_reached_pack(100,200, &msg,mission.currentId());
				       
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	
#if USE_TELEMETRY	
        Serial3.write(buf, len);
#else
	Serial.write(buf, len);
#endif

}

void gcs_send_hil_control(){

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	float rudder= rudderChannel.getControl();
	float sail= sailChannel.getControl();
	float aux= auxChannel.getControl();
	mavlink_msg_hil_controls_pack(100, 200, &msg,millis(), 0, 0, rudder, sail, aux, 0,0, 0, 0, 0);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	
#if USE_TELEMETRY	
        Serial3.write(buf, len);
#else
	Serial.write(buf, len);
#endif

}


void gcs_update(){
	
	mavlink_message_t msg;
	mavlink_status_t status;

#if USE_TELEMETRY	
	while(Serial3.available() > 0 )
#else
	while(Serial.available() > 0 )
#endif

	{
	
#if USE_TELEMETRY	
            uint8_t c = Serial3.read();
#else
            uint8_t c = Serial.read();
#endif
	 

	   // Try to get a new message
	   if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
                    int gcs_mode;
			   switch (msg.msgid)
			   {
			   	case MAVLINK_MSG_ID_HEARTBEAT:
                                    // Serial.print("MAVLINK_MSG_ID_HEARTBEAT \n");

			   	break;
                                case MAVLINK_MSG_ID_SET_MODE:
                                       Serial.print("MAVLINK_MSG_ID_SET_MODE \n");
                                        gcs_mode=gcs.handleSetModeMessage(&msg);
                                        overridedMode=true;
                                        Serial.printf("Switch to %d",gcs_mode);
                                       if(gcs_mode==MAV_MODE_GUIDED_ARMED)switch_mode(NAV_MODE_HEADHOLD);
                                       if(gcs_mode==MAV_MODE_MANUAL_ARMED)switch_mode(NAV_MODE_MANUAL);
                                       if(gcs_mode==156)switch_mode(NAV_MODE_AUTO);
                                       
			        break;

                                case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                                     
                                      Serial.print("MAVLINK_MSG_ID_MISSION_CLEAR_ALL \n");
                                     gcs.handleMissionClearMessage(&mission,&msg);
			   	break;
			         case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                                     
                                      Serial.print("MAVLINK_MSG_ID_MISSION_REQUEST_LIST \n");
                                       gcs.handleMissionRequestList(&mission,&msg);
			   	break;
                                case MAVLINK_MSG_ID_MISSION_REQUEST:
                                     
                                      Serial.print("MAVLINK_MSG_ID_MISSION_REQUEST\n");
                                       gcs.handleMissionRequest(&mission,&msg);
			   	break;
                                case MAVLINK_MSG_ID_MISSION_COUNT:
                                      Serial.print("MAVLINK_MSG_ID_MISSION_COUNT \n");
			   	      gcs.handleMissionCountMessage(&mission,&msg);
                                
                                
                                break;
                                case MAVLINK_MSG_ID_MISSION_ITEM:
                                      Serial.print("MAVLINK_MSG_ID_MISSION_ITEM \n");
                                      gcs.handleMissionItemMessage(&mission,&msg);

                                
                                break;
                                case MAVLINK_MSG_ID_HIL_STATE:
                                     // Serial.print("MAVLINK_MSG_ID_HIL_STATE \n");
                                      gcs.handleHilStateMessage(&gps,&sensor ,&msg);

                                
                                break;
                                
                               
			   	default:
                                      Serial.print(msg.msgid);
                                      Serial.print("MV\n");
			   	break;
			   }
		}   // And get the next one
		   
	}
}





