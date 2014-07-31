#define 	INT8_MAX   0x7f
#define 	INT8_MIN   (-INT8_MAX - 1)
#define 	UINT8_MAX   (__CONCAT(INT8_MAX, U) * 2U + 1U)
#define 	INT16_MAX   0x7fff
#define 	INT16_MIN   (-INT16_MAX - 1)
#define 	UINT16_MAX   (__CONCAT(INT16_MAX, U) * 2U + 1U)


void gcs_send_position(void){

 /* Serial.print (gps.lon);
  Serial.print (";");
  Serial.print (gps.lat);
  Serial.print("\n\r");
  */
 
	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint32_t timestamp = millis();
	
 //avlink_msg_global_position_int_pack( system_id,  component_id, * msg,
//						        time_boot_ms,  lat,  lon,  alt,  relative_alt,  vx,  vy,  vz,  hdg)
	//mavlink_msg_gps_raw_int_pack( 100,200, &msg,timestamp,1, gps.lat,gps.lon,gps.alt,  UINT16_MAX,  UINT16_MAX,  0,  gps.angle, 12);

	mavlink_msg_global_position_int_pack(100,200, &msg,timestamp, gps.lat,gps.lon,gps.alt,0,0,0,0, gps.angle);
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);



}

void gcs_send_heartbeat(void){
	
  int system_type = MAV_TYPE_GENERIC;
	int autopilot_type = MAV_AUTOPILOT_GENERIC;
	uint8_t base_mode;
	uint32_t  custom_mode;
	if(current_nav_mode==NAV_MODE_HEADHOLD){
		base_mode = MAV_MODE_FLAG_GUIDED_ENABLED;
		custom_mode = MAV_MODE_FLAG_GUIDED_ENABLED;
		}else{

			base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
			custom_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		}
		uint8_t system_status = MAV_STATE_ACTIVE;


	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type,base_mode,custom_mode,system_status);
	
	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);;

	
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
	
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);
	
	
}

void gcs_send_servo_out(){
	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	
	mavlink_msg_servo_output_raw_pack(100,200, &msg,millis(),1,  rudderChannel.getPwmOut(), sailChannel.getPwmOut(),UINT16_MAX ,UINT16_MAX,auxChannel.getPwmOut(),  UINT16_MAX ,UINT16_MAX ,UINT16_MAX);
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes)
	Serial.write(buf, len);
	
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
	Serial.write(buf, len);

}


void gcs_update(){
	
	mavlink_message_t msg;
	mavlink_status_t status;


	while(Serial.available() > 0 )
	{
	 
            uint8_t c = Serial.read();
	   // Try to get a new message
	   if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			   // Handle message

			   switch (msg.msgid)
			   {
			   	case MAVLINK_MSG_ID_HEARTBEAT:

			   	break;

			   	case MAV_CMD_NAV_WAYPOINT:


			   	break;


			   	default:

			   	break;
			   }
		}   // And get the next one
		   
	}
}






