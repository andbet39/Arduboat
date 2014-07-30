
#include <CGS_MAVLink.h>

//FastSerialPort0(Serial);

void CGS_MAVLink::init(){
	
	
	
	Serial.begin(57600);

}


//mavlink_msg_gps_raw_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)

void CGS_MAVLink::send_gps_raw_int(uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible){
	
	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	
	
	mavlink_msg_gps_raw_int_pack(100, 200, &msg,millis(),  fix_type, lat,  lon, alt,  eph,  epv,  vel,  cog,  satellites_visible);

// Copy the message to send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

// Send the message (.write sends as bytes)
	Serial.write(buf, len);
}


void CGS_MAVLink::send_heartbeat(){

		// Define the system type (see mavlink_types.h for list of possible types) 
	int system_type = MAV_TYPE_GENERIC;
	int autopilot_type = MAV_AUTOPILOT_GENERIC;

	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = MAV_MODE_FLAG_GUIDED_ENABLED;
	
	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	// Pack the message
	// mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
	//mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->type, heartbeat->autopilot, heartbeat->base_mode, heartbeat->custom_mode, heartbeat->system_status);

	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type,base_mode,custom_mode,system_status);
	
	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);
	


}


void send_attitude(float yaw, float pitch, float roll){

		
	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint32_t timestamp = millis();

	mavlink_msg_attitude_pack(100,200, &msg, timestamp,  roll,  pitch,  yaw, 0, 0, 0);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);
	

}


