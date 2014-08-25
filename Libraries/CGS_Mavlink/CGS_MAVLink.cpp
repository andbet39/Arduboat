
#include <CGS_MAVLink.h>

//FastSerialPort0(Serial);

void CGS_MAVLink::init(){

    Serial.begin(115200);

}

void CGS_MAVLink::handleMissionCountMessage(AS_Mission * mission , mavlink_message_t * msg){

}

bool mavlink_check_target(uint8_t sysid, uint8_t compid){

	if(sysid=100 && compid==200) return true;
	return false;
}

void CGS_MAVLink::handleMissionClearMessage(AS_Mission * mission , mavlink_message_t * msg){

    mavlink_mission_clear_all_t packet;
    mavlink_msg_mission_clear_all_decode(msg, &packet);

    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        return;
    }

    sendMissionAck();
}

void CGS_MAVLink::sendMissionAck(){



    mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_mission_ack_pack(100,200,&msg,255,255, MAV_RESULT_ACCEPTED);
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	Serial.write(buf, len);
  
}




