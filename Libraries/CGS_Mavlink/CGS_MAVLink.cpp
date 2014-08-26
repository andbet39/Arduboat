
#include <CGS_MAVLink.h>

//FastSerialPort0(Serial);

bool mavlink_check_target(uint8_t sysid, uint8_t compid){

    if(sysid=100 && compid==200) return true;
    return false;
}


void CGS_MAVLink::init(){

    Serial.begin(57600);

}

void CGS_MAVLink::handleMissionCountMessage(AS_Mission * mission , mavlink_message_t * msg){

    mavlink_mission_count_t count;
    mavlink_msg_mission_count_decode(msg, &count);

    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(count.target_system, count.target_component)) {
        return;
    }

    receive_count = count.count; //Totale da ricevere
    last_received = 0; //Contatore dei ricevuti
    isReceiving=true;
    Serial.print("\n Waypoint count :");
    Serial.print(receive_count);
    requestWP(last_received);

}


void CGS_MAVLink::handleMissionItemMessage(AS_Mission * mission , mavlink_message_t * msg){

    uint8_t result = MAV_MISSION_ACCEPTED;

    mavlink_mission_item_t item;
    mavlink_msg_mission_item_decode(msg, &item);

    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(item.target_system, item.target_component)) {
        return;
    }

    if (isReceiving)
    {   
            if (item.seq != last_received)
            {    result = MAV_MISSION_INVALID_SEQUENCE;
                 Serial.printf("Sequence error \n");

                 goto mission_ack;
            }

            


            Serial.printf("Received WP : %d \n",item.seq);
            last_received++;

            cmd_nav_to_wp cmd1;
            cmd1.cmd_id=item.seq;
            cmd1.latitude=item.x;
            cmd1.longitude=item.y;

            mission->addCmd(&cmd1);
            
            if (last_received >= receive_count)
            {
                Serial.printf("Sending ACK %d \n",last_received );
                isReceiving=false;
                goto mission_ack;
            }

            Serial.printf("Sending request for WP :%d \n",last_received);
            requestWP(last_received);
            return;
                
                
    }
    
    
mission_ack:
    sendMissionAck(result);



}


void CGS_MAVLink::handleMissionClearMessage(AS_Mission * mission , mavlink_message_t * msg){

    mavlink_mission_clear_all_t packet;
    mavlink_msg_mission_clear_all_decode(msg, &packet);

    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        return;
    }

    sendMissionAck(0);
}

void CGS_MAVLink::sendMissionAck(uint8_t result){


    mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_mission_ack_pack(100,200,&msg,255,0, result);
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	Serial.write(buf, len);
  
}

void CGS_MAVLink::requestWP(uint16_t wpnum){

/*
static inline uint16_t mavlink_msg_mission_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t seq)*/
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_mission_request_pack(100,200,&msg,255,0, wpnum);
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
    Serial.write(buf, len);

}



