
#include <CGS_MAVLink.h>


bool mavlink_check_target(uint8_t sysid, uint8_t compid){

    if(sysid=100 && compid==200) return true;
    return false;
}


void CGS_MAVLink::init(){

    Serial.begin(57600);
    Serial3.begin(57600);

    _sendingParameter=false;
    
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
    
    mission->clear();

    Serial.print("\n Waypoint count :");
    Serial.print(receive_count);
    requestWP(last_received);

}

uint8_t CGS_MAVLink::handleSetModeMessage(mavlink_message_t  * msg){

    mavlink_set_mode_t mode;
    mavlink_msg_set_mode_decode(msg, &mode);

    return mode.base_mode;

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
            
            mission->mavLinkCmdToMission(&item,&cmd1);
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

    mission->clear();
    sendMissionAck(0);
}


void CGS_MAVLink::handleMissionRequestList(AS_Mission * mission , mavlink_message_t * msg){
    
    mavlink_mission_request_list_t packet;
    mavlink_msg_mission_request_list_decode(msg, &packet);

    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        return;
    }

    isSending=true;
    last_sended=0;
    
    sendMissionCount(mission);

}


void CGS_MAVLink::handleMissionRequest(AS_Mission * mission , mavlink_message_t * msg){
        

        uint8_t result = MAV_MISSION_ACCEPTED;

    mavlink_mission_request_t item;
    mavlink_msg_mission_request_decode(msg, &item);

    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(item.target_system, item.target_component)) {
        return;
    }

    if (isSending)
    {
            sendMissionItem(mission,item.seq);
            last_sended++;


    }

mission_ack:
    sendMissionAck(result);


}

void CGS_MAVLink::handleHilStateMessage(AS_HILGPS * gps ,AS_HILSensor * sensor, mavlink_message_t * msg){

    mavlink_hil_state_t packet;
    mavlink_msg_hil_state_decode(msg, &packet);

/*
    // exit immediately if this command is not meant for this vehicle
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        return;
    }
*/
    gps->setHIL(packet.lat,packet.lon,packet.alt);
    sensor->setHIL(packet.yaw,packet.pitch,packet.roll);

}

void CGS_MAVLink::handleHilStateMessage(AS_GPS * gps ,AS_Sensor * sensor, mavlink_message_t * msg){

  
}



void CGS_MAVLink::sendMissionItem(AS_Mission * mission,uint16_t cmd_num)
{

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    cmd_nav_to_wp cmd1;

    mission->loadCommandFromEEprom(&cmd1,cmd_num);

    uint8_t current=0;
    if(mission->currentId()==cmd1.cmd_id){
        current=1;
    }

    mavlink_msg_mission_item_pack(100,200,&msg,255,0,cmd1.cmd_id, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, current, 0, 0, 0, 0, 0, cmd1.latitude/ 1.0e7f, cmd1.longitude/ 1.0e7f, 0);
    
    Serial.printf("Sending WP %d \n",cmd1.cmd_id);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
#if USE_TELEMETRY   
        Serial3.write(buf, len);
#else
        Serial.write(buf, len);
#endif  

}



void CGS_MAVLink::sendMissionAck(uint8_t result){


    mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_mission_ack_pack(100,200,&msg,255,0, result);
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
#if USE_TELEMETRY   
        Serial3.write(buf, len);
#else
    Serial.write(buf, len);
#endif  
}

void CGS_MAVLink::sendMissionCount(AS_Mission * mission){


    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
/*static inline uint16_t mavlink_msg_mission_count_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t count)*/
    mavlink_msg_mission_count_pack(100,200,&msg,255,0, mission->loadedCommand());
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
#if USE_TELEMETRY   
        Serial3.write(buf, len);
#else
    Serial.write(buf, len);
#endif  
}




void CGS_MAVLink::requestWP(uint16_t wpnum){

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_mission_request_pack(100,200,&msg,255,0, wpnum);
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
#if USE_TELEMETRY   
        Serial3.write(buf, len);
#else
        Serial.write(buf, len);
#endif
}



void CGS_MAVLink::handleParameters_request_list(mavlink_message_t * msg){

    if(!_sendingParameter){
        mavlink_param_request_list_t packet;
        mavlink_msg_param_request_list_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system,packet.target_component)) {
            return;
        }

        // Start sending parameters - next call to ::update will kick the first one out
        _last_sent_parameter = 0;
        _params_to_send=AS_Param::count();
        _sendingParameter=true;
    }
}

void CGS_MAVLink::handleParam_set_Message(mavlink_message_t * msg){


    AS_Param *vp;
    enum as_var_type var_type;

    mavlink_param_set_t packet;
    mavlink_msg_param_set_decode(msg, &packet);
    Serial.printf("Receved key : %s\n",packet.param_id );
          
    char key[MAX_PARAM_NAME_SIZE+1];
    strncpy(key, (char *)packet.param_id, MAX_PARAM_NAME_SIZE);
    key[MAX_PARAM_NAME_SIZE] = 0;


    // find the requested parameter
    vp = AS_Param::find(key, &var_type);

    if (NULL != vp){  
         float rounding_addition = 0.00;
        
  
        if (var_type == AS_PTYPE_FLOAT) {
            ((AS_Float *)vp)->set_and_save(packet.param_value);
        } else if (var_type == AS_PTYPE_INT32) {
            if (packet.param_value < 0) rounding_addition = -rounding_addition;
            float v = packet.param_value+rounding_addition;
            v = constrain_float(v, -2147483648.0, 2147483647.0);

            ((AS_Int32 *)vp)->set_and_save(v);
        } else if (var_type == AS_PTYPE_INT16) {
            if (packet.param_value < 0) rounding_addition = -rounding_addition;
            float v = packet.param_value+rounding_addition;
            v = constrain_float(v, -32768, 32767);

            Serial.printf("Received : %f of type AS_PTYPE_INT16 \n ",packet.param_value);

            ((AS_Int16 *)vp)->set_and_save(v);
        } else if (var_type == AS_PTYPE_INT8) {
            if (packet.param_value < 0) rounding_addition = -rounding_addition;
            float v = packet.param_value+rounding_addition;
            v = constrain_float(v, -128, 127);

            ((AS_Int8 *)vp)->set_and_save(v);
        } else {
            // we don't support mavlink set on this parameter
            return;
        }

        Serial.printf("Received : %f of type :%d \n ",vp->cast_to_float(var_type),toMAV_ParamType(var_type) );

        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        
        float value =  vp->cast_to_float(var_type);

        mavlink_msg_param_value_pack(100,200,&msg, key,vp->cast_to_float(var_type),toMAV_ParamType(var_type),AS_Param::count(),-1 );
    
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
        #if USE_TELEMETRY   
                Serial3.write(buf, len);
        #else
                Serial.write(buf, len);
        #endif 
    }

}


    
void CGS_MAVLink::sendParameter(uint16_t idx){

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    enum as_var_type p_type;
    
    AS_Param * vp = AS_Param::getByIndex(idx,&p_type);

    float value =  vp->cast_to_float(p_type);

    Serial.printf("Value : %f of type :%d \n ",value,toMAV_ParamType(p_type) );
    char param_name[MAX_PARAM_NAME_SIZE+1];

    vp->copy_name_token(param_name,MAX_PARAM_NAME_SIZE);
        
    //mavlink_msg_param_value_pack(100,200,&msg, param_name,value,toMAV_ParamType(p_type) ,_params_to_send,idx );
    mavlink_msg_param_value_pack(100,200,&msg, param_name,value,9,_params_to_send,idx );

    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
#if USE_TELEMETRY   
        Serial3.write(buf, len);
#else
        Serial.write(buf, len);
#endif  
}


void CGS_MAVLink::update(){

if( _sendingParameter){
    if(_last_sent_parameter!=_params_to_send ){

        sendParameter(_last_sent_parameter);
        Serial.printf("Send param idx: %d \n",_last_sent_parameter );

        _last_sent_parameter++;
    }

    if(_last_sent_parameter >= _params_to_send ){
        _sendingParameter=false;
        Serial.printf("Done sending parameter \n");
    }
}
}



uint8_t CGS_MAVLink::toMAV_ParamType(enum as_var_type type){

    switch (type) {
        case AS_PTYPE_INT32:
            return MAVLINK_TYPE_INT32_T;
        case AS_PTYPE_INT16:
            return MAVLINK_TYPE_INT16_T;
        case AS_PTYPE_UINT32:
            return MAVLINK_TYPE_UINT32_T;
        case AS_PTYPE_FLOAT:
            return MAVLINK_TYPE_FLOAT;
        case AS_PTYPE_INT8:
            return MAVLINK_TYPE_INT8_T;
        case AS_PTYPE_UINT8:
            return MAVLINK_TYPE_UINT8_T;
    }

    return MAVLINK_TYPE_FLOAT;
}


