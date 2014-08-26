/*
 * ASMission.cpp
 *
 *  Created on: Aug 22, 2014
 *      Author: aterzani
 */

#include "AS_Mission.h"


void AS_Mission::init(){

	last_inserted_cmd=0;
 	memory_count = eeprom_read_word(( uint16_t *)COUNT_ADDRESS);	
 	int i=0;



}


void AS_Mission::DebugPrint(){

	Serial.printf(" There is %d WP in memory\n",memory_count );

}

void AS_Mission::addCmd(cmd_nav_to_wp * cmd){

	eeprom_write_block((const void*)cmd, (void*)(BASE_ADDRESS + last_inserted_cmd * sizeof(cmd_nav_to_wp)), sizeof(cmd_nav_to_wp));
    eeprom_write_word(( uint16_t *)COUNT_ADDRESS,last_inserted_cmd+1);		
	Serial.printf("Saved on eeprom %d \n",last_inserted_cmd+1 );

	last_inserted_cmd++;



}


uint16_t AS_Mission::loadedCommand(){

 	memory_count = eeprom_read_word(( uint16_t *)COUNT_ADDRESS);
	
	Serial.printf("Command on eeprom %d \n",memory_count );

	return memory_count;

}

void AS_Mission::start(){

	current_cmd=0;
}

void AS_Mission::clear(){

	current_cmd=0;
	last_inserted_cmd=0;
	 
	eeprom_write_word(( uint16_t *)COUNT_ADDRESS ,0);		
	memory_count=0;
}

void AS_Mission::nextCmd(cmd_nav_to_wp * cmd){


	eeprom_read_block((void*)cmd, (void*)(BASE_ADDRESS+current_cmd*sizeof(cmd_nav_to_wp)), sizeof(cmd_nav_to_wp));

	current_cmd++;

}


void AS_Mission::loadCommandFromEEprom(cmd_nav_to_wp * cmd,uint16_t cmd_num)
{
	//TODO : check su numero di comando richiesto
	eeprom_read_block((void*)cmd, (void*)(BASE_ADDRESS+cmd_num*sizeof(cmd_nav_to_wp)), sizeof(cmd_nav_to_wp));

}
void AS_Mission::stop(){


}

void AS_Mission::mavLinkCmdToMission(mavlink_mission_item_t * orig,cmd_nav_to_wp * dest){

		dest->latitude = 1.0e7f * orig->x;
		dest->longitude = 1.0e7f * orig->y;
		dest->cmd_id = orig->seq;
	
}


void AS_Mission::missionToMavLink(mavlink_mission_item_t * dest,cmd_nav_to_wp * orig){

		dest->x = orig->latitude/ 1.0e7f;
		dest->y = orig->longitude/ 1.0e7f;
		dest->seq = orig->cmd_id;
	
}

