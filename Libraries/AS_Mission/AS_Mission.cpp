/*
 * ASMission.cpp
 *
 *  Created on: Aug 22, 2014
 *      Author: aterzani
 */

#include "AS_Mission.h"


void AS_Mission::init(){

	last_inserted_cmd=0;
	uint16_t  add = COUNT_ADDRESS;
	memory_count = eeprom_read_word(&add);	
	Serial.printf("Loaded mission %d 	\n",memory_count );
	int i=0;



}


void AS_Mission::DebugPrint(){

	Serial.printf(" There is last_inserted_cmd %d\n",memory_count );

}

void AS_Mission::addCmd(cmd_nav_to_wp * cmd){

	eeprom_write_block((const void*)cmd, (void*)(BASE_ADDRESS + last_inserted_cmd * sizeof(cmd_nav_to_wp)), sizeof(cmd_nav_to_wp));
		
	uint16_t  add = COUNT_ADDRESS;
	eeprom_write_word(&add,last_inserted_cmd);		

	last_inserted_cmd++;



}

void AS_Mission::start(){

	current_cmd=0;
}

void AS_Mission::clear(){

	current_cmd=0;
	last_inserted_cmd=0;
	uint16_t  add = COUNT_ADDRESS;
	eeprom_write_word(&add,last_inserted_cmd);		
	memory_count=0;
}

void AS_Mission::nextCmd(cmd_nav_to_wp * cmd){


	eeprom_read_block((void*)cmd, (void*)(BASE_ADDRESS+current_cmd*sizeof(cmd_nav_to_wp)), sizeof(cmd_nav_to_wp));

	current_cmd++;

}

void AS_Mission::stop(){


}

void AS_Mission::mavLinkCmdToMission(mavlink_mission_item_t * orig,cmd_nav_to_wp * dest){

		dest->latitude = orig->x;
		dest->longitude = orig->y;
		dest->cmd_id = orig->seq;
	
}