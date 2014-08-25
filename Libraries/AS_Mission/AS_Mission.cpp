/*
 * ASMission.cpp
 *
 *  Created on: Aug 22, 2014
 *      Author: aterzani
 */

#include "AS_Mission.h"


void AS_Mission::init(){

	last_inserted_cmd=0;
	int i=0;


}
void AS_Mission::addCmd(cmd_nav_to_wp * cmd){

	eeprom_write_block((const void*)cmd, (void*)(BASE_ADDRESS + last_inserted_cmd * sizeof(cmd_nav_to_wp)), sizeof(cmd_nav_to_wp));
	
	last_inserted_cmd++;


}

void AS_Mission::start(){

	current_cmd=0;
}

void AS_Mission::nextCmd(cmd_nav_to_wp * cmd){


	eeprom_read_block((void*)cmd, (void*)(BASE_ADDRESS+current_cmd*sizeof(cmd_nav_to_wp)), sizeof(cmd_nav_to_wp));

	current_cmd++;

}

void AS_Mission::stop(){


}

