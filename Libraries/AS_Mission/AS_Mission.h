/*
 * ASMission.h
 *
 *  Created on: Aug 22, 2014
 *      Author: aterzani
 */

#ifndef ASMISSION_H_
#define ASMISSION_H_

#include <avr/eeprom.h>


#define NUM_COMMANDS 4
#define CMD_SIZE 8
#define BASE_ADDRESS 100



struct __attribute__((__packed__)) cmd_nav_to_wp {
		uint8_t cmd_id;
		uint32_t latitude;
		uint32_t longitude;
	};

class AS_Mission {
public:


	void addCmd(cmd_nav_to_wp * cmd);
	void start();

	void nextCmd(cmd_nav_to_wp * cmd);

	void stop();
	void init();

private:

	cmd_nav_to_wp cmd_list[NUM_COMMANDS];
	uint8_t current_cmd;
	uint8_t last_inserted_cmd;


};

#endif /* ASMISSION_H_ */
