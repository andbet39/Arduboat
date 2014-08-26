/*
 * ASMission.h
 *
 *  Created on: Aug 22, 2014
 *      Author: aterzani
 */

#ifndef ASMISSION_H_
#define ASMISSION_H_

#include <avr/eeprom.h>
#include "../CGS_Mavlink/include/mavlink/v1.0/mavlink_types.h"
#include "../CGS_Mavlink/include/mavlink/v1.0/common/mavlink.h"
#include "../CGS_Mavlink/include/mavlink/v1.0/ardupilotmega/ardupilotmega.h"
#include "../FastSerial/FastSerial.h"

#define NUM_COMMANDS 4
#define CMD_SIZE 8
#define BASE_ADDRESS 100
#define COUNT_ADDRESS 98



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
	void clear();

	void mavLinkCmdToMission(mavlink_mission_item_t * orig,cmd_nav_to_wp * dest);


	void DebugPrint();
private:

	cmd_nav_to_wp cmd_list[NUM_COMMANDS];
	uint16_t current_cmd;
	uint16_t last_inserted_cmd;
	uint16_t memory_count;



};

#endif /* ASMISSION_H_ */
