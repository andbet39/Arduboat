// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GCS_MAVLink.h
/// @brief	One size fits all header for MAVLink integration.

#ifndef CGS_MAVLink_h
#define CGS_MAVLink_h

#include "../AS_Mission/AS_Mission.h"
#include "../FastSerial/FastSerial.h"

#include "include/mavlink/v1.0/mavlink_types.h"
#include "include/mavlink/v1.0/common/mavlink.h"
#include "include/mavlink/v1.0/ardupilotmega/ardupilotmega.h"

class CGS_MAVLink {
  
public:

	void init();

	void handleMissionCountMessage(AS_Mission * mission , mavlink_message_t * msg);
	void handleMissionClearMessage(AS_Mission * mission , mavlink_message_t * msg);
	void sendMissionAck();


private:

	bool IsReceiving;
	uint8_t last_received;
	

};

#endif

