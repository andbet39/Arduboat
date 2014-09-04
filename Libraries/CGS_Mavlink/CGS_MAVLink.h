// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GCS_MAVLink.h
/// @brief	One size fits all header for MAVLink integration.

#ifndef CGS_MAVLink_h
#define CGS_MAVLink_h

#include "../AS_Mission/AS_Mission.h"
#include "../AS_HILGPS/AS_HILGPS.h"
#include "../AS_Param/AS_Param.h"
#include "../AS_HILSensor/AS_HILSensor.h"
#include "../AS_GPS/AS_GPS.h"
#include "../AS_Sensor/AS_Sensor.h"

#include "include/mavlink/v1.0/mavlink_types.h"
#include "include/mavlink/v1.0/common/mavlink.h"
#include "include/mavlink/v1.0/ardupilotmega/ardupilotmega.h"

#define  USE_TELEMETRY true


class CGS_MAVLink {
  
public:

	void init();

	void handleMissionCountMessage(AS_Mission * mission , mavlink_message_t * msg);
	void handleMissionClearMessage(AS_Mission * mission , mavlink_message_t * msg);
	void handleMissionItemMessage(AS_Mission * mission , mavlink_message_t * msg);
	void handleMissionRequestList(AS_Mission * mission , mavlink_message_t * msg);
	void handleMissionRequest(AS_Mission * mission , mavlink_message_t * msg);
	uint8_t handleSetModeMessage(mavlink_message_t  * msg);
	void handleHilStateMessage(AS_HILGPS * gps ,AS_HILSensor * sensor, mavlink_message_t * msg);
	void handleHilStateMessage(AS_GPS * gps ,AS_Sensor * sensor, mavlink_message_t * msg);
	void requestWP(uint16_t wpnum);
	void sendMissionAck(uint8_t result);
	void sendMissionCount(AS_Mission * mission);
	void sendMissionItem(AS_Mission * mission,uint16_t cmd_num);

	void handleParameters_request_list(mavlink_message_t * msg);
	void sendParameter(uint16_t idx);
	void update();
private:

	bool isReceiving;
	bool isSending;
	uint16_t last_received;
	uint16_t receive_count;
	uint16_t last_sended;


	uint8_t _last_sent_parameter;
	uint8_t _params_to_send;
	bool _sendingParameter;
};

#endif

