// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GCS_MAVLink.h
/// @brief	One size fits all header for MAVLink integration.

#ifndef CGS_MAVLink_h
#define CGS_MAVLink_h

#include <util/crc16.h>
#include <FastSerial.h>
#include <Arduino.h>

#include "include/mavlink/v1.0/mavlink_types.h"
#include "include/mavlink/v1.0/common/mavlink.h"
#include "include/mavlink/v1.0/ardupilotmega/ardupilotmega.h"

class CGS_MAVLink {
  
public:

	void init();

	void send_heartbeat();
	
	void send_attitude(float yaw, float pitch, float roll);
    void send_gps_raw_int(uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible);	
private:

	

};

#endif

