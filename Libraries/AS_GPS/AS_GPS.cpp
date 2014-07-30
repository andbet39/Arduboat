#include "AS_GPS.h"


//AltSoftSerial mySerial;//RX TX
//Adafruit_GPS GPS(&mySerial);

void AS_GPS::init()
{
	/*
	
	
	GPS.begin(9600);    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude  
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // uncomment this line to turn on only the "minimum recommended" data  //
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since  // the parser doesn't care about other sentences at this time   
 // Set the update rate  
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate  // For the parsing code to work nicely and have time to sort thru the data, and  // print it out we don't suggest using anything higher than 1 Hz  // Request updates on antenna status, comment out to keep quiet  
	GPS.sendCommand(PGCMD_ANTENNA);
	
	*/

}

void AS_GPS::getSample()
{
	/*
	Serial.print("GPSREAD");
	
	char c = GPS.read();

Serial.print(c);
	if (GPS.newNMEAreceived()) {
		
		if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
		return;  // we can fail to parse a sentence in which case we should just wait for another
	}

	
	lat=GPS.latitude*10000000;
	lon=GPS.longitude*10000000;
	alt=GPS.altitude*1000;
	
	angle=GPS.angle*100;
	
	fix=GPS.fix;
	satellites=GPS.satellites;
	
		*/
	
	}
 


