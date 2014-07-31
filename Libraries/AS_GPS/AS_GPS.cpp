#include "AS_GPS.h"

Adafruit_GPS GPS(&Serial1);

#define GPSECHO  false

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


double toDeg(float val){

  double degree= floor(val/100);
  double decpos=degree+(val-(degree*100))/60;

  return decpos;

}



void AS_GPS::init()
{
	
	
	GPS.begin(9600);    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude  
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // uncomment this line to turn on only the "minimum recommended" data  //
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since  // the parser doesn't care about other sentences at this time   
 // Set the update rate  
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate  // For the parsing code to work nicely and have time to sort thru the data, and  // print it out we don't suggest using anything higher than 1 Hz  // Request updates on antenna status, comment out to keep quiet  
	GPS.sendCommand(PGCMD_ANTENNA);
	
	 useInterrupt(true);
 delay(1000);
}


void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
    Serial.print("INT");
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void AS_GPS::getSample()
{
	
	
 
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }


	float gps_lat=GPS.latitude;
	float gps_lon=GPS.longitude;
	
  
  lat=toDeg(gps_lat)*10000000;
  lon=toDeg(gps_lon)*10000000;

  alt=GPS.altitude*1000;
	angle=GPS.angle*100;
	
  fix=GPS.fix;
	satellites=GPS.satellites;
	
	
}
 
