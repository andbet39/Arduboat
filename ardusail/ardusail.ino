#include <FastSerial.h>
#include <Arduino.h>
#include "GCS_Mavlink.h"
#include "navigation.h"
#include "nav_mode.h"
#include "radio.h"



#define NO_PORTA_PINCHANGES
#define NO_PORTB_PINCHANGES
#define NO_PORTC_PINCHANGES
#define NO_PORTB_PINCHANGES // to indicate that port b will not be used for pin change interrupts
#define NO_PORTJ_PINCHANGES // to indicate that port c will not be used for pin change interrupts
#define NO_PORTK_PINCHANGES

#include <stdint.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
//#include <Adafruit_GPS.h>
#include <AS_Math.h>
#include <AS_Scheduler.h>
#include <AS_Sensor.h>
#include <AS_GPS.h>
#include <AS_Common.h>
#include <CGS_MAVLink.h>
#include <RC_Channel.h>
#include <RC_HAL.h>
#include <PID_v1.h>

#define UPDATE_SENSOR_TIME 20

#define NAV_MODE_MANUAL 0
#define NAV_MODE_HEADHOLD 1


int32_t last_millis;
int32_t elapsed;


RC_Channel rudderChannel;
RC_Channel sailChannel;
RC_Channel auxChannel;

FastSerialPort0(Serial);

#define PID_HEADING_SCALER 2.0

double Setpoint, Input, Output;
double consKp=5, consKi=0.1, consKd=2.25;
PID headingPID(&Input, &Output, &Setpoint,consKp,consKi,consKd, REVERSE);


static int16_t nav_bearing;
static int16_t nav_bearing_error;

static int16_t curr_heading;
static uint8_t current_nav_mode;

double headingPIDOutput;

static void update_heading();
static void fast_loop();
static void one_sec_loop();


static AS_Scheduler scheduler;
static AS_Sensor sensor;
static AS_GPS gps;

static CGS_MAVLink   gcs;


static const AS_Scheduler::Task scheduler_tasks[] = {
//	{ read_radio,				 1,   100 },
//	{ fast_loop,               1,   100 },
//	{ write_radio,				1,100},
//	{ update_heading,            2,   100 },
//	{ check_nav_mode,             10,   100 },
      { gcs_send_attitude,        5,   200 },
      { gcs_send_heartbeat,		50,	  100},
      { gcs_send_servo_out, 5,         200},
      { gcs_send_servo_in, 5,         200},		
	//{gcs_update,1,200}
	

};


void setup() {

  RC_HAL *hal=RC_HAL::getInstance();
  
  hal->init();
  
  current_nav_mode=NAV_MODE_MANUAL;
  
  headingPID.SetOutputLimits(-500,500);
  gcs.init();
//  gps.init();

rudderChannel.init(5,1);
sailChannel.init(6,2);
auxChannel.init(7,3);


init_navigation();


Serial.print( "Start init sensor");
sensor.init(UPDATE_SENSOR_TIME);


Serial.print( "Start init scheduler");
scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));



last_millis=0;
elapsed=0;

}


void loop() {
	
//  gcs_update();

  if(!sensor.getSample()){
    return;
  }

  scheduler.tick();
  scheduler.run(19500U);

}


static void gps_update(){
	
	gps.getSample();
	
	//Serial.print(gps.fix);
}



static void one_sec_loop(){


  
}


static void fast_loop()
{
	if(current_nav_mode==NAV_MODE_HEADHOLD){	
		update_nav();
	}
}

static void update_heading(){
  
  int temp  = 0;
  if (sensor.heading>0){
   curr_heading=sensor.heading;
   }else{
     curr_heading=360+sensor.heading;
   }


 }







