#include <FastSerial.h>
#include <Arduino.h>

#include "GCS_Mavlink.h"
#include "navigation.h"
#include "nav_mode.h"
#include "radio.h"



#include <stdint.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_GPS.h>
#include <AS_Math.h>
#include <AS_Scheduler.h>
#include <AS_Sensor.h>
#include <AS_HILSensor.h>
#include <AS_GPS.h>
#include <AS_HILGPS.h>
#include <AS_Mission.h>
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
FastSerialPort1(Serial1);

#define PID_HEADING_SCALER 2.0

double Setpoint, Input, Output;
double consKp=1.5, consKi=0.2, consKd=0.5;
PID headingPID(&Input, &Output, &Setpoint,consKp,consKi,consKd, REVERSE);


static int16_t nav_bearing;
static int16_t nav_bearing_error;

static int16_t curr_heading;
static uint8_t current_nav_mode;

static bool overridedMode;

double headingPIDOutput;

static void update_heading();
static void fast_loop();
static void one_sec_loop();


static AS_Scheduler scheduler;
//static AS_Sensor sensor;
static AS_HILGPS gps;
static AS_HILSensor sensor;


static AS_Mission mission;


static CGS_MAVLink   gcs;


static const AS_Scheduler::Task scheduler_tasks[] = {
	{ read_radio,				 1,   100 },
	{ fast_loop,               1,   100 },
	{ write_radio,				1,100},
	{ update_heading,            2,   100 },
	{ check_nav_mode,             10,   100 },
        {gps_update,                10,100},
        { gcs_send_attitude,        5,   200 },
      { gcs_send_heartbeat,		50,	  100},
      { gcs_send_servo_out, 5,         200},
//      { gcs_send_servo_in, 5,         200},		
      { gcs_send_position, 5,         200},
      {gcs_send_hil_control,5,200},
      {gcs_send_navcontroller,20,200}
	
	//{gcs_update,1,200}
	

};


void setup() {
      
      RC_HAL *hal=RC_HAL::getInstance();
      
      hal->init();
      overridedMode=false;
      current_nav_mode=NAV_MODE_MANUAL;
      
      headingPID.SetOutputLimits(-300,300);
      gcs.init();
      gps.init();
    
    rudderChannel.init(6,1);
    sailChannel.init(7,2);
    auxChannel.init(8,3);
    
    mission.init();
    mission.DebugPrint();
    init_navigation();
    
    
    Serial.print( "Start init sensor");
    sensor.init(UPDATE_SENSOR_TIME);  
    
    
    Serial.print( "Start init scheduler");
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
    
    
    
    last_millis=0;
    elapsed=0;

}


void loop() {
	
  gcs_update();

  if(!sensor.getSample()){
    return;
  }

  scheduler.tick();
  scheduler.run(19500U);

}


static void gps_update(){
	
	gps.getSample();
	
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







