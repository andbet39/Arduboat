#include <FastSerial.h>
#include <Arduino.h>


#define  USE_TELEMETRY true


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

#include <AS_Param.h>

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
#include <AS_Nav.h>


#include "parameters.h"


#define UPDATE_SENSOR_TIME 20

#define NAV_MODE_MANUAL 0
#define NAV_MODE_HEADHOLD 1
#define NAV_MODE_AUTO 2

#define WP_RADIUS 2


int32_t last_millis;
int32_t elapsed;


static Parameters g;

RC_Channel rudderChannel;
RC_Channel sailChannel;
RC_Channel auxChannel;

AS_Nav navigator;


FastSerialPort0(Serial);
FastSerialPort1(Serial1);
FastSerialPort3(Serial3);


#define PID_HEADING_SCALER 2.0

double Setpoint, Input, Output;

double consKp=g.rudder_pid_p, consKi=g.rudder_pid_i, consKd=g.rudder_pid_d;


PID headingPID(&Input, &Output, &Setpoint,consKp,consKi,consKd, REVERSE);


static int16_t nav_bearing;
static int32_t nav_distance;

static int16_t nav_bearing_error;

static int16_t curr_heading;
static uint8_t current_nav_mode;

static bool overridedMode;

double headingPIDOutput;

static void update_heading();
static void fast_loop();
static void one_sec_loop();


static AS_Scheduler scheduler;
static AS_Sensor sensor;
static AS_GPS gps;
//static AS_HILSensor sensor;


static AS_Mission mission;


static CGS_MAVLink   gcs;


static const AS_Scheduler::Task scheduler_tasks[] = {
	{ read_radio,				 1,   100 },
	{ fast_loop,               1,   100 },
	{ write_radio,				1,100},
	{ update_heading,            2,   100 },
	{ check_nav_mode,             10,   100 },
        {gps_update,                10,100},
        { gcs_send_attitude,        1,   200 },
        { gcs_send_heartbeat,		50,	  100},
      //{ gcs_send_servo_out, 5,         200},
      //{ gcs_send_servo_in, 5,         200},		
        { gcs_send_position, 5,         200},
       // {gcs_send_hil_control,10,200},
        {gcs_send_navcontroller,20,200}
      	
	//{gcs_update,1,200}
	

};

AS_Param param_loader(var_info, COUNT_ADDRESS);

void setup() {
      
      RC_HAL *hal=RC_HAL::getInstance();
      
      hal->init();
      overridedMode=false;
      
      AS_Param::setup_sketch_defaults();

      load_parameters();
      
      current_nav_mode=NAV_MODE_MANUAL;
      
      
      headingPID.SetOutputLimits(-300,300);
      headingPID.SetTunings( g.rudder_pid_p,g.rudder_pid_i,g.rudder_pid_d);
 
      gcs.init();
      gps.init();
      

    rudderChannel.init(6,1);
    sailChannel.init(7,2);
    auxChannel.init(8,3);
    
     mission.init();
     mission.DebugPrint();
   
     navigator.init(&gps,&mission);
   
    init_navigation();
    
   
    Serial3.print("Air comunication enabled");
    
    Serial.print( "Start init sensor");
    sensor.init(UPDATE_SENSOR_TIME);  
    
    
    Serial.print( "Start init scheduler");
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
    
    
    
    last_millis=0;
    elapsed=0;
    Serial.print("\n");
    Serial.print(g.rudder_pid_p);
    Serial.print("\n");
    Serial.print(g.rudder_pid_i);
    Serial.print("\n");
    Serial.print(g.rudder_pid_d);
    Serial.print("\n");



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
	if(current_nav_mode>0){	
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







