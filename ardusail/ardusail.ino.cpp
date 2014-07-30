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
	{ read_radio,				 1,   100 },
	{ fast_loop,               1,   100 },
	{ write_radio,				1,100},
//	{ update_heading,            2,   100 },
//	{ check_nav_mode,             10,   100 },
{ gcs_send_attitude,        5,   200 },
{ gcs_send_heartbeat,		50,	  100},
{ gcs_send_servo_out, 5,         200},
{ gcs_send_servo_in, 5,         200},		
	//{gcs_update,1,200}
	

};



static void gps_update();
static void one_sec_loop();
static void fast_loop();
static void update_heading();
void gcs_send_heartbeat(void);
void gcs_send_attitude();
void gcs_send_servo_out();
void gcs_send_servo_in();
void gcs_update();
void check_nav_mode();
void switch_mode(int navmode);
void init_navigation();
void update_nav();
void read_radio();
void write_radio();

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
	
  gcs_update();

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





#define 	INT8_MAX   0x7f
#define 	INT8_MIN   (-INT8_MAX - 1)
#define 	UINT8_MAX   (__CONCAT(INT8_MAX, U) * 2U + 1U)
#define 	INT16_MAX   0x7fff
#define 	INT16_MIN   (-INT16_MAX - 1)
#define 	UINT16_MAX   (__CONCAT(INT16_MAX, U) * 2U + 1U)


void gcs_send_heartbeat(void){
	

	int system_type = MAV_TYPE_GENERIC;
	int autopilot_type = MAV_AUTOPILOT_GENERIC;
	uint8_t base_mode;
	uint32_t  custom_mode;
	if(current_nav_mode==NAV_MODE_HEADHOLD){
		base_mode = MAV_MODE_FLAG_GUIDED_ENABLED;
		custom_mode = MAV_MODE_FLAG_GUIDED_ENABLED;
		}else{

			base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
			custom_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		}
		uint8_t system_status = MAV_STATE_ACTIVE;


	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type,base_mode,custom_mode,system_status);
	
	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);;

	
}



void gcs_send_attitude(){
	

	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint32_t timestamp = millis();
	float radHeading = toRadians(sensor.heading);
	float radRoll = toRadians(-sensor.roll);
	float radPitch = toRadians(-sensor.pitch);
	
	mavlink_msg_attitude_pack(100,200, &msg, timestamp,  radRoll,  radPitch, radHeading , 0, 0, 0);
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);
	
	
}

void gcs_send_servo_out(){
	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	
	mavlink_msg_servo_output_raw_pack(100,200, &msg,millis(),1,  rudderChannel.getPwmOut(), sailChannel.getPwmOut(),UINT16_MAX ,UINT16_MAX,auxChannel.getPwmOut(),  UINT16_MAX ,UINT16_MAX ,UINT16_MAX);
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes)
	Serial.write(buf, len);
	
}

void gcs_send_servo_in(){

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	uint16_t range1= rudderChannel.getMax()-rudderChannel.getMin();
	uint16_t range2= sailChannel.getMax()-sailChannel.getMin();
	uint16_t range3= auxChannel.getMax()-auxChannel.getMin();
	
	
	int16_t ch1Scaled = ((rudderChannel.pwmIn-rudderChannel.getMin())*(20000/range1))-10000;
	int16_t ch2Scaled = ((sailChannel.pwmIn-sailChannel.getMin())*(20000/range2))-10000;
	int16_t ch3Scaled = ((auxChannel.pwmIn-auxChannel.getMin())*(20000/range3))-10000;

	//Serial.print("PWMIN: ");	Serial.print(ch1Scaled);

	mavlink_msg_rc_channels_scaled_pack(100,200, &msg,millis(),1,  ch1Scaled,ch2Scaled,UINT16_MAX ,UINT16_MAX,ch3Scaled,  UINT16_MAX ,UINT16_MAX ,UINT16_MAX,255);
	
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes)
	Serial.write(buf, len);

}


void gcs_update(){
	
	mavlink_message_t msg;
	mavlink_status_t status;


	while(Serial.available() > 0 )
	{
		uint8_t c = Serial.read();
	   // Try to get a new message
	   if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			   // Handle message

			   switch (msg.msgid)
			   {
			   	case MAVLINK_MSG_ID_HEARTBEAT:

			   	break;

			   	case MAV_CMD_NAV_WAYPOINT:


			   	break;


			   	default:

			   	break;
			   }
		}	   // And get the next one
		   
	}
}






void check_nav_mode(){
	
	if(auxChannel.pwmIn>1500 && current_nav_mode!=NAV_MODE_MANUAL)
	{
		switch_mode(NAV_MODE_MANUAL);
		Serial.print("Switched to MANUAL");
	}
	
	if (auxChannel.pwmIn<1500 && current_nav_mode!=NAV_MODE_HEADHOLD)
	{
		switch_mode(NAV_MODE_HEADHOLD);
			Serial.print("Switched to AUTO");
	}
	
}


void switch_mode(int navmode){
	
	switch (navmode)
	{
		
		case NAV_MODE_MANUAL:
		
		current_nav_mode=NAV_MODE_MANUAL;
		
		break;
		
		
		case NAV_MODE_HEADHOLD:
			Serial.print("Switched to HEAD HOLD");
			nav_bearing = curr_heading;
			current_nav_mode=NAV_MODE_HEADHOLD;
			rudderChannel.fixCenterPos();
			 
			Setpoint=nav_bearing;
			break;
	}	
	
}




void init_navigation(){
	
	headingPID.SetMode(AUTOMATIC);
}

 void update_nav(){
	
	if(current_nav_mode==NAV_MODE_HEADHOLD){
		
		int errorHeading = nav_bearing-curr_heading;
		
		if(errorHeading>180 || errorHeading <-180){
			if(errorHeading>180){
				Setpoint=nav_bearing-360;
			}
			if(errorHeading<-180){
				Setpoint=nav_bearing+360;
			}
			
		}else{
			
			Setpoint=nav_bearing;
			
		}
		Input=curr_heading;
		
		headingPID.Compute();
		
		//Serial.print(" OUT: ");Serial.print(Output);Serial.print(" ERR: ");Serial.print(Input-Setpoint);Serial.print(" CUR_HD: ");Serial.print(Input);Serial.print(" TO_HD: ");Serial.print(Setpoint);Serial.print("\n\r");
		
		int16_t center=rudderChannel.center();
		
		rudderChannel.setPwm(center-Output);
		
	}
	
	
}


void read_radio(){

	RC_HAL *hal=RC_HAL::getInstance();
	hal->read_all();
		
		int pwm =  rudderChannel.readRadio();
		int pwm2 = sailChannel.readRadio();
		int pwm3 = auxChannel.readRadio();
		//Serial.print(pwm);Serial.print(" : ");Serial.print(pwm3);
		//Serial.print("\n\r");

	
	if(current_nav_mode==NAV_MODE_HEADHOLD){
		
			sailChannel.setPwm(pwm2);
			auxChannel.setPwm(pwm3);
	}else{
		
		// SE SIAMO IN MANUAL RIPORTA TUTTI I PWM COME LI LEGGE ( VANNO CMQ SCRITTI )
			
			rudderChannel.setPwm(pwm);
			sailChannel.setPwm(pwm2);
			auxChannel.setPwm(pwm3);
	}
 
	
}

void write_radio(){

	rudderChannel.writeCurrent();
	sailChannel.writeCurrent();
	auxChannel.writeCurrent();
}
