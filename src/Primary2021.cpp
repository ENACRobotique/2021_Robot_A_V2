#include "odometry.h"
#include "motorControl.h"
#include "params.h"
#include "Arduino.h"
#include "Metro.h"
#include "navigator.h"
#include "communication.h"
#include "./FMSSupervisor.h"
#include "raspberryParser.h"
#include <string.h>
#include <kalmanFilter.h>
#include "trajectoryv2.h"
#include "lidar.h"
#include "servoManager.h"
#include "ecocupManager.h"

Metro controlTime = Metro((unsigned long)(CONTROL_PERIOD * 1000));
Metro debugLed = Metro(2000);
Metro navTime = Metro((unsigned long)(NAVIGATOR_PERIOD * 1000)); //2000
Metro TestTime = Metro(1000);
Metro commXBee = Metro((unsigned long)(COMMUNICATION_PERIOD * 1000));
Metro stateTime = Metro((unsigned long)(STATE_PERIOD * 1000));
Metro lidarTime = Metro((unsigned long)(1000));

//The setup function is called once at startup of the sketch
void setup()
{
	
	pinMode(LED_BUILTIN, OUTPUT);
	//pinMode(MOT_LIDAR, OUTPUT);

	pinMode(COLOR, INPUT_PULLUP);
	//pinMode(BATT_CHARGE, INPUT);

	pinMode(TIRETTE,INPUT_PULLUP);

	//while(!Serial){}
	Serial1.begin(57600);
	Serial.begin(57600);
	Serial.println("INIT Serial");
	controlTime.reset();
	debugLed.reset();
	navTime.reset();
	TestTime.reset();
	Odometry::init();
	//position initiale
	Odometry::set_pos(traj1.get_current_WP().x,traj1.get_current_WP().y,180);

	MotorControl::init();
	navigator.init(traj1.get_current_WP().x,traj1.get_current_WP().y);
	fmsSupervisor.init();
	//MotorControl::set_cons(0,0);
	//navigator.move_to(500,500);

	IR_sel = IR_FC;
	navigator.set_sens(1);//0 ou 1

	kalmanFilter::init();
	Lidar::init();
	Lidar::changeReadSpace2(1);
	servoManager.init(init_tab);
	ecocupManager.init();
	
}

int mot1 = 100; //entre -255 et 255
int mot2 = -100;
int i;
int k = 0;

// The loop function is called in an endless loop
bool begin;

void loop() // ATTENTION  ne pas donner moins de 50 ms pour écrire sur le serial
{	
	if (!fmsSupervisor.test_mode)
	{	
		if (debugLed.check())
		{
			digitalToggle(LED_BUILTIN);
		}

		if (commXBee.check())
		{	
			Communication::update();
			//Lidar::readState();
			//tostr=String(Communication::get_value());
			//Serial1.print(tostr);
			//MotorControl::set_cons(10*Communication::get_value(),0);
		}

		if (controlTime.check())
		{
			Odometry::update();
			MotorControl::update();

		}

		if (navTime.check())
		{	
			navigator.update();
		}

		if (stateTime.check())
		{
			fmsSupervisor.update();
			Lidar::readState();
			servoManager.update();
			//Serial1.println(digitalRead(TIRETTE));
		}
		if (lidarTime.check()) {
			//Lidar::update_all();
		}
	}
	else
	{	
		Serial1.printf("val IR 1 = %f\n",navigator.volt_to_dist(analogRead(IR_sel)));
		Serial1.printf("val IR 2 = %f\n",navigator.volt_to_dist(analogRead(IR_sel+1)));
		Serial1.printf("val IR 3 = %f\n",navigator.volt_to_dist(analogRead(IR_sel+2)));
		delay(20);
	}
}
