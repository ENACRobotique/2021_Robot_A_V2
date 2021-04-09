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

Metro controlTime = Metro((unsigned long)(CONTROL_PERIOD * 1000));
Metro debugLed = Metro(2000);
Metro navTime = Metro((unsigned long)(NAVIGATOR_PERIOD * 1000)); //2000
Metro TestTime = Metro(1000);
Metro commXBee = Metro((unsigned long)(COMMUNICATION_PERIOD * 1000));
Metro stateTime = Metro((unsigned long)(STATE_PERIOD * 1000));

//The setup function is called once at startup of the sketch
void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	//pinMode(MOT_LIDAR, OUTPUT);

	pinMode(COLOR, INPUT_PULLUP);
	//pinMode(BATT_CHARGE, INPUT);


	//while(!Serial){}
	Serial1.begin(57600);
	Serial.begin(57600);
	Serial.println("INIT Serial");
	controlTime.reset();
	debugLed.reset();
	navTime.reset();
	TestTime.reset();
	Odometry::init();
	MotorControl::init();
//	fmsSupervisor.init();
	//MotorControl::set_cons(0,0);
	//navigator.move_to(500,500);

	kalmanFilter::init();
	SerialUSB.println("hi");
}

int mot1 = 100; //entre -255 et 255
int mot2 = -100;
int i;
int k = 0;

String tostr;
// The loop function is called in an endless loop

void loop() // ATTENTION  ne pas donner moins de 50 ms pour écrire sur le serial
{
	if (true)
	{	
		if (debugLed.check())
		{
			digitalToggle(LED_BUILTIN);
		}

		if (commXBee.check())
		{	
			Communication::update();
			//tostr=String(Communication::get_value());
			//Serial1.print(tostr);
			//MotorControl::set_cons(10*Communication::get_value(),0);
		}

		if (controlTime.check())
		{
			Odometry::update();
			MotorControl::update();
		}

		/*if(navTime.check()) {
		MotorControl::set_cons(speed_cons[speed_i], 0);
		speed_i = (speed_i + 1) % 4;
	}*/
		if (navTime.check())
		{	
	/*		if (millis()%4000<2000) {
				MotorControl::set_cons(0,0.2);

			}
			else {
				MotorControl::set_cons(0,0.0);

			}   */
			navigator.update();
			//Serial1.println(Odometry::get_pos_x());
		}
		if (stateTime.check())
		{
		//	fmsSupervisor.update();
		}
	}
	else
	{
		//étude consigne moteur/vitesse
		MotorControl::testmoteur(mot1, mot2);
		if (controlTime.check())
		{
			Serial.print(i);
			Serial.print("\t");
			Odometry::update();
			Serial.print("\t");
			Serial.println(mot2);
			i++;
		}
		if (TestTime.check())
			mot2 = (mot2 + 20) % 255;
	}
}
