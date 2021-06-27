/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "Arduino.h"
#include "ReleaseEcocup.h"
#include "../FsmSupervisor.h"
#include "Reajustement.h"
#include "../controlServo.h"
#include "../params.h"
#include "etat_test.h"
#include "navigator.h"
#include "../ecocupManager.h"

ReleaseEcocup releaseEcocup = ReleaseEcocup();

//ControlServo servo = ControlServo(); 

ReleaseEcocup::ReleaseEcocup() {
	time_start = 0;
}

ReleaseEcocup::~ReleaseEcocup() {
	// TODO Auto-generated destructor stub
}

void ReleaseEcocup::enter() {
	
	Serial1.println("etat release");
	ecocupManager.release();
	//servoManager.controlServo(servo_sel,180);
	time_start = millis();
}

/*	Serial1.println("entrée dans l'état capture éco cup");
	servo.defInitAngle(10);
	servo.init(SERVO3);
	delay(1000);
	servo.resetPos();
	servo.moveServo(120);

}*/

void ReleaseEcocup::leave() {
	Serial1.println("Leaving ReleaseEcocup");
}

void ReleaseEcocup::doIt() {
	if(servoManager.isFinished()){
		//fmsSupervisor.setNextState(&travel);
		Serial1.println("set something here !!");
	}
}


void ReleaseEcocup::reEnter(unsigned long interruptTime){
}

void ReleaseEcocup::forceLeave(){
}

void ReleaseEcocup::pauseNextState(){
}
