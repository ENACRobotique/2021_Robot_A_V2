/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "Arduino.h"
#include "ReleaseEcocup.h"
#include "../FMSSupervisor.h"
#include "Reajustement.h"
#include "../controlServo.h"
#include "../params.h"
#include "etat_test.h"
#include "navigator.h"
#include "../servoManager.h"
#include "travel.h"

ReleaseEcocup releaseEcocup = ReleaseEcocup();

//ControlServo servo = ControlServo(); 

ReleaseEcocup::ReleaseEcocup() {
	time_start = 0;
}

ReleaseEcocup::~ReleaseEcocup() {
	// TODO Auto-generated destructor stub
}

void ReleaseEcocup::enter() {
	
	if (IR_sel == IR_FR){
		navigator.set_sens(1);
	}
	servoManager.controlServo(servo_sel,180);
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
		fmsSupervisor.setNextState(&travel);
		
	}
}


void ReleaseEcocup::reEnter(unsigned long interruptTime){
}

void ReleaseEcocup::forceLeave(){
}

void ReleaseEcocup::pauseNextState(){
}
