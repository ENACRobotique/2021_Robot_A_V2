/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "Arduino.h"
#include "CaptureEcocup.h"
#include "../FMSSupervisor.h"
#include "Reajustement.h"
#include "../controlServo.h"
#include "../params.h"

CaptureEcocup captureEcocup = CaptureEcocup();

ControlServo servo = ControlServo(); 

CaptureEcocup::CaptureEcocup() {
	time_start = 0;
}

CaptureEcocup::~CaptureEcocup() {
	// TODO Auto-generated destructor stub
}

void CaptureEcocup::enter() {
	time_start = millis();
	Serial1.println("entrée dans l'état capture éco cup");
	servo.defInitAngle(10);
	servo.init(SERVO3);
	delay(1000);
	servo.resetPos();
	servo.moveServo(120);

}

void CaptureEcocup::leave() {
	Serial1.println("Leaving CaptureEcocup");
}

void CaptureEcocup::doIt() {
	Serial1.println(servo.returnPos());
	Serial1.println(millis() - time_start);
	if(((millis() - time_start) > SERVO_MOVEMENT_DURATION) && (servo.returnPos()<110)){
		servo.resetPos();
		fmsSupervisor.setNextState(&reajustement);
	}
	else if(servo.returnPos()>=110){
		
	}
}


void CaptureEcocup::reEnter(unsigned long interruptTime){
}

void CaptureEcocup::forceLeave(){
}

void CaptureEcocup::pauseNextState(){
}
