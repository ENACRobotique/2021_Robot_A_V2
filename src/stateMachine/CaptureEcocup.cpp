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
#include "etat_test.h"
#include "navigator.h"
#include "../servoManager.h"
#include "travel.h"

CaptureEcocup captureEcocup = CaptureEcocup();

//ControlServo servo = ControlServo(); 

CaptureEcocup::CaptureEcocup() {
	time_start = 0;
}

CaptureEcocup::~CaptureEcocup() {
	// TODO Auto-generated destructor stub
}

void CaptureEcocup::enter() {
	servoManager.controlServo(servo_sel,90);
	time_start = millis();
}

/*	Serial1.println("entrée dans l'état capture éco cup");
	servo.defInitAngle(10);
	servo.init(SERVO3);
	delay(1000);
	servo.resetPos();
	servo.moveServo(120);

}*/

void CaptureEcocup::leave() {
	IR_sel = (sensors)((int)IR_sel+1);
	Serial1.println("Leaving CaptureEcocup");
}

void CaptureEcocup::doIt() {

	servoManager.update();
	
	if(servoManager.isFinished()){
		fmsSupervisor.setNextState(&travel);
		
	}
}


void CaptureEcocup::reEnter(unsigned long interruptTime){
}

void CaptureEcocup::forceLeave(){
}

void CaptureEcocup::pauseNextState(){
}
