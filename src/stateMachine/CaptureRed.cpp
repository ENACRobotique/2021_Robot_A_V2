/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "Arduino.h"
#include "CaptureRed.h"
#include "../FMSSupervisor.h"
#include "Reajustement.h"
#include "../controlServo.h"
#include "../params.h"
#include "etat_test.h"
#include "navigator.h"
#include "../ecocupManager.h"
#include "travel.h"

CaptureRed captureEcocup = CaptureRed();

//ControlServo servo = ControlServo(); 

CaptureRed::CaptureRed() {
	time_start = 0;
}

CaptureRed::~CaptureRed() {
	// TODO Auto-generated destructor stub
}

void CaptureRed::enter() {
	servoManager.controlServo(servo_sel,100);
	time_start = millis();
}

/*	Serial1.println("entrée dans l'état capture éco cup");
	servo.defInitAngle(10);
	servo.init(SERVO3);
	delay(1000);
	servo.resetPos();
	servo.moveServo(120);

}*/

void CaptureRed::leave() {
	Serial1.println("Leaving CaptureRed");
}

void CaptureRed::doIt() {
	if(ecocupManager.isFinished()){
		fmsSupervisor.setNextState(&travel);
		
	}
}


void CaptureRed::reEnter(unsigned long interruptTime){
}

void CaptureRed::forceLeave(){
}

void CaptureRed::pauseNextState(){
}
