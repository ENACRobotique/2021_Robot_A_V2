/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "Arduino.h"
#include "CaptureEcocup.h"
#include "../FsmSupervisor.h"
#include "Reajustement.h"
#include "../controlServo.h"
#include "../params.h"
#include "etat_test.h"
#include "navigator.h"
//#include "../servoManager.h"
#include "../ecocupManager.h"
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
	//servoManager.controlServo(servo_sel,100);
	Serial1.println("etat capture ecocup");
	ecocupManager.capture();
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
	//IR_sel = (sensors)((int)IR_sel+1);
	//servo_sel = (servos)((int)servo_sel+1);
	//if (IR_sel == IR_BL){
	//	navigator.set_sens(1);
	//}
	Serial1.println("Leaving CaptureEcocup");
}

void CaptureEcocup::doIt() {


	
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
