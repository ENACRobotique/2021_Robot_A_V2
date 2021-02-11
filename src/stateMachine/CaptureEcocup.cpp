/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "Arduino.h"
#include "CaptureEcocup.h"
#include "../FMSSupervisor.h"
#include "Reajustement.h"

CaptureEcocup captureEcocup = CaptureEcocup();

CaptureEcocup::CaptureEcocup() {
	time_start = 0;
}

CaptureEcocup::~CaptureEcocup() {
	// TODO Auto-generated destructor stub
}

void CaptureEcocup::enter() {
	time_start = millis();
	Serial1.println("entrée dans l'état capture éco cup");
}

void CaptureEcocup::leave() {
	Serial1.println("Leaving CaptureEcocup");
}

void CaptureEcocup::doIt() {
	if(true){
		fmsSupervisor.setNextState(&reajustement);
	}
}


void CaptureEcocup::reEnter(unsigned long interruptTime){
}

void CaptureEcocup::forceLeave(){
}

void CaptureEcocup::pauseNextState(){
}
