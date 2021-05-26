/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "Arduino.h"
#include "etat_begin.h"
#include "../FMSSupervisor.h"
#include "Reajustement.h"
#include "../controlServo.h"
#include "../params.h"
#include "travel.h"
#include "CaptureEcocup.h"

Etat_begin etat_begin = Etat_begin();


Etat_begin::Etat_begin() {
	time_start = 0;
	name="Etat_begin";
}

Etat_begin::~Etat_begin() {
	// TODO Auto-generated destructor stub
}

void Etat_begin::enter() {
	time_start = millis();
	Serial1.println("Je suis en état begin");
}

void Etat_begin::leave() {
	Serial1.println("Leaving etat_begin");
}

void Etat_begin::doIt() {
	if (digitalRead(TIRETTE)==LOW) {
		fmsSupervisor.setNextState(&travel);
	};

}


void Etat_begin::reEnter(unsigned long interruptTime){
}

void Etat_begin::forceLeave(){
}

void Etat_begin::pauseNextState(){
}
