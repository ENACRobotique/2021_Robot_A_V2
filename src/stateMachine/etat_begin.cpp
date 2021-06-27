/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "Arduino.h"
#include "params.h"
#include "etat_begin.h"
#include "../FsmSupervisor.h"
#include "../controlServo.h"
#include "../params.h"


Etat_begin etat_begin = Etat_begin();


Etat_begin::Etat_begin() {
	time_start = 0;

}

Etat_begin::~Etat_begin() {
	// TODO Auto-generated destructor stub
}

void Etat_begin::enter() {
	time_start = millis();
	//SerialDebug.println("Je suis en état begin");
}

void Etat_begin::leave() {
	Serial1.println("Leaving etat_begin");
}

void Etat_begin::doIt() {


}


void Etat_begin::reEnter(unsigned long interruptTime){
}

void Etat_begin::forceLeave(){
}

void Etat_begin::pauseNextState(){
}
