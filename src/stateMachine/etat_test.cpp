/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "Arduino.h"
#include "etat_test.h"
#include "../FsmSupervisor.h"
#include "Reajustement.h"
#include "../controlServo.h"
#include "../params.h"

Etat_test etat_test = Etat_test();


Etat_test::Etat_test() {
	time_start = 0;
	name="Etat_test";
}

Etat_test::~Etat_test() {
	// TODO Auto-generated destructor stub
}

void Etat_test::enter() {
	time_start = millis();
	Serial1.println("Je suis en état test");
}

void Etat_test::leave() {
	Serial1.println("Leaving etat_test");
}

void Etat_test::doIt() {


}


void Etat_test::reEnter(unsigned long interruptTime){
}

void Etat_test::forceLeave(){
}

void Etat_test::pauseNextState(){
}
