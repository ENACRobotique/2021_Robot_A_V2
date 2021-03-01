/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "Arduino.h"
#include "Reajustement.h"
#include "../FMSSupervisor.h"
#include "CaptureEcocup.h"
#include "navigator.h"

Reajustement reajustement = Reajustement();

Reajustement::Reajustement() {
	time_start = 0;
}

Reajustement::~Reajustement() {
	// TODO Auto-generated destructor stub
}

void Reajustement::enter() {
	Serial1.println("entrée dans l'état réajustement");
	time_start = millis();
    navigator.adjust_rot(delta_ajust_deg);
}

void Reajustement::leave() {
    Serial1.println("Sortie Réajustment");
}

void Reajustement::doIt() {
    if (navigator.isTrajectoryFinished()){
        fmsSupervisor.setNextState(&captureEcocup);
    }
    else if (navigator.caperror()) {
        fmsSupervisor.setNextState(&next_cup);
    }
}


void Reajustement::reEnter(unsigned long interruptTime){
}

void Reajustement::forceLeave(){
}

void Reajustement::pauseNextState(){
}
