/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#include "Arduino.h"
#include "Reajustement.h"
#include "../FMSSupervisor.h"
#include "CaptureEcocup.h"
#include "../navigator.h"
#include "../params.h"
#include "travel.h"

Reajustement reajustement = Reajustement();

Reajustement::Reajustement() {
	time_start = 0;
    name = "Reajustement";
}

Reajustement::~Reajustement() {
	// TODO Auto-generated destructor stub
}

void Reajustement::enter() {
	Serial1.println("entrée dans l'état réajustement");
	time_start = millis();
    navigator.adjust_rot(nominal_delta_rot); //entrée dans l'état de capture dans le navigator
}

void Reajustement::leave() {
    Serial1.println("Sortie Réajustment");
}

void Reajustement::doIt() {
   //float v_r = VOLT_TO_DIST(analogRead(IR_sel));
   //Serial1.printf("IR: %d\n",(int)IR_sel);

   //Serial1.printf("IR read : %f\n", v_r);
    if (navigator.caperror()) {
        fmsSupervisor.setNextState(&travel);
    }

    else if (navigator.isTrajectoryFinished()){
        fmsSupervisor.setNextState(&captureEcocup);
    }
    
}


void Reajustement::reEnter(unsigned long interruptTime){
}

void Reajustement::forceLeave(){
}

void Reajustement::pauseNextState(){
}
