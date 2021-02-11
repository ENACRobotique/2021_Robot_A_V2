/*
 * FMSSupervisor.cpp
 *
 *  Created on: 11 avr. 2018
 *      Author: robot
 */

#include "FMSSupervisor.h"
#include "Arduino.h"
#include "AbstractState.h"
#include "CaptureEcocup.h"

FMSSupervisor fmsSupervisor = FMSSupervisor();

FMSSupervisor::FMSSupervisor() {
	// TODO Auto-generated constructor stub
	nextState = NULL;
	currentState = NULL;
	previousState = NULL;
	time_obstacle_left = 0;
}

FMSSupervisor::~FMSSupervisor() {
	// TODO Auto-generated destructor stub
}

void FMSSupervisor::setNextState(AbstractState* state) {
	nextState = state;
}

void FMSSupervisor::update() {
	/*if (millis() - tiretteState.get_time_start() > TIME_RACE){
		fmsSupervisor.setNextState(&deadState); //TODO Créer un état où le robot s'arrête
	}

	if(currentState == &pauseState && pauseState.isTooLong()){
		previousState->pauseNextState();
	}*/

	if(nextState != NULL && nextState != currentState){
		currentState->leave();
		nextState->enter();
		currentState = nextState;
		nextState = NULL;
	}

	currentState->doIt();

	/*if(currentState->getFlags() & E_ULTRASOUND){
		usManager.update();
		if(usManager.obstacleDetected()){
			time_obstacle_left = 0;
			if(currentState != &pauseState){			// on va dans l'état pause
				currentState->forceLeave();
				previousState = currentState;
				currentState = &pauseState;
				pauseState.enter();
			}

		}
		else {
			if(currentState == &pauseState && previousState != NULL){		// on revient dans l'état précédent !
				if(time_obstacle_left == 0){
					time_obstacle_left = millis();
				}
				if(millis() - time_obstacle_left > DETECTION_STOP_TIME){ //Permet de ne pas repartir directement quand on ne détecte plus d'adversaires
					pauseState.leave();
					previousState->reEnter(pauseState.getPauseTime());
					currentState = previousState;
					previousState = &pauseState;
					time_obstacle_left = 0; //May be useless (just like the primary)
				}
			}
		}
	}*/
}

/*
void FMSSupervisor::init(AbstractState* state) {
	currentState = state;
	state->enter();
}
*/
void FMSSupervisor::init(){
	currentState = &captureEcocup;
	currentState->enter();
}
