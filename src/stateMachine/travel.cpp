/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */
#include "travel.h"
#include "Arduino.h"
#include "../FMSSupervisor.h"
#include "Reajustement.h"
#include "../params.h"
#include "etat_test.h"
#include "pivot.h"
#include "etat_end.h"
#include "trajectory.h"
#include "../navigator.h"
#include "../ecocupManager.h"

Travel travel = Travel();

Travel::Travel()
{
	currentWp=Waypoint(0.0,0.0,BEGIN);
	time_start = 0;
}

Travel::~Travel()
{
	// TODO Auto-generated destructor stub
}

void Travel::enter()
{
	Serial1.println("Enter Travel");
	time_start = millis();
	currentWp = traj1.get_next_WP();
	Serial1.printf("position visée :%f %f\n",currentWp.x,currentWp.y);
		switch(currentWp.type){
		case ECOCUP_RED: ecocupManager.definir_action(true,true); break;
		case ECOCUP_GREEN: ecocupManager.definir_action(false,true); break;
		case RELEASE_GREEN: ecocupManager.definir_action(false,false); break;
		case RELEASE_RED: ecocupManager.definir_action(true,false); break;
		default: break;
	}
	//on pourrait aussi modifier la position à atteindre en fonction du servo selectionné
	if(!currentWp.angleAleatoire){
		//on avance jusqu'à un point en amont
		//vérifier selon disposition des axes/angles
		navigator.move_to(currentWp.x-dist_min*sin(currentWp.angle), currentWp.y-dist_min*cos(currentWp.angle));
		navigator.move_to(currentWp.x, currentWp.y);
		navigator.turn_to(currentWp.angle);//inutile en théorie, car on est déjà bien aligné
	} else if(currentWp.bouger){
		navigator.move_to(currentWp.x, currentWp.y);
	}

	
}

void Travel::leave()
{
	Serial1.println("Leaving Travel");
}

void Travel::doIt()
{

	if (navigator.isTrajectoryFinished())
	{
		if (currentWp.type==ECOCUP_RED) {
			fmsSupervisor.setNextState(&reajustement);
		}
		else if (currentWp.type==TURNPOINT) {
			fmsSupervisor.setNextState(&pivot);
		}
		else {
			fmsSupervisor.setNextState(&etat_end);
		}
	}
}

void Travel::reEnter(unsigned long interruptTime)
{
}

void Travel::forceLeave()
{

}

void Travel::pauseNextState()
{
}
