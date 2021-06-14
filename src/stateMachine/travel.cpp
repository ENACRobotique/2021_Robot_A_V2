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
#include "trajectoryv2.h"
#include "../navigator.h"
#include "../ecocupManager.h"
#include "ReleaseEcocup.h"

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
	actionAFinir = 0;
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
	
	if (currentWp.bouger){
		Serial1.println("mouvement1");
		actionAFinir = 1;
		//on avance jusqu'à un point en amont
		//vérifier selon disposition des axes/angles
		//navigator.move_to(currentWp.x-dist_min*sin(PI/180*currentWp.angle), currentWp.y-dist_min*cos(PI/180*currentWp.angle));
		navigator.move_to(currentWp.x, currentWp.y);
	}
	if(!currentWp.angleAleatoire){
		actionAFinir = 2;
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
		actionAFinir--;
		if (!currentWp.angleAleatoire){
			navigator.turn_to(currentWp.angle);
		}
	}
	if (actionAFinir<=0){
		if (currentWp.type==ECOCUP_RED || currentWp.type==ECOCUP_GREEN) {
			fmsSupervisor.setNextState(&reajustement);
		} 
		else if (currentWp.type==RELEASE_GREEN || currentWp.type==RELEASE_RED){
			fmsSupervisor.setNextState(&releaseEcocup);
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
