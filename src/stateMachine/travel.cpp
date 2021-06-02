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
	navigator.move_to(currentWp.x, currentWp.y);
}

void Travel::leave()
{
	Serial1.println("Leaving Travel");
}

void Travel::doIt()
{

	if (navigator.isTrajectoryFinished())
	{
		if (currentWp.type==ECOCUP) {
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
