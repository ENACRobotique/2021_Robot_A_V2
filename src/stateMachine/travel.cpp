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
	currentWp = traj.get_next_WP();
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
		fmsSupervisor.setNextState(&reajustement);
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
