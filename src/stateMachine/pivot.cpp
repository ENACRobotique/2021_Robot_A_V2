/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */
#include "pivot.h"
#include "Arduino.h"
#include "../FMSSupervisor.h"
#include "Reajustement.h"
#include "../params.h"
#include "etat_test.h"
#include "../navigator.h"
#include "travel.h"

Pivot pivot = Pivot();

Pivot::Pivot()
{
	currentWp=Waypoint(-3,0.0,0.0,BEGIN);//-3: pas d'id de point, ne correspon
	time_start = 0;
}

Pivot::~Pivot()
{
	// TODO Auto-generated destructor stub
}

void Pivot::enter()
{
	Serial1.println("Enter Pivot");
}

void Pivot::leave()
{
	Serial1.println("Leaving Pivot");
}

void Pivot::doIt()
{
	fmsSupervisor.setNextState(&travel);
}

void Pivot::reEnter(unsigned long interruptTime)
{
}

void Pivot::forceLeave()
{
}

void Pivot::pauseNextState()
{
}
