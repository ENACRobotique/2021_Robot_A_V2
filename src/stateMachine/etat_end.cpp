/*
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */
#include "etat_end.h"
#include "Arduino.h"
#include "../FsmSupervisor.h"
#include "Reajustement.h"
#include "../params.h"
#include "etat_test.h"
#include "../navigator.h"
#include "etat_begin.h"
#include "trajectoryv2.h"

Etat_end etat_end = Etat_end();

Etat_end::Etat_end()
{

}

Etat_end::~Etat_end()
{
	// TODO Auto-generated destructor stub
}

void Etat_end::enter()
{
	Serial1.println("Enter Etat end");
}

void Etat_end::leave()
{
	Serial1.println("Leaving Etat end");
}

void Etat_end::doIt()
{
	if (true) {
		fmsSupervisor.setNextState(&etat_begin);
		traj1.pos = 0;
	}
}

void Etat_end::reEnter(unsigned long interruptTime)
{
}

void Etat_end::forceLeave()
{
}

void Etat_end::pauseNextState()
{
}
