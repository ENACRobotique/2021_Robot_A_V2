/*
 * MoveToButtonPause.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_Travel_H_
#define STATEMACHINE_Travel_H_

#include "AbstractState.h"
#include "../trajectory.h"

class Travel : public AbstractState {
public:
	Travel();
	
	virtual ~Travel();

	void doIt();
	void leave();
	void enter();
	void reEnter(unsigned long interruptTime);
	void forceLeave();
	void pauseNextState();
	unsigned long get_time_start(){
		return time_start;
	}

private:
	Waypoint currentWp;
	unsigned long time_start;
};

extern Travel travel;

#endif 