/*
 * MoveToButtonPause.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_PIVOT_H_
#define STATEMACHINE_PIVOT_H_

#include "AbstractState.h"
#include "../parcours.h"

class Pivot : public AbstractState {
public:
	Pivot();
	
	virtual ~Pivot();

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

extern Pivot pivot;

#endif 