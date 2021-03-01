/*
 * MoveToButtonPause.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_REAJUSTEMENT_H_
#define STATEMACHINE_REAJUSTEMENT_H_

#include "AbstractState.h"

class Reajustement : public AbstractState {
public:
	int delta_ajust_deg = 10;
	Reajustement();
	virtual ~Reajustement();

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

	unsigned long time_start;
};

extern Reajustement reajustement;


#endif
