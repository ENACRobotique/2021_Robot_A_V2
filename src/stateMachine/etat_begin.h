/*
 * MoveToButtonPause.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_ETAT_BEGIN_H_
#define STATEMACHINE_ETAT_BEGIN_H_

#include "AbstractState.h"

class Etat_begin : public AbstractState {
public:
	Etat_begin();
	virtual ~Etat_begin();

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

extern Etat_begin etat_begin;

#endif 