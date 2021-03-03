/*
 * MoveToButtonPause.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_ETAT_TEST_H_
#define STATEMACHINE_ETAT_TEST_H_

#include "AbstractState.h"

class Etat_test : public AbstractState {
public:
	Etat_test();
	virtual ~Etat_test();

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

extern Etat_test etat_test;

#endif 