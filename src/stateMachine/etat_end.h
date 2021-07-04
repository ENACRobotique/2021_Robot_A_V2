/*
 * MoveToButtonPause.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_END_H_
#define STATEMACHINE_END_H_

#include "AbstractState.h"
#include "../parcours.h"

class Etat_end : public AbstractState {
public:
	Etat_end();
	
	virtual ~Etat_end();

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

extern Etat_end etat_end;

#endif 