/*
 * FMSSupervisor.h
 *
 *  Created on: 11 avr. 2018
 *      Author: robot
 */

#ifndef STATEMACHINE_FMSSUPERVISOR_H_
#define STATEMACHINE_FMSSUPERVISOR_H_

#include "AbstractState.h"

class FMSSupervisor {
public:
	FMSSupervisor();
	virtual ~FMSSupervisor();
	void setNextState(AbstractState* state);
	void update();
	void init();

private:

	AbstractState* nextState;
	AbstractState* currentState;
	AbstractState* previousState;
	unsigned long time_obstacle_left;
};

extern FMSSupervisor fmsSupervisor;

#endif /* STATEMACHINE_FSMSUPERVISOR_H_ */
