/*
 * MoveToButtonPause.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_CAPTUREECOCUP_H_
#define STATEMACHINE_CAPTUREECOCUP_H_

#include "AbstractState.h"

class CaptureEcocup : public AbstractState {
public:
	CaptureEcocup();
	virtual ~CaptureEcocup();

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

extern CaptureEcocup captureEcocup;

#endif 