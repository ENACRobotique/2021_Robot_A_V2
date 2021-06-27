/*
 * MoveToButtonPause.h
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#pragma once

#include "AbstractState.h"

/* recalibration is done on a wall perpendicular to the horizontal or vertical axis => we recalibrate only the x or y axis*/

class Recalibration_wall : public AbstractState {
public:
	Recalibration_wall(float targetPos, float targetTheta, bool isX); //targetTheta en radian !
	virtual ~Recalibration_wall();

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
	float wheel_cod_last_reading;
	bool isX;
	float targetPos;
	float targetTheta;
};

extern Recalibration_wall recalibration_wall_left;
extern Recalibration_wall recalibration_wall_top;
extern Recalibration_wall recalibration_wall_bottom;
extern Recalibration_wall recalibration_wall_right;