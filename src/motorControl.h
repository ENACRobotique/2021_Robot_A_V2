/*
 * motorControl.h
 *
 *  Created on: 20 mars 2018
 *      Author: elie
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

namespace MotorControl {
	void update();
	void init();

	float get_cons_speed();
	float get_cons_omega();
	void testmoteur(int cmde1, int cmde2);
	void set_cons(float speed, float omega);
}


#endif /* MOTORCONTROL_H_ */
