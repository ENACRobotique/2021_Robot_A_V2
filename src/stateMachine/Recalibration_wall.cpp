/*
 * MatchDirector.h
 *
 *  Created on: 09/06/21
 *      Author: Jonathan Muchler
 */
/*
#include "../FsmSupervisor.h"
//#include "../ai/MatchDirector.h"
#include "Recalibration_wall.h"
#include "etat_begin.h"
#include "navigator.h"
#include "odometry.h"
#include "math.h"

//Recalibration : valeurs configurés en supposant que le robot se reconfigure vers L'AVANT et pas l'arrière (faire attention en l'amenant devant le mur)
//prendre en compte pas les coordonnées dumur mais ceux du centre du robot
Recalibration_wall recalibration_wall_left = Recalibration_wall(150.0f, 3.14f, true);
Recalibration_wall recalibration_wall_top = Recalibration_wall(1850.f, 1.57f, false);
Recalibration_wall recalibration_wall_bottom = Recalibration_wall(100.0f, -1.57f, false);
Recalibration_wall recalibration_wall_right = Recalibration_wall(2900.0f, 0.0f, true);
*/
/**
 * targetPos : en absolue (reconvertie en local en doIt)
 * targetTheta : en radian
 * isX : si c'est à faux, ça reconfigure le y
 * **/
/*
Recalibration_wall::Recalibration_wall(float targetPos, float targetTheta, bool isX) { //if isX = false => Y,
	time_start = 0;
	this->targetPos = targetPos;
	this->targetTheta = targetTheta;
	this->isX = isX;
	wheel_cod_last_reading = -1; //arbitrary value outside of the terrain
}

Recalibration_wall::~Recalibration_wall() {
	// TODO Auto-generated destructor stub
}

void Recalibration_wall::enter() {
	Serial1.print("recalibrating");
	Serial1.print(targetPos);
	Serial1.print(isX);
	navigator.move(50.0f, 0); //40 mm/s
}

void Recalibration_wall::leave() {
	Serial1.print("end recalibrating wall");
}

void Recalibration_wall::doIt() {

	float cur_reading;
	if(isX)
	{
		//cur_reading = odometry_wheel.get_pos_x();
	}
	else
	{
		cur_reading = odometry_wheel.get_pos_y();
	}

	if(fabs(cur_reading-wheel_cod_last_reading) <= 0.01f && wheel_cod_last_reading != -1)
	{
		navigator.forceStop();
		float x = (isX) ? MatchDirector::abs_x_to_local(targetPos) : odometry_wheel.get_pos_x(); //on recoit des coordonnées locales, donc faut les reconvertir
		float y = (!isX) ? MatchDirector::abs_y_to_local(targetPos) : odometry_wheel.get_pos_y();
		odometry_wheel.set_pos(x,y, targetTheta);
		odometry_motor.set_pos(x,y, targetTheta);
		fsmSupervisor.setNextState(&etat_begin);
	}
	wheel_cod_last_reading = cur_reading;

}


void Recalibration_wall::reEnter(unsigned long interruptTime){
}

void Recalibration_wall::forceLeave(){
}

void Recalibration_wall::pauseNextState(){
}
*/