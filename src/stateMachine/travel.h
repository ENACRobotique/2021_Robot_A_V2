/*
 * 
 *
 *  Created on: 18 avr. 2018
 *      Author: Maxime
 */

#ifndef STATEMACHINE_Travel_H_
#define STATEMACHINE_Travel_H_

#include "AbstractState.h"
#include "../parcours.h"

class Travel : public AbstractState {
public:
	Travel();
	
	virtual ~Travel();

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
	int actionAFinir;
	//actionAFinir : pour définir plusieurs mouvements dans navigator.
	//pour suivre l'esprit, il faudrait créer des états "tourner" et "bouger", 
	// et leur passer des paramètres ou les superviser par "travel".. mais c'est lourd
	//une autre possibilité serait de modifier la valeur du navigator.istrajectory finished en fonction de l'action effectuée
};

extern Travel travel;

#endif 