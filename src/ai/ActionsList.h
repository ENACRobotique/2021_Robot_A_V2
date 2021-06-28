/*
 * MatchDirector.h
 *
 *  Created on: 09/06/21
 *      Author: Jonathan Muchler
 */

#ifndef ACTIONS_LIST_H
#define ACTIONS_LIST_H

#include "Action.h"
#include "../stateMachine/AbstractState.h"
namespace ActionList
{
   
    #define MINIMUM_FLOAT -3.4028235E+38
    extern Action NullAction;
    /**
    @brief return true if action == NullACtion */
    bool isNull(Action* action);
    void set_timer_counter(int timer);



    #pragma region partie_gauche

    //On redepose left ici
    //Puis on go final

    #pragma endregion
    //extern Action GetToFinal[2];
}


#endif
