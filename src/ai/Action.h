#ifndef ACTION_H
#define ACTION_H

#include "../stateMachine/AbstractState.h"

typedef enum 
{
    BEGIN,
    MOVING,
    TURNING,
    DOSTATE,
}ActionState;
struct Action
{
    float x;
    float y; //navigator turn automatically to go in straight line towards it
    float angle = 0;//en deg
    AbstractState *state;
    float countdownState = 0;//state to execute x time before reaching coords
};

/**
 * @brief à utiliser pour terminer un array d'actions, car les fonctions qui recoivent ces array dans matchdirector ne sont pas capable de savoir quand l'array se termine sans
 * */
extern Action NullAction;

#endif