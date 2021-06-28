#include "ActionsList.h"
#include "Action.h"

//#endif

/**
 * @brief action must have nullAction at the end, because it's an "infinite" length array in order to know in matchdirector when to stop and avoid overflow
 * @ingroup namespace
 */
namespace ActionList
{

    int *score_counter;

    Action NullAction = {MINIMUM_FLOAT, MINIMUM_FLOAT, MINIMUM_FLOAT, nullptr, MINIMUM_FLOAT};

    
    void set_timer_counter(int* timer)
    {
        score_counter = timer;
    }

    bool isNull(Action* action)
    {
        if (
            (*action).x == NullAction.x
            && (*action).y == NullAction.y
            && (*action).angle == NullAction.angle
            && (*action).state == NullAction.state
            && (*action).countdownState == NullAction.countdownState
        )
        {
            return true;
        }
        return false;

    }
    #pragma region ecocups

    #pragma endregion

    #pragma region switch_sections


    #pragma endregion

    

    Action RamassageEcocup[10] = {
        {-1250.0,500.0,-400.f, &deployLeftFrontServo, 0.0f},
        /*
        {-1200.0,500.0,ECOCUP_RED, true, false, 180},
        {-1200.0,1000.0,ECOCUP_RED, true, false, 180},
        //{-1200.0,100.0,ECOCUP_RED, true, false, 180},
        {-500.0,100.0, RELEASE_RED, true, false, 180},
        {-100.0,100.0, RELEASE_RED, true, false, 180},
        //{0.0,0.0,TURNPOINT},
        {-1250.0,500.0,END, true, false, 180}
        */
        NullAction 
    };
  
/*
    //X = 850
    Action EcocupsTopLeft[10] = {
        //{700.f, 1980.f - sizeHalfLengthRobot, 90.f, &deployFrontServo, 3.0f}, //Going a little bit before the wall + deploy servo before arrival
        {700.f, 1980.f - sizeHalfLengthRobot, 90.f, &recalibration_wall_top, 0.f},
        //{700.f, 2000.f- sizeHalfLengthRobot, 90.f, &wait_font_cup_succ, 0.f}, 
        //{700.f, 2000.f- sizeHalfLengthRobot, 90.f, &retractFrontServo, 0.f},
        {700.f, 1800.f, 90.f, &switchPhare, 0.f}, //backing off the wall$ //TODO : faire un SwitchSection
        NullAction 


    };
*/
    

    //Doit être modifié par Get_To_Final dans match director avec les bonnes coords/angle
    /*
    Action GetToFinal[2] =
    {
        {200.f, 100.f, 0.f, &etat_begin, 0.0f},
    };
    */




}
 
