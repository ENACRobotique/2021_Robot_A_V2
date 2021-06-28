#include "MatchDirector.h"
#include "math.h"
#include "ActionsList.h"
#include "../params.h"
#include "../navigator.h" 
#include "../odometry.h"
#include "../FsmSupervisor.h" 
#include "../actuatorSupervisor.h"
#include "Arduino.h" //NULL definition
#include "utils.h"

/**
 * @brief 
 * @file MatchDirector.cpp
 * @ingroup namespace

 */

/**
 * @brief Control navigator (which is the high level interface to move the robot/give it a certain orientation) 
and FSM (which controls various actuators & execute codes depending on various states) 
depending on the time left & an order of action. 
Manage the score total, the score of an action is stored in Action. 
Can be modified by states if needed (for example, if an action is interacting with a variable amount of object & the score depend on this amount).
 * 
 */
namespace MatchDirector
{
    


    //ActionOrder actionOrder;


    bool isStartingLeft = true;
    bool isDrivingBackward = false; //if move with navigator with positive number, robot goes backward

    Action *curSection = NULL; // undefined size of array
    //Section = multiple actions that usually lead to points at the end
    int curActIndex = 0;
    ActionState actionState = BEGIN;

    /**
     * @brief When there is a huge discrepancy between targeted position & position reached, this get reincremented and the same command is sent again
     *  @ingroup namespace
     */
    int nbReadjust = 0; //

    /**
     * @brief Number of time we can correct trajectory according to wheel encoder, in order not to be stuck trying to reach an unnatable precise position
     * 
     */
    int nbCorectionAuthorized = 0;
    float timer = 10; // en s, durée du match
    int score = 0;
    float offsetX = 0; //offsets au début du terrain par rapport à l'abs
    float offsetY = 0;
    uint32_t start_millis;
    bool moveBackToBase;
    /**
     * @brief Flag that is set depending if the activities is done in the 
     * 
     */
    int activitiesDoneFlag = 0; 

void init()
{
    start_millis = millis();
    moveBackToBase = false;
    //TODO : call only when match has started;
    //TODO : implement timer count;

    //en fonction de la taille du terrain 

    offsetY = 1000.0f; //largeur terrain/2
    offsetX = (isStartingLeft) ? robot_center_x : (3000.0f - robot_center_x); //3000-> longueur terrain
    //DEBUG : 
    //curSection = EcocupsTopLeft;
}

float distance_squared (float xA, float yA, float xB, float yB)
{
    return (xA-xB)*(xA-xB)+(yA-yB)*(yA-yB);
}

void abs_coords_to(float x, float y)
{  
    //Serial1.print("commande envoyé : move_to local : ");
    navigator.move_to(abs_x_to_local(x), abs_y_to_local(y));
    
}

float abs_x_to_local(float x)
{
    if((!isStartingLeft && !isDrivingBackward) || (isStartingLeft && isDrivingBackward))
    {
        return -x + offsetX;
    }
    else
    {
        return x - offsetX;
    }
}

float abs_y_to_local(float y)
{
    if((!isStartingLeft && !isDrivingBackward) || (isStartingLeft && isDrivingBackward))
    {
        return -y + offsetY;
    }
    else
    {
        return y - offsetY;
    }
}

float get_abs_x()
{
    if((!isStartingLeft && !isDrivingBackward) || (isStartingLeft && isDrivingBackward))
    {
        return -odometry_motor.get_pos_x() + offsetX;
    }
    else 
    {
        return odometry_motor.get_pos_x() + offsetX;
    }
}

float get_abs_y()
{
    if((!isStartingLeft && !isDrivingBackward) || (isStartingLeft && isDrivingBackward))
    {
        return -odometry_motor.get_pos_y() + offsetY;
    }
    else 
    {
        return odometry_motor.get_pos_y() + offsetY;
    }
}
float timeToReachCoords(float begX, float begY, float targetX, float targetY)
/* rough estimate using ACCEL MAX, doesn't take into account time to turn to a certain angle*/
{
    float time = 0;
    float distance = sqrt((targetX-begX)*(targetX-begX)+(begY-targetY)*(begY-targetY));
    
    if(distance >= ACCEL_MAX)
    {
         time = distance/(ACCEL_MAX) + 2; //trapèze de vitesse majorée
    }
    else
    {
        float speed = ACCEL_MAX / 2;
        time = distance/speed + 1;
    }
    return time;

}

//Handle the logic/state of an action : should it move, should it execute the next state,...
void action_dispatcher(Action action)
{
    //start only when fsmSupervisor has no state
    if(actionState == BEGIN && fsmSupervisor.is_no_state_set())
    {
        Serial1.print("new action :");
        Serial1.print("\t");
        Serial1.print(action.x);
        Serial1.print("\t");
        Serial1.print(action.y);
        Serial1.print("\t");
        Serial1.println(action.angle);
        //Serial1.println("actionState == begin");
        if(curActIndex == 0) //pathfinding avec waypoint uniquement entre les sections (car gros déplacement)
        {
            route_from_action(action.x,action.y);
        }
        else //pathfinding uniquement 
        {
            abs_coords_to(action.x,action.y);
        }

        
        actionState = MOVING;
    }

    else if(actionState == MOVING)
    {
        if (navigator.isTrajectoryFinished())
        {
                if(curActIndex == 0)
                {

                }
                // Si on est pas suffisament proche de la position et qu'on a le droit de se réajuster (permission lié à un "timeout" pour pas perdre trop de tps à se réajuster)
                else if (distance_squared(get_abs_x(), get_abs_y(), action.x,action.y) > ADMITTED_POSITION_ERROR*ADMITTED_POSITION_ERROR
                    && nbReadjust < nbCorectionAuthorized / 2) // on divise par 2 pour laiser la moitié de corrections autorisés à la correction d'angle
                {
                    Serial1.println("reajustement position car erreur trop grande (réel vs erreur admise): ");
                    nbReadjust++;
                }
                else
                {
                    actionState = TURNING;
                }
            if(!(action.angle <= -360.f)) // -180 <= action.angle <= 180° pour être pris en compte, (normalement donc on met 360 au cas où)
            {
                Serial1.print("turning to angle : ");
                Serial1.println(action.angle);
                navigator.turn_to(action.angle);
            }

            actionState = TURNING;
        }
        /*
        //le bloc ci-dessous se lance si on est avant le timer indiqué dans ActionList, ou si on est arrivé à destination/quasi destination 
        if ((fsmSupervisor.is_no_state_set() || !fsmSupervisor.is_switching_state())
            && ((timeToReachCoords(get_abs_x(), get_abs_y(), action.x,action.y) <= action.countdownState +1.1f) //car time toReachCoords >= 1 à tout moment
                || distance_squared(get_abs_x(), get_abs_y(), action.x,action.y) <= ADMITTED_POSITION_ERROR*ADMITTED_POSITION_ERROR))
        {
            Serial1.println("actionState - fsmSueprvisor nextState ");
            fsmSupervisor.setNextState(action.state);
        } 
        */
    }

    else if(actionState == TURNING && navigator.isTrajectoryFinished())// && fsmSupervisor.is_no_state_set())
    {          

        if(abs(action.angle-odometry_wheel.get_pos_theta()) > ADMITTED_DEG_ANGLE_ERROR && nbReadjust < nbCorectionAuthorized)
        {
            Serial1.println("reajustement angle");
            Serial1.println(odometry_wheel.get_pos_theta());
            nbReadjust++;
            actionState = MOVING;
        }


        //Serial1.println(navigator.theta_target);
        Serial1.println("actionState turning - fsm nextState ");
        Serial1.print("real pos : ");
        Serial1.print(get_abs_x());
        Serial1.print("\t");
        Serial1.println(get_abs_y());
         fsmSupervisor.setNextState(action.state);
        //Serial1.println("actionState - turning done");
        curActIndex++;
        nbReadjust = 0;
        actionState = BEGIN;
    } 
}

void update()
{ 
    Action curAction = curSection[curActIndex];
    //Une section : un ensemble d'actions, action_dispatcher s'occupe de voir ce qu'il doit faire avec une action ou passer à l'action suivante pour la prochaine loop
    if(curSection != NULL && !ActionList::isNull(&curAction))
    {
       action_dispatcher(curAction);
    }
    // remplacer avec un else if pas de retoru à la base vu le temps restant
    /*
    if(curSection == NULL || ActionList::isNull(&curAction))
    {
        Serial1.println("curSection is null or curAction is invalid !");

        //TODO : changement de curSection, dans un ordre prédeterminé sauf si le chemin est occupé:
        /*
        Recup ecueil 
        phare
        Dépose ecueil
        Récup ecueil
        Manche à air
        dépose écueil

        Checker si la zone est dispo, sinon faire autre chose
        Si rien n'est dispo : attendre

        Si plus rien à faire : rentrer à la base/circuit d'attente
        Si temps pressant (< 30s), on rentre à la base
            
        */
    //}
    
    if((millis()-start_millis > timer*1000-10000) & !moveBackToBase)
    {
        moveBackToBase = true;
        //set_current_action(*(compute_final_point()));
    }
    if(millis()-start_millis > timer*1000-5000) // -5000 : hardcode du pavillon qui doit se déclencher à 5s de la fin
    {
        //ActuatorSupervisor::deploy_pav();

    }
    //->passer à l'etat suivant si le fsm est à un état "final", càd sans état supplémentaire prévue par défaut
    //Par exemple, pour le récupérage de gobelet, on doit rester longtemps dans l'état, donc jusqu'à ce qu'il soit dans "deadState", on attends

}


void set_current_action(Action *action)
{
    curSection = action;
}

//to be called when receveing information from raspberry about girouette in communication.h
void compute_final_point(bool isGirouetteWhite) 
//White : Sud, Black : Nord
{
    /*
    float theta = 0.f;

    if(isStartingLeft)
    {
        ActionList::GetToFinal[0].x = 200.f;
        ActionList::GetToFinal[0].y = (isGirouetteWhite) ? 500.f : 1500.f;
        ActionList::GetToFinal[0].angle = 0.0f;
    }
    else
    {
        ActionList::GetToFinal[0].x = 2800.f;
        ActionList::GetToFinal[0].y = (isGirouetteWhite) ? 500.f : 1500.f;
        ActionList::GetToFinal[0].angle = 0.0f;
    }
    */
}

void addScore(int add)
{
    score += add;
}

PointSeq route_from_action(float actionX, float actionY)
{
        float entry[2] = {get_abs_x(), get_abs_y()};
        float exit[2] = {actionX, actionY};
        float robotPos[3] = {get_abs_x(), get_abs_y(), odometry_wheel.get_pos_theta()};
        return route_to_follow(entry, exit, robotPos);
}

PointSeq route_to_follow(float* entry, float* exit, float* robot_pos)
{
    
    Route route = ATC::find_route(&ATC::graph, entry, exit, ATC::lidar, robot_pos);
    return ATC::read_route(route);
    //ATC::
}
/* méthodes à rajouter :
Temps estimé pour réaliser action
(Heuristique action/temps réalsier une action)

externes : 
setArrivalCoordinate() ->comm.cpp pour arriver à bon port

(freeEcocup() -> comm.cpp pour autoriser d'autres coords ??)

*/




} // namespace MatchDirector
