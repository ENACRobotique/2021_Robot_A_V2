#include "communication.h"
#include "Arduino.h"
#include <stdlib.h>
#include "motorControl.h"
#include "navigator.h"
#include "odometry.h"
#include "FsmSupervisor.h"
//#include "ai/MatchDirector.h"
//#include "ai/ActionsList.h"
#include "params.h"
//#include "actuatorSupervisor.h"

#include "stateMachine/Recalibration_wall.h"

#include "stateMachine/travel.h"

#define COM_DEBUG

namespace Communication {
    
    static void parse_data();

    char buffer[100];
    int buff_index=0;
    void update() {
        //Serial.printf("entree update comm\n");

        int a;
        a = Serial1.available();
        
        if (a) {
            
            for (int k=0;k<a;k++) {
                char c=Serial1.read();
                //Serial1.write(c);
                if (c=='\n'){
                   
                   buffer[buff_index]='\0';
      //             Serial1.write(buffer,i);
                   parse_data();
                   //value=atoi(buffer);
                   buff_index=0;

                }
                else if (c=='\r') {}
                else {
                    buffer[buff_index]=c;
                    buff_index++;
                }
            }
        }



    }

    static void parse_data(){
        Serial.printf("buffer : %s\n",buffer);
        if(buffer[0] == 's') {
            MotorControl::set_cons(0,0);
            navigator.forceStop();
            fmsSupervisor.setNextState(&etat_begin);
            #ifdef COM_DEBUG
            Serial1.println("Stopped!");
            #endif
        }
        else if(buffer[0] == 'm') {
            float x,y;
            int nb = sscanf(buffer, "m %f %f", &x, &y);
            if(nb == 2) {   
                navigator.move_to(x, y);
                #ifdef COM_DEBUG
                Serial1.print("Moving to ");
                Serial1.print(x);
                Serial1.print("\t");
                Serial1.println(y);
                #endif
            }
        }
        else if(buffer[0] == 'o') {
            Serial1.print("pos - odometry_motor : ");
            Serial1.print(odometry_motor.get_pos_x());
            Serial1.print("\t");
            Serial1.print(odometry_motor.get_pos_y());
            Serial1.print("\t");
            Serial1.println(odometry_motor.get_pos_theta());
        }
        else if(buffer[0] == '2') {
            Serial1.print("pos - odometry_wheel : ");
            Serial1.print(odometry_wheel.get_pos_x());
            Serial1.print("\t");
            Serial1.print(odometry_wheel.get_pos_y());
            Serial1.print("\t");
            Serial1.println(odometry_wheel.get_pos_theta());
        } 
        else if(buffer[0] == '1') {
            Serial1.print("pos - absolue :  ");
            Serial1.print(MatchDirector::get_abs_x());
            Serial1.print("\t");
            Serial1.println(MatchDirector::get_abs_y());
        } 
        else if(buffer[0] == 't') {
            //in degrees
            float angle;
            int nb = sscanf(buffer, "t %f", &angle);
            if(nb == 1) {
                navigator.turn_to(angle);
            }
        }
        else if(buffer[0] == 'i'){
           // Serial.println("truc");
            raspberryparser.parseData(buffer);
            kalmanFilter::update(raspberryparser.getX(),raspberryparser.getY(),raspberryparser.gettheta());            
            Serial1.printf("odometry : %f %f %f\n",Odometry::get_pos_x(),Odometry::get_pos_y(),Odometry::get_pos_theta());
            kalmanFilter::print_state();


        }
        else if(buffer[0]=='c') {
            fmsSupervisor.setNextState(&travel);
        }
        else if(buffer[0]=='e') {
            fmsSupervisor.print_State();
        }
        else if (buffer[0]=='p') {
            MotorControl::testmoteur(50,50);
            Serial1.printf("pong\n");
        }
        else if (buffer[0]=='0') {
            Serial1.println("entre en test mode");
            fmsSupervisor.set_testmode(true);
        }

        buff_index = 0;
    }
}       
