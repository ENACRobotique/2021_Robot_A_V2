#include "communication.h"
#include "Arduino.h"
#include <stdlib.h>
#include "motorControl.h"
#include "navigator.h"
#include "odometry.h"
#include "raspberryParser.h"

#define COM_DEBUG

namespace Communication {
    
    static void parse_data();

    char buffer[50];
    int buff_index=0;
    void update() {
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
        //switch(buffer[0]) ?
        if(buffer[0] == 's') {
            MotorControl::set_cons(0,0);
            navigator.forceStop();
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
            Serial1.print("pos: ");
            Serial1.print(Odometry::get_pos_x());
            Serial1.print("\t");
            Serial1.print(Odometry::get_pos_y());
            Serial1.print("\t");
            Serial1.println(Odometry::get_pos_theta());
        } else if(buffer[0] == 't') {
            //in degrees
            float angle;
            int nb = sscanf(buffer, "t %f", &angle);
            if(nb == 1) {
                navigator.turn_to(angle);
            }
        }
        else if(buffer[0] == 'i'){
            raspberryparser.parseData(buffer);
        }
        else if(buffer[0] == 'v') {
            float v,omega;
            int nb = sscanf(buffer, "v %f %f", &v, &omega);
            if(nb == 2) {
                navigator.move(v, omega);
                #ifdef COM_DEBUG
                Serial1.print("Moving to ");
                Serial1.print(v);
                Serial1.print("\t");
                Serial1.println(omega);
                #endif
            }

        buff_index = 0;
    }
}       