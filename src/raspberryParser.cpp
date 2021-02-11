#include "raspberryParser.h"

#include "params.h"

raspberryParser raspberryparser = raspberryParser();

raspberryParser::raspberryParser(){
    x = 0;
    y = 0;
    z = 0;
    boussole = -1;
    retard = 0;
}

void raspberryParser::update(){
    char buffer[50];
    int buff_index=0;
    int a = Serial1.available();

    if(a){
        for (int k=0;k<a;k++) {
                char c=Serial1.read();
                if (c=='\n'){
                   buffer[buff_index]='\0';
                   parseData(buffer);
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

void raspberryParser::parseData(char* buffer){
    int id;
    float u;
    int nb = sscanf(buffer, "%d %d %f %f %f %f",&boussole, &id,&u,&u,&u,&u);
    if(id == ARUCO_ID && nb == 6){
        sscanf(buffer, "%d %d %f %f %f %f",&boussole, &id, &x, &y, &z, &retard);
    }
}



float raspberryParser::getX(){
    return x;
}
float raspberryParser::getY(){
    return y;
}
float raspberryParser::getZ(){
    return z;
}
int raspberryParser::getBoussole(){
    return boussole;
}
float raspberryParser::getRetard(){
    return retard;
}