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

void raspberryParser::parseData(char* buffer){
    int id;
    float u;
    int nb = sscanf(buffer, "i %d %d %f %f %f %f",&boussole, &id, &u, &u, &u, &u);
    if(id == ARUCO_ID && nb == 6){
        sscanf(buffer, "i %d %d %f %f %f %f",&boussole, &id, &x, &y, &z, &retard);
        Serial1.printf("i %d %d %f %f %f %f",boussole, id, x, y, z, retard);
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
void raspberryParser::getPos(){
}