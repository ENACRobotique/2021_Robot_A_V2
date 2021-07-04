#include "raspberryParser.h"

#include "params.h"

raspberryParser raspberryparser = raspberryParser();

raspberryParser::raspberryParser(){
    x = 0;
    y = 0;
    z = 0;
    theta = 0;
    boussole = -1;
    retard = 0;
}

void raspberryParser::parseData(char* buffer){
    int id;
    float u;
    int nb = sscanf(buffer, "i %d %d %f %f %f %f %f",&boussole, &id, &u, &u, &u, &u, &u);
    if(nb == 7){
        if (id == ARUCO_ID){
            sscanf(buffer, "i %d %d %f %f %f %f %f",&boussole, &id, &x, &y, &z, &theta, &retard);
            //Serial.printf("val rasp : i %d %d %f %f %f %f %f\n",boussole, id, x, y, z, theta, retard);
        } else {
            sscanf(buffer, "i %d %d %f %f %f %f %f",&u, &concurrent.id, &concurrent.pos.x, &concurrent.pos.y, &concurrent.z, &concurrent.theta, &concurrent.retard);
            traj1.modifieObstacle(concurrent.id, concurrent.pos);
        }
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
float raspberryParser::gettheta(){
    return theta;
}
int raspberryParser::getBoussole(){
    return boussole;
}
float raspberryParser::getRetard(){
    return retard;
}
void raspberryParser::getPos(){
}


Coords raspberryParser::getPosConcurrent(){
    return concurrent.pos;
}

int raspberryParser::getIDConcurrent(){
    return concurrent.id;
}