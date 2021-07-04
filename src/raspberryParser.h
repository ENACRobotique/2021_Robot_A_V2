#pragma once
#include "parcours.h"

struct autre {
    int id;
    Coords pos;
    // arguments suivants utile?
    float z;
    float theta;
    int boussole;
    float retard;
};

class raspberryParser{
public:
    raspberryParser();
    void update();
    float getX();
    float getY();
    float getZ();
    float gettheta();
    void getPos();
    int getBoussole();
    float getRetard();
    Coords getPosConcurrent();
    int getIDConcurrent();
    void parseData(char* buffer);


private:
    float x;
    float y;
    float z;
    float theta;
    int boussole;
    float retard;
    float id;
    struct autre concurrent; 

};

extern raspberryParser raspberryparser;