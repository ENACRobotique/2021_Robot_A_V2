#pragma once

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
    void parseData(char* buffer);


private:
    float x;
    float y;
    float z;
    float theta;
    int boussole;
    float retard;

};

extern raspberryParser raspberryparser;