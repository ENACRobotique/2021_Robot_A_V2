#pragma once

class raspberryParser{
public:
    raspberryParser();
    void update();
    float getX();
    float getY();
    float getZ();
    int getBoussole();
    float getRetard();
    void parseData(char* buffer);


private:
    float x;
    float y;
    float z;
    int boussole;
    float retard;

};

extern raspberryParser raspberryparser;