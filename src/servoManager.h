#pragma once
#include <i2c_t3.h>
#include <Adafruit_PWMServoDriver.h>
#include "params.h"


class ServoManager{

private:
    void controlservo_k();
    void setPulse(uint8_t n, double pulse);
    uint16_t consigne[8];
    uint16_t currentAngle[8];
    bool finished;
public: 
    ServoManager(){
        
    };
    void init(const uint16_t init_tab[8]);
    void controlServo(servos servo, uint16_t angle);
    void update();
    bool isFinished();

};

extern ServoManager servoManager;
extern servos servo_sel;