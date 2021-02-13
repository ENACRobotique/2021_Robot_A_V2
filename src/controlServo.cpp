#include "controlServo.h"
#include "Arduino.h"
#include "PWMServo.h"
#include "params.h"

ControlServo::ControlServo(){
}

ControlServo::~ControlServo(){
}

void ControlServo::defInitAngle(int angle){
    init_angle = angle;
}

void ControlServo::init(int ServoPIN){
    servo.attach(ServoPIN);
    servo.write(init_angle);
    delay(500);    
}

void ControlServo::moveServo(int angle){
    servo.write(angle);
    delay(15);
}

void ControlServo::resetPos(){
    servo.write(init_angle);
    delay(15);
}

int ControlServo::returnPos(){
    return(servo.read());
}