#include "Essai_Servo.h"
#include <Arduino.h>
#include <PWMServo.h>
#include "controlServo.h"

const int NB_SERVO = 6;
const int PIN_SERVO_CUP[NB_SERVO];
const int PIN_SERVO_FLAG;
const int SERVO_FLAG_ANGLE = 90;
const int SERVO_CUP_ANGLE = 90;
const int SERVO_POS_NULL = 0;

ControlServo servoTest;

ControlServo servoCupFront1, servoCupFront2, servoCupFront3, servoCupBack1, servoCupBack2, servoCupBack3;
ControlServo servoCup[NB_SERVO];
ControlServo servoFlag;

int i = 0;

void setup(){
    // Mettre à jour les valeurs avec les bons pins
    PIN_SERVO_CUP[0];
    PIN_SERVO_CUP[1];
    PIN_SERVO_CUP[2];
    PIN_SERVO_CUP[3];
    PIN_SERVO_CUP[4];
    PIN_SERVO_CUP[5];
    PIN_SERVO_FLAG;
    delay(3000);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    Serial.begin(115200);
    Serial.println("Init...");

    for(i=0; i<NB_SERVO; i++)
    {
        servoCup[i].init(PIN_SERVO_CUP[i]); 
        Serial.println(servoCup[i].returnPos()); 
    }
    
}

void loop()
{
    //tester un branchement et une séquence avec 6 servo pour les verres et un servo de Flag
    for(i=0; i<NB_SERVO; i++)
    {
        servoCup[i].moveServo(SERVO_CUP_ANGLE);
        delay(500);
    }
    for(i=0; i<NB_SERVO; i++)
    {
        servoCup[i].moveServo(SERVO_POS_NULL);
        delay(500);
    }
}

void initServoTest()
{
    servoTest.init(PIN_SERVO_CUP[0]); 
    Serial.println(servoTest.returnPos());
}

void testServoRotation()
{
   servoTest.moveServo(i);
    if (i == 1){
        delay(500);
    }
    Serial.println(servoTest.returnPos());
    i++;
    Serial.println("---");
    if (servoTest.returnPos() == 180){
        delay(1000);
        servoTest.resetPos();
        i = 0;
    } 
}