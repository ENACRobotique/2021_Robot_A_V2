#include "Arduino.h"
//#define test
//#define servo

#ifdef test
#include "testI2C.h"

void setup(){
    testi2c_s();
}

void loop(){
    testi2c_l();
}
#endif

#ifdef servo
#include "ServoI2C.h"

void setup(){
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,HIGH);
    controlservo_s();
}
void loop(){
    controlservo_l();
}
#endif