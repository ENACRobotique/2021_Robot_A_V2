#pragma once
#include <i2c_t3.h>
#include <Adafruit_PWMServoDriver.h>
#include "params.h"
#include "servoManager.h"

/*
    On suppose que le robot s'est placé et orienté comme il faut.
    action: relâche l'écocup. Prévoir à l'avance la position d'empilement dans trajectory
*/
class EcocupManager{

private:
    void controlservo_k();
    void setPulse(uint8_t n, double pulse);
    uint16_t consigne[8];
    uint16_t currentAngle[8];
    bool finished;
    int hauteurPileVerte;//entre 0 et 2
    int hauteurPileRouge;
    bool couleurRouge;
    bool typeCapture;

public: 
    int capture();
    
    void release();

    int definir_action(bool couleurRouge, bool typeCapture);
    bool isFinished(){
        return servoManager.isFinished();
    }
    

};

extern EcocupManager ecocupManager;