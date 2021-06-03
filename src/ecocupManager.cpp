#include "ecocupManager.h"
#include "servoManager.h"
#include "trajectory.h" //pour IR_sel
#include "navigator.h" //pour le sens


#define TAILLEMAXPILE 3
servos PileVerte[] = {servo_FL,servo_BR,servo_BC};
servos PileRouge[] = {servo_FR,servo_BL,servo_FC};

void EcocupManager::release(){
    servoManager.controlServo(servo_sel,180);
    if(couleurRouge && hauteurPileRouge>0){
        hauteurPileRouge--;
    }
    if(!couleurRouge && hauteurPileVerte>0){
        hauteurPileRouge--;
    }
}

int EcocupManager::capture(){
    ///pas fini, à corriger!!!!!!!!!!!!!!!
    servoManager.controlServo(servo_sel,100);
    if (this->hauteurPileRouge < TAILLEMAXPILE){
        servo_sel = PileRouge[this->hauteurPileRouge];
        
        (this->hauteurPileRouge)++;
        return 0;
    }
    return 1;
}

/* couleurRouge=true/false typeCapture=0(false)/1(true) */
int EcocupManager::definir_action(bool estRouge, bool estCapture){
    couleurRouge=estRouge;
    typeCapture=estCapture;

    if (couleurRouge){
        if(typeCapture && hauteurPileRouge < TAILLEMAXPILE){
            servo_sel = PileRouge[hauteurPileRouge+1];
        } else {
            servo_sel = PileRouge[hauteurPileRouge];
        }
    } else {
        if(typeCapture && hauteurPileVerte < TAILLEMAXPILE){
            servo_sel = PileVerte[this->hauteurPileVerte+1];
        } else {
            servo_sel = PileVerte[hauteurPileVerte];
        }
    }

    IR_sel = (sensors)((int)servo_sel + (int)IR_FL);
    if(servo_sel==servo_FL || servo_sel==servo_FR || servo_sel==servo_FC){
        navigator.set_sens(1);
    } else {
        navigator.set_sens(0);
    }

}
EcocupManager ecocupManager = EcocupManager();
