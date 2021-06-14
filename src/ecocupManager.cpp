#include "ecocupManager.h"
#include "servoManager.h"
#include "trajectoryv2.h" //pour IR_sel
#include "navigator.h" //pour le sens


#define TAILLEMAXPILE 2//on compte de 0 à n-1 
servos PileVerte[] = {servo_BR,servo_BL,servo_FR};
servos PileRouge[] = {servo_FL,servo_FC,servo_BC};

void EcocupManager::release(){
    servoManager.controlServo(servo_sel,180);
    if(couleurRouge && hauteurPileRouge>-1){
        hauteurPileRouge--;
    }
    if(!couleurRouge && hauteurPileVerte>-1){
        hauteurPileVerte--;
    }
}

void EcocupManager::capture(){
    servoManager.controlServo(servo_sel,100);
    if(couleurRouge && hauteurPileRouge<TAILLEMAXPILE){
        hauteurPileRouge++;
    }
    if(!couleurRouge && hauteurPileVerte<TAILLEMAXPILE){
        hauteurPileVerte++;
    }
}

/* couleurRouge=true/false typeCapture=0(false)/1(true) */
void EcocupManager::definir_action(bool estRouge, bool estCapture){
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
            servo_sel = PileVerte[hauteurPileVerte+1];
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

void EcocupManager::init(){
    hauteurPileRouge=-1;//-1 <=> 'pile vide'
    hauteurPileVerte=-1;
}

EcocupManager ecocupManager = EcocupManager();
