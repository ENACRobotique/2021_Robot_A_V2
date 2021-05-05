#include "Arduino.h"
#include <string>
#include "lidar.h"
#include "params.h"


namespace Lidar {
    Spot spots[3];

    int lidarIns[] = {LIDARIN1,LIDARIN2,LIDARIN3,LIDARIN4,LIDARIN5}; //à définir
    void changeReadSpace(Space zone){
        int zn = (int)(zone);
        int i;
        for(i=0; i<5; i++) {    
            zn=zn%2;
            digitalWrite(lidarIns[i],zn);
            zn=zn/2;
        }    
    }
    void changeReadSpace2(int zone){
        changeReadSpace((Space)zone);
    }

    //Space nextsectors[3];
    Space trans(Space etat) {
        switch (etat) {
            case DL:
                return DL1;
            case DC:
                return DC1;
            case DR:
                return DR1;
            case AL:
                return AL1;
            case AC:
                return AC1;
            case AR:
                return AR1;
            default:
                break;
        }
            if (3<=etat && etat<=6) {
                return etat;
            }
            if (7<=etat && etat<=10) {
                return etat;
            }
            if (11<=etat && etat<=14) {
                return etat;
            }
            return etat;
    }     

    void reset(int numspot) {
        spots[numspot].dist = (Space)((int)DL + numspot);
        spots[numspot].azim = (Space)((int)AL + numspot);
    }

    void init() {
        for(int i=0;i<5;i++){
            pinMode(lidarIns[i], OUTPUT);
        }
        pinMode(LIDAROUT1, INPUT_PULLDOWN);
        pinMode(LIDAROUT2, INPUT_PULLDOWN);
        pinMode(LIDAROUT3, INPUT_PULLDOWN);
        reset(0);
        reset(1);
        reset(2);
    }

    void update(int numspot, bool d_a=true) {
        Space zone;
        if (d_a){
            zone = spots[numspot].dist;
        }
        else {
            zone = spots[numspot].azim;
        }
        Space ns;
        ns=trans(zone);
        changeReadSpace(zone);
       
        if (ns<=11) {
            spots[numspot].dist=ns;
            spots[numspot].sector_dist=0;
            if (digitalRead(LIDAROUT1)==HIGH) { //détection dans le moins englobant
                spots[numspot].sector_dist=1;
            }
            else if (digitalRead(LIDAROUT2)==HIGH) {
                spots[numspot].sector_dist=2;
            }
            else if (digitalRead(LIDAROUT3) == HIGH) {
                 //détection dans le plus englobant
                spots[numspot].sector_dist=3;
            }
            else {
                reset(numspot);
            }
        }
        else {
            spots[numspot].azim = ns;
            spots[numspot].sector_azim=0;
            if (digitalRead(LIDAROUT1)==HIGH) {
                spots[numspot].sector_azim+=1;
            }
            if (digitalRead(LIDAROUT2)==HIGH) {
                spots[numspot].sector_azim+=2;
            }
            if (digitalRead(LIDAROUT3) == HIGH) {
                spots[numspot].sector_azim+=4;
            }
            if (spots[numspot].sector_azim==0) {
                reset(numspot);
            }
        }
    }

    void update_all() {
        int i;
        for (i=0;i<3;i++) {
            update(i,true);
            update(i,false);
        }
        readState();
    }

    static char *enumStrings[] = {"Left", "Center", "Right"};
    void readState() {
        int i;
        for (i=0;i<3;i++) {
            Serial1.printf("spot %s : D=%d A=%d\n",enumStrings[i],spots[i].dist,spots[i].azim);
        }
    }
}