//#include "Arduino.h"
#include <string>
#include "lidar.h"

namespace Lidar {
    Spot spots[3];

    int lidarIns[] = {lidarIn1,lidarIn2,lidarIn3,lidarIn4,lidarIn5}; //à définir
    void changeReadSpace(Space zone){
        int zn = (int)(zone);
        int i;
        for(i=0; i<5; i++) {    
            zn=zn%2;
            analogWrite(lidarIns[i],zn);
            zn=zn/2;
        }    
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
            if (3<=etat && etat<=6) {
                return etat;
            }
            if (7<=etat && etat<=10) {
                return etat;
            }
            if (11<=etat && etat<=14) {
                return etat;
            }
            else {
                return etat;
            }
        }     
    }

    void init() {
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
            if (digitalread(lidarOut1)==HIGH) { //détection dans le moins englobant
                spots[numspot].sector_dist=1;
            }
            else if (digitalread(lidarOut2)==HIGH) {
                spots[numspot].sector_dist=2;
            }
            else if (digitalread(lidarOut3) == HIGH) {
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
            if (digitalread(lidarOut1)==HIGH) {
                spots[numspot].sector_azim+=1;
            }
            if (digitalread(lidarOut2)==HIGH) {
                spots[numspot].sector_azim+=2;
            }
            if (digitalread(lidarOut3) == HIGH) {
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
    }

    void reset(int numspot) {
        spots[numspot].dist = (Space)((int)DL + numspot);
        spots[numspot].azim = (Space)((int)AL + numspot);
    }

    static char *enumStrings[] = {"Left", "Center", "Right"};
    void readState() {
        int i;
        for (i=0;i<3;i++) {
            Serial1.printf("spot %s : D=%d A=%d\n",enumStrings[i],spots[i].dist,spots[i].azim);
        }
    }
}