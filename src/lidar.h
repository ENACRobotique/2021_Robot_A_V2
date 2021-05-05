#ifndef _LIDAR_
#define _LIDAR_

namespace Lidar {
    enum Space {
        DL,DC,DR,
        DL1,DL2,DL3,
        DC1,DC2,DC3,
        DR1,DR2,DR3,
        AL,AC,AR,
        AL1,AL2,AL3,
        AC1,AC2,AC3,
        AR1,AR2,AR3
    };
    struct Spot {
        Space dist;
        Space azim;
        int sector_dist=0;
        int sector_azim=0;
    };
    void changeReadSpace2(int zone);
    void reset(int numspot);
    void init();
    void update_all();
    void readState();
    
    extern Spot spots[3];
}

#endif