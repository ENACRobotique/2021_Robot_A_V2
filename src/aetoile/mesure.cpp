#include <iostream>
#include <cmath>
#include <vector>
#include "mesure.hpp"
#include "grilnav.hpp"

float distance(Coords const &un, Coords const &deux){
    return sqrt(pow(un.x-deux.x,2) + pow(un.y-deux.y,2));
}

float mesure(const Noeud &dep, const Noeud &arr){
    return distance(dep.coords, arr.coords);
}

Coords orthogonal(Coords un){
    return Coords(un.y, -un.x);
}

bool estSurLeTrajet(Noeud const &n1, Noeud const &n2, Coords const &posRobot){
    //std::cout << "test de presence obstacle" << std::endl;
    Coords vec_lin = (n2.coords-n1.coords);
    Coords vec_orth=orthogonal(vec_lin)/distance(n1.coords,n2.coords);
    Coords vec_centre = (posRobot-n1.coords);
    float scal1 = scalaire(vec_centre, vec_orth);
    if (fabs(scal1) < RAYON_ADVERSE+RAYON_NOUS){
         
        /*
        vérifie si posRobot est dans la boite rectangulaire comme suit:
        
        <--distance(n1,n2)------>

        -------------------------   /\
        |                       |   |
        n1                      n2   largeur = 2*(RAYON_ADVERSE+RAYON_NOUS)
        |                       |   |
        -------------------------   \/

        */
        float scal2 = scalaire(vec_centre, vec_lin);
        if (scal2>0 && scal2<scalaire(vec_lin,vec_lin)){
            return true;
        }  
    } 
    if (distance(n1.coords,posRobot) < RAYON_ADVERSE+RAYON_NOUS || distance(n2.coords,posRobot) < RAYON_ADVERSE+RAYON_NOUS){
        /*
            Vérifie si posRobot est dans un cercle de rayon RAYON_ADVERSE+RAYON_NOUS 
        */
        return true;
    }
    return false;
}
