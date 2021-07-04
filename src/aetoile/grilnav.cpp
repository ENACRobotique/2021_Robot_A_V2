#include "grilnav.hpp"
#include <vector>

Coords _posAeviter[NB_CONCURRENTS] = {Coords(2,2), Coords(2,2)};


Noeud _noeuds[NBNOEUDS]={
    //chaque noeud a un id (Noeud.num) égal à sa place 
    // 0 pour le premier, 1 pour le 2ème ..
    Noeud(-1250.0,500.0), Noeud(-1250.0,0.0), Noeud(-1000.0,0.0), 
    Noeud(-1500.0,500.0), Noeud(-1250.0,250.0), Noeud(-1250.0,500.0),
    Noeud(-500, 0)
};

std::vector<int> _voisins[NBNOEUDS] = {
/* id des noeuds */
/* depart   arrivée */
/* 0  */       {1,2,3},
/* 1  */       {0,3,2},
/* 2  */       {1,3,5,6},
/* 3  */       {0,4,5},
/* 4  */       {2,5,6},
/* 5  */       {3,2,6,4},
/* 6  */       {4,5,2,1}
};