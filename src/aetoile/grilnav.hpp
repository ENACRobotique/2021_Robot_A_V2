#pragma once
#include <vector>
#include "structures.hpp"
#define NB_CONCURRENTS 4 //nb de robots potentiels sur le terrain = 3 +1 qui est détecté devant le robot par le lidar
#define COUTFIXE 0.2 //pour minimiser le nombre de noeud, on pénalise chaque noeud ajouté au chemin par un coût
#define RAYON_ADVERSE 0.1 // largeur du robot adverse, zone à éviter
#define RAYON_NOUS 0.1 // largeur de notre robot (matérialise un couloir)

#define HORS_DU_PLATEAU Coords(1e6,1e6) // pour placer un point très loin, car non vu
//#define DEBUG // à décommenter pour activer affichage résultats intermédiaires

#ifdef DEBUG
#define TRACE(A) A 
#endif
#ifndef DEBUG
#define TRACE(A)
#endif


#define NBNOEUDS 7

extern std::vector<int> _voisins[NBNOEUDS];
extern Noeud _noeuds[NBNOEUDS];


