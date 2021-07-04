#pragma once
#include <vector>
#include "structures.hpp"
#include "../params.h"

/* paramètres pour le maillage de la grille de navigation */

#define COUTFIXE 0.2 //pour minimiser le nombre de noeud, on pénalise chaque noeud ajouté au chemin par un coût
#define RAYON_ADVERSE 0.1 // largeur du robot adverse, zone à éviter
#define RAYON_NOUS 0.1 // largeur de notre robot (matérialise un couloir)
#define NBNOEUDS 7

#define HORS_DU_PLATEAU Coords(1e6,1e6) // pour placer un point très loin, car non vu
//#define DEBUG // à décommenter pour activer affichage résultats intermédiaires

#ifdef DEBUG
#define TRACE(A) A 
#endif
#ifndef DEBUG
#define TRACE(A)
#endif


//id des arucos des concurrents
const int NB_CONCURRENTS = 4; //nb de robots potentiels sur le terrain = 3 +1 qui est détecté devant le robot par le lidar
const int CONCURRENT1 = 12;
const int CONCURRENT2 = 13;
const int CONCURRENT3 = 14;
//dernier concurrent : par défaut, non nommé


extern const std::vector<int> _voisins[NBNOEUDS];
extern const Noeud _noeuds[NBNOEUDS];


