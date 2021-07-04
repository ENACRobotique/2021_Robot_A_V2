#pragma once
#include <vector>
#include "mesure.hpp"
#include <iostream>
#include "grilnav.hpp"

class Aetoile{
private:
    int idCourant, idArrivee;//id des noeuds de départ et arrivée
    std::list<chemin> chemins;
    bool cheminValable;
    Coords posAeviter[NB_CONCURRENTS] = {HORS_DU_PLATEAU, HORS_DU_PLATEAU, HORS_DU_PLATEAU, HORS_DU_PLATEAU};

    int succesChoixChemin();
    void initialise(int idNoeudCourant, int idNdArrivee);
    void nettoie();
    void miseAjour();//avance d'un voisin pour trouver la solution
    
public:
    Aetoile();
    void initialise(int idNdArrivee);
    int prepare(int ndCourant, int ndArrivee);//calcule le chemin  
    int prepare(int ndArrivee);//calcul du chemin à partir du point actuel.
    int prepare();//calcule le chemin sans changer départ ou arrivée
    int sequence();//passe au noeud suivant

    int verifChemin();//renvoie 1 si obstacle sur le chemin, 0 sinon -> 
    void definirConcurrent(int numero, Coords pos);
    void affichage();//affiche les noeud du chemin actuel et sa longueur
};

extern Aetoile aetoile;