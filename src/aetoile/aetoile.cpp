
#include "aetoile.hpp"


Aetoile::Aetoile():idCourant(0), idArrivee(0), cheminValable(false){};

int Aetoile::verifChemin(){
/*****************************************************************************************************************/
    // A FAIRE
/*****************************************************************************************************************/
    return 0;
}

void Aetoile::definirConcurrent(int numero, Coords pos){
    // à appeler dans communication et dans le lidar
    if (numero > 0 && numero < NB_CONCURRENTS){
        posAeviter[numero] = pos;
    }
    cheminValable=false;
}

int Aetoile::prepare(int ndCourant, int ndArrivee){
    if (cheminValable==true){
        return 0;
    }
    nettoie();
    initialise(ndCourant,ndArrivee);
    while(succesChoixChemin()>1);
    if (chemins.empty()){
        //std::cout << "erreur trajet impossible" << std::endl;
        return 1;
    }
    return 0;
}

int Aetoile::prepare(){
    return prepare(idCourant, idArrivee);
}

int Aetoile::sequence(){
    if (chemins.empty()){
        return -2;//pas de chemin en cours, passer à l'action suivante
    }
    auto liste_nds = (chemins.begin())->id_nds;
    if (liste_nds.empty()){
        nettoie();
        return -2;//le robot a terminé le chemin, passer à l'action suivante
    } else if (liste_nds.size()==1){
        idCourant = liste_nds.front();
        liste_nds.pop_front();
        return -1;//le robot passe au dernier waypoint
    } else {
        idCourant = liste_nds.front();//oblige le robot à aller à son point de départ
        liste_nds.pop_front();
        return idCourant;
    }
}

void Aetoile::affichage(){
    affiche(chemins.front());
};







int Aetoile::succesChoixChemin(){
    /*renvoie le résultat de la recherche de chemins : 0=chemin trouve, 1=erreur, 2=recherche en cours*/
    miseAjour();
    chemins.sort(
        [](chemin const &un, chemin const &deux){//fonction lambda
            return (un.poids < deux.poids);
        }
    );//assure que le chemin retourné sera optimal (selon fonction mesure)
    if ((chemins.begin())->fini){
        cheminValable = true;
        return 0;
    } else if (chemins.empty())
        return 1;
    else
        return 2;
}

void Aetoile::initialise(int idNdArrivee){
    idArrivee = idNdArrivee;
    cheminValable = false;
    chemins.push_back({(idNdArrivee==idCourant),0,0,{idCourant}});
    TRACE(std::cout << "res " << estSurLeTrajet(noeuds[0],noeuds[1],Coords(1.3,1.5)) << std::endl;)//1,1 1,2
}

void Aetoile::initialise(int idNoeudCourant, int idNdArrivee){
    idCourant = idNoeudCourant;
    idArrivee = idNdArrivee;
    chemins.push_back({(idNdArrivee==idNoeudCourant),0,0,{idNoeudCourant}});
    TRACE(std::cout << "res " << estSurLeTrajet(noeuds[0],noeuds[1],Coords(1.3,1.5)) << std::endl;)//1,1 1,2
}

void Aetoile::nettoie(){
    cheminValable = false;
    for (auto chem : chemins){
        chem.id_nds.clear();
    }
    chemins.clear();
}

void Aetoile::miseAjour(){
    int imax=chemins.size();
    auto cheminCourant = chemins.begin();
    for (int i = 0; i<imax; i++){//ajout des nouveaux chemins
        cheminCourant = chemins.begin();
        TRACE(std::cout << "chem listé:";)
        TRACE(affiche(*cheminCourant);)
        if (cheminCourant->fini){
            chemins.push_back(chemins.front());
            //sauvegarde le chemin si celui-ci est terminé, permet de comparer sa longueur avec autres chemins
        } else {
            int idNdActuel = cheminCourant->id_nds.back();
            for (auto idVoisin : _voisins[idNdActuel]){
                TRACE(std::cout << " v" << idVoisin;)
                bool cheminPossible = true;
                for (auto pt : posAeviter){//évite obstacles
                    if (estSurLeTrajet(_noeuds[idNdActuel], _noeuds[idVoisin], pt)){
                        TRACE(std::cout << " obstacle,";)
                        cheminPossible = false;
                        break;
                    }
                }
                for (auto nd : cheminCourant->id_nds){//élimine la boucle infinie
                    if (nd==idVoisin){
                        TRACE(std::cout << " boucle,";)
                        cheminPossible = false;
                        break;
                    }
                }
                if (cheminPossible){
                    TRACE(std::cout << " ajouté,";)
                    std::list<int> v(cheminCourant->id_nds);
                    chemin nouveauChemin = {cheminCourant->fini, cheminCourant->poids, cheminCourant->poids_present, v};
                    nouveauChemin.id_nds.push_back(idVoisin);
                    nouveauChemin.poids_present += mesure(_noeuds[idNdActuel], _noeuds[idVoisin]) + COUTFIXE;
                    nouveauChemin.poids = nouveauChemin.poids_present + mesure(_noeuds[idVoisin], _noeuds[idArrivee]);
                    if (idVoisin == idArrivee){
                        nouveauChemin.fini = true;
                    }
                    chemins.push_back(nouveauChemin);
                }
            }
        }
        TRACE(std::cout << std::endl;)
        chemins.pop_front();
    }
}



Aetoile aetoile = Aetoile();