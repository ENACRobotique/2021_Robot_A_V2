#include "parcours.h"
#include "aetoile/aetoile.hpp"
#include <vector>
#include <iostream>
#include <math.h>
#include "Arduino.h"

Action _actions[] = {
    //{idNd, type_point, bouger?, anglefinal?, valeurAngle}
    {1,BEGIN, false, false, 180},
    {2,ECOCUP_RED, true, false, 180},
    {3,ECOCUP_RED, true, false, 180},
    //{-1200.0,100.0,ECOCUP_RED, true, false, 180},
    {4, RELEASE_RED, true, false, 180},
    {5, RELEASE_RED, true, false, 180},
    //{0.0,0.0,TURNPOINT},
    {6,END, true, false, 180}
};



Waypoint::Waypoint() {
}

Waypoint::Waypoint(int id) : Coords(_noeuds[idPoint].coords), idPoint(id), type(TURNPOINT), bouger(true), angleAleatoire(true)
{
};

Waypoint::Waypoint(int id, float x0, float y0, WP_type t_type) : Coords(x0, y0), idPoint(id), type(t_type), bouger(true), angleAleatoire(true)
{//pour aller à un endroit non prévu. déconseillé
};

Waypoint::Waypoint(int id, WP_type t_type, bool bbouger, bool aa, float anglefinal) : Coords(_noeuds[idPoint].coords), idPoint(id), type(t_type), bouger(bbouger), angleAleatoire(aa), angle(anglefinal)
{
}; 

Waypoint::Waypoint(int id, float x0, float y0, WP_type t_type, bool bbouger, bool aa, float anglefinal) : Coords(x0, y0), idPoint(id), type(t_type), bouger(bbouger), angleAleatoire(aa), angle(anglefinal)
{//pour aller à un endroit non prévu. déconseillé
};                                                                     

Parcours::Parcours()
{
    pos=0;
    unsigned int i_loc=0;
    for (i_loc=0;i_loc<(int)(sizeof(_actions)/sizeof(Action));i_loc++) {
        parcours.push_back(Waypoint(_actions[i_loc].idNoeud,_actions[i_loc].type,_actions[i_loc].bouger,_actions[i_loc].angleAleatoire, _actions[i_loc].angle));
    }
};

Waypoint Parcours::get_current_WP()//deprecated?
{
    return parcours[pos];
}

void Parcours::verifObstacles(){
    if(aetoile.verifChemin()){
        //en cas de présence d'un concurrent sur le trajet, on en calcule un nouveau
        decideChemin();//recherche un autre itineraire
    }
}


void Parcours::decideChemin(){
    static int nbEchecs;
    if(aetoile.prepare()){
        //si pas de chemin possible
        if (pos == parcours.size()-1){
            //attendre si dernier element
            delay(200);
        } else {
            nbEchecs++;
            if (nbEchecs < 3){
                //attend que la situation évolue...
                delay(5000);
                //tente de recalculer le chemin
                decideChemin();
            }
            //sinon passer à l'action suivante. Automatique car aetoile::chemins est nul.
            // Peut ne pas avoir de sens
        }
        //rajouter une condition sur le temps restant? 
        //pour éviter de se retrouver au milieu du terrain à la fin du temps
    } else {
        //chemin trouvé, en route!!
        nbEchecs=0;
    }
}

Waypoint Parcours::get_next_WP()
{
    int id = aetoile.sequence();
    if (id > -1){
        return Waypoint(id);// cas où le chemin n'est pas terminé
    }
    if (id==-1){
        return parcours[pos];//cas où le chemin est terminé, on passe à l'action désirée
    }
    //le chemin est terminé, action suivante
    
    if (pos < parcours.size()) {
        aetoile.initialise(parcours[pos++].idPoint);//sélectionne l'action et incrémente pour la fois suivante
        decideChemin();
        return aetoile.sequence();
    } else {
        return parcours[pos];
    }
}
float Parcours::get_rad()//deprecated
{
    return parcours[pos].get_theta(parcours[pos - 1]);
}
float Parcours::currentX_track(float x0, float y0)//deprecated
{
    Coords p = Coords(x0, y0);
    return p.get_dist(parcours[pos - 1]) * sin(p.get_theta(parcours[pos - 1]) - parcours[pos].get_theta(parcours[pos - 1]));
}
float Parcours::currentX_angle(float x0, float y0)//deprecated
{
    Coords p = Coords(x0, y0);
    return p.get_theta(parcours[pos - 1]) - get_rad();
}

Parcours traj1 = Parcours();
