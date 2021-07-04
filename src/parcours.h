/*
 * trajectory.h
 *
 * Created on : 9 mars 2021
 * Author : Liam
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include "aetoile/structures.hpp"
#include "aetoile/grilnav.hpp"
#include <vector>

typedef enum
{
    TURNPOINT,
    ECOCUP_RED,
    ECOCUP_GREEN,
    RELEASE_RED,
    RELEASE_GREEN,
    END,
    BEGIN
} WP_type;

typedef struct {
    int idNoeud;
    WP_type type;
    bool bouger;//ne pas se déplacer
    bool angleAleatoire;//prend en compte la suite
    float angle;
}Action;



class Waypoint : public Coords
{
public:
    Waypoint();
    Waypoint(int id);
    Waypoint(int idPoint, float x, float y, WP_type type);
    Waypoint(int idPoint, WP_type type, bool bouger, bool angleAleatoire, float anglefinal);
    Waypoint(int idPoint, float x, float y, WP_type type, bool bouger, bool angleAleatoire, float anglefinal);
    int scalaire(float x, float y, float x2, float y2);
    int idPoint;
    WP_type type;
    bool bouger;
    bool angleAleatoire;
    float angle;
private:
    //float x,y;
   
};

class Parcours
{
public:
    Parcours();
    Waypoint get_current_WP();
    Waypoint get_next_WP();
    float get_rad();
    float currentX_track(float x0, float y0);
    float currentX_angle(float x0, float y0);
    int pos;
    void verifObstacles();//à appeler en cas de modification de position des concurrents
    void decideChemin();//ici on décide de l'action suivante. fonction cruciale, qui ne rend compte à personne

private:
    std::vector<Waypoint> parcours;
};

extern Parcours traj1;

#endif /* TRAJECTORY_H_ */
