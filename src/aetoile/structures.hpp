#pragma once
#include <iostream>
#include <list>


class Coords
{
public:
    Coords();
    Coords(float x, float y);
    float get_theta(Coords p);
    float get_dist(Coords p);
    float x, y;
};
Coords operator-(Coords const &un, Coords const &deux);

template <typename t>
Coords operator*(Coords const& un, t const& nombre);

//template <typename t>
Coords operator/(Coords const un, float const nombre);

float scalaire(Coords v1, Coords v2);



class Noeud {
public:
    static int num;
    Coords coords;
    Noeud(Coords pt);
    Noeud(float x, float y);
    Noeud();
};

typedef struct {
    bool fini;//indique si chemin terminé
    float poids;//poids estimé
    float poids_present;//jusqu'au noeud present
    std::list<int> id_nds;
} chemin;

void affiche(chemin& unchemin);