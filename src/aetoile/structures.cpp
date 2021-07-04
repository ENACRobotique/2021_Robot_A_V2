#include <iostream>
#include <cmath>
#include <list>
#include "structures.hpp"

#ifndef PI
#define PI 3.14159
#endif

Coords::Coords():x(0), y(0){};
Coords::Coords(float fx, float fy): x(fx), y(fy){};

float Coords::get_dist(Coords p)
{
    return sqrt(pow(x + p.x, 2) + pow(y + p.y, 2));
}

float Coords::get_theta(Coords p)
{ // donne le relevement du Coords p/r à p
    if ((x - p.x) > 0)
    {
        return PI / 2 - atan((y - p.y) / (x - p.x));
    }
    else if ((x - p.x) < 0)
    {
        return 3 * PI / 2 - atan((y - p.y) / (x - p.x));
    }
    else
    {
        return 0.0;
    }
};

Coords operator-(Coords const &un, Coords const &deux){
    return Coords(un.x-deux.x,un.y-deux.y);
}



template <typename t>
Coords operator*(Coords const& un, t const& nombre){
    return Coords(un.x*nombre,un.y*nombre);
}
//template <typename t>
Coords operator/(Coords const un, float const nombre){
    return Coords(un.x/nombre,un.y/nombre);
}

float scalaire(Coords v1, Coords v2){
    return v1.x*v2.x+v1.y*v2.y;
}

void affiche(chemin& unchemin){
    std::cout << "chemin poids:" << unchemin.poids_present << " noeuds: ";
    for (auto nd : unchemin.id_nds){
        std::cout << nd << " ";
    }
}

Noeud::Noeud(Coords pt):coords(pt){num++;}
Noeud::Noeud(float x, float y):coords(Coords(x,y)){num++;}
Noeud::Noeud(){num++;}
int Noeud::num = 0;