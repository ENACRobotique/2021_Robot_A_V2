#pragma once
#include <iostream>
#include <cmath>
#include "structures.hpp"

/* définition de la mesure de tri dans A* */

int succesChoixChemin();

float distance(Coords const &un, Coords const &deux);
float mesure(const Noeud &dep, const Noeud &arr);

Coords orthogonal(Coords un);

bool estSurLeTrajet(Noeud const &n1, Noeud const &n2, Coords const &posRobot);

