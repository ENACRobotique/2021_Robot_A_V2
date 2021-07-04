#pragma once
#include <iostream>
#include <cmath>
#include "structures.hpp"



int succesChoixChemin();

float distance(Coords const &un, Coords const &deux);
float mesure(Noeud &dep, Noeud &arr);

Coords orthogonal(Coords un);

bool estSurLeTrajet(Noeud const &n1, Noeud const &n2, Coords const &posRobot);

