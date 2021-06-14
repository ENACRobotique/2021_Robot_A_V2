#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include <kalman.h>

#define Nstate 3 // nombre de param d'état
#define Nobs 3   // nbr pram observation
#define Ncom 2 // nombre de param de commande

namespace kalmanFilter{
    void init();
    extern KALMAN<Nstate,Nobs,Ncom> K;
    template<typename tp>
    void init(tp x, tp y,tp theta);

    void predict(double u_v,double u_w);
    void update(double x, double y, double theta);
    void print_state();

}

#endif