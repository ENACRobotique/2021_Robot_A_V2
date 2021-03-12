#include <KalmanFilter.hpp>
#ifndef KALMAN_HS
#define KALMAN_HS

#define Nstate 3 // nombre de param d'état
#define Nobs 3   // nbr pram observation
#define Ncom 2 // nombre de param de commande

namespace Kalman{
    void update();
    void setOdomobs(float dx, float dy, float dtheta);
    void setcom(float v, float omega);
    void setpos(float x, float y, float theta);
    void init();
}
extern KALMAN<Nstate,Nobs,Ncom> K; // your Kalman filter
#endif