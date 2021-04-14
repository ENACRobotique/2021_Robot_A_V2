#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

namespace kalmanFilter{

    void init();
    void predict(double u_v,double u_w);
    void update(double x, double y, double theta);
    void print_state();


}

#endif