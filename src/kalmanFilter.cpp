#include <Kalman.h>
#include <kalman.hpp>
//using namespace BLA;
namespace kalmanFilter{
//------------------------------------
/****  MODELIZATION PARAMETERS  ****/
//------------------------------------

#define Nstate 3 // nombre de param d'état
#define Nobs 3   // nbr pram observation
#define Ncom 2 // nombre de param de commande

// measurement std
#define n_x 0.3
#define n_y 5.0
#define n_theta 5.0
// model std (1/inertia)
#define m_x 0.3
#define m_y 5.0
#define m_theta 5.0


BLA::Matrix<Nobs> obs; // observation vector
BLA::Matrix<Ncom> com;
KALMAN<Nstate,Nobs,Ncom> K; // your Kalman filter

//------------------------------------
/****    SIMULATOR PARAMETERS   ****/
//------------------------------------

unsigned long T;
float DT;

BLA::Matrix<Nstate> state; // true state vector for simulation
BLA::Matrix<Nobs> noise;   // additive noise for simulation

#define LOOP_DELAY 10  // add delay in the measurement loop
#define SIMUL_PERIOD 0.3 // oscillating period [s]
#define SIMUL_AMP 1.0

//------------------------------------
/****        SETUP & LOOP       ****/
//------------------------------------

void init() {

  Serial.begin(57600);

  // The model below is very simple since matrices are diagonal!
  state(0) = 0;
  state(1) = 0;
  state(2) = 0;

  obs.Fill(0.0);

  // time evolution matrix
  K.F = {1.0, 0.0, 0.0,
		 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0};

  K.B = {cos(state(2)), 0.0,
		      sin(state(2)), 0.0,
         0.0, 1.0};

  // measurement matrix
  K.H = {1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
         0.0, 0.0, 1.0};
  // measurement covariance matrix
  K.R = {n_x*n_x,   0.0, 0.0,
           0.0, n_y*n_y, 0.0,
           0.0, 0.0,n_theta*n_theta};
  // model covariance matrix
  K.Q = {m_x*m_x,     0.0,     0.0,
             0.0, m_y*m_y,     0.0,
			 0.0,     0.0, m_theta*m_theta};
  
  T = millis();
  
  
}

void update() {
  // TIME COMPUTATION
  DT = (millis()-T)/1000.0;
  T = millis();

  K.F = {1.0, 0.0, 0.0,
		 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0};

  K.B = {cos(state(2)), 0.0,
		      sin(state(2)), 0.0,
         0.0, 1.0};

  // MESURE X Y THETA
  
  // APPLY KALMAN FILTER
  K.update(obs,com);

  // PRINT RESULTS: true state, measures, estimated state
  Serial1 << state << ' ' << obs << ' ' << K.x << ' ' << K.P << '\n';
}

}