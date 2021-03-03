/*
 * params.h
 *
 *  Created on: 18 mars 2018
 *      Author: elie
 */

#ifndef PARAMS_H_
#define PARAMS_H_
#include "Arduino.h"

#define VOLT_TO_DIST(volt) (13 * pow (volt  * 0.0048828125, -1)) 

const int ARUCO_ID = 1;

const unsigned long TIME_RACE = 100000;


const int ENCODEUR1_A = 21;
const int ENCODEUR1_B = 20;
const int ENCODEUR2_A = 17;
const int ENCODEUR2_B = 16;

const int MOT1_PWM = 5;
const int MOT1_DIR = 6;
const int MOT2_PWM = 29;
const int MOT2_DIR = 28;

//const int BATT_CHARGE = 37;

// const int US_RANGE = 35;
// const int US_RANGE_DIMINUSHED = 25;

// const int SERVO1 = 39;
// const int SERVO2 = 15;
const int SERVO3 = 37;

// const int MOT_GALET_L = 35;
// const int MOT_LIDAR = 35;

const int COLOR = 26;
const int TIRETTE = 25;

// const int SPARE1 = 10;
// const int SPARE2 = 9;
// const int SPARE3 = 8;
// const int SPARE4 = 7;

// const int DYNAMIXEL_CONTROL = 30;
// const int DYNAMIXEL_ID =1;

const int EMERGENCY_BRAKE = 1800;
const int ACCEL_MAX = 500;
const int ACCEL_MAX_THROW = 100;
const int SPEED_MAX_CRUISE = 350;

const int ACCEL_MAX_CAP = 100;
const int SPEED_MAX_CAP = 100;

const float ACCEL_OMEGA_MAX_CRUISE = 1;
const float OMEGA_MAX_CRUISE = 1.5;

const float ACCEL_OMEGA_MAX_CAP = 0.3;
const float OMEGA_MAX_CAP = 0.5;


const float ADMITTED_OMEGA_ERROR = 0.03;
const float ADMITTED_SPEED_ERROR = 10;

const float ADMITTED_POSITION_ERROR = 5;
const float ADMITTED_ANGLE_ERROR = 0.01;

//const float MAX_DRIFT = 0.1;

//const float WHEEL_DIAMETER = 55.0;
const float WHEELBASE = 261.9790233;
const float INCR_TO_MM = 0.21922110552763818;   //PI*WHEEL_DIAMETER/805.12;
const float INCR_TO_MM_1 = INCR_TO_MM * 0.9833721540716956;
const float INCR_TO_MM_2 = INCR_TO_MM * 1.016909006279521;
//const float INCR_TO_TOURS = 1/805.12;
const float CONTROL_PERIOD = 0.02;
const float NAVIGATOR_PERIOD = 0.05;
const float COMMUNICATION_PERIOD = 0.1;
const float STATE_PERIOD = 0.1;

const unsigned long THROW_DURATION = 20000;
const unsigned long SERVO_MOVEMENT_DURATION = 1000;
const unsigned long DETECTION_STOP_TIME = 1500;

const int GREEN = 1;
const int ORANGE = !GREEN;
const int PURPLE = 1;
const int YELLOW = !PURPLE;


enum sensors {IR1=33,IR2,IR3,IR4,IR5,IR6,IR_test};
sensors IR_sel=IR_test;


//params de capture ecocup
float dist_min=5,dist_max=20;//valeurs initial turn
float delta_step_forward=4;
float nominal_delta_rot=10;//degres
float dist_opt=5;

// const int NB_US = 4;
#endif /* PARAMS_H_ */
