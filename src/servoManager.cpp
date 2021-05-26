

#include <i2c_t3.h>
#include <Adafruit_PWMServoDriver.h>
#include <servoManager.h>
#include "params.h"
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x65, Wire2);
ServoManager servoManager = ServoManager();
#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  515 // This is the 'maximum' pulse length count (out of 4096)
#define FACTOR (SERVOMAX - SERVOMIN)/180 // Facteur de convertion de degrés en pulselength
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define NB_SERVO 8

// our servo # counter

servos servo_sel = servo_FL;
uint8_t servonum = 0;


void ServoManager::init(const uint16_t init_tab[8] ) {
  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  for(int i =0; i<8;i++){
            consigne[i] = init_tab[i];
            currentAngle[i] = init_tab[i];
  }
  delay(10);
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void ServoManager::setPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  //Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  //Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  //Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void ServoManager::controlservo_k() {
  // Drive each servo one at a time using setPWM()
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen+=10) {
    pwm.setPWM(servonum, 0, pulselen);
    delay(10);
  }

  //delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen-=10) {
    pwm.setPWM(servonum, 0, pulselen);
    delay(10);
  }

  servonum++;
  if (servonum > 5) servonum = 0; // Testing the first 8 servo channels
}

//numero du servo et angle en deg
void ServoManager::controlServo(servos servo,uint16_t angle){
  if(angle < 0){angle = 0;}
  else if(angle > 180){angle = 180;}
  consigne[servo] = angle;
}

void ServoManager::update(){
  
  for(int i = 0;i<NB_SERVO;i++){
    finished = true;
    if(consigne[i]< currentAngle[i]){
      currentAngle[i] -= 27;
      finished = false;
    }
    else if(consigne[i]> currentAngle[i]){
      currentAngle[i] += 27;
      finished = false;
    }
  //currentAngle[i] = consigne[i];
  if(abs(consigne[i] - currentAngle[i])<=27){
    currentAngle[i] = consigne[i];
    }
  //if(!finished){
  pwm.setPWM(i,0,currentAngle[i]*FACTOR + SERVOMIN);
  //}
  }
}

bool ServoManager::isFinished(){
    return finished;
}