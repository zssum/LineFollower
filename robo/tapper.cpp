//HelperFunctions.cpp
#include "tapper.h"
#include "Arduino.h"

Tapper::Tapper(int channelNumber){
  servonum=channelNumber;
  pwm = Adafruit_PWMServoDriver();
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void Tapper::tap(){
  pwm.setPWM(servonum, 0, 350); 
}

void Tapper::liftUp(){
  pwm.setPWM(servonum, 0, 420); 
}

