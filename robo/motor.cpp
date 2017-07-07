//HelperFunctions.cpp
#include "motor.h"
#include "Arduino.h"

Motor::Motor(int lm1, int lm2, int lm_pwm, int rm1, int rm2, int rm_pwm, int m_GRD, int m_5V){
    m_lm1=lm1;
    m_lm2=lm2;
    m_lm_pwm=lm_pwm;
    m_rm1=rm1;
    m_rm2=rm2;
    m_rm_pwm=rm_pwm;
    pinMode(m_lm1,OUTPUT);
    pinMode(m_lm2,OUTPUT);
    pinMode(m_rm1,OUTPUT);
    pinMode(m_rm2,OUTPUT);
    pinMode(m_GRD,OUTPUT);
    pinMode(m_5V,OUTPUT);
    digitalWrite(m_lm1,LOW);
    digitalWrite(m_lm2,LOW);
    digitalWrite(m_rm1,LOW);
    digitalWrite(m_rm2,LOW);
    digitalWrite(m_GRD,LOW);
    digitalWrite(m_5V,HIGH);
}

 
void Motor::motorFwd(int motorspeed){
    analogWrite(m_lm_pwm, motorspeed);
    digitalWrite(m_lm1,HIGH);
    digitalWrite(m_lm2,LOW);
    analogWrite(m_rm_pwm,motorspeed+motorCorrection);
    digitalWrite(m_rm1,HIGH);
    digitalWrite(m_rm2,LOW);
}

void Motor::motorBack(int motorspeed){
    analogWrite(m_lm_pwm, motorspeed);
    digitalWrite(m_lm1,LOW);
    digitalWrite(m_lm2,HIGH);
    analogWrite(m_rm_pwm,motorspeed+motorCorrection);
    digitalWrite(m_rm1,LOW);
    digitalWrite(m_rm2,HIGH);
}

void Motor::motorLeft(int motorspeed){
    analogWrite(m_rm_pwm,motorspeed+motorCorrection);
    digitalWrite(m_rm1,HIGH);
    digitalWrite(m_rm2,LOW);
    analogWrite(m_lm_pwm,motorspeed);
    digitalWrite(m_lm1,LOW);
    digitalWrite(m_lm2,HIGH);
    
}

void Motor::motorRight(int motorspeed){
    analogWrite(m_rm_pwm,motorspeed+motorCorrection);
    digitalWrite(m_rm1,LOW);
    digitalWrite(m_rm2,HIGH);
    analogWrite(m_lm_pwm,motorspeed);
    digitalWrite(m_lm1,HIGH);
    digitalWrite(m_lm2,LOW);
    
}

void Motor::changeSpeed(int leftMotorSpeed, int rightMotorSpeed){
  analogWrite(m_lm_pwm,leftMotorSpeed);
  analogWrite(m_rm_pwm,rightMotorSpeed);
}

void Motor::softAccelerateToSpeed(int leftMotorSpeed, int rightMotorSpeed){
   for(int i=5; i>0;i--){
      changeSpeed(leftMotorSpeed/i,rightMotorSpeed/i);
      delay(70);
   }
}

void Motor::softBrakeFromSpeed(){
  changeSpeed(0,0);
  delay(60);
  motorStop();
}

void Motor::motorStop(){
    analogWrite(m_lm_pwm,0);
    digitalWrite(m_lm1,LOW);
    digitalWrite(m_lm2,LOW);
    analogWrite(m_rm_pwm,0);
    digitalWrite(m_rm1,LOW);
    digitalWrite(m_rm2,LOW);
}
