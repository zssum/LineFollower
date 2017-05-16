//HelperFunctions.cpp
#include "motor.h"
#include "Arduino.h"

Motor::Motor(int lm1, int lm2, int lm_pwm, int rm1, int rm2, int rm_pwm, int motorspeed){
    m_lm1=lm1;
    m_lm2=lm2;
    m_lm_pwm=lm_pwm;
    m_rm1=rm1;
    m_rm2=rm2;
    m_rm_pwm=rm_pwm;
    m_motorspeed=motorspeed;
    pinMode(m_lm1,OUTPUT);
    pinMode(m_lm2,OUTPUT);
    pinMode(m_rm1,OUTPUT);
    pinMode(m_rm2,OUTPUT);
    digitalWrite(m_lm1,LOW);
    digitalWrite(m_lm2,LOW);
    digitalWrite(m_rm1,LOW);
    digitalWrite(m_rm2,LOW);
}

 
void Motor::motorFwd(int motorspeed){
    analogWrite(m_lm_pwm, motorspeed);
    digitalWrite(m_lm1,HIGH);
    digitalWrite(m_lm2,LOW);
    analogWrite(m_rm_pwm,motorspeed);
    digitalWrite(m_rm1,HIGH);
    digitalWrite(m_rm2,LOW);
}

void Motor::motorBack(int motorspeed){
    analogWrite(m_lm_pwm, motorspeed);
    digitalWrite(m_lm1,LOW);
    digitalWrite(m_lm2,HIGH);
    analogWrite(m_rm_pwm,motorspeed);
    digitalWrite(m_rm1,LOW);
    digitalWrite(m_rm2,HIGH);
}

void Motor::motorLeft(int motorspeed){
    analogWrite(m_lm_pwm,motorspeed);
    digitalWrite(m_lm1,LOW);
    digitalWrite(m_lm2,HIGH);
    analogWrite(m_rm_pwm,motorspeed);
    digitalWrite(m_rm1,HIGH);
    digitalWrite(m_rm2,LOW);
}

void Motor::motorRight(int motorspeed){
    analogWrite(m_lm_pwm,motorspeed);
    digitalWrite(m_lm1,HIGH);
    digitalWrite(m_lm2,LOW);
    analogWrite(m_rm_pwm,motorspeed);
    digitalWrite(m_rm1,LOW);
    digitalWrite(m_rm2,HIGH);
}

void Motor::motorSlight(int leftMotorSpeed,int rightMotorSpeed){
    analogWrite(m_lm_pwm,leftMotorSpeed);
    digitalWrite(m_lm1,HIGH);
    digitalWrite(m_lm2,LOW);
    analogWrite(m_rm_pwm,rightMotorSpeed);
    digitalWrite(m_rm1,HIGH);
    digitalWrite(m_rm2,LOW);

}

void Motor::motorStop(){
    analogWrite(m_lm_pwm,0);
    digitalWrite(m_lm1,LOW);
    digitalWrite(m_lm2,LOW);
    analogWrite(m_rm_pwm,0);
    digitalWrite(m_rm1,LOW);
    digitalWrite(m_rm2,LOW);
}