//HelperFunctions.cpp
#include "distSensor.h"
#include "Arduino.h"

DistSensor::DistSensor(int trigPin, int echoPin, int sonicVcc, int sonicGnd){
    _trigPin = trigPin;
    _echoPin = echoPin;
    _sonicVcc = sonicVcc;
    _sonicGnd = sonicGnd;
    pinMode(_trigPin,OUTPUT);
    pinMode(_echoPin,INPUT);
    pinMode(_sonicVcc,OUTPUT);
    pinMode(_sonicGnd,OUTPUT);
    digitalWrite(_sonicVcc,HIGH);
    digitalWrite(_sonicGnd,LOW);
}

bool DistSensor::rangeIsClear(int distanceInCentimetres){
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(_echoPin, INPUT);
  long timeoutForDistance = 1.2*distanceInCentimetres*29.1*2; // calculate time taken if within the stated distance, all is clear in front of robot
  duration = pulseIn(_echoPin, HIGH, timeoutForDistance); //Time out set to increase the speed of detection at the expense of detection range

  if(duration==0){
    cm = distanceInCentimetres; // On timeout and no detection, objects are at least specified distance away from the distance sensor
  } else{
    cm = (duration/2) / 29.1; //calculation of distance with speed of sound
  } 
  
  if(cm<distanceInCentimetres) return false;
  else return true;
}

