#include <QTRSensors.h>
#include "motor.h"

//#define DEBUG //comment out to disable debugging
#ifdef DEBUG
#define debug(x)     Serial.print(x)
#define debugln(x)   Serial.println(x)
#else
#define debug(x)     // define empty, so macro does nothing
#define debugln(x)
#endif

//QTR Settings
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   23     // emitter is controlled by digital pin 23

//Motor Settings
#define LM1     25       
#define LM2     27
#define LM_PWM  2 
#define RM1     29
#define RM2     31
#define RM_PWM  3
#define MOTORSPEED  60

// Ultrasound Distance Detector Settings
#define  trigPin  51    
#define  echoPin  49    
#define  sonicVcc 53
#define  sonicGnd 47


// Line Follower PID constants
#define Kp 0.08 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0.12 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define RIGHT_MAX_SPEED 130 // previously 120 max speed of the robot
#define LEFT_MAX_SPEED  130// previously 120 max speed of the robot
#define RIGHT_BASE_SPEED 70 // previoiusly 60 this is the speed at which the motors should spin when the robot is perfectly on the line
#define LEFT_BASE_SPEED 70  // previously 60 this is the speed at which the motors should spin when the robot is perfectly on the line

// Initialising motors and QTR(ir) sensor
QTRSensorsRC qtrrc((unsigned char[]) {22, 24, 26, 28, 30, 32, 34, 36},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
Motor motor(LM1, LM2, LM_PWM, RM1, RM2, RM_PWM, MOTORSPEED);

unsigned int sensorValues[NUM_SENSORS];
String action; // Robot State
String inputString; // Command builder
int lastError = 0; //Last Line Following Error
long duration, cm; //distance detection variables
boolean shouldDetect=true;

void setup()
{
  pinMode(13, OUTPUT); //Arduino LED
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(sonicVcc,OUTPUT);
  pinMode(sonicGnd, OUTPUT);
  digitalWrite(sonicVcc,HIGH);
  digitalWrite(sonicGnd,LOW);
  
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  Serial.setTimeout(80);
  Serial1.begin(9600); // set the data rate in bits per second for serial data transmission
 
 
  qtrrc.calibrate();
  
  //956	692	536	640	488	488	536	692	go for plus 700
  
  qtrrc.calibratedMinimumOn[0]=1656;
  qtrrc.calibratedMinimumOn[1]=1392;
  qtrrc.calibratedMinimumOn[2]=1236;
  qtrrc.calibratedMinimumOn[3]=1340;
  qtrrc.calibratedMinimumOn[4]=1188;
  qtrrc.calibratedMinimumOn[5]=1188;
  qtrrc.calibratedMinimumOn[6]=1236;
  qtrrc.calibratedMinimumOn[7]=1392;
  
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    //qtrrc.calibratedMaximumOn[i]=600;
    qtrrc.calibratedMaximumOn[i]=2500;
  }
  
}


void loop()
{
  if(action=="read") read();
  else if (action=="readlline") readline();
  else if (action=="calibrate") calibrate();
  else if (action=="f") motor.motorFwd(MOTORSPEED);
  else if (action=="b") motor.motorBack(MOTORSPEED);
  else if (action=="l") motor.motorLeft(MOTORSPEED);
  else if (action=="r") motor.motorRight(MOTORSPEED);
  else if (action=="s") motor.motorStop();
  else if (action=="go") drive();
  else motor.motorStop();
  

  detectRange();
  while(cm<15){
    detectRange();
    motor.motorStop();
    debugln("object in front");
  }
}

void read(){
  // read raw sensor values
  qtrrc.read(sensorValues);

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();
  
  delay(250);
}

void readline(){
    // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int position = qtrrc.readLine(sensorValues); 

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(position); // comment this line out if you are using raw values
  
  delay(250);
}

void calibrate(){
  
  delay(500); 
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 200; i++)  // make the calibration take about 5 seconds
  {
    if(i<60) motor.motorLeft(65); 
    else if (i>60&&i<140) motor.motorStop(); 
    else if (i>141&& i<199) motor.motorRight(65);
    else  motor.motorStop(); //Robot rotates anticlockwise for first 1.5s stops for 2s and rotates clockwise for 1.5s
      
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);  
  action= "s";
}



void drive(){
  //unsigned long starting=micros();
  unsigned int position = qtrrc.readLine(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = 3500-position;
  //debugln("read timeing");
  //debugln(micros()-starting);

  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = RIGHT_BASE_SPEED - motorSpeed;
  int leftMotorSpeed = LEFT_BASE_SPEED + motorSpeed;
  
  if (rightMotorSpeed > RIGHT_MAX_SPEED ) rightMotorSpeed = RIGHT_MAX_SPEED; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > LEFT_MAX_SPEED ) leftMotorSpeed = LEFT_MAX_SPEED; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  //debugln("xxx");
  //debugln(micros()-starting);
  
  
  if(error==-3500 ){ // if line is on the left of the robot, stop line detection and rotate to the anti-clockwise for 0.2s
    motor.motorLeft(70);
    debug("lockleft");    
    debugln();
    delay(300);
  } else if (error==3500){
    debug("lockright");
    debugln();
    motor.motorRight(70);
    delay(300);
  } else { 
    motor.motorSlight(leftMotorSpeed,rightMotorSpeed);
  }
  
  //debugln("read timeing1");
  //debugln(micros()-starting);
  
  int black=0;
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {    
    if(sensorValues[i]==1000) black++;
  }
  if(black==8) action="s";
  
  debugln(position);
  delay(1);
  //debugln("timeing");
  //debugln(micros()-starting);
}

void detectRange()
{
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH,1750);

  if(duration==0){
    cm=30;
  } else{
    cm = (duration/2) / 29.1;
  }

 

  // convert the time into a distance
  
  
  /*debug(inches);
  debug("in, ");
  debug(cm);
  debug("cm");
  debugln();*/
  
}

void serialEvent1() {
  while (Serial1.available()) {  
    // get the new byte:
    char inChar = (char)Serial1.read();
    // add it to the inputString:
    
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '#') {
      action=inputString;
      inputString="";
    } else{
      inputString += inChar;
    }
  }
}

void serialEvent() {
   while(Serial.available()){
    action=Serial.readString();
  }
}
//1116 788 588 636 528 488 488 636go


//2500 2500 2500 2500 2500 2500 2500 2500 

//604 692 344 348 244 244 296 344
