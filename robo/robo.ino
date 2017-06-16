#include <QTRSensors.h>
#include "motor.h"
#include "distSensor.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//#define DEBUG //comment out to disable debugging
#ifdef DEBUG
#define debug(x)     Serial.print(x)
#define debugln(x)   Serial.println(x)
#else
#define debug(x)     // define empty, so macro does nothing
#define debugln(x)
#endif

//IR sensor Settings
#define NUM_SENSORS   6     // number of sensors used
#define TIMEOUT       4000  // waits for 4500 microseconds for sensor outputs to go low (Adjust higher for better sensitivity to offset increased distance between floor and sensor)
#define EMITTER_PIN   23    // emitter is controlled by digital pin 23

//Motor Settings
#define LM1     33       
#define LM2     35
#define LM_PWM  2 
#define L_GND   37 // TODO: check which todelete
#define L_5V    39 // TODO: check which todelete  delete left
#define RM1     25
#define RM2     27
#define RM_PWM  3
#define R_GND   29
#define R_5V    31
#define ROBOT_MAX_SPEED_FACTOR  1.7 // Factor of speedSelected to allow for speed variation between left and right wheels

// Ultrasound Distance Detector Settings
#define  trigPin  51    
#define  echoPin  49    
#define  sonicVcc 53
#define  sonicGnd 47

#define servonum  0 // tapper is attached to channel 0 out of 16 of the pwm controlboard

// Initialising motors and QTR(ir) sensor, tapper and distance sensor
QTRSensorsRC qtrrc((unsigned char[]) {24, 26, 28, 30, 32, 34},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
Motor motor(LM1, LM2, LM_PWM, RM1, RM2, RM_PWM);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
DistSensor distSensor(trigPin, echoPin, sonicVcc, sonicGnd);

#define INITIAL_SPEED 45
int speedSelected= INITIAL_SPEED; //Inital speed 
float Kp=0.035; // LineFollower proportional constant
unsigned int sensorValues[NUM_SENSORS]; //IR Sensor values

String action; // Robot's state
String inputString; // Command builder for serial strings
boolean directionTogg=true; // Toggle between clockwise and anti-clockwise rotation movements as a feedback to user
boolean isLaunchFromStop=true; // Used to soften acceleration if robot is launching from rest 
boolean tapped=false; //tapper state

void setup()
{
  pinMode(13, OUTPUT); //Arduino LED
  
  pinMode(R_GND,OUTPUT);
  pinMode(R_5V, OUTPUT);
  digitalWrite(R_GND,LOW);
  digitalWrite(R_5V,HIGH);
  pinMode(L_GND,OUTPUT); // TODO: check which todelete
  pinMode(L_5V, OUTPUT); // TODO: check which todelete
  digitalWrite(R_GND,LOW); // TODO: check which todelete
  digitalWrite(R_5V,HIGH); // TODO: check which todelete
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  Serial.setTimeout(80);
  Serial1.begin(9600);  //set bluetooth baud rate
  
  //tapper initialisation
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm.setPWM(servonum, 0, 420); // Raise arm up
 
 
  qtrrc.calibrate();
  // Manual callibration by recording read() values  
  // Raw values at KCD floor: 1104	1104	1020	1224	1264	1388		plus 1000 for noise reduction
  qtrrc.calibratedMinimumOn[0]=2104;
  qtrrc.calibratedMinimumOn[1]=2104;
  qtrrc.calibratedMinimumOn[2]=2020;
  qtrrc.calibratedMinimumOn[3]=2224;
  qtrrc.calibratedMinimumOn[4]=2264;
  qtrrc.calibratedMinimumOn[5]=2388;
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    qtrrc.calibratedMaximumOn[i]=TIMEOUT;
  }
  
}


void loop()
{
  if(action=="read") read(); // Read raw values of line sensor
  else if (action=="readline") readline(); // Read calibrated values and position of Robot on the line
  else if (action=="calibrate") calibrate(); //Recalibrate line sensor to a different surface
  else if (action=="select") selectSpeed();  
  else if (action=="go") go(); // Switching the motors in the forward direction before moving on to the main robot program
  else if (action=="d") drive(); // Main robot program
  else if (action=="tap") tapCard();
  // Manual control of the movement of the robot
  else if (action=="f") motor.motorFwd(40);
  else if (action=="b") motor.motorBack(40);
  else if (action=="l") motor.motorLeft(75); // Rotate Anti-Clockwise
  else if (action=="r") motor.motorRight(75); //Rotate Clockwise
  else if (action=="s") motor.motorStop();
  else if (action=="check") {
    Serial.println(distSensor.rangeIsClear(150)); 
    Serial.print(distSensor.cm);
    Serial.print("cm");
    Serial.println();
  }
  else motor.motorStop();
  
  //Ensures robot will brake if an object is in front
  //The robot will continue to move if the obstacle is removed   

  
  while(!distSensor.rangeIsClear(30)){  //TODO: check logic if it restarts
    analogWrite(LM_PWM,0);
    analogWrite(RM_PWM,0);
    debugln("object in front");
    Serial.println("object in front");
    Serial.println(distSensor.cm);
    Serial.println("cm");
  }
  
}


void read(){
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

//Sensors on the left of the black line and excecute calibration so that the robot will rotate through the black line for all the sensors
void calibrate(){  
  delay(500); 
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 200; i++)  
  {
    if(i<60) motor.motorLeft(65); 
    else if (i>60&&i<140) motor.motorStop(); 
    else if (i>141&& i<199) motor.motorRight(65);
    else  motor.motorStop(); //Robot rotates anticlockwise for first 1.5s stops for 2s and rotates clockwise for 1.5s
      
    qtrrc.calibrate();       // reads all sensors 10 times at TIMEOUT us per read (i.e. ~TIMEOUT*10 ms per call)
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

//Program to select the motor's speed from 70-110 in increments of 10
//User presses "selectSpeed" button to start the program, then presses "+" button to increment 10 to the base speed of 70
//Robot responds with a jerk on press of the increment button
//speedSelecting takes values 0 to 4. A selection of 0 gives speed of 70 while a selection of 4 gives speed of 110
//At the end of selection, user presses "selectSpeed" button to end the program. The robot will jerk the number of times to feedback the selected speed to the user.
void selectSpeed(){
  digitalWrite(13,HIGH); //Turn on indicators to feedback the selectSpeed program is ongoing
  motor.motorStop();
  boolean done=false;
  int speedSelecting=0;
  while (!done){
    while (Serial1.available()) {  
      char inChar = (char)Serial1.read();
      if (inChar == '+') {
        speedSelecting++;
        jerk();
        if(speedSelecting==5){
          done=true;
          digitalWrite(13,LOW);
          delay(500);
        }
      } else if(inChar=='#'){
        done=true;
        digitalWrite(13,LOW);
        delay(500);
      }
    }
  }
  debugln("Speed selected: ");
  debug(speedSelecting);
  for(int i=0;i<speedSelecting;i++){
    jerk();
  }
  //adjust speed and linefollowing constants
  speedSelected=INITIAL_SPEED+speedSelecting*10; 
  Kp= 0.035*40/speedSelected;
  action="s";
}

void go(){
  analogWrite(LM_PWM,0);
  digitalWrite(LM1,HIGH);
  digitalWrite(LM2,LOW);
  analogWrite(RM_PWM,0);
  digitalWrite(RM1,HIGH);
  digitalWrite(RM2,LOW);
  action="d";
}

void drive(){
  unsigned int position = qtrrc.readLine(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = 2500-position;
  int motorSpeed = Kp * error;
  int rightMotorSpeed = speedSelected - motorSpeed;
  int leftMotorSpeed = speedSelected + motorSpeed;
  
  if (rightMotorSpeed > ROBOT_MAX_SPEED_FACTOR*speedSelected ) rightMotorSpeed = ROBOT_MAX_SPEED_FACTOR*speedSelected; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > ROBOT_MAX_SPEED_FACTOR*speedSelected ) leftMotorSpeed = ROBOT_MAX_SPEED_FACTOR*speedSelected; 
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; 
  
  
  if(error==-2500 ){ // if line is on the left of the robot, stop line detection and rotate anti-clockwise until line is at nearer to the center of the line before moving off
    //motor.motorStop();
    //delay(200);
    while(error==-2500){
      motor.motorLeft(75);
      error = 2500-qtrrc.readLine(sensorValues);      
    }
    //motor.motorStop();
    //delay(200);
    debug("lockleft");    
    debugln();
    go();
  } else if (error==2500){  // otherwise if line is on the right, vice versa
    //motor.motorStop();
    //delay(200);
    while(error==2500){
      motor.motorRight(75);
      error = 2500-qtrrc.readLine(sensorValues);
    }
    //motor.motorStop();
    //delay(200);
    debug("lockleft");    
    debugln();
    go();
  } else { // speed correction when robot is on the line
    if(isLaunchFromStop){ //damped acceleration if robot is moving from a stop
      for(int i=5; i>0;i--){
        analogWrite(LM_PWM,leftMotorSpeed/i);
        analogWrite(RM_PWM,rightMotorSpeed/i);
        delay(70);
      }
      isLaunchFromStop=false;
    } else{
      analogWrite(LM_PWM,leftMotorSpeed);
      analogWrite(RM_PWM,rightMotorSpeed);
    }    
  }

  //Checking for all black signal
  int black=0;
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {    
    if(sensorValues[i]>950) black++;
  }
  if(black==NUM_SENSORS) {
    for(int i=1;i<6;i++){
    analogWrite(LM_PWM,leftMotorSpeed/2^i);
    analogWrite(RM_PWM,rightMotorSpeed/2^i);
    delay(10);
    }
    motor.motorStop();
    isLaunchFromStop=true;
    action="tap";
  }
  debugln(position);
  delay(1); // stabilise robot
  
}


//Movement to give feedback and information to user when controlling robot
void jerk(){
  if(directionTogg){
          motor.motorLeft(60);
        } else {
          motor.motorRight(60);
        }
    directionTogg=!directionTogg;        
    delay(200);
    motor.motorStop();
    delay(200);
}

void tapCard(){
  delay(3500);//bypass 10s
  bool goodToGo=false;
  pwm.setPWM(servonum, 0, 350);
  unsigned long lastTapTime=millis();
  while(!goodToGo){
    if(millis()-lastTapTime>1500){
      pwm.setPWM(servonum,0,420);
      delay(400);
      pwm.setPWM(servonum,0,350);
      lastTapTime=millis();
    }
    int checkNumber=0;
    for (int i=0;i<15;i++){
    if(distSensor.rangeIsClear(100)) checkNumber++;
    else break;
    }
    if (checkNumber==15) goodToGo=true; 
  }
  analogWrite(LM_PWM,45);
  digitalWrite(LM1,HIGH);
  digitalWrite(LM2,LOW);
  analogWrite(RM_PWM,45);
  digitalWrite(RM1,HIGH);
  digitalWrite(RM2,LOW);
  
  Serial.println("lets go");
  action="go";
  pwm.setPWM(servonum, 0, 420);
  
  
}


//tap or untap tapper
void toggleTapper(){
  if (tapped){
    for (int i=420;i<=510;i++)
    {
      pwm.setPWM(servonum, 0, i);
      delay(1);
    }
    delay(50);
    pwm.setPWM(servonum, 0, 0);
  } else{
    pwm.setPWM(servonum, 0, 400);
    /*for (int i=510;i>=420;i--)
    {
      pwm.setPWM(servonum, 0, i);
      delay(5);
    }*/
    delay(50);
    //pwm.setPWM(servonum, 0, 0);
  }
  tapped=!tapped;
}

//Bluetooth Serial interrupt
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

//USB Serial interrupt
void serialEvent() {
   while(Serial.available()){
    action=Serial.readString();
  }
}

