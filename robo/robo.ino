#include <QTRSensors.h>
#include <Wire.h>
#include "motor.h"
#include "distSensor.h"
#include "tapper.h"

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
#define RM1     25
#define RM2     27
#define RM_PWM  3
#define MOTOR_GND   29
#define MOTOR_5V    31
#define ROBOT_MAX_SPEED_FACTOR  1.7 // Factor of speedSelected to allow for speed variation between left and right wheels
#define INITIAL_SPEED 45
#define INITIAL_Kp 0.035

// Ultrasound Distance Detector Settings
#define  trigPin  51
#define  echoPin  49
#define  sonicVcc 53
#define  sonicGnd 47

#define servonum  0 // tapper is attached to channel 0 out of 16 of the pwm controlboard

// Initialising line sensor, motors, distance sensor and tapper
QTRSensorsRC qtrrc((unsigned char[]) { 24, 26, 28, 30, 32, 34 }, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
Motor motor(LM1, LM2, LM_PWM, RM1, RM2, RM_PWM, MOTOR_GND, MOTOR_5V);
DistSensor distSensor(trigPin, echoPin, sonicVcc, sonicGnd);
Tapper tapper;

int speedSelected = INITIAL_SPEED; //Inital speed
float Kp = INITIAL_Kp; // LineFollower proportional constant
unsigned int sensorValues[NUM_SENSORS]; //IR Sensor values
String action; // Robot's state
String inputString; // Command builder for serial strings
boolean directionTogg = true; // Toggle between clockwise and anti-clockwise rotation movements as a feedback to user
boolean isLaunchFromStop = true; // Used to soften acceleration if robot is launching from rest
boolean tapped = false; //tapper state

void setup()
{
  
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  Serial.setTimeout(80);
  Serial1.begin(9600);  //set bluetooth baud rate

  pinMode(13, OUTPUT); //Arduino LED
  tapper.initialise(servonum);
  tapper.liftUp();

  qtrrc.calibrate();
  // Manual callibration by recording read() values
  // Raw values at KCD floor: 1104	1104	1020	1224	1264	1388		plus 1000 for noise reduction
  qtrrc.calibratedMinimumOn[0] = 2104;
  qtrrc.calibratedMinimumOn[1] = 2104;
  qtrrc.calibratedMinimumOn[2] = 2020;
  qtrrc.calibratedMinimumOn[3] = 2224;
  qtrrc.calibratedMinimumOn[4] = 2264;
  qtrrc.calibratedMinimumOn[5] = 2388;

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    qtrrc.calibratedMaximumOn[i] = TIMEOUT;
  }

}


void loop()
{
  if (action == "calibrate") calibrate(); //Recalibrate line sensor to a different surface
  else if (action == "d") drive(); // Main robot program
  else if (action == "go") go(); // Switching the motors in the forward direction before moving on to the main robot program
  else if (action == "read") read(); // Read raw values of line sensor
  else if (action == "readline") readline(); // Read calibrated values and position of Robot on the line
  else if (action == "select") selectSpeed();
  else if (action == "tap") tapCard();
  // Manual control of the movement of the robot
  else if (action == "f") motor.motorFwd(40);
  else if (action == "b") motor.motorBack(40);
  else if (action == "l") motor.motorLeft(75); // Rotate Anti-Clockwise
  else if (action == "r") motor.motorRight(75); //Rotate Clockwise
  else if (action == "s") motor.motorStop();
  else if (action == "check") {
    Serial.println(distSensor.rangeIsClear(150));
    Serial.print(distSensor.cm);
    Serial.print("cm");
    Serial.println();
  }
  else motor.motorStop();

  //Ensures robot will brake if an object is in front
  //The robot will continue to move if the obstacle is removed


  while (!distSensor.rangeIsClear(30)) { //TODO: check logic if it restarts
    motor.changeSpeed(0, 0);
    debugln("object in front");
    Serial.println("object in front");
    Serial.println(distSensor.cm);
    Serial.println("cm");
  }

}

//Sensors on the left of the black line and excecute calibration so that the robot will rotate through the black line for all the sensors
void calibrate() {
  motor.motorStop();
  delay(500);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 200; i++)
  {
    if (i < 60) motor.motorLeft(65);
    else if (i > 60 && i < 140) motor.motorStop();
    else if (i > 141 && i < 199) motor.motorRight(65);
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
  action = "s";
}

void go() {
  analogWrite(LM_PWM, 0);
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  analogWrite(RM_PWM, 0);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  action = "d";
}

void drive() {
  unsigned int position = qtrrc.readLine(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = 2500 - position;
  int motorSpeed = Kp * error;
  int rightMotorSpeed = speedSelected - motorSpeed;
  int leftMotorSpeed = speedSelected + motorSpeed;

  if (rightMotorSpeed > ROBOT_MAX_SPEED_FACTOR * speedSelected ) rightMotorSpeed = ROBOT_MAX_SPEED_FACTOR * speedSelected; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > ROBOT_MAX_SPEED_FACTOR * speedSelected ) leftMotorSpeed = ROBOT_MAX_SPEED_FACTOR * speedSelected;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;


  if (error == -2500 ) { // if line is on the left (i.e. error=-2500) of the robot, stop line detection and rotate anti-clockwise until detects the line before moving off
    while (error == -2500) {
      motor.motorLeft(75);
      error = 2500 - qtrrc.readLine(sensorValues);
    }
    debug("lockleft");
    debugln();
    go();
  } else if (error == 2500) { // otherwise if line is on the right, vice versa
    while (error == 2500) {
      motor.motorRight(75);
      error = 2500 - qtrrc.readLine(sensorValues);
    }
    debug("lockleft");
    debugln();
    go();
  } else { // speed correction when robot is on the line
    if (isLaunchFromStop) { //damped acceleration if robot is moving from a stop
      motor.softAccelerateToSpeed(leftMotorSpeed, rightMotorSpeed);
      isLaunchFromStop = false;
    } else motor.changeSpeed(leftMotorSpeed, rightMotorSpeed);
  }

  //Checking for all black signal
  if (robotIsAtStopPoint()) {
    motor.softBrakeFromSpeed(leftMotorSpeed, rightMotorSpeed);
    isLaunchFromStop = true;
    action = "tap";
  }

  debugln(position);
  delay(1); // stabilise robot
}

void read() {
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

void readline() {
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

//Program to select the motor's speed from 70-110 in increments of 10
//User presses "selectSpeed" button to start the program, then presses "+" button to increment 10 to the base speed of 70
//Robot responds with a jerk on press of the increment button
//speedSelecting takes values 0 to 4. A selection of 0 gives speed of 70 while a selection of 4 gives speed of 110
//At the end of selection, user presses "selectSpeed" button to end the program. The robot will jerk the number of times to feedback the selected speed to the user.
void selectSpeed() {
  digitalWrite(13, HIGH); //Turn on indicators to feedback the selectSpeed program is ongoing
  motor.motorStop();
  boolean done = false;
  int speedSelecting = 0;
  while (!done) {
    while (Serial1.available()) {
      char inChar = (char)Serial1.read();
      if (inChar == '+') {
        speedSelecting++;
        jerk();
        if (speedSelecting == 2) {
          done = true;
          digitalWrite(13, LOW);
          delay(500);
        }
      } else if (inChar == '#') {
        done = true;
        digitalWrite(13, LOW);
        delay(500);
      }
    }
  }
  debugln("Speed selected: ");
  debug(speedSelecting);
  for (int i = 0; i < speedSelecting; i++) {
    jerk();
  }
  //adjust speed and linefollowing constants
  speedSelected = INITIAL_SPEED + speedSelecting * 10;
  Kp = INITIAL_Kp * INITIAL_SPEED / speedSelected;
  action = "s";
}

void tapCard() {
  delay(3500);//bypass 10s
  bool goodToGo = false;
  tapper.tap();
  unsigned long lastTapTime = millis();
  int attemptNumber=0;
  while (!goodToGo&&attemptNumber<4) {
    if (millis() - lastTapTime > 2500) {
      tapper.liftUp();
      delay(600);
      tapper.tap();
      lastTapTime = millis();
      attemptNumber++;
    }
    int checkNumber = 0;
    for (int i = 0; i < 10; i++) {
      if (distSensor.rangeIsClear(100)) {
        checkNumber++;
        delay(1);
      }
      else break;
    }
    if (checkNumber == 10) goodToGo = true; // Proceed only after readings stabilized, i.e. for 10 readings
  }
  
  tapper.liftUp();
  if(goodToGo){
    motor.motorFwd(speedSelected);
    Serial.println("lets go");
    action = "go";
    
  } else{
    action ="s";
  }
}


//Movement to give feedback and information to user when controlling robot
void jerk() {
  if (directionTogg) {
    motor.motorLeft(60);
  } else {
    motor.motorRight(60);
  }
  directionTogg = !directionTogg;
  delay(200);
  motor.motorStop();
  delay(200);
}



bool robotIsAtStopPoint() {
  int black = 0;
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorValues[i] > 950) black++;
  }
  if (black == NUM_SENSORS) return true;
  else return false;
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
      action = inputString;
      inputString = "";
    } else {
      inputString += inChar;
    }
  }
}

//USB Serial interrupt
void serialEvent() {
  while (Serial.available()) {
    action = Serial.readString();
  }
}

