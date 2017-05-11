#include <QTRSensors.h>


#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   23     // emitter is controlled by digital pin 2
#define LM1     25        //(25,27,29,31 burnt)
#define LM2     27
#define LM_PWM  2 //(2,3 burnt)
#define RM1     29
#define RM2     31
#define RM_PWM  3
#define MOTORSPEED  50



#define Kp 0.08 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0.1 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define RIGHT_MAX_SPEED 120 // max speed of the robot
#define LEFT_MAX_SPEED  120// max speed of the robot
#define RIGHT_BASE_SPEED 50 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define LEFT_BASE_SPEED 50  // this is the speed at which the motors should spin when the robot is perfectly on the line

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {22, 24, 26, 28, 30, 32, 34, 36},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];


String action;
int lastError = 0;

void setup()
{
  pinMode(13, OUTPUT); //Arduino LED
  pinMode(LM1,OUTPUT);
  pinMode(LM2,OUTPUT);
  pinMode(RM1,OUTPUT);
  pinMode(RM2,OUTPUT);
  digitalWrite(LM1,LOW);
  digitalWrite(LM2,LOW);
  digitalWrite(RM1,LOW);
  digitalWrite(RM2,LOW);
  
  
  
  delay(500);
  Serial.setTimeout(80);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);
  
  qtrrc.calibrate();
  
  qtrrc.calibratedMinimumOn[0]=1008;
  qtrrc.calibratedMinimumOn[1]=692;
  qtrrc.calibratedMinimumOn[2]=588;
  qtrrc.calibratedMinimumOn[3]=588;
  qtrrc.calibratedMinimumOn[4]=484;
  qtrrc.calibratedMinimumOn[5]=488;
  qtrrc.calibratedMinimumOn[6]=484;
  qtrrc.calibratedMinimumOn[7]=640;
  
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    //qtrrc.calibratedMaximumOn[i]=600;
    qtrrc.calibratedMaximumOn[i]=2500;
  }
  
}


void loop()
{
  while(Serial.available()){
    action=Serial.readString();
  }


  if(action=="read") read();
  else if (action=="calibrate") calibrate();
  else if (action=="blah") blah();
  else if (action=="f") motorFwd(MOTORSPEED);
  else if (action=="b") motorBack(MOTORSPEED);
  else if (action=="l") motorLeft(MOTORSPEED);
  else if (action=="r") motorRight(MOTORSPEED);
  else if (action=="s") motorStop();
  else if (action=="go") drive();
  else motorStop();
}

void blah(){
  Serial.print("ok");
  delay(1000);
  Serial.print("wheee");
}

void read(){
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
    /*switch (i){
      Serial.print(i);
      case 0: motorLeft(60);
      case 95: motorStop();
      case 105: motorRight(60);
      case 199: motorStop();      
    }*/
    if(i<60) motorLeft(50);
    else if (i>60&&i<140) motorStop();
    else if (i>141&& i<199) motorRight(50);
    else  motorStop();
      
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
  action= "idle";
}




void motorFwd(int motorspeed){
    analogWrite(LM_PWM, motorspeed);
    digitalWrite(LM1,HIGH);
    digitalWrite(LM2,LOW);
    analogWrite(RM_PWM,motorspeed);
    digitalWrite(RM1,HIGH);
    digitalWrite(RM2,LOW);
}

void motorBack(int motorspeed){
    analogWrite(LM_PWM, motorspeed);
    digitalWrite(LM1,LOW);
    digitalWrite(LM2,HIGH);
    analogWrite(RM_PWM,motorspeed);
    digitalWrite(RM1,LOW);
    digitalWrite(RM2,HIGH);
}

void motorLeft(int motorspeed){
    analogWrite(LM_PWM,motorspeed);
    digitalWrite(LM1,LOW);
    digitalWrite(LM2,HIGH);
    analogWrite(RM_PWM,motorspeed);
    digitalWrite(RM1,HIGH);
    digitalWrite(RM2,LOW);
}

void motorRight(int motorspeed){
    analogWrite(LM_PWM,motorspeed);
    digitalWrite(LM1,HIGH);
    digitalWrite(LM2,LOW);
    analogWrite(RM_PWM,motorspeed);
    digitalWrite(RM1,LOW);
    digitalWrite(RM2,HIGH);
}

void motorStop(){
    analogWrite(LM_PWM,0);
    digitalWrite(LM1,LOW);
    digitalWrite(LM2,LOW);
    analogWrite(RM_PWM,0);
    digitalWrite(RM1,LOW);
    digitalWrite(RM2,LOW);
}

void drive(){
  unsigned int position = qtrrc.readLine(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = 3500-position;

  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = RIGHT_BASE_SPEED - motorSpeed;
  int leftMotorSpeed = LEFT_BASE_SPEED + motorSpeed;
  
  if (rightMotorSpeed > RIGHT_MAX_SPEED ) rightMotorSpeed = RIGHT_MAX_SPEED; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > LEFT_MAX_SPEED ) leftMotorSpeed = LEFT_MAX_SPEED; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  
  if(error==-3500 ){
    //motorStop();
    //delay(5);
    motorLeft(70);
    Serial.print("lockleft");    
    Serial.println();
    delay(200);
  } else if (error==3500){
    Serial.print("lockright");
    Serial.println();
    //motorStop();
    //delay(5);
    motorRight(70);
    delay(200);
  } else {
    analogWrite(LM_PWM,leftMotorSpeed);
    digitalWrite(LM1,HIGH);
    digitalWrite(LM2,LOW);
    analogWrite(RM_PWM,rightMotorSpeed);
    digitalWrite(RM1,HIGH);
    digitalWrite(RM2,LOW);
    //Serial.print("drive");
    //Serial.println();
  }
  
  int black=0;
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {    
    if(sensorValues[i]==1000) black++;
  }
  if(black==8) action="s";
  Serial.println(position);
  delay(2);
  
  
}

//1052 776 632 676 576 576 580 788 

//2500 2500 2500 2500 2500 2500 2500 2500 


