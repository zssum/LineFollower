#include <QTRSensors.h>


#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   23     // emitter is controlled by digital pin 2
#define LM1     25        
#define LM2     27
#define LM_PWM  2
#define RM1     29
#define RM2     31
#define RM_PWM  3
#define MOTORSPEED  40

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {22, 24, 26, 28, 30, 32, 34, 36},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];
String action;


void setup()
{
  pinMode(13, OUTPUT); //Arduino LED
  pinMode(LM1,OUTPUT);
  pinMode(LM2,OUTPUT);
  pinMode(RM1,OUTPUT);
  pinMode(RM2,OUTPUT);
  delay(500);
  Serial.setTimeout(10);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);
}


void loop()
{
  while(Serial.available()){
    action=Serial.readString();
  }


  if(action=="read") read();
  else if (action=="calibrate");
  else if (action=="blah") blah;
  else return;
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
    switch (i){
      case 0: motorLeft(30);
      case 95: motorStop();
      case 105: motorRight(30);
      case 199: motorStop();      
    }
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

