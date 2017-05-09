#define LM1     25
#define LM2     27
#define LM_PWM  2
#define RM1     29
#define RM2     31
#define RM_PWM  3
#define MOTORSPEED  40

String action

void setup() {
  // put your setup code here, to run once:
  pinMode(LM1,OUTPUT);
  pinMode(LM2,OUTPUT);
  pinMode(RM1,OUTPUT);
  pinMode(RM2,OUTPUT);
  delay(500);
  Serial.setTimeout(10);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);
}

void loop() {
  while(Serial.available()){
    action=Serial.readString();
  }
  
  if (action=="f") {
      analogWrite(LM_PWM,MOTORSPEED);
      digitalWrite(LM1,HIGH);
      digitalWrite(LM2,LOW);
      analogWrite(RM_PWM,MOTORSPEED);
      digitalWrite(RM1,HIGH);
      digitalWrite(RM2,LOW);
  } else if (action =="b") {
      analogWrite(LM_PWM,MOTORSPEED);
      digitalWrite(LM1,LOW);
      digitalWrite(LM2,HIGH);
      analogWrite(RM_PWM,MOTORSPEED);
      digitalWrite(RM1,LOW);
      digitalWrite(RM2,HIGH);
  } else if (action =="l") {
      analogWrite(LM_PWM,MOTORSPEED);
      digitalWrite(LM1,LOW);
      digitalWrite(LM2,HIGH);
      analogWrite(RM_PWM,MOTORSPEED);
      digitalWrite(RM1,HIGH);
      digitalWrite(RM2,LOW);
  } else if (action=="r") {
      analogWrite(LM_PWM,MOTORSPEED);
      digitalWrite(LM1,HIGH);
      digitalWrite(LM2,LOW);
      analogWrite(RM_PWM,MOTORSPEED);
      digitalWrite(RM1,LOW);
      digitalWrite(RM2,HIGH);
  } else (action=="s") {
      analogWrite(LM_PWM,0);
      digitalWrite(LM1,LOW);
      digitalWrite(LM2,LOW);
      analogWrite(RM_PWM,0);
      digitalWrite(RM1,LOW);
      digitalWrite(RM2,LOW);
  }
  

}
