#define ledPin 13
int state = 0;
String inputString="";
String action="";

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial1.begin(9600); // Default communication rate of the Bluetooth module
  Serial.begin(9600);
}

void loop() {
 /* if(Serial1.available() > 0){ // Checks whether data is comming from the serial port
    state = Serial1.read(); // Reads the data from the serial port
 }*/
  if (action=="a"){
    digitalWrite(ledPin,HIGH);
  } else{
    digitalWrite(ledPin,LOW);
  }
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
