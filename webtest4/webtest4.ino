#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <ESP8266HTTPClient.h>

#define USE_SERIAL Serial
#define LED 16

String state="";

ESP8266WiFiMulti WiFiMulti;

void setup() {
    pinMode(LED,OUTPUT);
    digitalWrite(LED,LOW);
    USE_SERIAL.begin(115200);
   // USE_SERIAL.setDebugOutput(true);

    USE_SERIAL.println();
    USE_SERIAL.println();
    USE_SERIAL.println();

    for(uint8_t t = 4; t > 0; t--) {
        USE_SERIAL.printf("[SETUP] WAIT %d...\n", t);
        USE_SERIAL.flush();
        delay(1000);
    }

    WiFiMulti.addAP("LAPTOP-7N2NRTRM 3678", "aaa12345");

    while(WiFiMulti.run() != WL_CONNECTED){
      delay(500);
    }
    digitalWrite(LED,HIGH);

}

void loop() {
  while(Serial.available()){
    state=Serial.readString();
    if (state=="checkGate1") checkGate1();
    else if(state=="checkGate2") checkGate2();
  }
  
   

}

void checkGate1(){
        HTTPClient http;
        http.begin("http://192.168.137.210/1");
 
        int httpCode = http.GET();
        if(httpCode == HTTP_CODE_OK)
        {
           Serial.print("HTTP response code ");
           Serial.println(httpCode);
           String response = http.getString();
           Serial.println(response);
        }
        else
        {
           Serial.println("Error in HTTP request");
        }
         
        http.end();
        state ="idle";
}


void checkGate2(){
        HTTPClient http;
        http.begin("http://192.168.137.210/2");
 
        int httpCode = http.GET();
        if(httpCode == HTTP_CODE_OK)
        {
           Serial.print("HTTP response code ");
           Serial.println(httpCode);
           String response = http.getString();
           Serial.println(response);
        }
        else
        {
           Serial.println("Error in HTTP request");
        }
         
        http.end();
        state ="idle";
}


