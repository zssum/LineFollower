#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <ESP8266HTTPClient.h>

#define USE_SERIAL Serial
#define LED 16

String state="";

//const char* ssid = "LAPTOP-7N2NRTRM 3678";
//const char* password = "aaa12345";

const char* ssid = "MyPi";
const char* password = "raspberry";

void setup() {
    pinMode(LED,OUTPUT);
    digitalWrite(LED,HIGH);
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

    WiFi.begin(ssid, password);

    while(WiFi.status() != WL_CONNECTED){
      WiFi.disconnect();
      delay(3000);
      WiFi.begin(ssid, password);
      delay(7000);
      Serial.print(".");
    }
    digitalWrite(LED,LOW);
}

void loop() {
  while(USE_SERIAL.available()){
    state=USE_SERIAL.readString();
    USE_SERIAL.println(state);
    if (state=="checkGate1") checkGate1();
    else if(state=="checkGate2") checkGate2();
  }
  
   

}

void checkGate1(){
        HTTPClient http;
        http.begin("http://192.168.42.101/1");
 
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
        http.begin("http://192.168.42.101/2");
 
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


