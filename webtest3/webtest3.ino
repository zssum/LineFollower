/*********
  Rui Santos
  Complete project details at http://randomnerdtutorials.com  
*********/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#define GATE1 12
#define GATE2 14

MDNSResponder mdns;

// Replace with your network credentials
//const char* ssid = "LAPTOP-7N2NRTRM 3678";
//const char* password = "aaa12345";
const char* ssid = "MyPi";
const char* password = "raspberry";

ESP8266WebServer server(80);

String webPage = "";

void setup(void){
  webPage += "<h1>ESP8266 Web Server</h1><p>Socket #1 <a href=\"socket1On\"><button>ON</button></a>&nbsp;<a href=\"socket1Off\"><button>OFF</button></a></p>";
  webPage += "<p>Socket #2 <a href=\"socket2On\"><button>ON</button></a>&nbsp;<a href=\"socket2Off\"><button>OFF</button></a></p>";
  
  // preparing GPIOs
  pinMode(GATE1,INPUT);
  pinMode(GATE2,INPUT);
  
  delay(3000);
  Serial.begin(115200);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    delay(3000);
    WiFi.begin(ssid, password);
    delay(7000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  if (mdns.begin("esp8266", WiFi.localIP())) {
    Serial.println("MDNS responder started");
  }

  server.on("/1", checkIfGate1Open);
  server.on("/2", checkIfGate2Open);
  server.on("/", handleRoot);
  
  server.begin();
  Serial.println("HTTP server started");
}
 
void loop(void){
  server.handleClient();
} 

void checkIfGate1Open(){
  while(digitalRead(GATE1)==LOW){
    delay(50);
  }
  server.send(200, "text/html", "k");
}

void checkIfGate2Open(){
  while(digitalRead(GATE2)==LOW){
    delay(50);
  }
  server.send(200, "text/html", "k");
}

void handleRoot() {
    server.send(200, "text/html", "<h1>Hello! from arduino-er!</h1>");
}
