/*
 * This is code made to propagate the WiFi network
 */

//We need exactly softAP function from the WiFi.h library
#include <WiFi.h>

//Define the internal led
#define led 2

const char *ssid = "MyESP32"; //name
const char *password = "testpassword"; //pass
//Defines default network's settings
IPAddress local_IP(192,168,4,22);
IPAddress gateway(192,168,4,9);
IPAddress subnet(255,255,255,0);

WiFiServer server(80);

void setup() {  
  //Start writing in the IDE monitor at port 11520
  Serial.begin(115200);
  while(!Serial) {
    ; //wait for serial port
  } 
  Serial.println();

  pinMode(led, OUTPUT);

  WiFi.begin(ssid, password);
  
  Serial.print("Setting Access Point...");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
  //Start propagating the network
  Serial.println("Running Access Point");
  //channel = 1, not hided, max_connection = 1
  Serial.println(WiFi.softAP(ssid, password, 1, 0, 1) ? "Ready" : "Failed!");
  Serial.println("WiFi address: ");
  //Shows the ESP32 IP
  Serial.println(WiFi.softAPIP());

  Serial.print(WiFi.localIP());

  server.begin();
}

void loop() {
  checkIfConnected();
}

void checkIfConnected() {
  //Check for the active connection
  if(WiFi.softAPgetStationNum() >= 1){
    digitalWrite(led, HIGH);
  }else{
    digitalWrite(led, LOW);
  }
}


