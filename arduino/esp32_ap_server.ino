#include <WiFi.h>

//Define the internal led
#define led 2
int pin = 21;

const char *ssid = "MYESP32"; //name
const char *password = "testpassword"; //pass

//Defines default network's settings
IPAddress local_IP(192,168,4,22);
IPAddress gateway(192,168,4,9);
IPAddress subnet(255,255,255,0);

WiFiServer server(80);

void setup() {
    Serial.begin(115200);
    pinMode(led, OUTPUT);
    pinMode(pin, OUTPUT);      // set the LED pin mode
    delay(10);

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

int value = 0;

void loop() {
  checkIfConnected();

   WiFiClient client = server.available();   // listen for incoming clients

    if (client) {                             // if you get a client,
      Serial.println("New Client.");           // print a message out the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      while (client.connected()) {            // loop while the client's connected
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
          if (c == '\n') {                    // if the byte is a newline character
  
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println();
  
              // the content of the HTTP response follows the header:
              client.print("Click <a href=\"/H\">here</a> to turn the LED on pin 5 on.<br>");
              client.print("Click <a href=\"/L\">here</a> to turn the LED on pin 5 off.<br>");
  
              // The HTTP response ends with another blank line:
              client.println();
              // break out of the while loop:
              break;
            } else {    // if you got a newline, then clear currentLine:
              currentLine = "";
            }
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }
  
          // Check to see if the client request was "GET /H" or "GET /L":
          if (currentLine.endsWith("GET /H")) {
            digitalWrite(pin, HIGH);               // GET /H turns the LED on
            Serial.println("-------led ON-------");
          }
          if (currentLine.endsWith("GET /L")) {
            digitalWrite(pin, LOW);               // GET /L turns the LED off
            Serial.println("-------led OFF-------");
          }
        }
      }
      // close the connection:
      client.stop();
      Serial.println("Client Disconnected.");
    }
}

void checkIfConnected() {

  //Check for the active connection
  if(WiFi.softAPgetStationNum() >= 1){
    digitalWrite(led, HIGH);
  }else{
    digitalWrite(led, LOW);
  }
}
