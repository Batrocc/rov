#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include "MS5837.h"
MS5837 sensor;

const char* ssid = "Hell's Gate";         // Replace with your network SSID (name)
const char* password = "Blossom007";      // Replace with your network password

const char* udpAddress = "192.168.34.17"; // IP address of your PC
const int udpPort = 10010;                // Port on which the PC is listening
const int esp8266Port = 10011;            // Port on which ESP8266 is listening

WiFiUDP udp;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  delay(1000);

  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Wire.begin();
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  // Start UDP
  udp.begin(esp8266Port);
  Serial.print("Listening on UDP port ");
  Serial.println(esp8266Port);

  // Send initial message to PC with ESP8266 IP address
  String initialMessage = "Hello from ESP8266. My IP is: ";
  initialMessage += WiFi.localIP().toString();
  sendUDPMessage(initialMessage.c_str());
}

void loop() {
  sensor.read();
  
  // Check for incoming messages
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }

    Serial.printf("Received message from PC: %s\n", incomingPacket);

    // Send a response back to the PC
    String  response = String(sensor.pressure()/1000);
    response += " bar/"+String(sensor.temperature())+" deg C/";
    sendUDPMessage(response.c_str());
  }

  // Serial.print("Pressure: "); 
  // Serial.print(sensor.pressure()); 
  // Serial.println(" mbar");
  
  // Serial.print("Temperature: ");
  // Serial.print(sensor.temperature()); 
  // Serial.println(" deg C");
  
  // Serial.print("Depth: "); 
  // Serial.print(sensor.depth()); 
  // Serial.println(" m");
  
  // Serial.print("Altitude: "); 
  // Serial.print(sensor.altitude()); 
  // Serial.println(" m above mean sea level");
 
  delay(1000);
}

void sendUDPMessage(const char* message) {
  udp.beginPacket(udpAddress, udpPort);
  udp.print(message);
  udp.endPacket();
  Serial.printf("Sent message to PC: %s\n", message);
}
