#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

const char* ssid = "NCR-2.4G";         // Replace with your network SSID (name)
const char* password = "choreNCRke";   // Replace with your network password

const char* udpAddress = "192.168.1.17"; // IP address of your PC
const int udpPort = 10010;               // Port on which the PC is listening
const int esp8266Port = 10011;           // Port on which ESP8266 is listening

WiFiUDP udp;

void sendUDPMessage(const char* message);

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
  Wire.setClock(100000);  // Reduce I2C speed for stability

  // Initialize MS5837
  while (!sensor.init()) {
    Serial.println("Failed to initialize MS5837 sensor!");
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

    // Collect sensor data
    String message = "Pressure: ";
    message += String(sensor.pressure() / 1000);
    message += " bar, Temperature: ";
    message += String(sensor.temperature());
    message += " deg C";

    // Send the message to PC
    sendUDPMessage(message.c_str());
  }

  delay(1000);
}

void sendUDPMessage(const char* message) {
  udp.beginPacket(udpAddress, udpPort);
  udp.print(message);
  udp.endPacket();
  Serial.printf("Sent message to PC: %s\n", message);
}