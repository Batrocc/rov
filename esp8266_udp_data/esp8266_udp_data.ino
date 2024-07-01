#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "MS5837.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

MS5837 sensor;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

const char* ssid = "AndroidAP_1390";         // Replace with your network SSID (name)
const char* password = "Bazooka4457#";      // Replace with your network password

const char* udpAddress = "192.168.131.17"; // IP address of your PC
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
  // while (!sensor.init()) {
  //   Serial.println("Init failed!");
  //   Serial.println("Are SDA/SCL connected correctly?");
  //   Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
  //   Serial.println("\n\n\n");
  //   delay(5000);
  // }
  
  // sensor.setModel(MS5837::MS5837_30BA);
  // sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);

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
  // sensor.read();
  
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
    String message = "";
    // String message = "Pressure: ";
    // message += String(sensor.pressure()/1000);
    // message += " bar, Temperature: ";
    // message += String(sensor.temperature());
    // message += " deg C, ";

    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    message += formatEvent("Orientation", &orientationData);
    message += formatEvent("Gyroscope", &angVelocityData);
    message += formatEvent("Linear Acceleration", &linearAccelData);
    message += formatEvent("Magnetometer", &magnetometerData);
    message += formatEvent("Accelerometer", &accelerometerData);
    message += formatEvent("Gravity", &gravityData);

    // Send the message to PC
    sendUDPMessage(message.c_str());
  }

  delay(1000);
}

String formatEvent(const char* type, sensors_event_t* event) {
  String message = type;
  message += ": x=";
  message += String(event->acceleration.x);
  message += ", y=";
  message += String(event->acceleration.y);
  message += ", z=";
  message += String(event->acceleration.z);
  message += " ";
  return message;
}

void sendUDPMessage(const char* message) {
  udp.beginPacket(udpAddress, udpPort);
  udp.print(message);
  udp.endPacket();
  Serial.printf("Sent message to PC: %s\n", message);
}
