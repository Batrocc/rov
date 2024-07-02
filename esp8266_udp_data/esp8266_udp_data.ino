#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "MS5837.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ArduinoOTA.h>

MS5837 sensor;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

#ifndef STASSID
#define STASSID "NCR-2.4G"
#define STAPSK "choreNCRke"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;

const char* udpAddress = "192.168.1.4";   // IP address of your PC
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

  // Initialize MS5837 sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(2000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  // Initialize BNO055 sensor
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // Start UDP
  udp.begin(esp8266Port);
  Serial.print("Listening on UDP port ");
  Serial.println(esp8266Port);

  // Send initial message to PC with ESP8266 IP address
  String initialMessage = "Hello from ESP8266. My IP is: ";
  initialMessage += WiFi.localIP().toString();
  sendUDPMessage(initialMessage.c_str());

  // OTA setup
  //---------------------------
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  
  ArduinoOTA.begin();
  Serial.println("Ready for OTA updates");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
  //---------------------------

void loop() {
  ArduinoOTA.handle();

  // Main code begins 
  sensor.read();

  // Check for incoming messages
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    // Serial.printf("Received message from PC: %s\n", incomingPacket);
  }

  // Collect sensors data
  String message = "temp-";
  message += String(sensor.temperature());
  message += "/pressure-";
  message += String(sensor.pressure() / 1000);

  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  message += "/acc-";
  message += String(accelerometerData.acceleration.x);
  message += "/";
  message += String(accelerometerData.acceleration.y);
  message += "/";
  message += String(accelerometerData.acceleration.z);

  message += "/gravity-";
  message += String(gravityData.acceleration.x);
  message += "/";
  message += String(gravityData.acceleration.y);
  message += "/";
  message += String(gravityData.acceleration.z);

  message += "/gyro-";
  message += String(angVelocityData.gyro.x);
  message += "/";
  message += String(angVelocityData.gyro.y);
  message += "/";
  message += String(angVelocityData.gyro.z);

  message += "/mag-";
  message += String(magnetometerData.magnetic.x);
  message += "/";
  message += String(magnetometerData.magnetic.y);
  message += "/";
  message += String(magnetometerData.magnetic.z);

  // Send the message to PC
  sendUDPMessage(message.c_str());
}

void sendUDPMessage(const char* message) {
  udp.beginPacket(udpAddress, udpPort);
  udp.print(message);
  udp.endPacket();
  // Serial.printf("Sent message to PC: %s\n", message);
}
