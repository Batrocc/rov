#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "MS5837.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ArduinoOTA.h>

MS5837 sensor;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

#ifndef STASSID
#define STASSID "hi"
#define STAPSK "hello123"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;

const char* udpAddress = "192.168.0.108"; // IP address of your PC
const int udpPort = 10010;                // Port on which the PC is listening
const int esp32Port = 10011;              // Port on which ESP32 is listening

WiFiUDP udp;

// Servo objects for thrusters
Servo thruster1;
Servo thruster2;
Servo thruster3;
Servo thruster4;
Servo thruster5;
Servo thruster6;

unsigned long previousMillis = 0;
const long interval = 1000;  // Interval to send sensor data (in milliseconds)

// Pin definitions for additional data reception
const int dataPin = 4; // Data pin
const int clockPin = 5; // Clock pin
const int ledPin = 2; // ESP32 built-in LED for feedback

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

  // Initialize servos
  thruster1.attach(13);
  thruster2.attach(14);
  thruster3.attach(32);
  thruster4.attach(27);
  thruster5.attach(26);
  thruster6.attach(25);

  // Start UDP
  udp.begin(esp32Port);
  Serial.print("Listening on UDP port ");
  Serial.println(esp32Port);

  // Send initial message to PC with ESP32 IP address
  String initialMessage = "Hello from ESP32. My IP is: ";
  initialMessage += WiFi.localIP().toString();
  sendUDPMessage(initialMessage.c_str());

  // OTA setup
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
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

  // Initialize additional data reception pins
  pinMode(dataPin, INPUT);
  pinMode(clockPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.println("ESP32 ready to receive data...");
}

void loop() {
  ArduinoOTA.handle();

  // Check for incoming thruster commands
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }

    // Print the incoming string as it is
    Serial.print("Received string: ");
    Serial.println(incomingPacket);

    // Process the incoming packet
    String msg = String(incomingPacket);
    int thrusterValues[6];
    int startIndex = 0;
    int endIndex = msg.indexOf('/');

    for (int i = 0; i < 6; i++) {
      thrusterValues[i] = msg.substring(startIndex, endIndex).toInt();
      startIndex = endIndex + 1;
      endIndex = msg.indexOf('/', startIndex);
      if (endIndex == -1) {
        endIndex = msg.length();
      }
    }

    // Print PWM values to serial monitor
    for (int i = 0; i < 6; i++) {
      Serial.print("Thruster ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(thrusterValues[i]);
    }

    // Update thruster values immediately
    thruster1.writeMicroseconds(thrusterValues[0]);
    thruster2.writeMicroseconds(thrusterValues[1]);
    thruster3.writeMicroseconds(thrusterValues[2]);
    thruster4.writeMicroseconds(thrusterValues[3]);
    thruster5.writeMicroseconds(thrusterValues[4]);
    thruster6.writeMicroseconds(thrusterValues[5]);
  }

  // Send sensor data at regular intervals
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Collect sensors data
    sensor.read();
    String message = "TEMPERATURE: ";
    message += String(sensor.temperature());
    message += "/PRESSURE: ";
    message += String(sensor.pressure() / 1000);

    sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    message += "/ACCELEROMETER: ";
    message += String(accelerometerData.acceleration.x);
    message += "/";
    message += String(accelerometerData.acceleration.y);
    message += "/";
    message += String(accelerometerData.acceleration.z);

    message += "/GRAVITY: ";
    message += String(gravityData.acceleration.x);
    message += "/";
    message += String(gravityData.acceleration.y);
    message += "/";
    message += String(gravityData.acceleration.z);

    message += "/GYROSCOPE: ";
    message += String(angVelocityData.gyro.x);
    message += "/";
    message += String(angVelocityData.gyro.y);
    message += "/";
    message += String(angVelocityData.gyro.z);

    message += "/MAGNETOMETER: ";
    message += String(magnetometerData.magnetic.x);
    message += "/";
    message += String(magnetometerData.magnetic.y);
    message += "/";
    message += String(magnetometerData.magnetic.z);

    // Receive additional data and append to the message
    // String received = receiveString();
    // if (received.length() > 0) {
    //   message += "/";
    //   message += received;
    // }

    // Send the message to PC
    sendUDPMessage(message.c_str());
  }
}

void sendUDPMessage(const char* message) {
  udp.beginPacket(udpAddress, udpPort);
  udp.print(message);
  udp.endPacket();
}

bool receiveBit() {
  while (digitalRead(clockPin) == LOW); // Wait for the clock to go high
  bool bit = digitalRead(dataPin);
  while (digitalRead(clockPin) == HIGH); // Wait for the clock to go low
  return bit;
}

char receiveByte() {
  char byte = 0;
  for (int i = 0; i < 8; i++) {
    bitWrite(byte, i, receiveBit());
  }
  return byte;
}

String receiveString() {
  String received = "DEPTH: ";
  char byte;
  do {
    byte = receiveByte();
    if (byte != '\0') {
      received += byte;
    }
  } while (byte != '\0');

  if (received.length() > 0) {
    Serial.println("Received: " + received); // Print the received string to the Serial Monitor

    // Toggle the LED to show valid data was received
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
  }
  
  return received;
}