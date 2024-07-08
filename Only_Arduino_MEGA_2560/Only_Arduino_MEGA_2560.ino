#include <Ethernet.h> // Load Ethernet Library
#include <EthernetUdp.h> // Load UDP Library
#include <SPI.h> // Load the SPI Library
#include <Servo.h> // Load thrusters library
#include <Wire.h> // Load I2C library
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h> // IMU Library
#include <utility/imumaths.h>
#include "MS5837.h" // Bar30 library
#include "ping1d.h" // Sonar library
#include "SoftwareSerial.h"

// Servo objects
Servo ESC, fthruster1, fthruster2, rthruster, lthruster, dthruster1, dthruster2;

// IMU object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Bar30 sensor object
MS5837 sensor;

// Sonar SoftwareSerial pins & object declaration 
SoftwareSerial pingSerial = SoftwareSerial(53, 52);
static Ping1D ping { pingSerial };

// Thruster values
int f1, f2, l, r, d1, d2 = 1500;
int prevf1, prevf2, prevl , prevr, prevd1, prevd2 = 1500;
String ft1, ft2, lt, rt, dt1, dt2 = "";
String message = "";

// Ethernet settings
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE }; // Assign a mac address
IPAddress ip(192, 168, 1, 14); // Assign my IP address
unsigned int localPort = 5000; // Assign a Port to talk over
char packetBuffer[30];
String datReq; // String for our data
int packetSize; // Size of Packet
EthernetUDP Udp; // Define UDP Object

// IMU data sampling delay
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

void setup() {
  // Attach servos
  fthruster1.attach(6, 1100, 1900);
  fthruster2.attach(4, 1100, 1900);
  lthruster.attach(3, 1100, 1900);
  rthruster.attach(5, 1100, 1900);
  dthruster1.attach(2, 1100, 1900);
  dthruster2.attach(7, 1100, 1900);

  Serial.begin(9600); // Turn on Serial Port
  Ethernet.begin(mac, ip); // Initialize Ethernet
  Udp.begin(localPort); // Initialize UDP

  // Initialize IMU
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000); // Give the IMU time to initialize

  // Initialize Bar30 sensor
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

  // Initialize Sonar
  pingSerial.begin(9600);
      while (!ping.initialize()) {
        Serial.println("\nPing device failed to initialize!");
        delay(1000);
    }
}

void write(String ft1, String ft2, String lt , String rt, String dt1, String dt2) {
  Serial.print("I am going to write : ");
  Serial.println(String(ft1) + '/' + String(ft2) + '/' + String(lt) + '/' + String(rt) + '/' +  String(dt1) + '/' + String(dt2));

  f1 = ft1.toInt();
  if (f1 <= 1900 && f1 >= 1100) {
    fthruster1.writeMicroseconds(f1);
  } else {
    f1 = prevf1;
    fthruster1.writeMicroseconds(f1);
  }

  f2 = ft2.toInt();
  if (f2 <= 1900 && f2 >= 1100) {
    fthruster2.writeMicroseconds(f2);
  } else {
    f2 = prevf2;
    fthruster2.writeMicroseconds(f2);
  }

  d1 = dt1.toInt();
  if (d1 <= 1900 && d1 >= 1100) {
    dthruster1.writeMicroseconds(d1);
  } else {
    d1 = prevd1;
    dthruster1.writeMicroseconds(d1);
  }

  d2 = dt2.toInt();
  if (d2 <= 1900 && d2 >= 1100) {
    dthruster2.writeMicroseconds(d2);
  } else {
    d2 = prevd2;
    dthruster2.writeMicroseconds(d2);
  }
}

void decode(String message) {
  int i;
  for (i = 0; i < 4; i++) {
    ft1 += message[i];
  }
  int val = ft1.toInt();
  if (val <= 1100 || val >= 1900) {
    ft1 = "1500"; 
  }
  for (i = 5; i < 9; i++) {
    ft2 += message[i];  
  }
  int val1 = ft2.toInt();
  if (val1 <= 1100 || val1 >= 1900) {
    ft2 = "1500"; 
  }
  for (i = 10; i < 14; i++) {
    lt += message[i];
  }
  int val2 = lt.toInt();
  if (val2 <= 1100 || val2 >= 1900) {
    l = "1500"; 
  }
  for (i = 15; i < 19; i++) {
    rt += message[i];
  }
  int val3 = rt.toInt();
  if (val3 <= 1100 || val3 >= 1900) {
    r = "1500"; 
  }
  for (i = 20; i < 24; i++) {
    dt1 += message[i];
  }
  int val4 = dt1.toInt();
  if (val4 <= 1100 || val4 >= 1900) {
    d1 = "1500"; 
  }
  for (i = 25; i < 29; i++) {
    dt2 += message[i];
  }
  int val5 = dt2.toInt();
  if (val5 <= 1100 || val5 >= 1900) {
    d2 = "1500"; 
  }

  Serial.print("The received commands are : ");
  Serial.println(String(ft1) + '/' + String(ft2) + '/' + String(lt) + '/' + String(rt) + '/' +  String(dt1) + '/' + String(dt2));
  write(ft1, ft2, lt, rt, dt1, dt2);
  ft1 = "";
  ft2 = "";
  lt = "";
  rt = "";
  dt1 = "";
  dt2 = "";
}

void loop() {
  packetSize = Udp.parsePacket(); // Read the packetSize

  if (packetSize > 0) { // Check to see if a request is present
    Udp.read(packetBuffer, 30); // Reading the data request on the Udp
    String datReq(packetBuffer); // Convert packetBuffer array to string datReq
    message = datReq.substring(0, 29);
    decode(message);
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  // Initialize Packet send
    Udp.print(String(random(0, 50))); // Send string back to client 
    Udp.endPacket();
    memset(packetBuffer, 0, 30);
  }

  // Read IMU data
  sensors_event_t orientationData, angVelocityData, accelerometerData, linearAccelData, magnetometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  // Create IMU data string
  String message = "ACC: " + String(accelerometerData.acceleration.x, 1) + "/" + 
                   String(accelerometerData.acceleration.y, 1) + "/" + 
                   String(accelerometerData.acceleration.z, 1) + "/" +
                   "GYRO: " + String(angVelocityData.gyro.x, 1) + "/" + 
                   String(angVelocityData.gyro.y, 1) + "/" + 
                   String(angVelocityData.gyro.z, 1) + "/" +
                   "ORIENT: " + String(orientationData.orientation.x, 1) + "/" + 
                   String(orientationData.orientation.y, 1) + "/" + 
                   String(orientationData.orientation.z, 1) + "/" +
                   "MAG: " + String(magnetometerData.magnetic.x, 1) + "/" + 
                   String(magnetometerData.magnetic.y, 1) + "/" + 
                   String(magnetometerData.magnetic.z, 1) + "/" +
                   "LINEAR: " + String(linearAccelData.acceleration.x, 1) + "/" + 
                   String(linearAccelData.acceleration.y, 1) + "/" + 
                   String(linearAccelData.acceleration.z, 1) + "/" +
                   "GRAV: " + String(gravityData.acceleration.x, 1) + "/" + 
                   String(gravityData.acceleration.y, 1) + "/" + 
                   String(gravityData.acceleration.z, 1) + "/";

  // Read Bar30 sensor data
  sensor.read();
  message += "PRESSURE: " + String(sensor.pressure()/1000, 1) + " bar/" +
                     "TEMP: " + String(sensor.temperature(), 1) + " °C/";

  // Read Sonar data
  if (ping.update()) {
  message += "DEPTH: " + String(ping.distance(), 1) + " m (" +
                           String(ping.confidence(), 1) + ")";
  } else {
        Serial.println("No update received!");
  }

  Serial.println(message);

  // Send Bar30 sensor data over UDP
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.print(message);
  Udp.endPacket();

  // delay(BNO055_SAMPLERATE_DELAY_MS); // Delay between samples
}
