#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Hell's Gate";         // Replace with your network SSID (name)
const char* password = "Blossom007"; // Replace with your network password

const char* udpAddress = "192.168.34.17"; // IP address of your PC
const int udpPort = 10010;                // Port on which the PC is listening
const int esp32Port = 10011;              // Port on which ESP32 is listening

WiFiUDP udp;

unsigned long previousMillis = 0;
const long interval = 1000;

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

  // Start UDP
  udp.begin(esp32Port);
  Serial.print("Listening on UDP port ");
  Serial.println(esp32Port);

  // Send initial message to PC with ESP32 IP address
  String initialMessage = "Hello from ESP32. My IP is: ";
  initialMessage += WiFi.localIP().toString();
  sendUDPMessage(initialMessage.c_str());
}

void loop() {
  // Check for incoming messages
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }

    Serial.printf("Received message from PC: %s\n", incomingPacket);

    // Optionally, send a response back to the PC
    String response = "ESP32 received: ";
    response += incomingPacket;
    sendUDPMessage(response.c_str());
  }

  // Send a message to the PC every second
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    String message = "Hello from ESP32 at " + String(currentMillis);
    sendUDPMessage(message.c_str());
  }
}

void sendUDPMessage(const char* message) {
  udp.beginPacket(udpAddress, udpPort);
  udp.printf(message);
  udp.endPacket();
  Serial.printf("Sent message to PC: %s\n", message);
}