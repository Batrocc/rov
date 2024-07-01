#include <iostream>
#include <WinSock2.h>
#include<string>
#define PORT 10010
<<<<<<< HEAD
#define SERVER_IP "192.168.1.108"  // Replace with your PC's IP
#define ESP8266_IP "192.168.1.110"   // Replace with your ESP8266's IP
=======
#define SERVER_IP "192.168.131.17"  // Replace with your PC's IP
#define ESP8266_IP "192.168.131.140"   // Replace with your ESP8266's IP
>>>>>>> c92481a3755808fad2e448f21da35e231b204392
#define ESP8266_PORT 10011           // Replace with the port on which ESP8266 is listening

int main() {
    // Initialize Winsock
    WSADATA wsData;
    WORD ver = MAKEWORD(2, 2);
    int wsOk = WSAStartup(ver, &wsData);
    if (wsOk != 0) {
        std::cerr << "Can't initialize Winsock! Quitting" << std::endl;
        return -1;
    }
    
    // Create a socket
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Can't create socket! Quitting" << std::endl;
        WSACleanup();
        return -1;
    }
    
    // Bind the socket to an IP address and port
    sockaddr_in serverHint;
    serverHint.sin_addr.s_addr = inet_addr(SERVER_IP);  // IP address of your PC
    serverHint.sin_family = AF_INET;
    serverHint.sin_port = htons(PORT);  // Use custom port
    
    if (bind(sock, (sockaddr*)&serverHint, sizeof(serverHint)) == SOCKET_ERROR) {
        std::cerr << "Can't bind socket! Error: " << WSAGetLastError() << std::endl;
        closesocket(sock);
        WSACleanup();
        return -1;
    }
    
    std::cout << "Server is listening on " << SERVER_IP << ":" << PORT << std::endl;

    // Setup the address structure for the ESP8266
    sockaddr_in ESP8266Hint;
    ESP8266Hint.sin_addr.s_addr = inet_addr(ESP8266_IP);  // IP address of the ESP8266
    ESP8266Hint.sin_family = AF_INET;
    ESP8266Hint.sin_port = htons(ESP8266_PORT);  // Port on the ESP8266

    // Main loop to receive and send data
    sockaddr_in client;
    int clientLength = sizeof(client);
    char buf[1024];

    // To send a message every second
    DWORD previousMillis = GetTickCount();
    const DWORD interval = 1000;
    
    while (true) {
        ZeroMemory(&client, clientLength);
        ZeroMemory(buf, 1024);
        
        // Wait for message
        int bytesIn = recvfrom(sock, buf, 1024, 0, (sockaddr*)&client, &clientLength);
        if (bytesIn == SOCKET_ERROR) {
            std::cerr << "Error receiving from client! Error: " << WSAGetLastError() << std::endl;
            break;
        }
        
        // Display message from client
        std::cout << "Message from " << inet_ntoa(client.sin_addr) << ":" << ntohs(client.sin_port) << " - " << buf << std::endl;

        // Send a response to ESP8266
        std::string response = "Message received: ";
        response += buf;
        int sendOk = sendto(sock, response.c_str(), response.size() + 1, 0, (sockaddr*)&ESP8266Hint, sizeof(ESP8266Hint));
        if (sendOk == SOCKET_ERROR) {
            std::cerr << "Error sending to ESP8266! Error: " << WSAGetLastError() << std::endl;
        }

        // Send a message to the ESP8266 every second
        DWORD currentMillis = GetTickCount();
        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
            std::string message = "Hello from PC at " + std::to_string(currentMillis);
            int sendOk = sendto(sock, message.c_str(), message.size() + 1, 0, (sockaddr*)&ESP8266Hint, sizeof(ESP8266Hint));
            if (sendOk == SOCKET_ERROR) {
                std::cerr << "Error sending to ESP8266! Error: " << WSAGetLastError() << std::endl;
            } else {
                std::cout << "" << message << std::endl;
            }
        }
    }
    
    // Close socket
    closesocket(sock);
    
    // Cleanup Winsock
    WSACleanup();
    
    return 0;
}
