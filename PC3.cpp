#include <iostream>
#include <WinSock2.h>

#define PORT 10010
#define SERVER_IP "192.168.34.17"  // Replace with your PC's IP
#define ESP32_IP "192.168.34.244"   // Replace with your ESP32's IP
#define ESP32_PORT 10011           // Replace with the port on which ESP32 is listening

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

    // Setup the address structure for the ESP32
    sockaddr_in esp32Hint;
    esp32Hint.sin_addr.s_addr = inet_addr(ESP32_IP);  // IP address of the ESP32
    esp32Hint.sin_family = AF_INET;
    esp32Hint.sin_port = htons(ESP32_PORT);  // Port on the ESP32

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

        // Send a response to ESP32
        std::string response = "Message received: ";
        response += buf;
        int sendOk = sendto(sock, response.c_str(), response.size() + 1, 0, (sockaddr*)&esp32Hint, sizeof(esp32Hint));
        if (sendOk == SOCKET_ERROR) {
            std::cerr << "Error sending to ESP32! Error: " << WSAGetLastError() << std::endl;
        }

        // Send a message to the ESP32 every second
        DWORD currentMillis = GetTickCount();
        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
            std::string message = "Hello from PC at " + std::to_string(currentMillis);
            int sendOk = sendto(sock, message.c_str(), message.size() + 1, 0, (sockaddr*)&esp32Hint, sizeof(esp32Hint));
            if (sendOk == SOCKET_ERROR) {
                std::cerr << "Error sending to ESP32! Error: " << WSAGetLastError() << std::endl;
            } else {
                std::cout << "Sent message to ESP32: " << message << std::endl;
            }
        }
    }
    
    // Close socket
    closesocket(sock);
    
    // Cleanup Winsock
    WSACleanup();
    
    return 0;
}
