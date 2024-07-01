#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <thread>
#include <chrono>

#pragma comment(lib, "Ws2_32.lib")

#define SERVER_IP "192.168.0.18"
#define SERVER_PORT 8888
#define CLIENT_PORT 8889
#define BUFFER_SIZE 1024

void initWinsock() {
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) {
        std::cerr << "WSAStartup failed: " << result << std::endl;
        exit(1);
    }
}

void cleanupWinsock() {
    WSACleanup();
}

int main() {
    initWinsock();

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Socket creation failed: " << WSAGetLastError() << std::endl;
        cleanupWinsock();
        return 1;
    }

    sockaddr_in clientAddr;
    clientAddr.sin_family = AF_INET;
    clientAddr.sin_port = htons(CLIENT_PORT);
    clientAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (SOCKADDR*)&clientAddr, sizeof(clientAddr)) == SOCKET_ERROR) {
        std::cerr << "Bind failed: " << WSAGetLastError() << std::endl;
        closesocket(sock);
        cleanupWinsock();
        return 1;
    }

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr);

    char buffer[BUFFER_SIZE];

    // Send initial message to Arduino
    std::string message = "Hello from PC";
    if (sendto(sock, message.c_str(), message.size(), 0, (SOCKADDR*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Send failed: " << WSAGetLastError() << std::endl;
    }

    while (true) {
        sockaddr_in senderAddr;
        int senderAddrSize = sizeof(senderAddr);

        int recvLen = recvfrom(sock, buffer, BUFFER_SIZE, 0, (SOCKADDR*)&senderAddr, &senderAddrSize);
        if (recvLen == SOCKET_ERROR) {
            std::cerr << "Receive failed: " << WSAGetLastError() << std::endl;
            continue;
        }

        std::cout << "Received: ";
        std::cout.write(buffer, recvLen);
        std::cout << std::endl;

        // Send reply back to Arduino
        std::string reply = "Message received";
        if (sendto(sock, reply.c_str(), reply.size(), 0, (SOCKADDR*)&senderAddr, sizeof(senderAddr)) == SOCKET_ERROR) {
            std::cerr << "Send reply failed: " << WSAGetLastError() << std::endl;
        }

        // Send message to Arduino every second
        std::this_thread::sleep_for(std::chrono::seconds(1));
        message = "Hello from PC";
        if (sendto(sock, message.c_str(), message.size(), 0, (SOCKADDR*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
            std::cerr << "Send failed: " << WSAGetLastError() << std::endl;
        }
    }

    closesocket(sock);
    cleanupWinsock();
    return 0;
}
