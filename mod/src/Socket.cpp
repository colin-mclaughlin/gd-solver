#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include "Socket.hpp"
#include <cstdio>

static SOCKET g_listenSocket = INVALID_SOCKET;
static SOCKET g_clientSocket = INVALID_SOCKET;

void startServer() {
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);

	g_listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	sockaddr_in service;
	service.sin_family = AF_INET;
	service.sin_addr.s_addr = inet_addr("127.0.0.1");
	service.sin_port = htons(9999);

	bind(g_listenSocket, (SOCKADDR*)&service, sizeof(service));
	listen(g_listenSocket, 1);

	u_long mode = 1;
	ioctlsocket(g_listenSocket, FIONBIO, &mode);

	printf("Socket server listening on port 9999\n");
}

void sendToClient(const std::string& message) {
	if (g_clientSocket == INVALID_SOCKET) {
		g_clientSocket = accept(g_listenSocket, nullptr, nullptr);
		if (g_clientSocket != INVALID_SOCKET) {
			u_long mode = 1;
			ioctlsocket(g_clientSocket, FIONBIO, &mode);
			printf("Client connected!\n");
		}
		return;
	}

	std::string msg = message + "\n";
	int result = send(g_clientSocket, msg.c_str(), (int)msg.size(), 0);

	if (result == SOCKET_ERROR) {
		int err = WSAGetLastError();
		if (err != WSAEWOULDBLOCK) {
			printf("Client disconnected (send failed), resetting.\n");
			closesocket(g_clientSocket);
			g_clientSocket = INVALID_SOCKET;
		}
	}
}

int receiveAction() {
	if (g_clientSocket == INVALID_SOCKET) {
		return -1;
	}

	char buf[16] = {};
	int result = recv(g_clientSocket, buf, sizeof(buf) - 1, 0);

	if (result > 0) {
		if (buf[0] == '1') return 1;
		if (buf[0] == '0') return 0;
	} else if (result == 0) {
		printf("Client disconnected (recv 0), resetting.\n");
		closesocket(g_clientSocket);
		g_clientSocket = INVALID_SOCKET;
	} else {
		int err = WSAGetLastError();
		if (err != WSAEWOULDBLOCK) {
			printf("Client disconnected (recv error), resetting.\n");
			closesocket(g_clientSocket);
			g_clientSocket = INVALID_SOCKET;
		}
	}

	return -1;
}

static SOCKET g_frameListenSocket = INVALID_SOCKET;
static SOCKET g_frameClientSocket = INVALID_SOCKET;

void startFrameServer() {
	g_frameListenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	sockaddr_in service;
	service.sin_family = AF_INET;
	service.sin_addr.s_addr = inet_addr("127.0.0.1");
	service.sin_port = htons(9998); // different port from state socket (9999)

	bind(g_frameListenSocket, (SOCKADDR*)&service, sizeof(service));
	listen(g_frameListenSocket, 1);

	u_long mode = 1;
	ioctlsocket(g_frameListenSocket, FIONBIO, &mode);

	printf("Frame server listening on port 9998\n");
}

static bool sendAll(SOCKET s, const char* data, int len) {
    int sent = 0;
    while (sent < len) {
        int n = send(s, data + sent, len - sent, 0);
        if (n == SOCKET_ERROR) {
            if (WSAGetLastError() == WSAEWOULDBLOCK) continue;
            return false;
        }
        sent += n;
    }
    return true;
}

void sendLabeledFrame(const unsigned char* grayData, int width, int height, int label, int frameId) {
    if (g_frameClientSocket == INVALID_SOCKET) {
        g_frameClientSocket = accept(g_frameListenSocket, nullptr, nullptr);
        if (g_frameClientSocket != INVALID_SOCKET) {
            u_long mode = 1;
            ioctlsocket(g_frameClientSocket, FIONBIO, &mode);
            printf("Frame client connected!\n");
        }
        return;
    }

    int size = width * height; // grayscale, 1 byte per pixel
    int header[4] = {width, height, label, frameId};
    if (!sendAll(g_frameClientSocket, (const char*)header, sizeof(header)) ||
        !sendAll(g_frameClientSocket, (const char*)grayData, size)) {
        closesocket(g_frameClientSocket);
        g_frameClientSocket = INVALID_SOCKET;
    }
}