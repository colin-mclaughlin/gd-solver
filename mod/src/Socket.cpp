// winsock2.h MUST be included before anything drags in windows.h, and this
// translation unit must never include Geode headers. See Socket.hpp.
// WIN32_LEAN_AND_MEAN is defined project-wide in CMakeLists.txt.
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include "Socket.hpp"
#include <cstdio>

static SOCKET g_listenSocket = INVALID_SOCKET;
static SOCKET g_clientSocket = INVALID_SOCKET;
static bool   g_wsaStarted   = false;

static constexpr unsigned short kProgressPort = 9999;

// How long a single send may wait for the window to open before we give up and
// drop the message. Bounded on purpose: the previous implementation spun on
// WSAEWOULDBLOCK with `continue`, which is an unbounded busy-wait.
static constexpr long kSendTimeoutUs = 2000; // 2 ms

void startProgressServer() {
	if (g_listenSocket != INVALID_SOCKET) return;

	if (!g_wsaStarted) {
		WSADATA wsaData;
		if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
			std::printf("[gd-solver] WSAStartup failed\n");
			return;
		}
		g_wsaStarted = true;
	}

	g_listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (g_listenSocket == INVALID_SOCKET) return;

	sockaddr_in service{};
	service.sin_family      = AF_INET;
	service.sin_addr.s_addr = inet_addr("127.0.0.1");
	service.sin_port        = htons(kProgressPort);

	if (bind(g_listenSocket, (SOCKADDR*)&service, sizeof(service)) == SOCKET_ERROR ||
	    listen(g_listenSocket, 1) == SOCKET_ERROR) {
		closesocket(g_listenSocket);
		g_listenSocket = INVALID_SOCKET;
		std::printf("[gd-solver] progress server failed to bind port %u\n", kProgressPort);
		return;
	}

	u_long mode = 1;
	ioctlsocket(g_listenSocket, FIONBIO, &mode);
	std::printf("[gd-solver] progress server listening on 127.0.0.1:%u\n", kProgressPort);
}

void stopProgressServer() {
	if (g_clientSocket != INVALID_SOCKET) {
		closesocket(g_clientSocket);
		g_clientSocket = INVALID_SOCKET;
	}
	if (g_listenSocket != INVALID_SOCKET) {
		closesocket(g_listenSocket);
		g_listenSocket = INVALID_SOCKET;
	}
	if (g_wsaStarted) {
		WSACleanup();
		g_wsaStarted = false;
	}
}

static void dropClient(const char* why) {
	std::printf("[gd-solver] progress client disconnected (%s)\n", why);
	closesocket(g_clientSocket);
	g_clientSocket = INVALID_SOCKET;
}

// Sends the whole buffer or gives up. Partial writes on a non-blocking socket
// are silently truncating otherwise, which is how large frames were being
// corrupted in the previous incarnation of this file.
static bool sendAll(SOCKET s, const char* data, int len) {
	int sent = 0;
	while (sent < len) {
		int n = send(s, data + sent, len - sent, 0);
		if (n == SOCKET_ERROR) {
			if (WSAGetLastError() != WSAEWOULDBLOCK) return false;

			// Wait briefly for writability instead of spinning.
			fd_set wr;
			FD_ZERO(&wr);
			FD_SET(s, &wr);
			timeval tv{};
			tv.tv_sec  = 0;
			tv.tv_usec = kSendTimeoutUs;

			int ready = select(0, nullptr, &wr, nullptr, &tv);
			if (ready <= 0) return false; // timed out or errored: drop the message
			continue;
		}
		sent += n;
	}
	return true;
}

bool sendProgress(std::string const& message) {
	if (g_listenSocket == INVALID_SOCKET) return false;

	if (g_clientSocket == INVALID_SOCKET) {
		g_clientSocket = accept(g_listenSocket, nullptr, nullptr);
		if (g_clientSocket == INVALID_SOCKET) return false;

		u_long mode = 1;
		ioctlsocket(g_clientSocket, FIONBIO, &mode);
		std::printf("[gd-solver] progress client connected\n");
	}

	std::string msg = message;
	msg.push_back('\n');

	if (!sendAll(g_clientSocket, msg.c_str(), (int)msg.size())) {
		// A timeout is not necessarily a dead peer, but for a progress channel
		// a slow reader and a dead one warrant the same response.
		dropClient("send failed or timed out");
		return false;
	}
	return true;
}
