#pragma once
#include <string>

void startServer();
void sendToClient(const std::string& message);
int receiveAction(); // returns 1 = jump, 0 = no action, -1 = nothing received
void startFrameServer();
void sendLabeledFrame(const unsigned char* grayData, int width, int height, int label, int frameId);