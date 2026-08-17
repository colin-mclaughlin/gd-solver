#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

/**
 * Include the Geode headers.
 */
#include <Geode/Geode.hpp>
#include <gl/GL.h>
#include <vector>
#include <fstream>

/**
 * Brings cocos2d and all Geode namespaces to the current scope.
 */
using namespace geode::prelude;

#include "Socket.hpp"

float g_startX = -1.f;
bool g_hasJumped = false;
bool g_isHolding = false;
int g_frameId = 0;

bool g_firstAttemptDone = false;
int g_firstAttemptWarmupFrames = 0;
const int FIRST_ATTEMPT_WARMUP = 100; // you observed ~36 frames of pan-in, padded for margin

bool g_levelFinished = false;

int g_currentAction = 0;

bool g_wasDead = false;

const int TARGET_W = 128;
const int TARGET_H = 128;

void downsampleGrayscale(const unsigned char* rgb, int srcW, int srcH, unsigned char* outGray, int dstW, int dstH) {
    for (int y = 0; y < dstH; y++) {
        int srcYStart = y * srcH / dstH;
        int srcYEnd = (y + 1) * srcH / dstH;
        if (srcYEnd <= srcYStart) srcYEnd = srcYStart + 1;
        for (int x = 0; x < dstW; x++) {
            int srcXStart = x * srcW / dstW;
            int srcXEnd = (x + 1) * srcW / dstW;
            if (srcXEnd <= srcXStart) srcXEnd = srcXStart + 1;

            long sum = 0;
            int count = 0;
            for (int sy = srcYStart; sy < srcYEnd && sy < srcH; sy++) {
                for (int sx = srcXStart; sx < srcXEnd && sx < srcW; sx++) {
                    const unsigned char* px = &rgb[(sy * srcW + sx) * 3];
                    int gray = (px[0] * 299 + px[1] * 587 + px[2] * 114) / 1000;
                    sum += gray;
                    count++;
                }
            }
            outGray[y * dstW + x] = (unsigned char)(count > 0 ? sum / count : 0);
        }
    }
}

#include <Geode/modify/GJBaseGameLayer.hpp>
class $modify(MyPlayLayer, GJBaseGameLayer) {
	void update(float dt) {
		GJBaseGameLayer::update(dt);

		bool playerExists = this->m_player1 != nullptr;
		bool isDeadNow = playerExists && this->m_player1->m_isDead;
		bool justDied = isDeadNow && !g_wasDead;
		g_wasDead = isDeadNow;

		bool inValidGameplay = PlayLayer::get() && PlayLayer::get()->isGameplayActive() && !g_levelFinished && playerExists && !isDeadNow;

		if (inValidGameplay) {

			if (!g_firstAttemptDone) {
				g_firstAttemptWarmupFrames++;
				if (g_firstAttemptWarmupFrames >= FIRST_ATTEMPT_WARMUP) {
					g_firstAttemptDone = true;
				}
			}

			if (g_firstAttemptDone) {
				static int frameCount = 0;
				frameCount++;

				if (frameCount % 4 == 0) {
					GLint vp[4] = {0, 0, 0, 0};
					glGetIntegerv(GL_VIEWPORT, vp);
					int w = vp[2];
					int h = vp[3];

					glPixelStorei(GL_PACK_ALIGNMENT, 1);
					std::vector<unsigned char> pixels(w * h * 3);
					glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

					std::vector<unsigned char> flipped(w * h * 3);
					for (int row = 0; row < h; row++) {
						memcpy(&flipped[row * w * 3], &pixels[(h - 1 - row) * w * 3], w * 3);
					}

					std::vector<unsigned char> gray(TARGET_W * TARGET_H);
					downsampleGrayscale(flipped.data(), w, h, gray.data(), TARGET_W, TARGET_H);

					bool jumpHeld = (GetAsyncKeyState(VK_SPACE) & 0x8000) ||
									(GetAsyncKeyState(VK_UP) & 0x8000) ||
									(GetAsyncKeyState(VK_LBUTTON) & 0x8000);

					sendLabeledFrame(gray.data(), TARGET_W, TARGET_H, jumpHeld ? 1 : 0, g_frameId++);

					float yPos = this->m_player1->getPositionY();
					bool isDead = this->m_player1->m_isDead;
					bool isShip = this->m_player1->m_isShip;
					bool isOnGround = this->m_player1->m_isOnGround;
					bool touchingRing = this->m_player1->m_touchingRings && this->m_player1->m_touchingRings->count() > 0;

					log::debug("OnGround: {} | TouchingRings: {}", isOnGround, this->m_player1->m_touchingRings ? this->m_player1->m_touchingRings->count() : -1);

					auto pl = PlayLayer::get();
					float percent = pl ? pl->getCurrentPercent() : 0.f;

					char buffer[300];
					snprintf(buffer, sizeof(buffer),
						"{\"y\": %.2f, \"dead\": %s, \"percent\": %.2f, \"on_ground\": %s, \"is_ship\": %s, \"touching_ring\": %s}",
						yPos, isDead ? "true" : "false", percent, isOnGround ? "true" : "false", isShip ? "true" : "false", touchingRing ? "true" : "false");
					sendToClient(buffer);

					int action = receiveAction();
					if (action == 0 || action == 1) {
						g_currentAction = action;
					}
				}
			}
		}
		else if (justDied) {
			float yPos = this->m_player1->getPositionY();
			auto pl = PlayLayer::get();
			float percent = pl ? pl->getCurrentPercent() : 0.f;
			bool touchingRing = this->m_player1->m_touchingRings && this->m_player1->m_touchingRings->count() > 0;

			char buffer[300];
			snprintf(buffer, sizeof(buffer),
				"{\"y\": %.2f, \"dead\": true, \"percent\": %.2f, \"on_ground\": %s, \"is_ship\": %s, \"touching_ring\": %s}",
				yPos, percent, this->m_player1->m_isOnGround ? "true" : "false", this->m_player1->m_isShip ? "true" : "false", touchingRing ? "true" : "false");
			sendToClient(buffer);
			receiveAction(); // drain the queued response byte, unused here
		}

		// Reassert the current decision every frame, independent of capture cadence
		if (this->m_player1) {
			if (g_currentAction == 1) {
				this->handleButton(true, (int)PlayerButton::Jump, true);
				g_isHolding = true;
			} else {
				if (g_isHolding) {
					this->handleButton(false, (int)PlayerButton::Jump, true);
				}
				g_isHolding = false;
			}
		}
	}
};

#include <Geode/modify/PlayLayer.hpp>
class $modify(MyResetLayer, PlayLayer) {
	void resetLevel() {
		PlayLayer::resetLevel();
		g_startX = -1.f;
		g_hasJumped = false;
		g_isHolding = false;
		g_levelFinished = false;
		g_currentAction = 0;
		log::debug("Level reset!");
	}

	bool init(GJGameLevel* level, bool useReplay, bool dontCreateObjects) {
		if (!PlayLayer::init(level, useReplay, dontCreateObjects)) return false;
		g_startX = -1.f;
		g_hasJumped = false;
		g_isHolding = false;
		g_firstAttemptDone = false;
		g_firstAttemptWarmupFrames = 0;
		g_levelFinished = false;
		g_currentAction = 0;
		log::debug("Level started!");
		return true;
	}

	void updateProgressbar() {
		PlayLayer::updateProgressbar();
		float percent = this->getCurrentPercent();
		log::debug("Percent: {:.2f}%", percent);
	}

	virtual void playEndAnimationToPos(cocos2d::CCPoint position) {
		PlayLayer::playEndAnimationToPos(position);
		g_levelFinished = true;
		log::debug("End animation triggered!");
	}
};

#include <Geode/modify/MenuLayer.hpp>
class $modify(MyMenuLayer, MenuLayer) {
	bool init() {
		if (!MenuLayer::init()) {
			return false;
		}

		log::debug("Capturing full frame...");
		GLint viewport[4] = {0, 0, 0, 0};
		glGetIntegerv(GL_VIEWPORT, viewport);
		int width = viewport[2];
		int height = viewport[3];

		glPixelStorei(GL_PACK_ALIGNMENT, 1); // tell OpenGL not to pad rows

		std::vector<unsigned char> pixels(width * height * 3);
		glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

		std::vector<unsigned char> flipped(width * height * 3);
		for (int row = 0; row < height; row++) {
			memcpy(
				&flipped[row * width * 3],
				&pixels[(height - 1 - row) * width * 3],
				width * 3
			);
		}

		std::ofstream file("C:/projects/RLDash/test_frame.ppm", std::ios::binary);
		file << "P6\n" << width << " " << height << "\n255\n";
		file.write(reinterpret_cast<char*>(flipped.data()), flipped.size());
		file.close();
		log::debug("Saved frame to test_frame.ppm");

		static bool serverStarted = false;
		if (!serverStarted) {
			startServer();
			startFrameServer();
			log::debug("Frame server started");
			serverStarted = true;
		}

		log::debug("Hello from my MenuLayer::init hook! This layer has {} children.", this->getChildrenCount());

		return true;
	}
};