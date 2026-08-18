#pragma once
#include <string>

// Optional localhost progress/log channel.
//
// IMPORTANT (project plan 6.1, 9): this is NEVER to be called from the physics
// stepping path. It exists for coarse progress reporting only - sampled on a
// wall-clock interval or on new-best-percent, never per tick. The solver
// decides its own inputs, so there is deliberately no inbound action channel.
//
// The implementation lives in Socket.cpp specifically so that <winsock2.h>
// never enters a translation unit that includes Geode headers. That isolation
// is load-bearing; winsock2.h and windows.h conflict in ways that are annoying
// to rediscover. Do not include winsock headers anywhere else.

void startProgressServer();
void stopProgressServer();

// Non-blocking. Returns false if the message was dropped (no client attached,
// or the send window stayed full past a short timeout). Dropping progress
// output is always preferable to stalling the game.
bool sendProgress(std::string const& message);
