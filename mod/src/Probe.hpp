#pragma once

// Phase 0 measurement substrate.
//
// Everything here is designed to be callable from the physics stepping path:
// no allocation after reserve(), no formatting, no I/O. Rows are accumulated
// into a preallocated buffer and only turned into text at flush time.
//
// Determinism traces hash RAW BIT PATTERNS, never formatted floats. A decimal
// rendering can compare equal while the underlying doubles differ in the low
// bits, which is exactly the divergence we are hunting.

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace probe {

// ---------------------------------------------------------------------------
// Raw bit access
// ---------------------------------------------------------------------------

inline uint32_t bits(float v) {
	uint32_t u;
	std::memcpy(&u, &v, sizeof(u));
	return u;
}

inline uint64_t bits(double v) {
	uint64_t u;
	std::memcpy(&u, &v, sizeof(u));
	return u;
}

inline float asFloat(uint32_t u) {
	float v;
	std::memcpy(&v, &u, sizeof(v));
	return v;
}

inline double asDouble(uint64_t u) {
	double v;
	std::memcpy(&v, &u, sizeof(v));
	return v;
}

// ---------------------------------------------------------------------------
// Per-step state flags
// ---------------------------------------------------------------------------

// Bit layout of TraceRow::flags. The four on-ground variants are all recorded
// because 2.2081 PlayerObject has m_isOnGround, 2, 3 and 4, and the project
// plan's claim that m_isOnGround is the correct branching signal was validated
// cube-only against older bindings (Probe 7 settles which one to trust).
enum Flag : uint32_t {
	FlagDead          = 1u << 0,
	FlagOnGround      = 1u << 1,
	FlagOnGround2     = 1u << 2,
	FlagOnGround3     = 1u << 3,
	FlagOnGround4     = 1u << 4,
	FlagJumpBuffered  = 1u << 5,
	FlagUpsideDown    = 1u << 6,
	FlagShip          = 1u << 7,
	FlagBird          = 1u << 8,  // UFO
	FlagBall          = 1u << 9,
	FlagDart          = 1u << 10, // wave
	FlagRobot         = 1u << 11,
	FlagSpider        = 1u << 12,
	FlagSwing         = 1u << 13,
	FlagSideways      = 1u << 14,
	FlagPressed       = 1u << 15, // the input we asserted, or recorded, this step

	// Candidate sources for observing human input. Recorded in parallel so a
	// single run identifies which one actually reports button state, instead
	// of guessing at the input plumbing one hypothesis per run.
	FlagSrcHandleButton = 1u << 24, // GJBaseGameLayer::handleButton observer
	FlagSrcPushButton   = 1u << 25, // PlayerObject::pushButton/releaseButton observer
	FlagSrcHoldingMap   = 1u << 26, // PlayerObject::m_holdingButtons[Jump]
	FlagSrcAsyncKey     = 1u << 27, // GetAsyncKeyState (space/up/LMB)

	// m_holdingButtons[Jump], read DIRECTLY rather than through an observer, and
	// deliberately outside kInputSourceMask so it is compared, not masked.
	//
	// A solver-vs-replay divergence showed m_jumpBuffered differing while all six
	// physics fields were bit-identical. Input handling is then the only thing
	// that can differ, and whether the game already believes the button is held
	// decides whether a press registers as a new edge at all. That state lives in
	// a container, so it is outside the PlayerObject field table the restore
	// writes back - which makes it exactly the blind spot worth instrumenting.
	FlagHoldingJump     = 1u << 28,

	// Set on the first step recorded after a restore. Solver-only bookkeeping:
	// a clean replay never restores, so it is masked out of comparisons.
	//
	// "Nearest decision at or before the divergence" turned out to be weak
	// evidence - decisions are dense, so one is almost always nearby whether or
	// not a restore happened there. This answers the question directly.
	FlagPostRestore     = 1u << 29,

	// GJBaseGameLayer::m_queuedButtons was non-empty at the end of this step.
	//
	// handleButton does not act on the player directly - it appends a
	// PlayerButtonCommand to a layer-level queue that processQueuedButtons
	// drains during update. That queue is not part of any checkpoint, and
	// resetLevel clears it, so a restore can leave it holding a command from
	// before the rewind. Compared, not masked: if the solver carries a queue
	// entry the clean replay does not, that is the defect.
	FlagQueuedButtons   = 1u << 30,
};

// Bits that exist only in the solver's own trace and must never take part in a
// solver-vs-replay comparison.
constexpr uint32_t kSolverOnlyMask = FlagPostRestore;

// All four observation bits, for masking in comparisons.
constexpr uint32_t kInputSourceMask =
	FlagSrcHandleButton | FlagSrcPushButton | FlagSrcHoldingMap | FlagSrcAsyncKey;

// m_touchingRings is a CCArray, i.e. a proximity count (observed 0..3), not a
// boolean. The count is packed into the high bits rather than flattened to a
// bool so Probe 7 can see the actual distribution.
constexpr uint32_t kRingCountShift = 16;
constexpr uint32_t kRingCountMask  = 0xFFu << kRingCountShift;

inline uint32_t packRingCount(int count) {
	if (count < 0) count = 0;
	if (count > 255) count = 255;
	return (static_cast<uint32_t>(count) << kRingCountShift) & kRingCountMask;
}

inline int unpackRingCount(uint32_t flags) {
	return static_cast<int>((flags & kRingCountMask) >> kRingCountShift);
}

// ---------------------------------------------------------------------------
// Trace rows
// ---------------------------------------------------------------------------

// Field order is chosen so the struct has no interior padding: eight 4-byte
// members (32 bytes, leaving the offset 8-aligned) followed by two 8-byte
// members. Padding bytes would be uninitialised and would poison the hash.
struct TraceRow {
	int32_t  step;        // OUR per-attempt step counter, incremented once per update()
	int32_t  engineStep;  // GJBaseGameLayer::m_currentStep, recorded as an observation
	uint32_t x;           // raw bits, float
	uint32_t y;           // raw bits, float
	uint32_t rotation;    // raw bits, float
	uint32_t percent;     // raw bits, float
	uint32_t playerSpeed; // raw bits, float
	uint32_t flags;
	uint64_t yVelocity;   // raw bits, double
	uint64_t gravity;     // raw bits, double
};

static_assert(sizeof(TraceRow) == 48, "TraceRow must be padding-free for stable hashing");

// ---------------------------------------------------------------------------
// FNV-1a 64
// ---------------------------------------------------------------------------

constexpr uint64_t kFnvOffset = 1469598103934665603ull;
constexpr uint64_t kFnvPrime  = 1099511628211ull;

inline uint64_t fnv1a(const void* data, size_t len, uint64_t seed = kFnvOffset) {
	const auto* p = static_cast<const uint8_t*>(data);
	uint64_t h = seed;
	for (size_t i = 0; i < len; i++) {
		h ^= p[i];
		h *= kFnvPrime;
	}
	return h;
}

// ---------------------------------------------------------------------------
// Trace buffer
// ---------------------------------------------------------------------------

// Fixed-capacity by design. push() past capacity drops the row and bumps a
// counter rather than reallocating, because a realloc mid-search would be an
// unbounded stall in the stepping path. An overflow makes the run invalid and
// is reported loudly at flush time.
class Trace {
public:
	void reserve(size_t rows);
	void clear();

	void push(TraceRow const& row);

	size_t   size()      const { return m_rows.size(); }
	size_t   overflow()  const { return m_overflow; }
	bool     valid()     const { return m_overflow == 0; }
	uint64_t hash()      const;

	// Hash ignoring the first `skip` rows. Step 0 can carry state left over
	// from before the reset (m_isOnGround4 was observed stale there), which
	// would otherwise mask an otherwise-perfect physics match.
	uint64_t hashFrom(size_t skip) const;

	TraceRow const& operator[](size_t i) const { return m_rows[i]; }

	// Returns the index of the first row differing from `other`, or SIZE_MAX
	// if the common prefix matches. Length mismatch reports at min(size).
	size_t firstDivergence(Trace const& other) const;

	// As above, but ignores the flag bits in `ignoreFlags`. Needed to compare a
	// recorded human run against a replay of it: the input-source observers are
	// suppressed during injection, so those bits differ by construction even
	// when the physics are identical.
	size_t firstDivergenceMasked(Trace const& other, uint32_t ignoreFlags) const;

	// Percent (decoded) of the last row, or 0 if empty.
	float lastPercent() const;

	// Writes both the raw hex bits and the decoded value for every field, so
	// the CSV is readable by a human and exactly reconstructible by a diff.
	bool writeCsv(std::string const& path) const;

private:
	std::vector<TraceRow> m_rows;
	size_t m_overflow = 0;
};

// ---------------------------------------------------------------------------
// Timing (QueryPerformanceCounter)
// ---------------------------------------------------------------------------

uint64_t nowTicks();
double   ticksToMicros(uint64_t ticks);

struct Stats {
	double min    = 0.0;
	double median = 0.0;
	double p99    = 0.0;
	double max    = 0.0;
	double mean   = 0.0;
	size_t count  = 0;
};

// Sorts `samples` in place.
Stats summarize(std::vector<double>& samples);

// ---------------------------------------------------------------------------
// Output paths
// ---------------------------------------------------------------------------

// Absolute path to `name` inside the mod's save directory.
std::string outputPath(std::string const& name);

} // namespace probe
