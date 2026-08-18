// GD Solver - Phase 0 instrumentation.
//
// This mod measures; it does not solve. Every probe emits a number that gates a
// later architectural decision (see docs/PROJECT_PLAN.md 7, Phase 0).
//
// Hard rules for anything in this file (project plan 9):
//   - No glReadPixels, ever. It is a GPU->CPU sync point and would cap the
//     whole project at real-time framerates.
//   - No logging inside the stepping path. String formatting at 10,000+
//     steps/sec dominates runtime. Report by sampling or at attempt end.
//   - Anything holding cross-frame state resets in BOTH PlayLayer::init and
//     PlayLayer::resetLevel. A global that survives a reset once held jump
//     forever and produced a scrambled trajectory that looked like a
//     flag-detection bug.

// WIN32_LEAN_AND_MEAN is defined project-wide in CMakeLists.txt.
#include <Windows.h>
#include <psapi.h>
#pragma comment(lib, "psapi.lib")

#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <Geode/modify/PlayerObject.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "Probe.hpp"
#include "Socket.hpp"

using namespace geode::prelude;

namespace {

// The real gameplay tick. This never changes: we execute the same 240 Hz ticks
// faster than wall-clock, we do not change the tick rate. A macro found at any
// other dt would not be valid in normal play.
constexpr double kPhysicsDt = 1.0 / 240.0;

constexpr size_t kMaxTraceRows    = 32768; // ~136 s at 240 tps
constexpr int    kDeterminismRuns = 10;
constexpr int    kJumpButton      = 1;     // PlayerButton::Jump

// Config (c) of Probe 1: an arbitrary fixed value, only its constancy matters.
constexpr uint64_t kForcedSeed = 0x5EEDC0FFEEULL;

// ---------------------------------------------------------------------------
// Runtime configuration
// ---------------------------------------------------------------------------

struct Config {
	// Project plan 5.2 parts 1+2: hook getModifiedDelta (calling the original
	// first, because it mutates m_extraDelta), and feed our own delta into
	// update's original call. Part 3 (the fractionOf240 midhook) is peony's
	// LOW-tps fix and is deliberately not implemented - Probe 3 decides
	// whether the high-tps direction needs it at all.
	bool physicsFix = false;

	// Force m_randomSeed / m_replayRandSeed to a constant on reset. Isolates
	// whether RNG state actually affects physics or only cosmetics.
	bool forceSeeds = false;

	// How many 1/240 physics steps to run per rendered frame when physicsFix
	// is on.
	//
	// Measured: GJBaseGameLayer::update is called once per rendered frame with
	// dt = 1/refresh (1/120 on this machine), and vanilla GD subdivides that
	// into refresh/240 sub-steps internally. Returning 1/240 from
	// getModifiedDelta collapses that to exactly one sub-step per call, which
	// is the single-tick primitive we want - but it also runs the game at
	// (240/refresh)x slow motion. Running that many steps per frame restores
	// real-time pace while keeping full 240Hz input granularity.
	//
	// Auto-detected per attempt from the observed native delta, so this is
	// correct on any refresh rate. Raising it beyond the detected value is the
	// Probe 6 throughput lever.
	int stepsPerFrame = 2;

	// How many steps AHEAD to assert injected input.
	//
	// Measured: handleButton does not take effect on the step it is called. A
	// human press recorded as entering step N produced its jump during step N,
	// but injecting at step N produced the jump at N+1 - one step of latency,
	// consistent with the press being queued into m_queuedButtons and consumed
	// by the following processQueuedButtons.
	//
	// So the press must be asserted one step early to land on the intended
	// step. Sweepable with F9/F10 to confirm 1 is exactly right rather than
	// approximately right.
	int injectionLeadSteps = 1;
};

Config g_config;

// ---------------------------------------------------------------------------
// Probe state
// ---------------------------------------------------------------------------

enum class Mode {
	Idle,
	RecordInput,  // capture what the human presses, per step
	Determinism,  // Probe 1: replay fixed input N times, compare trace hashes
};

struct ProbeState {
	Mode mode = Mode::Idle;

	// --- per attempt (reset in BOTH init and resetLevel) ---
	bool wasDead           = false;
	bool finished          = false;
	int  endAnimStep       = -1;
	int  levelCompleteStep = -1;
	bool isHolding         = false;
	bool injecting         = false;
	// Our own per-attempt step counter, incremented once per update() call.
	//
	// Do NOT index off GJBaseGameLayer::m_currentStep. It lives in the replay
	// cluster (m_recordInputs, m_recordString, m_queuedRecordedButtons,
	// m_queuedReplayButtons) and is evidently only maintained while the game's
	// own replay system is active: it read 0 for an entire Stereo Madness
	// completion. Indexing off it collapsed every step onto index 0.
	// m_currentStep is still recorded per row as an observation, so we can see
	// empirically if and when it ever advances.
	int  stepCounter       = 0;
	bool attemptStarted    = false;
	probe::Trace trace;

	// --- per level (reset in init only) ---
	std::vector<uint8_t> scripted;  // input to replay, indexed by our step counter
	std::vector<uint8_t> recorded;  // input being captured this attempt

	// Four independent readings of "is the jump button held". handleButton
	// alone reported false for an entire run in which the level was beaten, so
	// all four are sampled in parallel until one is proven correct.
	bool srcHandleButton = false;  // event: GJBaseGameLayer::handleButton
	bool srcPushButton   = false;  // event: PlayerObject::pushButton/releaseButton
	bool srcHoldingMap   = false;  // state: PlayerObject::m_holdingButtons
	bool srcAsyncKey     = false;  // state: GetAsyncKeyState

	// Per-attempt tally of how many steps each source read as held.
	int heldSteps[4] = {0, 0, 0, 0};

	// What the game actually passes as handleButton's third argument for
	// player-1 jump input. The bindings call it isPlayer1, but filtering on it
	// discarded every human press, so the name is suspect. Injection passes
	// true; if the game passes false, injection may be targeting the wrong
	// player - which would only show up in dual sections.
	int humanIsPlayer1True  = 0;
	int humanIsPlayer1False = 0;

	// The recorded human run, kept so a replay of it can be checked for
	// fidelity, not just for run-to-run consistency. Cleared per level, not
	// per attempt.
	probe::Trace humanReference;
	float humanReferencePercent = 0.f;

	// Probe 2: how many of the first few getModifiedDelta calls of this attempt
	// have been logged. The native per-call delta was ASSUMED to be 1/240; the
	// replay ran at half the human run's progress per step, so it is measured
	// now instead.
	int deltaLogCount = 0;

	// Native frame delta, measured from the OUTER update hook's dt.
	//
	// It must be measured there, not inside getModifiedDelta: with the fix on
	// we pass 1/240 into the original update(), which calls getModifiedDelta
	// with that value, so sampling there measures our own injection and made
	// stepsPerFrame oscillate 2 -> 1 -> 2 between attempts. The outer hook's dt
	// is always the genuine scheduler delta, because our call goes to the
	// original and does not re-enter the hook.
	//
	// A median over many frames, because resets produce real hitches (deltas of
	// 1/35, 1/47, 1/30 were all observed at attempt start). Locked once per
	// level so it cannot drift mid-sweep.
	std::vector<float> dtSamples;
	bool   nativeDtLocked = false;
	double nativeDt       = 0.0;

	// Config the loaded recording was captured under. A recording made at 240Hz
	// granularity replayed at 120Hz consumes the input array at half rate, so
	// fidelity cannot pass across a mismatch - that is expected, not a bug.
	bool recordedPhysicsFix = false;
	bool haveRecordingMeta  = false;

	// --- determinism sweep (spans attempts; cleared when a sweep starts) ---
	int  runIndex    = 0;
	int  runsLeft    = 0;
	bool resetPending = false;
	std::vector<uint64_t> hashes;
	std::vector<uint64_t> hashesSkip1;
	probe::Trace reference;

	void resetPerAttempt() {
		wasDead           = false;
		finished          = false;
		endAnimStep       = -1;
		levelCompleteStep = -1;
		isHolding         = false;
		injecting         = false;
		srcHandleButton   = false;
		srcPushButton     = false;
		srcHoldingMap     = false;
		srcAsyncKey       = false;
		heldSteps[0] = heldSteps[1] = heldSteps[2] = heldSteps[3] = 0;
		humanIsPlayer1True  = 0;
		humanIsPlayer1False = 0;
		deltaLogCount       = 0;
		stepCounter       = 0;
		attemptStarted    = false;
		trace.reserve(kMaxTraceRows);
		recorded.clear();
		recorded.reserve(kMaxTraceRows);
	}

	void resetPerLevel() {
		resetPerAttempt();
		mode         = Mode::Idle;
		runIndex     = 0;
		runsLeft     = 0;
		resetPending = false;
		hashes.clear();
		hashesSkip1.clear();
		reference.clear();
		scripted.clear();
		humanReference.clear();
		humanReferencePercent = 0.f;
		dtSamples.clear();
		nativeDtLocked     = false;
		nativeDt           = 0.0;
		recordedPhysicsFix = false;
		haveRecordingMeta  = false;
	}

	static ProbeState& get() {
		static ProbeState s;
		return s;
	}
};

// ---------------------------------------------------------------------------
// Recorded input file format
// ---------------------------------------------------------------------------
//
// Deliberately plain text: one '0'/'1' per physics step, 80 per line, so a
// recording can be eyeballed and hand-edited.

std::string inputFilePath() {
	return probe::outputPath("input.txt");
}

bool saveInput(std::vector<uint8_t> const& input) {
	std::FILE* f = std::fopen(inputFilePath().c_str(), "wb");
	if (!f) return false;

	// One char per recorded step. With the physics fix on that is one true
	// 1/240 physics step; with it off it is one rendered frame, which the
	// engine subdivides internally - so a recording is only comparable to a
	// replay made under the same setting.
	std::fprintf(f, "# gd-solver input v1, %zu steps, physics_fix=%d\n",
	             input.size(), g_config.physicsFix ? 1 : 0);
	for (size_t i = 0; i < input.size(); i++) {
		std::fputc(input[i] ? '1' : '0', f);
		if ((i + 1) % 80 == 0) std::fputc('\n', f);
	}
	std::fputc('\n', f);
	std::fclose(f);
	return true;
}

bool loadInput(std::vector<uint8_t>& out, bool* physicsFixOut = nullptr) {
	std::FILE* f = std::fopen(inputFilePath().c_str(), "rb");
	if (!f) return false;

	out.clear();
	out.reserve(kMaxTraceRows);

	std::string comment;
	bool inComment = false;
	int c;
	while ((c = std::fgetc(f)) != EOF) {
		if (c == '#') { inComment = true; }
		else if (c == '\n') { inComment = false; }
		else if (inComment) { comment.push_back(static_cast<char>(c)); }
		else if (c == '0' || c == '1') { out.push_back(c == '1' ? 1 : 0); }
	}
	std::fclose(f);

	if (physicsFixOut) {
		auto at = comment.find("physics_fix=");
		*physicsFixOut = (at != std::string::npos && comment[at + 12] == '1');
	}
	return true;
}

// A deterministic fallback pattern, used when nothing has been recorded yet.
// ~10% press rate with presses held 8-20 steps, roughly matching the jump rate
// observed in the behaviour-cloning dataset. A fixed LCG, so it is identical
// on every run - which is the only property Probe 1 actually requires.
std::vector<uint8_t> makeFallbackInput(size_t steps) {
	std::vector<uint8_t> v(steps, 0);
	uint32_t rng = 0x1234ABCDu;
	auto next = [&rng]() {
		rng = rng * 1664525u + 1013904223u;
		return rng >> 16;
	};

	size_t i = 0;
	while (i < steps) {
		if ((next() % 100) < 4) {
			size_t hold = 8 + (next() % 13);
			for (size_t j = 0; j < hold && i < steps; j++, i++) v[i] = 1;
		} else {
			i++;
		}
	}
	return v;
}

// ---------------------------------------------------------------------------
// Determinism sweep bookkeeping
// ---------------------------------------------------------------------------

// Includes the injection lead so sweeps that differ only by lead do not
// overwrite each other's CSVs - which is how the lead=1 evidence was lost.
std::string configName() {
	std::string base = !g_config.physicsFix ? "a_no_fix"
	                 : !g_config.forceSeeds  ? "b_physics_fix"
	                                         : "c_fix_plus_seeds";
	return base + "_lead" + std::to_string(g_config.injectionLeadSteps);
}

void startDeterminismSweep(bool physicsFix, bool forceSeeds) {
	auto& st = ProbeState::get();
	auto* pl = PlayLayer::get();
	if (!pl) {
		log::warn("Probe 1: not in a level");
		return;
	}

	g_config.physicsFix = physicsFix;
	g_config.forceSeeds = forceSeeds;

	if (st.scripted.empty()) {
		bool fileFix = false;
		if (loadInput(st.scripted, &fileFix) && !st.scripted.empty()) {
			st.recordedPhysicsFix = fileFix;
			st.haveRecordingMeta  = true;
			log::info("Probe 1: loaded {} steps of recorded input (physics_fix={})",
			          st.scripted.size(), fileFix ? 1 : 0);
		} else {
			st.scripted = makeFallbackInput(kMaxTraceRows);
			log::warn("Probe 1: no recorded input found, using synthetic fallback pattern. "
			          "Record a real attempt with F5 for a much stronger test.");
		}
	}

	st.mode         = Mode::Determinism;
	st.runIndex     = 0;
	st.runsLeft     = kDeterminismRuns;
	st.resetPending = true;
	st.hashes.clear();
	st.hashesSkip1.clear();
	st.reference.clear();

	log::info("Probe 1: starting {} runs, config {}, injection lead {} step(s)",
	          kDeterminismRuns, configName(), g_config.injectionLeadSteps);
}

void finishDeterminismSweep() {
	auto& st = ProbeState::get();

	auto countDistinct = [](std::vector<uint64_t> v) {
		std::sort(v.begin(), v.end());
		v.erase(std::unique(v.begin(), v.end()), v.end());
		return v.size();
	};

	const size_t distinct     = countDistinct(st.hashes);
	const size_t distinctSk1  = countDistinct(st.hashesSkip1);

	log::info("================ Probe 1: determinism, config {} ================", configName());
	for (size_t i = 0; i < st.hashes.size(); i++) {
		log::info("  run {:2}  full {:016X}   from-step-1 {:016X}{}", i,
		          st.hashes[i], st.hashesSkip1[i],
		          (i > 0 && st.hashes[i] != st.hashes[0]) ? "   <-- DIVERGED" : "");
	}
	log::info("  full trace:      {} distinct hash(es) -> {}",
	          distinct, distinct == 1 ? "DETERMINISTIC" : "NONDETERMINISTIC");
	log::info("  ignoring step 0: {} distinct hash(es) -> {}",
	          distinctSk1, distinctSk1 == 1 ? "DETERMINISTIC" : "NONDETERMINISTIC");
	if (distinct != 1 && distinctSk1 == 1) {
		log::info("  => physics are deterministic; the ONLY variation is stale state "
		          "at step 0 carried over from before the reset.");
	}
	log::info("================================================================");

	std::string path = probe::outputPath(std::string("probe1_") + configName() + "_summary.txt");
	if (std::FILE* f = std::fopen(path.c_str(), "wb")) {
		std::fprintf(f,
			"probe 1 determinism\n"
			"config: %s\n"
			"runs: %zu\n"
			"full trace:      %zu distinct -> %s\n"
			"ignoring step 0: %zu distinct -> %s\n\n",
			configName().c_str(), st.hashes.size(),
			distinct,    distinct    == 1 ? "DETERMINISTIC" : "NONDETERMINISTIC",
			distinctSk1, distinctSk1 == 1 ? "DETERMINISTIC" : "NONDETERMINISTIC");
		for (size_t i = 0; i < st.hashes.size(); i++) {
			std::fprintf(f, "run %zu  full %016llX  from-step-1 %016llX\n", i,
			             static_cast<unsigned long long>(st.hashes[i]),
			             static_cast<unsigned long long>(st.hashesSkip1[i]));
		}
		std::fclose(f);
		log::info("Probe 1: summary written to {}", path);
	}

	st.mode     = Mode::Idle;
	st.runsLeft = 0;
}

// Called once an attempt has ended (death, completion, or trace full).
void onAttemptEnded() {
	auto& st = ProbeState::get();

	if (st.mode == Mode::RecordInput) {
		static const char* kSourceNames[4] = {
			"handleButton", "pushButton", "m_holdingButtons", "GetAsyncKeyState"
		};

		log::info("Probe 0: input source tallies over {} steps:", st.recorded.size());
		for (int i = 0; i < 4; i++) {
			log::info("    {:<18} {} steps held", kSourceNames[i], st.heldSteps[i]);
		}

		// Prefer the source closest to the game's own state, but only among
		// those that actually reported anything.
		const int preference[4] = {1, 2, 0, 3}; // pushButton, holdingMap, handleButton, asyncKey
		int chosen = -1;
		for (int i = 0; i < 4; i++) {
			if (st.heldSteps[preference[i]] > 0) { chosen = preference[i]; break; }
		}

		if (chosen < 0) {
			log::error("Probe 0: NO input source reported a single held step. "
			           "Recording not saved - the existing input file is left intact.");
			st.mode = Mode::Idle;
			return;
		}

		log::info("Probe 0: using '{}' as the input source ({} steps held)",
		          kSourceNames[chosen], st.heldSteps[chosen]);

		std::vector<uint8_t> flat(st.recorded.size(), 0);
		for (size_t i = 0; i < st.recorded.size(); i++) {
			flat[i] = (st.recorded[i] >> chosen) & 1u;
		}

		log::info("Probe 0: handleButton isPlayer1 arg for human jump input: "
		          "true x{}, false x{}", st.humanIsPlayer1True, st.humanIsPlayer1False);

		// Keep the human run so a replay can be checked against it. Run-to-run
		// consistency alone cannot catch a systematically wrong injection: that
		// would be perfectly reproducible and still wrong.
		st.humanReference        = st.trace;
		st.humanReferencePercent = st.trace.lastPercent();
		log::info("Probe 0: human reference trace kept, {} steps, reached {:.2f}%",
		          st.humanReference.size(), st.humanReferencePercent);

		if (saveInput(flat)) {
			log::info("Probe 0: recorded {} steps of input -> {}", flat.size(), inputFilePath());
			st.scripted           = flat;
			st.recordedPhysicsFix = g_config.physicsFix;
			st.haveRecordingMeta  = true;
		} else {
			log::error("Probe 0: failed to write {}", inputFilePath());
		}
		st.mode = Mode::Idle;
		return;
	}

	if (st.mode != Mode::Determinism) return;

	const uint64_t h = st.trace.hash();
	st.hashes.push_back(h);
	st.hashesSkip1.push_back(st.trace.hashFrom(1));

	// One row is written per update() call, so trace.size() is the number of
	// update() calls. m_currentStep is recorded alongside purely as an
	// observation: if it stays flat it is confirmed to be replay-only
	// bookkeeping, and if it advances, the delta over the attempt divided by
	// the update() count is how many 240Hz steps one update() absorbs - which
	// is Probe 3's question, answered here for free.
	if (st.trace.size() > 0) {
		const int engineDelta = st.trace[st.trace.size() - 1].engineStep - st.trace[0].engineStep;
		if (engineDelta == 0) {
			log::info("Probe 1: run {}  {} update() calls, m_currentStep FLAT at {} "
			          "(confirmed replay-only, not a physics step counter)",
			          st.runIndex, st.trace.size(), st.trace[0].engineStep);
		} else {
			log::info("Probe 1: run {}  {} update() calls, m_currentStep advanced {} "
			          "(ratio {:.3f} engine steps per update call)",
			          st.runIndex, st.trace.size(), engineDelta,
			          static_cast<double>(engineDelta) / static_cast<double>(st.trace.size()));
		}
	}

	const bool diverged = !st.hashes.empty() && h != st.hashes[0];

	if (st.runIndex == 0) {
		st.reference = st.trace;
		std::string p = probe::outputPath(std::string("probe1_") + configName() + "_run0.csv");
		st.trace.writeCsv(p);
		log::info("Probe 1: run 0  hash {:016X}  {} rows -> {}", h, st.trace.size(), p);

		// FIDELITY, as distinct from determinism. Does replaying the recorded
		// input through our injection actually reproduce the human run? A
		// wrong injection is perfectly reproducible, so the 10-run hash check
		// cannot see it. This can.
		if (st.haveRecordingMeta && st.recordedPhysicsFix != g_config.physicsFix) {
			// Not a bug: a 240Hz-granularity recording replayed at 120Hz
			// consumes the input array at half rate, so the timing is stretched
			// and the run cannot match. Reported as a skip, not a failure.
			log::info("Probe 1: fidelity SKIPPED - recording was made with "
			          "physics_fix={} but this sweep runs with physics_fix={}. "
			          "Input granularity differs, so a match is impossible by "
			          "construction. Re-record under this config to test fidelity.",
			          st.recordedPhysicsFix ? 1 : 0, g_config.physicsFix ? 1 : 0);
		} else if (st.humanReference.size() > 0) {
			const size_t at = st.humanReference.firstDivergenceMasked(
				st.trace, probe::kInputSourceMask);
			const float replayPercent = st.trace.lastPercent();

			if (at == SIZE_MAX) {
				log::info("Probe 1: FIDELITY OK - replay is bit-identical to the "
				          "human run over all {} steps ({:.2f}%)",
				          st.humanReference.size(), replayPercent);
			} else {
				log::error("Probe 1: FIDELITY FAILED - replay diverges from the human "
				           "run at step {} of {}. Human reached {:.2f}%, replay reached "
				           "{:.2f}%. Injection or recording is wrong; determinism "
				           "results below are about a DIFFERENT run than the one "
				           "recorded.",
				           at, st.humanReference.size(),
				           st.humanReferencePercent, replayPercent);
				std::string hp = probe::outputPath("probe1_human_reference.csv");
				st.humanReference.writeCsv(hp);
				log::error("Probe 1: human reference written to {} for diffing", hp);
			}
		} else {
			log::warn("Probe 1: no human reference recorded (F5), so injection "
			          "fidelity is UNVERIFIED - this run only tests determinism");
		}
	} else if (diverged) {
		size_t at = st.reference.firstDivergence(st.trace);
		std::string p = probe::outputPath(
			std::string("probe1_") + configName() + "_run" + std::to_string(st.runIndex) + "_DIVERGED.csv");
		st.trace.writeCsv(p);
		log::error("Probe 1: run {}  hash {:016X}  DIVERGED from run 0 at row {} (step {}) -> {}",
		           st.runIndex, h, at,
		           at < st.trace.size() ? st.trace[at].step : -1, p);
	} else {
		log::info("Probe 1: run {}  hash {:016X}  {} rows  ok", st.runIndex, h, st.trace.size());
	}

	if (!st.trace.valid()) {
		log::error("Probe 1: run {} overflowed the trace buffer - result is INVALID", st.runIndex);
	}

	st.runIndex++;
	st.runsLeft--;

	if (st.runsLeft > 0) st.resetPending = true;
	else                 finishDeterminismSweep();
}

// ---------------------------------------------------------------------------
// Probe 5: savestate cost
// ---------------------------------------------------------------------------
//
// "If savestate restore costs 50ms you get ~20 branches/sec and no algorithm
// saves you. If it costs 0.1ms you get thousands and naive DFS clears Stereo
// Madness." This is that number.
//
// Also measures MARGINAL memory per live checkpoint at 1/100/1000, which bounds
// K for a width-K beam and sets the point where retaining a deferred branch as
// a live savestate stops beating storing its input prefix and replaying.

size_t processMemoryBytes() {
	PROCESS_MEMORY_COUNTERS pmc{};
	pmc.cb = sizeof(pmc);
	if (!GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) return 0;
	return pmc.WorkingSetSize;
}

void runProbe5(int count) {
	auto* pl = PlayLayer::get();
	if (!pl || !pl->m_player1) {
		log::warn("Probe 5: not in an active level");
		return;
	}

	log::info("Probe 5: savestate cost, {} iterations...", count);

	// --- capture cost, retaining every checkpoint so memory can be measured ---
	std::vector<CheckpointObject*> kept;
	kept.reserve(count);
	std::vector<double> createUs;
	createUs.reserve(count);

	const size_t memStart = processMemoryBytes();
	size_t memAt1 = 0, memAt100 = 0;

	for (int i = 0; i < count; i++) {
		const uint64_t t0 = probe::nowTicks();
		CheckpointObject* cp = pl->createCheckpoint();
		const uint64_t t1 = probe::nowTicks();
		createUs.push_back(probe::ticksToMicros(t1 - t0));

		if (cp) { cp->retain(); kept.push_back(cp); }

		if (i == 0)  memAt1   = processMemoryBytes();
		if (i == 99) memAt100 = processMemoryBytes();
	}
	const size_t memAtN = processMemoryBytes();

	// --- restore cost, against a single checkpoint ---
	std::vector<double> loadUs;
	loadUs.reserve(count);
	if (!kept.empty()) {
		for (int i = 0; i < count; i++) {
			const uint64_t t0 = probe::nowTicks();
			pl->loadFromCheckpoint(kept.front());
			const uint64_t t1 = probe::nowTicks();
			loadUs.push_back(probe::ticksToMicros(t1 - t0));
		}
	}

	const probe::Stats cs = probe::summarize(createUs);
	const probe::Stats ls = probe::summarize(loadUs);

	log::info("  createCheckpoint   min {:.1f}us  median {:.1f}us  p99 {:.1f}us  max {:.1f}us",
	          cs.min, cs.median, cs.p99, cs.max);
	log::info("  loadFromCheckpoint min {:.1f}us  median {:.1f}us  p99 {:.1f}us  max {:.1f}us",
	          ls.min, ls.median, ls.p99, ls.max);

	const double perState = kept.size() > 1
		? static_cast<double>(memAtN - memStart) / static_cast<double>(kept.size())
		: 0.0;
	log::info("  memory: start {} KB, after 1 {} KB, after 100 {} KB, after {} {} KB",
	          memStart / 1024, memAt1 / 1024, memAt100 / 1024, kept.size(), memAtN / 1024);
	log::info("  marginal per live checkpoint: ~{:.0f} bytes ({:.1f} KB)", perState, perState / 1024.0);

	// Gate from the plan: <1ms viable, 1-10ms constrains the search design,
	// >10ms forces an architecture rethink.
	const char* verdict = ls.median < 1000.0  ? "VIABLE (<1ms): savestate search is cheap"
	                    : ls.median < 10000.0 ? "CONSTRAINED (1-10ms): search must minimise restores"
	                                          : "PROBLEM (>10ms): architecture rethink";
	log::info("  restore median {:.3f} ms -> {}", ls.median / 1000.0, verdict);

	for (auto* cp : kept) cp->release();
	log::info("Probe 5: released {} checkpoints", kept.size());
}

// ---------------------------------------------------------------------------
// Hotkeys
// ---------------------------------------------------------------------------
//
// Polled once per rendered frame, never inside the stepping loop.

bool keyPressedEdge(int vk) {
	static bool prev[256] = {};
	const bool down = (GetAsyncKeyState(vk) & 0x8000) != 0;
	const bool edge = down && !prev[vk & 0xFF];
	prev[vk & 0xFF] = down;
	return edge;
}

void pollHotkeys() {
	auto& st = ProbeState::get();

	if (keyPressedEdge(VK_F1)) startDeterminismSweep(false, false); // config a
	if (keyPressedEdge(VK_F2)) startDeterminismSweep(true,  false); // config b
	if (keyPressedEdge(VK_F3)) startDeterminismSweep(true,  true);  // config c

	if (keyPressedEdge(VK_F5)) {
		st.mode = Mode::RecordInput;
		st.resetPending = true;
		log::info("Probe 0: recording input on next attempt - play the level, "
		          "recording stops on death or completion");
	}

	if (keyPressedEdge(VK_F6)) {
		bool fileFix = false;
		if (loadInput(st.scripted, &fileFix) && !st.scripted.empty()) {
			st.recordedPhysicsFix = fileFix;
			st.haveRecordingMeta  = true;
			log::info("Probe 0: loaded {} steps from {} (physics_fix={})",
			          st.scripted.size(), inputFilePath(), fileFix ? 1 : 0);
		} else {
			log::warn("Probe 0: no input file at {}", inputFilePath());
		}
	}

	if (keyPressedEdge(VK_F10)) {
		g_config.injectionLeadSteps++;
		log::info("Probe 0: injection lead now {} steps", g_config.injectionLeadSteps);
	}
	if (keyPressedEdge(VK_F9)) {
		g_config.injectionLeadSteps = std::max(0, g_config.injectionLeadSteps - 1);
		log::info("Probe 0: injection lead now {} steps", g_config.injectionLeadSteps);
	}

	if (keyPressedEdge(VK_F8)) {
		g_config.physicsFix = !g_config.physicsFix;
		log::info("Probe 0: physics fix now {}. Record (F5) with this ON once the "
		          "delta is correct, so the human reference is itself captured under "
		          "fixed dt - comparing a fixed-dt replay against a vanilla recording "
		          "can never match exactly, because vanilla is not reproducible.",
		          g_config.physicsFix ? "ENABLED" : "disabled");
	}

	if (keyPressedEdge(VK_F11)) runProbe5(1000);

	if (keyPressedEdge(VK_F7)) {
		std::string p = probe::outputPath("trace_manual.csv");
		st.trace.writeCsv(p);
		log::info("Probe 0: {} rows, hash {:016X} -> {}", st.trace.size(), st.trace.hash(), p);
	}
}

} // namespace

// ---------------------------------------------------------------------------
// GJBaseGameLayer
// ---------------------------------------------------------------------------

class $modify(SolverBaseLayer, GJBaseGameLayer) {

	// Project plan 5.2 part 1. The original MUST be called first: it mutates
	// internal state (m_extraDelta) that the rest of the frame depends on.
	// Returns double in 2.2081 - peony's published snippet says float and is
	// out of date.
	double getModifiedDelta(float dt) {
		auto& st = ProbeState::get();

		const double extraBefore = m_extraDelta;
		const double original    = GJBaseGameLayer::getModifiedDelta(dt);
		const double extraAfter  = m_extraDelta;

		const double timeWarp = static_cast<double>(m_gameState.m_timeWarp);
		const double ours     = kPhysicsDt * std::fmin(timeWarp, 1.0);

		// Probe 2. First few calls of each attempt only - three lines per
		// attempt, never per tick. This is the measurement that says what the
		// game's native per-call delta actually is, rather than assuming 1/240.
		if (st.deltaLogCount < 3) {
			st.deltaLogCount++;
			log::info("Probe 2: dt_in={:.9f} original_returned={:.9f} ours={:.9f} "
			          "timeWarp={:.6f} extraDelta {:.9f} -> {:.9f} "
			          "(1/dt_in={:.2f}, 1/original={:.2f})",
			          static_cast<double>(dt), original, ours, timeWarp,
			          extraBefore, extraAfter,
			          dt      > 0.0 ? 1.0 / static_cast<double>(dt) : 0.0,
			          original > 0.0 ? 1.0 / original                : 0.0);
		}

		if (!g_config.physicsFix) return original;
		return ours;
	}

	// Assert the input for the step about to be simulated. Re-asserted every
	// step regardless of how often decisions are made: button-hold precision
	// must be decoupled from decision cadence.
	void applyInput(bool pressed) {
		auto& st = ProbeState::get();
		st.injecting = true;
		if (pressed) {
			this->handleButton(true, kJumpButton, true);
			st.isHolding = true;
		} else if (st.isHolding) {
			this->handleButton(false, kJumpButton, true);
			st.isHolding = false;
		}
		st.injecting = false;
	}

	// Read the two state-based input sources. The two event-based ones
	// (handleButton, pushButton) are maintained by their own hooks.
	void sampleStateInputSources() {
		auto& st = ProbeState::get();

		st.srcAsyncKey = (GetAsyncKeyState(VK_SPACE)   & 0x8000) != 0
		              || (GetAsyncKeyState(VK_UP)      & 0x8000) != 0
		              || (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;

		st.srcHoldingMap = false;
		if (auto* p = m_player1) {
			auto it = p->m_holdingButtons.find(kJumpButton);
			if (it != p->m_holdingButtons.end()) st.srcHoldingMap = it->second;
		}
	}

	// Capture the state at the END of the step just simulated.
	//
	// `relStep` is the step index relative to the start of this attempt, not
	// the absolute m_currentStep. Traces are compared byte-for-byte across
	// runs, so an absolute counter that happens to start at a different value
	// would make two physically identical runs hash differently.
	void recordStep(bool pressed, int relStep) {
		auto& st = ProbeState::get();
		auto* p  = m_player1;
		if (!p) return;

		auto* pl = PlayLayer::get();

		probe::TraceRow row{};
		row.step        = relStep;
		row.engineStep  = m_currentStep;
		row.x           = probe::bits(p->getPositionX());
		row.y           = probe::bits(p->getPositionY());
		row.rotation    = probe::bits(p->getRotation());
		row.percent     = probe::bits(pl ? pl->getCurrentPercent() : 0.f);
		row.playerSpeed = probe::bits(p->m_playerSpeed);
		row.yVelocity   = probe::bits(p->m_yVelocity);
		row.gravity     = probe::bits(p->m_gravity);

		uint32_t f = 0;
		if (p->m_isDead)       f |= probe::FlagDead;
		if (p->m_isOnGround)   f |= probe::FlagOnGround;
		if (p->m_isOnGround2)  f |= probe::FlagOnGround2;
		if (p->m_isOnGround3)  f |= probe::FlagOnGround3;
		if (p->m_isOnGround4)  f |= probe::FlagOnGround4;
		if (p->m_jumpBuffered) f |= probe::FlagJumpBuffered;
		if (p->m_isUpsideDown) f |= probe::FlagUpsideDown;
		if (p->m_isShip)       f |= probe::FlagShip;
		if (p->m_isBird)       f |= probe::FlagBird;
		if (p->m_isBall)       f |= probe::FlagBall;
		if (p->m_isDart)       f |= probe::FlagDart;
		if (p->m_isRobot)      f |= probe::FlagRobot;
		if (p->m_isSpider)     f |= probe::FlagSpider;
		if (p->m_isSwing)      f |= probe::FlagSwing;
		if (p->m_isSideways)   f |= probe::FlagSideways;
		if (pressed)           f |= probe::FlagPressed;
		if (st.srcHandleButton) f |= probe::FlagSrcHandleButton;
		if (st.srcPushButton)   f |= probe::FlagSrcPushButton;
		if (st.srcHoldingMap)   f |= probe::FlagSrcHoldingMap;
		if (st.srcAsyncKey)     f |= probe::FlagSrcAsyncKey;
		f |= probe::packRingCount(p->m_touchingRings ? p->m_touchingRings->count() : 0);
		row.flags = f;

		st.trace.push(row);
	}

	void update(float dt) {
		pollHotkeys();

		auto& st = ProbeState::get();
		auto* pl = PlayLayer::get();

		// Measure the native frame delta once per level, from the genuine
		// scheduler dt, and derive how many 1/240 steps make one frame.
		if (!st.nativeDtLocked && dt > 0.f) {
			st.dtSamples.push_back(dt);
			if (st.dtSamples.size() >= 61) {
				std::vector<float> s = st.dtSamples;
				std::sort(s.begin(), s.end());
				st.nativeDt       = static_cast<double>(s[s.size() / 2]);
				st.nativeDtLocked = true;
				g_config.stepsPerFrame =
					std::max(1, static_cast<int>(std::lround(st.nativeDt / kPhysicsDt)));
				log::info("Probe 2: native frame delta (median of {}) = {:.9f} s "
				          "(1/{:.1f}) -> stepsPerFrame = {}",
				          s.size(), st.nativeDt, 1.0 / st.nativeDt,
				          g_config.stepsPerFrame);
			}
		}

		if (st.resetPending && pl) {
			st.resetPending = false;
			st.resetPerAttempt();
			pl->resetLevelFromStart();
			return;
		}

		const bool active = pl && m_player1 && !m_player1->m_isDead && !st.finished;

		if (st.mode == Mode::Idle || !active) {
			GJBaseGameLayer::update(dt);
			// Death is checked outside `active` on purpose: isGameplayActive()
			// goes false at the exact moment of death, so anything gated behind
			// it can never observe the transition.
			const bool deadNow = m_player1 && m_player1->m_isDead;
			if (deadNow && !st.wasDead && st.mode != Mode::Idle) {
				st.wasDead = true;
				onAttemptEnded();
			}
			st.wasDead = deadNow;
			return;
		}

		if (!st.attemptStarted) {
			st.attemptStarted = true;
			// Logged once per attempt, never per step.
			log::debug("attempt start: m_currentStep = {}", m_currentStep);
		}
		// With the fix on, one update(1/240) call is exactly one physics step,
		// so several run per rendered frame to hold real-time pace. Without it,
		// one call advances however many sub-steps the engine decides and the
		// trace is correspondingly coarser.
		const int steps = g_config.physicsFix ? g_config.stepsPerFrame : 1;

		for (int i = 0; i < steps; i++) {
			const size_t idx = static_cast<size_t>(st.stepCounter);

			if (st.mode == Mode::RecordInput) {
				// Sample BEFORE stepping, so recorded[idx] is the input state
				// ENTERING step idx. Replay does applyInput(idx) then steps, so
				// sampling after the step recorded the state leaving it and put
				// the whole track one step out of phase - harmless across a
				// 15-step hold, fatal on a frame-perfect one.
				sampleStateInputSources();

				// Store all four readings per step. Which one is authoritative
				// is decided at save time from the tallies.
				uint8_t mask = 0;
				if (st.srcHandleButton) { mask |= 1u << 0; st.heldSteps[0]++; }
				if (st.srcPushButton)   { mask |= 1u << 1; st.heldSteps[1]++; }
				if (st.srcHoldingMap)   { mask |= 1u << 2; st.heldSteps[2]++; }
				if (st.srcAsyncKey)     { mask |= 1u << 3; st.heldSteps[3]++; }

				if (idx < kMaxTraceRows) {
					st.recorded.resize(std::max(st.recorded.size(), idx + 1), 0);
					st.recorded[idx] = mask;
				}

				GJBaseGameLayer::update(g_config.physicsFix ? static_cast<float>(kPhysicsDt) : dt);
				recordStep(mask != 0, static_cast<int>(idx));
			} else {
				// Assert the input for a step `injectionLeadSteps` ahead, so it
				// takes effect on the step it belongs to.
				const size_t src = idx + static_cast<size_t>(g_config.injectionLeadSteps);
				const bool asserted = src < st.scripted.size() && st.scripted[src] != 0;

				// ...but record the input INTENDED for this step, so the trace
				// lines up with the human recording's semantics.
				const bool intended = idx < st.scripted.size() && st.scripted[idx] != 0;

				applyInput(asserted);
				GJBaseGameLayer::update(g_config.physicsFix ? static_cast<float>(kPhysicsDt) : dt);
				recordStep(intended, static_cast<int>(idx));
			}
			st.stepCounter++;

			// Stop mid-frame rather than stepping a dead or finished player.
			if ((m_player1 && m_player1->m_isDead) || st.finished || !st.trace.valid()) break;
		}

		const bool deadNow = m_player1 && m_player1->m_isDead;
		if ((deadNow && !st.wasDead) || st.finished || !st.trace.valid()) {
			st.wasDead = true;
			onAttemptEnded();
		}
		st.wasDead = deadNow;
	}

	// Observe real button input so recording captures what the human pressed,
	// while ignoring our own injected presses.
	void handleButton(bool down, int button, bool isPlayer1) {
		auto& st = ProbeState::get();
		if (!st.injecting && button == kJumpButton) {
			// isPlayer1 is deliberately NOT filtered on here. Requiring it
			// discarded every human press, so whatever the argument means it is
			// not "this event belongs to player 1". Tallied rather than assumed.
			st.srcHandleButton = down;
			if (isPlayer1) st.humanIsPlayer1True++;
			else           st.humanIsPlayer1False++;
		}
		GJBaseGameLayer::handleButton(down, button, isPlayer1);
	}
};

// ---------------------------------------------------------------------------
// PlayLayer
// ---------------------------------------------------------------------------

class $modify(SolverPlayLayer, PlayLayer) {

	bool init(GJGameLevel* level, bool useReplay, bool dontCreateObjects) {
		if (!PlayLayer::init(level, useReplay, dontCreateObjects)) return false;
		ProbeState::get().resetPerLevel();
		log::info("gd-solver: level loaded. F1/F2/F3 = determinism sweep (config a/b/c), "
		          "F5 = record input, F6 = load input, F7 = dump trace");
		return true;
	}

	void resetLevel() {
		PlayLayer::resetLevel();
		ProbeState::get().resetPerAttempt();

		// Log the seeds BEFORE overwriting them. Configs (b) and (c) produced
		// byte-identical traces, which either means RNG does not affect physics
		// or means this write is a no-op. If the pre-force values differ
		// between attempts while the traces stay identical, it is the former.
		log::info("Probe 1: seeds at reset: m_randomSeed={:016X} m_replayRandSeed={:016X}{}",
		          m_randomSeed, m_replayRandSeed,
		          g_config.forceSeeds ? "  (forcing to constant)" : "");

		if (g_config.forceSeeds) {
			m_randomSeed     = kForcedSeed;
			m_replayRandSeed = kForcedSeed;
		}
	}

	// Both completion signals are hooked so Probe 7 can see which fires first
	// and whether both fire. m_hasCompletedLevel is useless for this: it only
	// flips once the completion menu appears, far too late.
	void playEndAnimationToPos(cocos2d::CCPoint position) {
		PlayLayer::playEndAnimationToPos(position);
		auto& st = ProbeState::get();
		st.endAnimStep = m_currentStep;
		st.finished    = true;
	}

	void levelComplete() {
		PlayLayer::levelComplete();
		ProbeState::get().levelCompleteStep = m_currentStep;
	}
};

// ---------------------------------------------------------------------------
// PlayerObject
// ---------------------------------------------------------------------------

// pushButton/releaseButton are where player button state actually changes, so
// this observer sits downstream of whatever input plumbing the game uses -
// direct dispatch, queued commands, or replay playback.
class $modify(SolverPlayerObject, PlayerObject) {

	bool pushButton(PlayerButton button) {
		auto& st = ProbeState::get();
		if (!st.injecting && button == PlayerButton::Jump) st.srcPushButton = true;
		return PlayerObject::pushButton(button);
	}

	bool releaseButton(PlayerButton button) {
		auto& st = ProbeState::get();
		if (!st.injecting && button == PlayerButton::Jump) st.srcPushButton = false;
		return PlayerObject::releaseButton(button);
	}
};

$on_mod(Loaded) {
	log::info("gd-solver Phase 0 instrumentation loaded. Output dir: {}",
	          Mod::get()->getSaveDir().string());
}
