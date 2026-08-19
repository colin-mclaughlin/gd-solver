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
#include <cstring>
#include <cctype>
#include <memory>

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

// Wall-clock the solver may spend per rendered frame before yielding. Keeps the
// window pumping messages so the process never goes Not Responding, at the cost
// of capping throughput at (budget / step cost) x refresh.
constexpr double kSolverFrameBudgetUs = 12'000.0; // 12 ms

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

	// Branch granularity for modes where input matters every frame (ship, wave,
	// UFO, swing). 1 would be exhaustive and correct but explodes; this is the
	// knob to tighten if a section proves unsolvable.
	int airBranchInterval = 4;

	// Minimum gap between decisions while grounded. Branching on EVERY grounded
	// frame is the complete formulation, but it produced a 3400-deep stack over
	// a third of Stereo Madness - which both blew memory and, per Probe 5,
	// degraded restore cost badly enough to cut throughput 15x. Landings and
	// orb contacts still always branch, since those are timing-critical.
	int groundBranchInterval = 4;

	// Escape from the DFS thrash documented in plan section 6.3: when the fatal
	// mistake happened long before the death, DFS exhausts an innocent subtree
	// near the death instead of backing up to the culprit. After this many
	// deaths with no improvement in best%, abandon a chunk of the stack to force
	// exploration further back.
	//
	// This TRADES AWAY strict completeness, which was the main argument for DFS
	// over beam. Set stallLimit to 0 to disable and get pure DFS back.
	int stallLimit  = 400;
	int escapeJump  = 200;

	// Iterative deepening on air toggle count. Starts at 1 so the very first
	// things tried are "never change the input" and "change it exactly once" -
	// the latter being "hold from ship entry onward".
	int toggleBudget    = 1;
	int maxToggleBudget = 24;

	// Prefix commitment (plan section 7, "segment decomposition").
	//
	// Measured: once the air toggles at a given budget were exhausted, the
	// search fell back into the ALREADY-SOLVED cube prefix and re-searched it
	// exhaustively - 2.9 million steps at 1099 steps per death, best% frozen,
	// and the toggle budget could never deepen because an 800-decision cube
	// tree never exhausts. Freezing the prefix behind the frontier fixes all
	// three at once.
	//
	// Measured in GAME STEPS behind the best progress, not stack indices.
	//
	// The first version stored an absolute stack index taken when best% improved
	// - i.e. when the stack was DEEPEST. Backtracking later shrank the stack to
	// exactly that index, so commitDepth == stack.size() and the search froze
	// solid: budgets 1..24 exhausted in one second with 87 total deaths.
	// A step-based cutoff is stable under stack growth and shrinkage.
	// 1 s at 240Hz. Must be well BELOW the length of a section, or the floor can
	// never advance within it: the ship is ~330 steps, so a 480-step window
	// would still span the whole thing and freeze nothing.
	int commitLookbackSteps = 240;

	// Give up on a branch that survives this long without a decision point or
	// death, to avoid an unbounded descent.
	int maxSegmentSteps = 2000;

	// Hard cap on search depth. Each frame retains a ~22 KB checkpoint, and
	// holding thousands of them also degrades restore cost badly (measured:
	// 43 ms -> 165 ms at 1000 live states), so an unbounded stack strangles
	// throughput long before it exhausts memory.
	int maxStackDepth = 4000;

	// Probe 4b experiment switch: force all captured scalar state after restore.
	// Off: the targeted field list above should now be sufficient. Turning this
	// back on is the fallback if a restore goes unsound again.

	// Periodic resync: how far the frontier may advance before the committed
	// prefix is re-validated by a clean replay from frame 0.
	//
	// Restores are not bit-exact in ship sections and six rounds of byte-level
	// comparison have not found the missing state. Rather than require a perfect
	// substrate, the search periodically re-derives its own prefix with no
	// savestates at all: that both PROVES the prefix works in normal play and
	// leaves the player in a provably clean state to continue from, so restore
	// error cannot accumulate beyond one interval.
	int resyncIntervalSteps = 2400; // 10 s of gameplay

	// OFF by default. Resync was added as a correctness net, but it regressed the
	// solver from a full-level solve in 24 s to stalling at 45%, and its own
	// bookkeeping is buggy (counter resets, lastGoodStep reads 0 after being
	// set). Debugging the safety net instead of the thing it protects is the
	// wrong order. F8 toggles it back on.
	bool resyncEnabled = false;

	// Search WITHOUT savestates: on backtrack, replay from frame 0 to the
	// decision instead of restoring a checkpoint.
	//
	// GD's practice checkpoints are documented as not 1:1 with real gameplay,
	// and specifically approximate in ship/UFO/wave because of momentum - they
	// are generated a set distance BEHIND the icon rather than at it. That is an
	// engine design choice, not a bug we can patch, which is why restoring 557
	// scalar fields changed nothing. Where fidelity matters more than speed,
	// replay is the only exact option.
	bool noSavestates = false;
};

Config g_config;

// ---------------------------------------------------------------------------
// Probe state
// ---------------------------------------------------------------------------

enum class Mode {
	Idle,
	RecordInput,  // capture what the human presses, per step
	Determinism,  // Probe 1: replay fixed input N times, compare trace hashes
	RestoreTest,  // Probe 4b: does a restore reproduce the future, bit-for-bit?
	Solve,        // DFS search for an input sequence that clears the level
	Verify,       // replay a found macro from frame 0, no savestates, no practice
};

// Which save/restore mechanism the restore test exercises.
enum class RestoreKind {
	Full,        // PlayLayer::createCheckpoint / loadFromCheckpoint
	PlayerOnly,  // PlayerObject::saveToCheckpoint / loadFromCheckpoint
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

	// Set immediately before a reset the solver itself requested, so the
	// suppression in resetLevel lets that one through.
	bool solverWantsReset = false;

	// Step at which the last verification failed, so restore-fidelity testing
	// can target the region that actually breaks instead of the level opening.
	int lastVerifyFailStep = -1;

	// Step at which a resync replay died. This is drift caught in normal play,
	// and a far better anchor for restore-fidelity testing than a guess.
	int lastResyncFailStep = -1;

	// Iterative repair: the portion of a macro that has been PROVEN to replay
	// from frame 0. Verification reports the exact step where a savestate-derived
	// path stops being real, so everything before it (less a margin) is known
	// good. The next solve starts by replaying this, then searches forward from a
	// provably clean state - turning each failed verification into locked-in
	// progress without needing to explain the drift.
	std::vector<uint8_t> verifiedPrefix;

	// Sanitised level name, used to namespace every output file. Without this a
	// solve on one level silently overwrites another's macro, and a stale
	// verified prefix from a previous level would be replayed against the wrong
	// geometry.
	std::string levelKey = "unknown";

	// True while the solver is deliberately using resetLevel as a respawn to a
	// chosen checkpoint. The reset must run for real, but must NOT wipe the
	// solver's per-attempt bookkeeping.
	bool solverRestoring = false;

	// --- Probe 4b: restore fidelity ---
	// Drift-zero at t=0 is necessary but not sufficient. The real question is
	// whether the SAME input from a restored state produces the same future.
	RestoreKind  restoreKind   = RestoreKind::Full;
	int          restorePhase  = 0;   // 0 = running to anchor, 1 = segment A, 2 = segment B
	int          restoreAnchor = 480; // step at which to capture (2 s in)
	int          restoreLen    = 600; // steps to simulate per segment
	probe::Trace segmentA;
	probe::Trace segmentB;
	probe::TraceRow anchorRow{};   // state captured at the anchor
	bool           haveAnchorRow = false;
	// Probe 4a: raw PlayerObject bytes at capture, for a field-level diff after
	// restore. The boundary state compares EXACT on position/velocity yet the
	// next step diverges, so whatever differs is a field not being compared.
	std::vector<uint8_t> anchorBytes;
	double anchorGameModeChangedTime = 0.0;
	bool   anchorUnkA29 = false;
	// PlayerObject now compares clean at the boundary except cosmetics and
	// SeedValueRSV noise, yet the next step still diverges - so the missing
	// state is on the LAYER, not the player.
	std::vector<uint8_t> anchorLayerBytes;
	double aExtraDelta = 0.0, aTimePlayed = 0.0, aTimestamp = 0.0;
	int    aTickIndex = 0, aClickIndex = 0, aResumeTimer = 0;
	bool   aJumping = false;
	double aAttemptTime = 0.0, aBestAttemptTime = 0.0, aCurrentTime = 0.0;
	bool   aHasJumped = false;
	CheckpointObject* fullCp   = nullptr;
	PlayerCheckpoint* playerCp = nullptr;

	void releaseCheckpoints() {
		if (fullCp)   { fullCp->release();   fullCp   = nullptr; }
		if (playerCp) { playerCp->release(); playerCp = nullptr; }
	}

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
		releaseCheckpoints();
		restorePhase = 0;
		segmentA.clear();
		segmentB.clear();
		dtSamples.clear();
		nativeDtLocked     = false;
		nativeDt           = 0.0;
		recordedPhysicsFix = false;
		haveRecordingMeta  = false;

		// Per-level, not per-attempt: these describe a specific level's macro and
		// are meaningless (worse, actively wrong) once a different level loads.
		verifiedPrefix.clear();
		lastVerifyFailStep  = -1;
		lastResyncFailStep  = -1;
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

// All solver/probe output is namespaced by level.
std::string levelFilePath(std::string const& suffix) {
	return probe::outputPath(ProbeState::get().levelKey + "_" + suffix);
}

std::string inputFilePath() {
	return levelFilePath("input.txt");
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

bool loadMacroFile(std::string const& path, std::vector<uint8_t>& out,
                   bool* physicsFixOut = nullptr) {
	std::FILE* f = std::fopen(path.c_str(), "rb");
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

bool loadInput(std::vector<uint8_t>& out, bool* physicsFixOut = nullptr) {
	return loadMacroFile(inputFilePath(), out, physicsFixOut);
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
// Verification: does the macro reproduce in a clean run?
// ---------------------------------------------------------------------------
//
// A solution found via savestates is an artifact until it replays from frame 0
// with no restores and no practice mode. This is the check that makes an
// unsound restore impossible to mistake for a success.

void startVerify(bool practiceMode) {
	auto& st = ProbeState::get();
	auto* pl = PlayLayer::get();
	if (!pl) { log::warn("Verify: not in a level"); return; }

	std::string path = levelFilePath("solution.txt");
	bool fileFix = false;
	if (!loadMacroFile(path, st.scripted, &fileFix) || st.scripted.empty()) {
		log::error("Verify: no macro at {}", path);
		return;
	}

	g_config.physicsFix = fileFix;          // replay as it was recorded
	pl->m_isPracticeMode = practiceMode;

	// Isolating one variable: the search ran in practice mode, the first
	// verification in normal mode, and it failed deterministically at step 1222.
	// If the macro passes WITH practice mode on and fails without it, practice
	// mode alters physics and the search must account for it. If it fails in
	// both, practice mode is innocent and restore soundness is the suspect.
	st.mode = Mode::Verify;
	st.resetPending = true;
	log::info("Verify: replaying {} steps from frame 0, practice mode {}, "
	          "no savestates, no restores. physics_fix={}",
	          st.scripted.size(), practiceMode ? "ON" : "OFF", fileFix ? 1 : 0);
}

void finishVerify(bool completed, int atStep, float pct) {
	auto& st = ProbeState::get();
	auto* plv = PlayLayer::get();
	log::info("================ VERIFICATION (practice {}) ================",
	          plv && plv->m_isPracticeMode ? "ON" : "OFF");
	if (completed) {
		log::info("  PASSED - the macro clears the level from frame 0 in normal mode.");
		log::info("  {} steps, reached {:.2f}%", atStep, pct);
		log::info("  This is an independently reproducible solution, not a savestate artifact.");
	} else {
		st.lastVerifyFailStep = atStep;

		// Lock in everything up to a margin before the failure.
		const int margin = 600; // 2.5 s
		const int keep   = std::max(0, atStep - margin);
		if (keep > static_cast<int>(st.verifiedPrefix.size())) {
			st.verifiedPrefix.assign(st.scripted.begin(), st.scripted.begin() + keep);
			log::info("  Locked a verified prefix of {} steps. F2 will replay it and "
			          "search forward from there.", keep);
		}

		log::error("  FAILED - died at step {} of {} ({:.2f}%).",
		           atStep, st.scripted.size(), pct);
		log::error("  Probe 4b (F12) will now anchor near this step to test whether "
		           "restores are still sound this late in the level.");
		log::error("  The macro does not reproduce. Either a restore was unsound, or "
		           "practice mode differs from normal play, or injection timing differs.");
	}
	log::info("==============================================");
	st.mode = Mode::Idle;
}

// ---------------------------------------------------------------------------
// The solver
// ---------------------------------------------------------------------------
//
// DFS with backtracking over decision points, pruning on death.
//
// The action at a decision point is "the button state to hold until the next
// decision point", which covers both a cube's tap-on-landing and a ship's
// sustained hold with one model.
//
// Ordering: try NOT pressing first. The behaviour-cloning dataset showed a
// ~10% jump rate, so no-press is the likelier branch and finding it first
// prunes more.

// Where input can change the outcome, per game mode (plan section 6.3). Getting
// this wrong in the permissive direction costs a slightly-too-frequent branch;
// getting it wrong in the restrictive direction can make a level unsolvable
// because the one frame that mattered was never considered.
bool isDecisionPoint(PlayerObject* p, int stepsSinceLast) {
	if (!p) return false;

	const bool ship  = p->m_isShip;
	const bool ufo   = p->m_isBird;
	const bool wave  = p->m_isDart;
	const bool swing = p->m_isSwing;

	if (ship || wave || swing || ufo) {
		// Hold state controls the trajectory continuously. Branching every
		// single frame is correct but explodes, so branch on a fixed grid and
		// tighten it only if a section proves unsolvable at this granularity.
		return stepsSinceLast >= g_config.airBranchInterval;
	}

	// Cube, ball, spider: only on a surface or touching an orb. Robot also
	// varies jump height with hold duration, which this does not yet model.
	const bool onGround = p->m_isOnGround;
	const bool onRing   = p->m_touchingRings && p->m_touchingRings->count() > 0;
	return onGround || onRing;
}

struct Decision {
	CheckpointObject* cp        = nullptr; // state entering this decision
	int               step      = 0;       // solver step index at capture
	uint8_t           tried     = 0;       // bit0 = tried release, bit1 = tried hold
	bool              choice    = false;   // action currently being explored
	bool              airPolicy = false;   // which branch policy created this

	// Iterative deepening on toggle count, for air sections.
	//
	// Good ship paths have FEW input changes: "hold from ship entry" is one
	// toggle, "hold then release to level off" is two. DFS on raw hold/release
	// reaches those last, having exhausted astronomically many high-toggle
	// sequences first. Bounding the toggle count and raising the bound on
	// exhaustion searches simple paths before complicated ones, and stays
	// complete as the bound grows.
	bool enteringHold  = false; // hold state on arrival, so a toggle is well-defined
	int  togglesBefore = 0;     // air toggles used on the path up to this decision
	bool modeTransition = false; // pushed because the game mode changed here

	// State the vanilla checkpoint does NOT restore, found by byte-diffing
	// PlayerObject across a restore (Probe 4a). Everything else that differed
	// was cocos render bookkeeping, particle/streak flags, or SeedValueRSV
	// anti-cheat obfuscation whose raw bytes change without the value changing.
	double gameModeChangedTime = 0.0;
	bool   unkA29 = false;

	// Layer state the checkpoint also drops (Probe 4a on GJBaseGameLayer).
	// m_extraDelta is the accumulator getModifiedDelta mutates; the rest are
	// time/tick bookkeeping that time-based physics can read.
	double layerExtraDelta = 0.0;
	double layerTimePlayed = 0.0;
	double layerTimestamp  = 0.0;
	int    layerTickIndex  = 0;
	int    layerClickIndex = 0;
	int    layerResumeTimer = 0;
	bool   layerJumping    = false;

	// PlayLayer's OWN region, above sizeof(GJBaseGameLayer) = 14240, which every
	// earlier byte comparison was blind to. Restoring the full forced set here
	// made a transition-window restore bit-identical, and these are the
	// time-carrying fields in that region.
};

struct Solver {
	bool                  running    = false;
	std::vector<Decision> stack;
	std::vector<uint8_t>  macro;      // input applied per step, truncated on backtrack
	bool                  hold       = false;
	int                   step       = 0;
	int                   lastBranch = 0;
	float                 bestPct    = 0.f;
	uint64_t              deaths     = 0;
	uint64_t              restores   = 0;
	uint64_t              steps      = 0;
	uint64_t              startTicks = 0;
	uint64_t              lastReport = 0;
	float                 diedAtX      = -1.f;
	bool                  prevOnGround = false;
	bool                  prevAirMode  = false;
	int                   togglesUsed  = 0;
	size_t                commitDepth  = 0; // cached for logging; see solverCommitFloor()
	int                   bestStep     = 0; // solver step at which bestPct was reached
	uint64_t              deathsAtBest = 0;
	uint64_t              escapes      = 0;
	struct PendingExtra {
		double gameModeChangedTime = 0.0;
		bool   unkA29 = false;
		double extraDelta = 0.0, timePlayed = 0.0, timestamp = 0.0;
		int    tickIndex = 0, clickIndex = 0, resumeTimer = 0;
		bool   jumping = false;
		double attemptTime = 0.0, bestAttemptTime = 0.0, currentTime = 0.0;
		bool   hasJumped = false;
		bool   valid = false;
	};
	PendingExtra          pendingExtra{};

	bool                  resyncing      = false;
	bool                  resyncForBacktrack = false;
	bool                  resumeHold     = false;
	int                   resumeToggles  = 0;
	int                   resumeBranch   = 0;
	uint64_t              replayBacktracks = 0;
	int                   resyncTarget   = 0;
	int                   lastResyncStep = 0;
	uint64_t              resyncs        = 0;
	uint64_t              resyncFailures = 0;
	std::vector<uint8_t>  resyncMacro;      // prefix being validated
	std::vector<uint8_t>  lastGoodMacro;    // last prefix that replayed clean
	int                   lastGoodStep   = 0;

	void clear() {
		// Drop our checkpoint out of GD's array first. We swap our own object
		// into m_checkpointArray to drive the practice respawn; releasing ours
		// while the game still references it leaves a dangling pointer, which
		// is a plausible cause of the crashes on returning to the level screen.
		if (auto* pl = PlayLayer::get()) {
			if (auto* arr = pl->m_checkpointArray) arr->removeAllObjects();
		}
		for (auto& d : stack) if (d.cp) d.cp->release();
		stack.clear();
		macro.clear();
		running = false;
		hold = false;
		step = 0;
		lastBranch = 0;
		togglesUsed = 0;
		resyncing = false;
		resyncForBacktrack = false;
		replayBacktracks = 0;
		resyncTarget = 0;
		lastResyncStep = 0;
		resyncs = resyncFailures = 0;
		resyncMacro.clear();
		lastGoodMacro.clear();
		lastGoodStep = 0;
		prevAirMode = false;
		prevOnGround = false;
		commitDepth = 0;
		bestStep = 0;
		bestPct = 0.f;
		deaths = restores = steps = 0;
	}

	static Solver& get() { static Solver s; return s; }
};

void solverWriteMacro(const char* name) {
	auto& sv = Solver::get();
	std::string path = levelFilePath(name);
	if (std::FILE* f = std::fopen(path.c_str(), "wb")) {
		std::fprintf(f, "# gd-solver macro v1, %zu steps, physics_fix=%d, 240Hz\n",
		             sv.macro.size(), g_config.physicsFix ? 1 : 0);
		for (size_t i = 0; i < sv.macro.size(); i++) {
			std::fputc(sv.macro[i] ? '1' : '0', f);
			if ((i + 1) % 80 == 0) std::fputc('\n', f);
		}
		std::fputc('\n', f);
		std::fclose(f);
		log::info("Solver: macro written to {}", path);
	}
}

// ---------------------------------------------------------------------------
// Probe 4b: restore fidelity
// ---------------------------------------------------------------------------

const char* restoreKindName(RestoreKind k) {
	return k == RestoreKind::Full ? "FULL (PlayLayer checkpoint)"
	                              : "PLAYER-ONLY (PlayerObject checkpoint)";
}

void startRestoreTest(RestoreKind kind) {
	auto& st = ProbeState::get();
	if (!PlayLayer::get()) { log::warn("Probe 4b: not in a level"); return; }

	// Prefer the solution macro: its first ~1200 steps are known to replay
	// cleanly from frame 0, so anchor 480 + 600 steps stays inside verified
	// territory. Testing restore fidelity on top of an input that does not
	// replay faithfully would measure nothing.
	bool fix = false;
	bool loaded = loadMacroFile(levelFilePath("solution.txt"), st.scripted, &fix);
	if (loaded && !st.scripted.empty()) {
		log::info("Probe 4b: using solution.txt ({} steps)", st.scripted.size());
	} else if (loadInput(st.scripted, &fix) && !st.scripted.empty()) {
		log::warn("Probe 4b: no solution.txt, falling back to input.txt - note this "
		          "recording is known not to replay faithfully, so a divergence may "
		          "be the replay rather than the restore.");
	} else {
		st.scripted = makeFallbackInput(kMaxTraceRows);
		fix = true;
		log::warn("Probe 4b: no recorded input, using synthetic pattern");
	}

	// The macro's granularity depends on this. Leaving it at the default meant
	// each update() covered two physics steps, stretching all input timing and
	// killing the run before the anchor was ever reached.
	g_config.physicsFix = fix;

	PlayLayer::get()->m_isPracticeMode = true; // checkpoint respawn requires it

	// Restores were sound at step 480 yet verification still failed at 18855,
	// so soundness must be checked where it breaks, not only at the opening.
	if (st.lastResyncFailStep > 400) {
		// Anchor just BEFORE the failure so the divergence happens inside the
		// measured window. The resync failure localised the drift to a 183-step
		// window at the cube->ship transition.
		st.restoreAnchor = std::max(240, st.lastResyncFailStep - 300);
		st.restoreLen    = 400;
		log::info("Probe 4b: anchoring at {} (resync failed at {} - the mode "
		          "transition window)", st.restoreAnchor, st.lastResyncFailStep);
	} else if (st.lastVerifyFailStep > st.restoreLen + 240) {
		st.restoreAnchor = st.lastVerifyFailStep - st.restoreLen - 120;
		log::info("Probe 4b: anchoring at {} (just before the last verification "
		          "failure at {})", st.restoreAnchor, st.lastVerifyFailStep);
	}

	st.restoreKind  = kind;
	st.mode         = Mode::RestoreTest;
	st.restorePhase = 0;
	st.segmentA.reserve(kMaxTraceRows);
	st.segmentB.reserve(kMaxTraceRows);
	st.releaseCheckpoints();
	st.resetPending = true;

	log::info("Probe 4b: restore fidelity, mechanism = {}. Anchor at step {}, "
	          "{} steps per segment.",
	          restoreKindName(kind), st.restoreAnchor, st.restoreLen);
}

void finishRestoreTest() {
	auto& st = ProbeState::get();

	const uint64_t hA = st.segmentA.hash();
	const uint64_t hB = st.segmentB.hash();
	const size_t   at = st.segmentA.firstDivergenceMasked(st.segmentB, probe::kInputSourceMask);

	log::info("================ Probe 4b: restore fidelity ================");
	log::info("  mechanism: {}", restoreKindName(st.restoreKind));
	log::info("  segment A (original): {} rows, hash {:016X}", st.segmentA.size(), hA);
	log::info("  segment B (restored): {} rows, hash {:016X}", st.segmentB.size(), hB);

	if (at == SIZE_MAX && st.segmentA.size() == st.segmentB.size()) {
		log::info("  RESTORE IS SOUND - {} steps after restore are bit-identical.",
		          st.segmentA.size());
	} else {
		log::error("  RESTORE IS NOT SOUND - diverges {} step(s) after the restore point.", at);
		std::string base = st.restoreKind == RestoreKind::Full ? "probe4b_full" : "probe4b_playeronly";
		st.segmentA.writeCsv(probe::outputPath(base + "_A.csv"));
		st.segmentB.writeCsv(probe::outputPath(base + "_B.csv"));
		log::error("  segments written to {}_A.csv / _B.csv for diffing", base);
	}
	log::info("===========================================================");

	st.releaseCheckpoints();
	st.mode = Mode::Idle;
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

// Total wall-clock budget per phase. The first version ran a flat 1000
// restores and hung an object-heavy demon for ~94 seconds, long enough for
// Windows to mark the process Not Responding. Each phase now stops early once
// it has spent this long, so cost scales the sample count down automatically.
constexpr double kProbe5BudgetUs = 4'000'000.0; // 4 s

void logStats(const char* label, std::vector<double>& us) {
	const probe::Stats s = probe::summarize(us);
	log::info("  {:<20} n={:<5} min {:.1f}us  median {:.1f}us  p99 {:.1f}us  max {:.1f}us",
	          label, s.count, s.min, s.median, s.p99, s.max);
}

// Bucketed means answer "uniform or degrading?" directly, instead of leaving it
// to be inferred from the gap between min and median.
void logTrend(const char* label, std::vector<double> const& us) {
	if (us.size() < 10) return;
	const size_t buckets = 5;
	const size_t per     = us.size() / buckets;
	std::string line;
	for (size_t b = 0; b < buckets; b++) {
		double sum = 0.0;
		for (size_t i = b * per; i < (b + 1) * per; i++) sum += us[i];
		line += fmt::format("{:.0f}us ", sum / static_cast<double>(per));
		if (b + 1 < buckets) line += "-> ";
	}
	const double first = us.front(), last = us.back();
	log::info("  {} trend over time: {}", label, line);
	log::info("  {} first={:.0f}us last={:.0f}us ratio={:.2f}x -> {}",
	          label, first, last, first > 0 ? last / first : 0.0,
	          (first > 0 && last / first > 1.5) ? "DEGRADING" : "uniform");
}

void runProbe5(int count) {
	auto* pl = PlayLayer::get();
	if (!pl || !pl->m_player1) {
		log::warn("Probe 5: not in an active level");
		return;
	}

	log::info("Probe 5: savestate cost, up to {} iterations per phase "
	          "({:.0f}s budget each)...", count, kProbe5BudgetUs / 1e6);
	log::warn("Probe 5: DISABLE other mods that hook createCheckpoint or "
	          "loadFromCheckpoint (QOLMod does) - Geode chains hooks, so their "
	          "code runs inside every timed call.");

	// --- Phase A: capture cost, with NO accumulation ---------------------
	// Create and immediately release, so this measures capture alone rather
	// than capture-while-N-checkpoints-are-live.
	std::vector<double> createUs;
	createUs.reserve(count);
	{
		const uint64_t start = probe::nowTicks();
		for (int i = 0; i < count; i++) {
			const uint64_t t0 = probe::nowTicks();
			CheckpointObject* cp = pl->createCheckpoint();
			const uint64_t t1 = probe::nowTicks();
			createUs.push_back(probe::ticksToMicros(t1 - t0));
			if (cp) { cp->retain(); cp->release(); }
			if (probe::ticksToMicros(probe::nowTicks() - start) > kProbe5BudgetUs) break;
		}
	}
	logStats("createCheckpoint", createUs);

	// --- Phase A2: player-only capture/restore ---------------------------
	// PlayLayer::createCheckpoint bundles player state WITH level state
	// (object states, effect manager, sequence triggers). The player-only pair
	// skips all of that, so it should be O(1) in object count. Correctness is
	// a separate question - see the restore-fidelity test (F12).
	if (auto* p = pl->m_player1) {
		if (PlayerCheckpoint* pc = PlayerCheckpoint::create()) {
			pc->retain();

			std::vector<double> saveUs, restoreUs;
			saveUs.reserve(count);
			restoreUs.reserve(count);

			uint64_t start = probe::nowTicks();
			for (int i = 0; i < count; i++) {
				const uint64_t t0 = probe::nowTicks();
				p->saveToCheckpoint(pc);
				const uint64_t t1 = probe::nowTicks();
				saveUs.push_back(probe::ticksToMicros(t1 - t0));
				if (probe::ticksToMicros(probe::nowTicks() - start) > kProbe5BudgetUs) break;
			}

			start = probe::nowTicks();
			for (int i = 0; i < count; i++) {
				const uint64_t t0 = probe::nowTicks();
				p->loadFromCheckpoint(pc);
				const uint64_t t1 = probe::nowTicks();
				restoreUs.push_back(probe::ticksToMicros(t1 - t0));
				if (probe::ticksToMicros(probe::nowTicks() - start) > kProbe5BudgetUs) break;
			}

			const probe::Stats ss = probe::summarize(saveUs);
			const probe::Stats rs = probe::summarize(restoreUs);
			logStats("player saveToCP", saveUs);
			logStats("player loadFromCP", restoreUs);
			log::info("  player-only restore median {:.3f} ms", rs.median / 1000.0);
			(void)ss;

			pc->release();
		}
	}

	// --- Phase B: restore cost against exactly ONE live checkpoint -------
	// This is the number that gates the architecture, and the condition a real
	// search actually runs under.
	std::vector<double> loadUs;
	loadUs.reserve(count);
	if (CheckpointObject* cp = pl->createCheckpoint()) {
		cp->retain();
		const uint64_t start = probe::nowTicks();
		for (int i = 0; i < count; i++) {
			const uint64_t t0 = probe::nowTicks();
			pl->loadFromCheckpoint(cp);
			const uint64_t t1 = probe::nowTicks();
			loadUs.push_back(probe::ticksToMicros(t1 - t0));
			if (probe::ticksToMicros(probe::nowTicks() - start) > kProbe5BudgetUs) break;
		}
		cp->release();
	}
	std::vector<double> loadOrdered = loadUs; // keep chronological order for the trend
	logStats("loadFromCheckpoint", loadUs);
	logTrend("loadFromCheckpoint", loadOrdered);

	const probe::Stats ls = probe::summarize(loadUs);

	// --- Phase C: memory scaling, measured separately --------------------
	{
		std::vector<CheckpointObject*> kept;
		const int memN = 200;
		kept.reserve(memN);
		const size_t memStart = processMemoryBytes();
		size_t memAt10 = 0, memAt100 = 0;
		for (int i = 0; i < memN; i++) {
			if (CheckpointObject* cp = pl->createCheckpoint()) { cp->retain(); kept.push_back(cp); }
			if (i == 9)  memAt10  = processMemoryBytes();
			if (i == 99) memAt100 = processMemoryBytes();
		}
		const size_t memEnd = processMemoryBytes();
		const double perState = kept.size() > 1
			? static_cast<double>(memEnd - memStart) / static_cast<double>(kept.size()) : 0.0;
		log::info("  memory: start {} KB, at 10 {} KB, at 100 {} KB, at {} {} KB",
		          memStart / 1024, memAt10 / 1024, memAt100 / 1024, kept.size(), memEnd / 1024);
		log::info("  marginal per live checkpoint: ~{:.0f} bytes ({:.1f} KB)",
		          perState, perState / 1024.0);
		for (auto* c : kept) c->release();
	}

	// Gate from the plan: <1ms viable, 1-10ms constrains the search, >10ms
	// forces an architecture rethink.
	const char* verdict = ls.median < 1000.0  ? "VIABLE (<1ms): savestate search is cheap"
	                    : ls.median < 10000.0 ? "CONSTRAINED (1-10ms): search must minimise restores"
	                                          : "PROBLEM (>10ms): architecture rethink";
	log::info("  restore median {:.3f} ms -> {}", ls.median / 1000.0, verdict);
	log::info("Probe 5: done");
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

	// Config (c), forced seeds, is retired: m_randomSeed differs on every reset
	// and ten runs under ten different seeds were byte-identical, so RNG is
	// proven irrelevant to physics.
	if (keyPressedEdge(VK_F1)) startDeterminismSweep(false, false); // vanilla dt

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

	// Player-only checkpoints are deliberately NOT offered. PlayerCheckpoint
	// carries no level state, so restoring one discards trigger and
	// moving-object state - fine on a 2013 level, wrong everywhere this project
	// is aiming. The full CheckpointObject path is the only correct one.

	if (keyPressedEdge(VK_F8)) {
		g_config.resyncEnabled = !g_config.resyncEnabled;
		log::info("Solver: periodic resync {}", g_config.resyncEnabled ? "ENABLED" : "disabled");
	}

	if (keyPressedEdge(VK_F2)) {
		auto& sv = Solver::get();
		if (sv.running) {
			log::info("Solver: stopped by user at best {:.2f}%", sv.bestPct);
			solverWriteMacro("partial.txt");
			sv.clear();
			st.mode = Mode::Idle;
		} else if (PlayLayer::get()) {
			g_config.physicsFix = true; // the solver requires fixed-dt stepping
			g_config.toggleBudget = 1;  // deepening is per-search, not global
			g_config.noSavestates = false; // re-enabled below if a verified prefix exists

			// Practice mode is what makes resetLevel respawn to a checkpoint
			// instead of the level start, which is the only revive path we have.
			PlayLayer::get()->m_isPracticeMode = true;

			sv.clear();
			sv.running    = true;
			sv.startTicks = probe::nowTicks();
			sv.lastReport = sv.startTicks;
			st.mode       = Mode::Solve;
			st.resetPending = true;

			if (!st.verifiedPrefix.empty()) {
				sv.resyncMacro  = st.verifiedPrefix;
				sv.resyncTarget = static_cast<int>(st.verifiedPrefix.size());
				sv.resyncing    = true;
				sv.step         = 0;
				sv.hold         = false;
				g_config.noSavestates = true;
				log::info("Solver: starting from a VERIFIED prefix of {} steps. Searching "
				          "forward WITHOUT savestates - every branch replays from frame 0, "
				          "so the result is exact by construction.", sv.resyncTarget);
			}
			log::info("Solver: starting DFS. air branch interval {} steps, "
			          "release-before-hold ordering. F2 again to stop.",
			          g_config.airBranchInterval);
		}
	}

	if (keyPressedEdge(VK_F11)) runProbe5(1000);
	if (keyPressedEdge(VK_F12)) startRestoreTest(RestoreKind::Full);
	if (keyPressedEdge(VK_F4))  startVerify(false); // normal mode
	if (keyPressedEdge(VK_F3))  startVerify(true);  // practice mode

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

	// Advances the Probe 4b state machine one step. Returns false when the
	// enclosing per-frame step loop should stop early (a restore just moved the
	// player, so continuing to step this frame would run from the wrong state).
	bool driveRestoreTest() {
		auto& st = ProbeState::get();
		auto* pl = PlayLayer::get();
		if (!pl || !m_player1) return true;

		const int since = st.stepCounter - st.restoreAnchor;

		if (st.restorePhase == 0) {
			if (st.stepCounter < st.restoreAnchor) return true;

			// Reached the anchor: capture with BOTH mechanisms so the same run
			// can be replayed against either.
			if (CheckpointObject* cp = pl->createCheckpoint()) { cp->retain(); st.fullCp = cp; }
			if (PlayerCheckpoint* pc = PlayerCheckpoint::create()) {
				pc->retain();
				m_player1->saveToCheckpoint(pc);
				st.playerCp = pc;
			}
			st.restorePhase = 1;
			// Snapshot the exact state at capture time, so the state after
			// restore+realign can be compared against it field by field. This
			// separates "the restore lands somewhere else" from "the restore is
			// right but the next step differs".
			if (st.trace.size() > 0) {
				st.anchorRow = st.trace[st.trace.size() - 1];
				st.haveAnchorRow = true;
			}
			if (m_player1) {
				const auto* raw = reinterpret_cast<const uint8_t*>(m_player1);
				st.anchorBytes.assign(raw, raw + sizeof(PlayerObject));
				st.anchorGameModeChangedTime = m_player1->m_gameModeChangedTime;
				st.anchorUnkA29              = m_player1->m_unkA29;
				// sizeof(GJBaseGameLayer) is 14240 but PlayLayer is larger, so
				// everything in PlayLayer's own region - m_attemptTime among it -
				// was never being compared at all.
				const auto* lraw = reinterpret_cast<const uint8_t*>(PlayLayer::get());
				st.anchorLayerBytes.assign(lraw, lraw + sizeof(PlayLayer));
				st.aExtraDelta  = m_extraDelta;
				st.aTimePlayed  = m_timePlayed;
				st.aTimestamp   = m_timestamp;
				st.aTickIndex   = m_tickIndex;
				st.aClickIndex  = m_clickIndex;
				st.aResumeTimer = m_resumeTimer;
				st.aJumping     = m_jumping;
				if (auto* pl2 = PlayLayer::get()) {
					st.aAttemptTime     = pl2->m_attemptTime;
					st.aBestAttemptTime = pl2->m_bestAttemptTime;
					st.aCurrentTime     = pl2->m_currentTime;
					st.aHasJumped       = pl2->m_hasJumped;
				}
			}
			log::info("Probe 4b: anchored at step {}", st.stepCounter);
			return true;
		}

		// Record the segment currently being simulated.
		probe::Trace& seg = (st.restorePhase == 1) ? st.segmentA : st.segmentB;
		if (!st.trace.size()) return true;
		seg.push(st.trace[st.trace.size() - 1]);

		if (since < st.restoreLen) return true;

		if (st.restorePhase == 1) {
			// Restore and replay the identical input from the anchor.
			if (st.restoreKind == RestoreKind::Full) {
				// The practice-respawn path, exactly as the solver restores.
				// Testing loadFromCheckpoint instead would measure a path the
				// solver never takes.
				const size_t prev = st.restoreAnchor > 0
				                  ? static_cast<size_t>(st.restoreAnchor - 1) : 0;
				const bool realign = prev < st.scripted.size() && st.scripted[prev] != 0;
				// Exercise the same extra-state restoration the solver uses.
				Solver::PendingExtra e;
				e.gameModeChangedTime = st.anchorGameModeChangedTime;
				e.unkA29      = st.anchorUnkA29;
				e.extraDelta  = st.aExtraDelta;
				e.timePlayed  = st.aTimePlayed;
				e.timestamp   = st.aTimestamp;
				e.tickIndex   = st.aTickIndex;
				e.clickIndex  = st.aClickIndex;
				e.resumeTimer = st.aResumeTimer;
				e.jumping     = st.aJumping;
				e.attemptTime     = st.aAttemptTime;
				e.bestAttemptTime = st.aBestAttemptTime;
				e.currentTime     = st.aCurrentTime;
				e.hasJumped       = st.aHasJumped;
				e.valid       = true;
				Solver::get().pendingExtra = e;
				solverRestoreCheckpoint(st.fullCp, realign);
			} else {
				if (st.playerCp) m_player1->loadFromCheckpoint(st.playerCp);
			}
			st.stepCounter  = st.restoreAnchor;
			st.restorePhase = 2;

			// Compare the restored state against the captured one directly.
			if (st.haveAnchorRow && m_player1) {
				auto const& a = st.anchorRow;
				auto* p = m_player1;
				auto* pl2 = PlayLayer::get();
				const float  x  = p->getPositionX(),  y = p->getPositionY();
				const float  r  = p->getRotation();
				const double vy = p->m_yVelocity,     g = p->m_gravity;
				const bool same =
					probe::bits(x)  == a.x  && probe::bits(y) == a.y &&
					probe::bits(r)  == a.rotation &&
					probe::bits(vy) == a.yVelocity && probe::bits(g) == a.gravity;
				log::info("Probe 4b: state at restore boundary is {}", same ? "EXACT" : "DIFFERENT");
				log::info("  captured : x={:.6f} y={:.6f} rot={:.6f} yVel={:.6f} onGround={} ship={}",
				          probe::asFloat(a.x), probe::asFloat(a.y), probe::asFloat(a.rotation),
				          probe::asDouble(a.yVelocity),
				          (a.flags & probe::FlagOnGround) ? 1 : 0,
				          (a.flags & probe::FlagShip) ? 1 : 0);
				log::info("  restored : x={:.6f} y={:.6f} rot={:.6f} yVel={:.6f} onGround={} ship={}",
				          x, y, r, vy, p->m_isOnGround ? 1 : 0, p->m_isShip ? 1 : 0);
				log::info("  holdingJump={}  jumpBuffered={}  percent={:.4f}",
				          ProbeState::get().isHolding ? 1 : 0, p->m_jumpBuffered ? 1 : 0,
				          pl2 ? pl2->getCurrentPercent() : 0.f);
			}

			// Probe 4a: which bytes of PlayerObject did the restore fail to bring
			// back? Reported as ranges with the nearest known field, so the
			// answer is a field list rather than a guess.
			if (!st.anchorBytes.empty() && m_player1) {
				const auto* now = reinterpret_cast<const uint8_t*>(m_player1);
				struct FieldRef { const char* name; size_t off; size_t size; };
				// Generated from bindings/2.2081 - every named PlayerObject member,
				// so a differing byte range is reported as a FIELD NAME rather than
				// a bare offset I then have to guess at.
				static const FieldRef fields[] = {
					{"m_mainLayer", offsetof(PlayerObject, m_mainLayer), sizeof(PlayerObject::m_mainLayer)},
					{"m_wasTeleported", offsetof(PlayerObject, m_wasTeleported), sizeof(PlayerObject::m_wasTeleported)},
					{"m_fixGravityBug", offsetof(PlayerObject, m_fixGravityBug), sizeof(PlayerObject::m_fixGravityBug)},
					{"m_reverseSync", offsetof(PlayerObject, m_reverseSync), sizeof(PlayerObject::m_reverseSync)},
					{"m_yVelocityBeforeSlope", offsetof(PlayerObject, m_yVelocityBeforeSlope), sizeof(PlayerObject::m_yVelocityBeforeSlope)},
					{"m_dashX", offsetof(PlayerObject, m_dashX), sizeof(PlayerObject::m_dashX)},
					{"m_dashY", offsetof(PlayerObject, m_dashY), sizeof(PlayerObject::m_dashY)},
					{"m_dashAngle", offsetof(PlayerObject, m_dashAngle), sizeof(PlayerObject::m_dashAngle)},
					{"m_dashStartTime", offsetof(PlayerObject, m_dashStartTime), sizeof(PlayerObject::m_dashStartTime)},
					{"m_dashRing", offsetof(PlayerObject, m_dashRing), sizeof(PlayerObject::m_dashRing)},
					{"m_slopeStartTime", offsetof(PlayerObject, m_slopeStartTime), sizeof(PlayerObject::m_slopeStartTime)},
					{"m_justPlacedStreak", offsetof(PlayerObject, m_justPlacedStreak), sizeof(PlayerObject::m_justPlacedStreak)},
					{"m_maybeLastGroundObject", offsetof(PlayerObject, m_maybeLastGroundObject), sizeof(PlayerObject::m_maybeLastGroundObject)},
					{"m_collisionLogTop", offsetof(PlayerObject, m_collisionLogTop), sizeof(PlayerObject::m_collisionLogTop)},
					{"m_collisionLogBottom", offsetof(PlayerObject, m_collisionLogBottom), sizeof(PlayerObject::m_collisionLogBottom)},
					{"m_collisionLogLeft", offsetof(PlayerObject, m_collisionLogLeft), sizeof(PlayerObject::m_collisionLogLeft)},
					{"m_collisionLogRight", offsetof(PlayerObject, m_collisionLogRight), sizeof(PlayerObject::m_collisionLogRight)},
					{"m_lastCollisionBottom", offsetof(PlayerObject, m_lastCollisionBottom), sizeof(PlayerObject::m_lastCollisionBottom)},
					{"m_lastCollisionTop", offsetof(PlayerObject, m_lastCollisionTop), sizeof(PlayerObject::m_lastCollisionTop)},
					{"m_lastCollisionLeft", offsetof(PlayerObject, m_lastCollisionLeft), sizeof(PlayerObject::m_lastCollisionLeft)},
					{"m_lastCollisionRight", offsetof(PlayerObject, m_lastCollisionRight), sizeof(PlayerObject::m_lastCollisionRight)},
					{"m_unk50C", offsetof(PlayerObject, m_unk50C), sizeof(PlayerObject::m_unk50C)},
					{"m_unk510", offsetof(PlayerObject, m_unk510), sizeof(PlayerObject::m_unk510)},
					{"m_currentSlope2", offsetof(PlayerObject, m_currentSlope2), sizeof(PlayerObject::m_currentSlope2)},
					{"m_preLastGroundObject", offsetof(PlayerObject, m_preLastGroundObject), sizeof(PlayerObject::m_preLastGroundObject)},
					{"m_slopeAngle", offsetof(PlayerObject, m_slopeAngle), sizeof(PlayerObject::m_slopeAngle)},
					{"m_slopeSlidingMaybeRotated", offsetof(PlayerObject, m_slopeSlidingMaybeRotated), sizeof(PlayerObject::m_slopeSlidingMaybeRotated)},
					{"m_quickCheckpointMode", offsetof(PlayerObject, m_quickCheckpointMode), sizeof(PlayerObject::m_quickCheckpointMode)},
					{"m_collidedObject", offsetof(PlayerObject, m_collidedObject), sizeof(PlayerObject::m_collidedObject)},
					{"m_lastGroundObject", offsetof(PlayerObject, m_lastGroundObject), sizeof(PlayerObject::m_lastGroundObject)},
					{"m_collidingWithLeft", offsetof(PlayerObject, m_collidingWithLeft), sizeof(PlayerObject::m_collidingWithLeft)},
					{"m_collidingWithRight", offsetof(PlayerObject, m_collidingWithRight), sizeof(PlayerObject::m_collidingWithRight)},
					{"m_maybeSavedPlayerFrame", offsetof(PlayerObject, m_maybeSavedPlayerFrame), sizeof(PlayerObject::m_maybeSavedPlayerFrame)},
					{"m_scaleXRelated2", offsetof(PlayerObject, m_scaleXRelated2), sizeof(PlayerObject::m_scaleXRelated2)},
					{"m_groundYVelocity", offsetof(PlayerObject, m_groundYVelocity), sizeof(PlayerObject::m_groundYVelocity)},
					{"m_yVelocityRelated", offsetof(PlayerObject, m_yVelocityRelated), sizeof(PlayerObject::m_yVelocityRelated)},
					{"m_scaleXRelated3", offsetof(PlayerObject, m_scaleXRelated3), sizeof(PlayerObject::m_scaleXRelated3)},
					{"m_scaleXRelated4", offsetof(PlayerObject, m_scaleXRelated4), sizeof(PlayerObject::m_scaleXRelated4)},
					{"m_scaleXRelated5", offsetof(PlayerObject, m_scaleXRelated5), sizeof(PlayerObject::m_scaleXRelated5)},
					{"m_isCollidingWithSlope", offsetof(PlayerObject, m_isCollidingWithSlope), sizeof(PlayerObject::m_isCollidingWithSlope)},
					{"m_dashFireSprite", offsetof(PlayerObject, m_dashFireSprite), sizeof(PlayerObject::m_dashFireSprite)},
					{"m_isBallRotating", offsetof(PlayerObject, m_isBallRotating), sizeof(PlayerObject::m_isBallRotating)},
					{"m_unk669", offsetof(PlayerObject, m_unk669), sizeof(PlayerObject::m_unk669)},
					{"m_currentPotentialSlope", offsetof(PlayerObject, m_currentPotentialSlope), sizeof(PlayerObject::m_currentPotentialSlope)},
					{"m_currentSlope", offsetof(PlayerObject, m_currentSlope), sizeof(PlayerObject::m_currentSlope)},
					{"m_collidingWithSlopeId", offsetof(PlayerObject, m_collidingWithSlopeId), sizeof(PlayerObject::m_collidingWithSlopeId)},
					{"m_slopeFlipGravityRelated", offsetof(PlayerObject, m_slopeFlipGravityRelated), sizeof(PlayerObject::m_slopeFlipGravityRelated)},
					{"m_particleSystems", offsetof(PlayerObject, m_particleSystems), sizeof(PlayerObject::m_particleSystems)},
					{"m_slopeAngleRadians", offsetof(PlayerObject, m_slopeAngleRadians), sizeof(PlayerObject::m_slopeAngleRadians)},
					{"m_rotateObjectsRelated", offsetof(PlayerObject, m_rotateObjectsRelated), sizeof(PlayerObject::m_rotateObjectsRelated)},
					{"m_potentialSlopeMap", offsetof(PlayerObject, m_potentialSlopeMap), sizeof(PlayerObject::m_potentialSlopeMap)},
					{"m_rotationSpeed", offsetof(PlayerObject, m_rotationSpeed), sizeof(PlayerObject::m_rotationSpeed)},
					{"m_rotateSpeed", offsetof(PlayerObject, m_rotateSpeed), sizeof(PlayerObject::m_rotateSpeed)},
					{"m_isRotating", offsetof(PlayerObject, m_isRotating), sizeof(PlayerObject::m_isRotating)},
					{"m_isBallRotating2", offsetof(PlayerObject, m_isBallRotating2), sizeof(PlayerObject::m_isBallRotating2)},
					{"m_hasGlow", offsetof(PlayerObject, m_hasGlow), sizeof(PlayerObject::m_hasGlow)},
					{"m_isHidden", offsetof(PlayerObject, m_isHidden), sizeof(PlayerObject::m_isHidden)},
					{"m_ghostType", offsetof(PlayerObject, m_ghostType), sizeof(PlayerObject::m_ghostType)},
					{"m_ghostTrail", offsetof(PlayerObject, m_ghostTrail), sizeof(PlayerObject::m_ghostTrail)},
					{"m_iconSprite", offsetof(PlayerObject, m_iconSprite), sizeof(PlayerObject::m_iconSprite)},
					{"m_iconSpriteSecondary", offsetof(PlayerObject, m_iconSpriteSecondary), sizeof(PlayerObject::m_iconSpriteSecondary)},
					{"m_iconSpriteWhitener", offsetof(PlayerObject, m_iconSpriteWhitener), sizeof(PlayerObject::m_iconSpriteWhitener)},
					{"m_iconGlow", offsetof(PlayerObject, m_iconGlow), sizeof(PlayerObject::m_iconGlow)},
					{"m_vehicleSprite", offsetof(PlayerObject, m_vehicleSprite), sizeof(PlayerObject::m_vehicleSprite)},
					{"m_vehicleSpriteSecondary", offsetof(PlayerObject, m_vehicleSpriteSecondary), sizeof(PlayerObject::m_vehicleSpriteSecondary)},
					{"m_birdVehicle", offsetof(PlayerObject, m_birdVehicle), sizeof(PlayerObject::m_birdVehicle)},
					{"m_vehicleSpriteWhitener", offsetof(PlayerObject, m_vehicleSpriteWhitener), sizeof(PlayerObject::m_vehicleSpriteWhitener)},
					{"m_vehicleGlow", offsetof(PlayerObject, m_vehicleGlow), sizeof(PlayerObject::m_vehicleGlow)},
					{"m_swingFireMiddle", offsetof(PlayerObject, m_swingFireMiddle), sizeof(PlayerObject::m_swingFireMiddle)},
					{"m_swingFireBottom", offsetof(PlayerObject, m_swingFireBottom), sizeof(PlayerObject::m_swingFireBottom)},
					{"m_swingFireTop", offsetof(PlayerObject, m_swingFireTop), sizeof(PlayerObject::m_swingFireTop)},
					{"m_dashSpritesContainer", offsetof(PlayerObject, m_dashSpritesContainer), sizeof(PlayerObject::m_dashSpritesContainer)},
					{"m_regularTrail", offsetof(PlayerObject, m_regularTrail), sizeof(PlayerObject::m_regularTrail)},
					{"m_shipStreak", offsetof(PlayerObject, m_shipStreak), sizeof(PlayerObject::m_shipStreak)},
					{"m_waveTrail", offsetof(PlayerObject, m_waveTrail), sizeof(PlayerObject::m_waveTrail)},
					{"m_speedMultiplier", offsetof(PlayerObject, m_speedMultiplier), sizeof(PlayerObject::m_speedMultiplier)},
					{"m_yStart", offsetof(PlayerObject, m_yStart), sizeof(PlayerObject::m_yStart)},
					{"m_gravity", offsetof(PlayerObject, m_gravity), sizeof(PlayerObject::m_gravity)},
					{"m_trailingParticleLife", offsetof(PlayerObject, m_trailingParticleLife), sizeof(PlayerObject::m_trailingParticleLife)},
					{"m_unk648", offsetof(PlayerObject, m_unk648), sizeof(PlayerObject::m_unk648)},
					{"m_gameModeChangedTime", offsetof(PlayerObject, m_gameModeChangedTime), sizeof(PlayerObject::m_gameModeChangedTime)},
					{"m_padRingRelated", offsetof(PlayerObject, m_padRingRelated), sizeof(PlayerObject::m_padRingRelated)},
					{"m_maybeReducedEffects", offsetof(PlayerObject, m_maybeReducedEffects), sizeof(PlayerObject::m_maybeReducedEffects)},
					{"m_maybeIsFalling", offsetof(PlayerObject, m_maybeIsFalling), sizeof(PlayerObject::m_maybeIsFalling)},
					{"m_shouldTryPlacingCheckpoint", offsetof(PlayerObject, m_shouldTryPlacingCheckpoint), sizeof(PlayerObject::m_shouldTryPlacingCheckpoint)},
					{"m_playEffects", offsetof(PlayerObject, m_playEffects), sizeof(PlayerObject::m_playEffects)},
					{"m_maybeCanRunIntoBlocks", offsetof(PlayerObject, m_maybeCanRunIntoBlocks), sizeof(PlayerObject::m_maybeCanRunIntoBlocks)},
					{"m_hasGroundParticles", offsetof(PlayerObject, m_hasGroundParticles), sizeof(PlayerObject::m_hasGroundParticles)},
					{"m_hasShipParticles", offsetof(PlayerObject, m_hasShipParticles), sizeof(PlayerObject::m_hasShipParticles)},
					{"m_isOnGround3", offsetof(PlayerObject, m_isOnGround3), sizeof(PlayerObject::m_isOnGround3)},
					{"m_checkpointTimeout", offsetof(PlayerObject, m_checkpointTimeout), sizeof(PlayerObject::m_checkpointTimeout)},
					{"m_lastCheckpointTime", offsetof(PlayerObject, m_lastCheckpointTime), sizeof(PlayerObject::m_lastCheckpointTime)},
					{"m_lastJumpTime", offsetof(PlayerObject, m_lastJumpTime), sizeof(PlayerObject::m_lastJumpTime)},
					{"m_lastFlipTime", offsetof(PlayerObject, m_lastFlipTime), sizeof(PlayerObject::m_lastFlipTime)},
					{"m_flashTime", offsetof(PlayerObject, m_flashTime), sizeof(PlayerObject::m_flashTime)},
					{"m_flashDuration", offsetof(PlayerObject, m_flashDuration), sizeof(PlayerObject::m_flashDuration)},
					{"m_flashDelay", offsetof(PlayerObject, m_flashDelay), sizeof(PlayerObject::m_flashDelay)},
					{"m_flashMainColor", offsetof(PlayerObject, m_flashMainColor), sizeof(PlayerObject::m_flashMainColor)},
					{"m_flashSecondColor", offsetof(PlayerObject, m_flashSecondColor), sizeof(PlayerObject::m_flashSecondColor)},
					{"m_lastSpiderFlipTime", offsetof(PlayerObject, m_lastSpiderFlipTime), sizeof(PlayerObject::m_lastSpiderFlipTime)},
					{"m_unkBool5", offsetof(PlayerObject, m_unkBool5), sizeof(PlayerObject::m_unkBool5)},
					{"m_maybeIsVehicleGlowing", offsetof(PlayerObject, m_maybeIsVehicleGlowing), sizeof(PlayerObject::m_maybeIsVehicleGlowing)},
					{"m_switchWaveTrailColor", offsetof(PlayerObject, m_switchWaveTrailColor), sizeof(PlayerObject::m_switchWaveTrailColor)},
					{"m_practiceDeathEffect", offsetof(PlayerObject, m_practiceDeathEffect), sizeof(PlayerObject::m_practiceDeathEffect)},
					{"m_accelerationOrSpeed", offsetof(PlayerObject, m_accelerationOrSpeed), sizeof(PlayerObject::m_accelerationOrSpeed)},
					{"m_snapDistance", offsetof(PlayerObject, m_snapDistance), sizeof(PlayerObject::m_snapDistance)},
					{"m_ringJumpRelated", offsetof(PlayerObject, m_ringJumpRelated), sizeof(PlayerObject::m_ringJumpRelated)},
					{"m_ringRelatedSet", offsetof(PlayerObject, m_ringRelatedSet), sizeof(PlayerObject::m_ringRelatedSet)},
					{"m_objectSnappedTo", offsetof(PlayerObject, m_objectSnappedTo), sizeof(PlayerObject::m_objectSnappedTo)},
					{"m_pendingCheckpoint", offsetof(PlayerObject, m_pendingCheckpoint), sizeof(PlayerObject::m_pendingCheckpoint)},
					{"m_onFlyCheckpointTries", offsetof(PlayerObject, m_onFlyCheckpointTries), sizeof(PlayerObject::m_onFlyCheckpointTries)},
					{"m_robotSprite", offsetof(PlayerObject, m_robotSprite), sizeof(PlayerObject::m_robotSprite)},
					{"m_spiderSprite", offsetof(PlayerObject, m_spiderSprite), sizeof(PlayerObject::m_spiderSprite)},
					{"m_maybeSpriteRelated", offsetof(PlayerObject, m_maybeSpriteRelated), sizeof(PlayerObject::m_maybeSpriteRelated)},
					{"m_playerGroundParticles", offsetof(PlayerObject, m_playerGroundParticles), sizeof(PlayerObject::m_playerGroundParticles)},
					{"m_trailingParticles", offsetof(PlayerObject, m_trailingParticles), sizeof(PlayerObject::m_trailingParticles)},
					{"m_shipClickParticles", offsetof(PlayerObject, m_shipClickParticles), sizeof(PlayerObject::m_shipClickParticles)},
					{"m_vehicleGroundParticles", offsetof(PlayerObject, m_vehicleGroundParticles), sizeof(PlayerObject::m_vehicleGroundParticles)},
					{"m_ufoClickParticles", offsetof(PlayerObject, m_ufoClickParticles), sizeof(PlayerObject::m_ufoClickParticles)},
					{"m_robotBurstParticles", offsetof(PlayerObject, m_robotBurstParticles), sizeof(PlayerObject::m_robotBurstParticles)},
					{"m_dashParticles", offsetof(PlayerObject, m_dashParticles), sizeof(PlayerObject::m_dashParticles)},
					{"m_swingBurstParticles1", offsetof(PlayerObject, m_swingBurstParticles1), sizeof(PlayerObject::m_swingBurstParticles1)},
					{"m_swingBurstParticles2", offsetof(PlayerObject, m_swingBurstParticles2), sizeof(PlayerObject::m_swingBurstParticles2)},
					{"m_useLandParticles0", offsetof(PlayerObject, m_useLandParticles0), sizeof(PlayerObject::m_useLandParticles0)},
					{"m_landParticles0", offsetof(PlayerObject, m_landParticles0), sizeof(PlayerObject::m_landParticles0)},
					{"m_landParticles1", offsetof(PlayerObject, m_landParticles1), sizeof(PlayerObject::m_landParticles1)},
					{"m_landParticlesAngle", offsetof(PlayerObject, m_landParticlesAngle), sizeof(PlayerObject::m_landParticlesAngle)},
					{"m_landParticleRelatedY", offsetof(PlayerObject, m_landParticleRelatedY), sizeof(PlayerObject::m_landParticleRelatedY)},
					{"m_playerStreak", offsetof(PlayerObject, m_playerStreak), sizeof(PlayerObject::m_playerStreak)},
					{"m_streakStrokeWidth", offsetof(PlayerObject, m_streakStrokeWidth), sizeof(PlayerObject::m_streakStrokeWidth)},
					{"m_disableStreakTint", offsetof(PlayerObject, m_disableStreakTint), sizeof(PlayerObject::m_disableStreakTint)},
					{"m_alwaysShowStreak", offsetof(PlayerObject, m_alwaysShowStreak), sizeof(PlayerObject::m_alwaysShowStreak)},
					{"m_shipStreakType", offsetof(PlayerObject, m_shipStreakType), sizeof(PlayerObject::m_shipStreakType)},
					{"m_slopeRotation", offsetof(PlayerObject, m_slopeRotation), sizeof(PlayerObject::m_slopeRotation)},
					{"m_currentSlopeYVelocity", offsetof(PlayerObject, m_currentSlopeYVelocity), sizeof(PlayerObject::m_currentSlopeYVelocity)},
					{"m_unk3d0", offsetof(PlayerObject, m_unk3d0), sizeof(PlayerObject::m_unk3d0)},
					{"m_blackOrbRelated", offsetof(PlayerObject, m_blackOrbRelated), sizeof(PlayerObject::m_blackOrbRelated)},
					{"m_unk3e0", offsetof(PlayerObject, m_unk3e0), sizeof(PlayerObject::m_unk3e0)},
					{"m_unk3e1", offsetof(PlayerObject, m_unk3e1), sizeof(PlayerObject::m_unk3e1)},
					{"m_isAccelerating", offsetof(PlayerObject, m_isAccelerating), sizeof(PlayerObject::m_isAccelerating)},
					{"m_isCurrentSlopeTop", offsetof(PlayerObject, m_isCurrentSlopeTop), sizeof(PlayerObject::m_isCurrentSlopeTop)},
					{"m_collidedTopMinY", offsetof(PlayerObject, m_collidedTopMinY), sizeof(PlayerObject::m_collidedTopMinY)},
					{"m_collidedBottomMaxY", offsetof(PlayerObject, m_collidedBottomMaxY), sizeof(PlayerObject::m_collidedBottomMaxY)},
					{"m_collidedLeftMaxX", offsetof(PlayerObject, m_collidedLeftMaxX), sizeof(PlayerObject::m_collidedLeftMaxX)},
					{"m_collidedRightMinX", offsetof(PlayerObject, m_collidedRightMinX), sizeof(PlayerObject::m_collidedRightMinX)},
					{"m_fadeOutStreak", offsetof(PlayerObject, m_fadeOutStreak), sizeof(PlayerObject::m_fadeOutStreak)},
					{"m_canPlaceCheckpoint", offsetof(PlayerObject, m_canPlaceCheckpoint), sizeof(PlayerObject::m_canPlaceCheckpoint)},
					{"m_originalMainColor", offsetof(PlayerObject, m_originalMainColor), sizeof(PlayerObject::m_originalMainColor)},
					{"m_originalSecondColor", offsetof(PlayerObject, m_originalSecondColor), sizeof(PlayerObject::m_originalSecondColor)},
					{"m_hasCustomGlowColor", offsetof(PlayerObject, m_hasCustomGlowColor), sizeof(PlayerObject::m_hasCustomGlowColor)},
					{"m_glowColor", offsetof(PlayerObject, m_glowColor), sizeof(PlayerObject::m_glowColor)},
					{"m_maybeIsColliding", offsetof(PlayerObject, m_maybeIsColliding), sizeof(PlayerObject::m_maybeIsColliding)},
					{"m_jumpBuffered", offsetof(PlayerObject, m_jumpBuffered), sizeof(PlayerObject::m_jumpBuffered)},
					{"m_stateRingJump", offsetof(PlayerObject, m_stateRingJump), sizeof(PlayerObject::m_stateRingJump)},
					{"m_wasJumpBuffered", offsetof(PlayerObject, m_wasJumpBuffered), sizeof(PlayerObject::m_wasJumpBuffered)},
					{"m_wasRobotJump", offsetof(PlayerObject, m_wasRobotJump), sizeof(PlayerObject::m_wasRobotJump)},
					{"m_stateJumpBuffered", offsetof(PlayerObject, m_stateJumpBuffered), sizeof(PlayerObject::m_stateJumpBuffered)},
					{"m_stateRingJump2", offsetof(PlayerObject, m_stateRingJump2), sizeof(PlayerObject::m_stateRingJump2)},
					{"m_touchedRing", offsetof(PlayerObject, m_touchedRing), sizeof(PlayerObject::m_touchedRing)},
					{"m_touchedCustomRing", offsetof(PlayerObject, m_touchedCustomRing), sizeof(PlayerObject::m_touchedCustomRing)},
					{"m_touchedGravityPortal", offsetof(PlayerObject, m_touchedGravityPortal), sizeof(PlayerObject::m_touchedGravityPortal)},
					{"m_maybeTouchedBreakableBlock", offsetof(PlayerObject, m_maybeTouchedBreakableBlock), sizeof(PlayerObject::m_maybeTouchedBreakableBlock)},
					{"m_jumpRelatedAC2", offsetof(PlayerObject, m_jumpRelatedAC2), sizeof(PlayerObject::m_jumpRelatedAC2)},
					{"m_touchedPad", offsetof(PlayerObject, m_touchedPad), sizeof(PlayerObject::m_touchedPad)},
					{"m_yVelocity", offsetof(PlayerObject, m_yVelocity), sizeof(PlayerObject::m_yVelocity)},
					{"m_fallSpeed", offsetof(PlayerObject, m_fallSpeed), sizeof(PlayerObject::m_fallSpeed)},
					{"m_isOnSlope", offsetof(PlayerObject, m_isOnSlope), sizeof(PlayerObject::m_isOnSlope)},
					{"m_wasOnSlope", offsetof(PlayerObject, m_wasOnSlope), sizeof(PlayerObject::m_wasOnSlope)},
					{"m_slopeVelocity", offsetof(PlayerObject, m_slopeVelocity), sizeof(PlayerObject::m_slopeVelocity)},
					{"m_maybeUpsideDownSlope", offsetof(PlayerObject, m_maybeUpsideDownSlope), sizeof(PlayerObject::m_maybeUpsideDownSlope)},
					{"m_isShip", offsetof(PlayerObject, m_isShip), sizeof(PlayerObject::m_isShip)},
					{"m_isBird", offsetof(PlayerObject, m_isBird), sizeof(PlayerObject::m_isBird)},
					{"m_isBall", offsetof(PlayerObject, m_isBall), sizeof(PlayerObject::m_isBall)},
					{"m_isDart", offsetof(PlayerObject, m_isDart), sizeof(PlayerObject::m_isDart)},
					{"m_isRobot", offsetof(PlayerObject, m_isRobot), sizeof(PlayerObject::m_isRobot)},
					{"m_isSpider", offsetof(PlayerObject, m_isSpider), sizeof(PlayerObject::m_isSpider)},
					{"m_isUpsideDown", offsetof(PlayerObject, m_isUpsideDown), sizeof(PlayerObject::m_isUpsideDown)},
					{"m_isDead", offsetof(PlayerObject, m_isDead), sizeof(PlayerObject::m_isDead)},
					{"m_isOnGround", offsetof(PlayerObject, m_isOnGround), sizeof(PlayerObject::m_isOnGround)},
					{"m_isGoingLeft", offsetof(PlayerObject, m_isGoingLeft), sizeof(PlayerObject::m_isGoingLeft)},
					{"m_isSideways", offsetof(PlayerObject, m_isSideways), sizeof(PlayerObject::m_isSideways)},
					{"m_isSwing", offsetof(PlayerObject, m_isSwing), sizeof(PlayerObject::m_isSwing)},
					{"m_reverseRelated", offsetof(PlayerObject, m_reverseRelated), sizeof(PlayerObject::m_reverseRelated)},
					{"m_maybeReverseSpeed", offsetof(PlayerObject, m_maybeReverseSpeed), sizeof(PlayerObject::m_maybeReverseSpeed)},
					{"m_maybeReverseAcceleration", offsetof(PlayerObject, m_maybeReverseAcceleration), sizeof(PlayerObject::m_maybeReverseAcceleration)},
					{"m_xVelocityRelated2", offsetof(PlayerObject, m_xVelocityRelated2), sizeof(PlayerObject::m_xVelocityRelated2)},
					{"m_isDashing", offsetof(PlayerObject, m_isDashing), sizeof(PlayerObject::m_isDashing)},
					{"m_dashFireFrame", offsetof(PlayerObject, m_dashFireFrame), sizeof(PlayerObject::m_dashFireFrame)},
					{"m_groundObjectMaterial", offsetof(PlayerObject, m_groundObjectMaterial), sizeof(PlayerObject::m_groundObjectMaterial)},
					{"m_vehicleSize", offsetof(PlayerObject, m_vehicleSize), sizeof(PlayerObject::m_vehicleSize)},
					{"m_playerSpeed", offsetof(PlayerObject, m_playerSpeed), sizeof(PlayerObject::m_playerSpeed)},
					{"m_shipRotation", offsetof(PlayerObject, m_shipRotation), sizeof(PlayerObject::m_shipRotation)},
					{"m_lastPortalPos", offsetof(PlayerObject, m_lastPortalPos), sizeof(PlayerObject::m_lastPortalPos)},
					{"m_unkUnused3", offsetof(PlayerObject, m_unkUnused3), sizeof(PlayerObject::m_unkUnused3)},
					{"m_isOnGround2", offsetof(PlayerObject, m_isOnGround2), sizeof(PlayerObject::m_isOnGround2)},
					{"m_lastLandTime", offsetof(PlayerObject, m_lastLandTime), sizeof(PlayerObject::m_lastLandTime)},
					{"m_platformerVelocityRelated", offsetof(PlayerObject, m_platformerVelocityRelated), sizeof(PlayerObject::m_platformerVelocityRelated)},
					{"m_maybeIsBoosted", offsetof(PlayerObject, m_maybeIsBoosted), sizeof(PlayerObject::m_maybeIsBoosted)},
					{"m_scaleXRelatedTime", offsetof(PlayerObject, m_scaleXRelatedTime), sizeof(PlayerObject::m_scaleXRelatedTime)},
					{"m_decreaseBoostSlide", offsetof(PlayerObject, m_decreaseBoostSlide), sizeof(PlayerObject::m_decreaseBoostSlide)},
					{"m_unkA29", offsetof(PlayerObject, m_unkA29), sizeof(PlayerObject::m_unkA29)},
					{"m_isLocked", offsetof(PlayerObject, m_isLocked), sizeof(PlayerObject::m_isLocked)},
					{"m_controlsDisabled", offsetof(PlayerObject, m_controlsDisabled), sizeof(PlayerObject::m_controlsDisabled)},
					{"m_lastGroundedPos", offsetof(PlayerObject, m_lastGroundedPos), sizeof(PlayerObject::m_lastGroundedPos)},
					{"m_touchingRings", offsetof(PlayerObject, m_touchingRings), sizeof(PlayerObject::m_touchingRings)},
					{"m_touchedRings", offsetof(PlayerObject, m_touchedRings), sizeof(PlayerObject::m_touchedRings)},
					{"m_lastActivatedPortal", offsetof(PlayerObject, m_lastActivatedPortal), sizeof(PlayerObject::m_lastActivatedPortal)},
					{"m_hasEverJumped", offsetof(PlayerObject, m_hasEverJumped), sizeof(PlayerObject::m_hasEverJumped)},
					{"m_hasEverHitRing", offsetof(PlayerObject, m_hasEverHitRing), sizeof(PlayerObject::m_hasEverHitRing)},
					{"m_playerColor1", offsetof(PlayerObject, m_playerColor1), sizeof(PlayerObject::m_playerColor1)},
					{"m_playerColor2", offsetof(PlayerObject, m_playerColor2), sizeof(PlayerObject::m_playerColor2)},
					{"m_position", offsetof(PlayerObject, m_position), sizeof(PlayerObject::m_position)},
					{"m_isSecondPlayer", offsetof(PlayerObject, m_isSecondPlayer), sizeof(PlayerObject::m_isSecondPlayer)},
					{"m_unkA99", offsetof(PlayerObject, m_unkA99), sizeof(PlayerObject::m_unkA99)},
					{"m_totalTime", offsetof(PlayerObject, m_totalTime), sizeof(PlayerObject::m_totalTime)},
					{"m_isBeingSpawnedByDualPortal", offsetof(PlayerObject, m_isBeingSpawnedByDualPortal), sizeof(PlayerObject::m_isBeingSpawnedByDualPortal)},
					{"m_audioScale", offsetof(PlayerObject, m_audioScale), sizeof(PlayerObject::m_audioScale)},
					{"m_unkAngle1", offsetof(PlayerObject, m_unkAngle1), sizeof(PlayerObject::m_unkAngle1)},
					{"m_yVelocityRelated3", offsetof(PlayerObject, m_yVelocityRelated3), sizeof(PlayerObject::m_yVelocityRelated3)},
					{"m_defaultMiniIcon", offsetof(PlayerObject, m_defaultMiniIcon), sizeof(PlayerObject::m_defaultMiniIcon)},
					{"m_swapColors", offsetof(PlayerObject, m_swapColors), sizeof(PlayerObject::m_swapColors)},
					{"m_switchDashFireColor", offsetof(PlayerObject, m_switchDashFireColor), sizeof(PlayerObject::m_switchDashFireColor)},
					{"m_followRelated", offsetof(PlayerObject, m_followRelated), sizeof(PlayerObject::m_followRelated)},
					{"m_playerFollowFloats", offsetof(PlayerObject, m_playerFollowFloats), sizeof(PlayerObject::m_playerFollowFloats)},
					{"m_unk838", offsetof(PlayerObject, m_unk838), sizeof(PlayerObject::m_unk838)},
					{"m_stateOnGround", offsetof(PlayerObject, m_stateOnGround), sizeof(PlayerObject::m_stateOnGround)},
					{"m_stateUnk", offsetof(PlayerObject, m_stateUnk), sizeof(PlayerObject::m_stateUnk)},
					{"m_stateNoStickX", offsetof(PlayerObject, m_stateNoStickX), sizeof(PlayerObject::m_stateNoStickX)},
					{"m_stateNoStickY", offsetof(PlayerObject, m_stateNoStickY), sizeof(PlayerObject::m_stateNoStickY)},
					{"m_stateUnk2", offsetof(PlayerObject, m_stateUnk2), sizeof(PlayerObject::m_stateUnk2)},
					{"m_stateBoostX", offsetof(PlayerObject, m_stateBoostX), sizeof(PlayerObject::m_stateBoostX)},
					{"m_stateBoostY", offsetof(PlayerObject, m_stateBoostY), sizeof(PlayerObject::m_stateBoostY)},
					{"m_maybeStateForce2", offsetof(PlayerObject, m_maybeStateForce2), sizeof(PlayerObject::m_maybeStateForce2)},
					{"m_stateScale", offsetof(PlayerObject, m_stateScale), sizeof(PlayerObject::m_stateScale)},
					{"m_platformerXVelocity", offsetof(PlayerObject, m_platformerXVelocity), sizeof(PlayerObject::m_platformerXVelocity)},
					{"m_holdingRight", offsetof(PlayerObject, m_holdingRight), sizeof(PlayerObject::m_holdingRight)},
					{"m_holdingLeft", offsetof(PlayerObject, m_holdingLeft), sizeof(PlayerObject::m_holdingLeft)},
					{"m_leftPressedFirst", offsetof(PlayerObject, m_leftPressedFirst), sizeof(PlayerObject::m_leftPressedFirst)},
					{"m_scaleXRelated", offsetof(PlayerObject, m_scaleXRelated), sizeof(PlayerObject::m_scaleXRelated)},
					{"m_maybeHasStopped", offsetof(PlayerObject, m_maybeHasStopped), sizeof(PlayerObject::m_maybeHasStopped)},
					{"m_xVelocityRelated", offsetof(PlayerObject, m_xVelocityRelated), sizeof(PlayerObject::m_xVelocityRelated)},
					{"m_maybeGoingCorrectSlopeDirection", offsetof(PlayerObject, m_maybeGoingCorrectSlopeDirection), sizeof(PlayerObject::m_maybeGoingCorrectSlopeDirection)},
					{"m_isSliding", offsetof(PlayerObject, m_isSliding), sizeof(PlayerObject::m_isSliding)},
					{"m_maybeSlopeForce", offsetof(PlayerObject, m_maybeSlopeForce), sizeof(PlayerObject::m_maybeSlopeForce)},
					{"m_isOnIce", offsetof(PlayerObject, m_isOnIce), sizeof(PlayerObject::m_isOnIce)},
					{"m_physDeltaRelated", offsetof(PlayerObject, m_physDeltaRelated), sizeof(PlayerObject::m_physDeltaRelated)},
					{"m_isOnGround4", offsetof(PlayerObject, m_isOnGround4), sizeof(PlayerObject::m_isOnGround4)},
					{"m_maybeSlidingTime", offsetof(PlayerObject, m_maybeSlidingTime), sizeof(PlayerObject::m_maybeSlidingTime)},
					{"m_maybeSlidingStartTime", offsetof(PlayerObject, m_maybeSlidingStartTime), sizeof(PlayerObject::m_maybeSlidingStartTime)},
					{"m_changedDirectionsTime", offsetof(PlayerObject, m_changedDirectionsTime), sizeof(PlayerObject::m_changedDirectionsTime)},
					{"m_slopeEndTime", offsetof(PlayerObject, m_slopeEndTime), sizeof(PlayerObject::m_slopeEndTime)},
					{"m_isMoving", offsetof(PlayerObject, m_isMoving), sizeof(PlayerObject::m_isMoving)},
					{"m_platformerMovingLeft", offsetof(PlayerObject, m_platformerMovingLeft), sizeof(PlayerObject::m_platformerMovingLeft)},
					{"m_platformerMovingRight", offsetof(PlayerObject, m_platformerMovingRight), sizeof(PlayerObject::m_platformerMovingRight)},
					{"m_isSlidingRight", offsetof(PlayerObject, m_isSlidingRight), sizeof(PlayerObject::m_isSlidingRight)},
					{"m_maybeChangedDirectionAngle", offsetof(PlayerObject, m_maybeChangedDirectionAngle), sizeof(PlayerObject::m_maybeChangedDirectionAngle)},
					{"m_unkUnused2", offsetof(PlayerObject, m_unkUnused2), sizeof(PlayerObject::m_unkUnused2)},
					{"m_isPlatformer", offsetof(PlayerObject, m_isPlatformer), sizeof(PlayerObject::m_isPlatformer)},
					{"m_stateNoAutoJump", offsetof(PlayerObject, m_stateNoAutoJump), sizeof(PlayerObject::m_stateNoAutoJump)},
					{"m_stateDartSlide", offsetof(PlayerObject, m_stateDartSlide), sizeof(PlayerObject::m_stateDartSlide)},
					{"m_stateHitHead", offsetof(PlayerObject, m_stateHitHead), sizeof(PlayerObject::m_stateHitHead)},
					{"m_stateFlipGravity", offsetof(PlayerObject, m_stateFlipGravity), sizeof(PlayerObject::m_stateFlipGravity)},
					{"m_gravityMod", offsetof(PlayerObject, m_gravityMod), sizeof(PlayerObject::m_gravityMod)},
					{"m_stateForce", offsetof(PlayerObject, m_stateForce), sizeof(PlayerObject::m_stateForce)},
					{"m_stateForceVector", offsetof(PlayerObject, m_stateForceVector), sizeof(PlayerObject::m_stateForceVector)},
					{"m_affectedByForces", offsetof(PlayerObject, m_affectedByForces), sizeof(PlayerObject::m_affectedByForces)},
					{"m_jumpPadRelated", offsetof(PlayerObject, m_jumpPadRelated), sizeof(PlayerObject::m_jumpPadRelated)},
					{"m_lastMovedTime", offsetof(PlayerObject, m_lastMovedTime), sizeof(PlayerObject::m_lastMovedTime)},
					{"m_playerSpeedAC", offsetof(PlayerObject, m_playerSpeedAC), sizeof(PlayerObject::m_playerSpeedAC)},
					{"m_fixRobotJump", offsetof(PlayerObject, m_fixRobotJump), sizeof(PlayerObject::m_fixRobotJump)},
					{"m_holdingButtons", offsetof(PlayerObject, m_holdingButtons), sizeof(PlayerObject::m_holdingButtons)},
					{"m_inputsLocked", offsetof(PlayerObject, m_inputsLocked), sizeof(PlayerObject::m_inputsLocked)},
					{"m_currentRobotAnimation", offsetof(PlayerObject, m_currentRobotAnimation), sizeof(PlayerObject::m_currentRobotAnimation)},
					{"m_gv0123", offsetof(PlayerObject, m_gv0123), sizeof(PlayerObject::m_gv0123)},
					{"m_iconRequestID", offsetof(PlayerObject, m_iconRequestID), sizeof(PlayerObject::m_iconRequestID)},
					{"m_robotBatchNode", offsetof(PlayerObject, m_robotBatchNode), sizeof(PlayerObject::m_robotBatchNode)},
					{"m_spiderBatchNode", offsetof(PlayerObject, m_spiderBatchNode), sizeof(PlayerObject::m_spiderBatchNode)},
					{"m_unk958", offsetof(PlayerObject, m_unk958), sizeof(PlayerObject::m_unk958)},
					{"m_robotFire", offsetof(PlayerObject, m_robotFire), sizeof(PlayerObject::m_robotFire)},
					{"m_unkUnused", offsetof(PlayerObject, m_unkUnused), sizeof(PlayerObject::m_unkUnused)},
					{"m_gameLayer", offsetof(PlayerObject, m_gameLayer), sizeof(PlayerObject::m_gameLayer)},
					{"m_parentLayer", offsetof(PlayerObject, m_parentLayer), sizeof(PlayerObject::m_parentLayer)},
					{"m_actionManager", offsetof(PlayerObject, m_actionManager), sizeof(PlayerObject::m_actionManager)},
					{"m_isOutOfBounds", offsetof(PlayerObject, m_isOutOfBounds), sizeof(PlayerObject::m_isOutOfBounds)},
					{"m_fallStartY", offsetof(PlayerObject, m_fallStartY), sizeof(PlayerObject::m_fallStartY)},
					{"m_disablePlayerSqueeze", offsetof(PlayerObject, m_disablePlayerSqueeze), sizeof(PlayerObject::m_disablePlayerSqueeze)},
					{"m_robotAnimation1Enabled", offsetof(PlayerObject, m_robotAnimation1Enabled), sizeof(PlayerObject::m_robotAnimation1Enabled)},
					{"m_robotAnimation2Enabled", offsetof(PlayerObject, m_robotAnimation2Enabled), sizeof(PlayerObject::m_robotAnimation2Enabled)},
					{"m_spiderAnimationEnabled", offsetof(PlayerObject, m_spiderAnimationEnabled), sizeof(PlayerObject::m_spiderAnimationEnabled)},
					{"m_ignoreDamage", offsetof(PlayerObject, m_ignoreDamage), sizeof(PlayerObject::m_ignoreDamage)},
					{"m_enable22Changes", offsetof(PlayerObject, m_enable22Changes), sizeof(PlayerObject::m_enable22Changes)},
					{"m_enableImpulseFix", offsetof(PlayerObject, m_enableImpulseFix), sizeof(PlayerObject::m_enableImpulseFix)}
				};

				auto nearestField = [&](size_t off) -> const char* {
					for (auto const& f : fields)
						if (off >= f.off && off < f.off + f.size) return f.name;
					return "";
				};

				size_t diffBytes = 0, ranges = 0;
				std::string detail;
				size_t i = 0;
				while (i < st.anchorBytes.size()) {
					if (st.anchorBytes[i] == now[i]) { i++; continue; }
					const size_t start = i;
					while (i < st.anchorBytes.size() && st.anchorBytes[i] != now[i]) i++;
					diffBytes += (i - start);
					ranges++;
					if (ranges <= 40) {
						log::info("    +0x{:04X}..0x{:04X} ({} bytes) {}",
						          start, i - 1, i - start, nearestField(start));
					}
				}
				log::info("Probe 4a: PlayerObject is {} bytes; {} differ across {} range(s)",
				          sizeof(PlayerObject), diffBytes, ranges);
				(void)detail;
			}

			if (!st.anchorLayerBytes.empty()) {
				const auto* now = reinterpret_cast<const uint8_t*>(PlayLayer::get());
				struct LField { const char* name; size_t off; size_t size; };
				static const LField lfields[] = {
					{"m_gameState", offsetof(GJBaseGameLayer, m_gameState), sizeof(GJBaseGameLayer::m_gameState)},
					{"m_level", offsetof(GJBaseGameLayer, m_level), sizeof(GJBaseGameLayer::m_level)},
					{"m_playbackMode", offsetof(GJBaseGameLayer, m_playbackMode), sizeof(GJBaseGameLayer::m_playbackMode)},
					{"m_lowDetailMode", offsetof(GJBaseGameLayer, m_lowDetailMode), sizeof(GJBaseGameLayer::m_lowDetailMode)},
					{"m_extraLDM", offsetof(GJBaseGameLayer, m_extraLDM), sizeof(GJBaseGameLayer::m_extraLDM)},
					{"m_ignoreDamage", offsetof(GJBaseGameLayer, m_ignoreDamage), sizeof(GJBaseGameLayer::m_ignoreDamage)},
					{"m_enable22Changes", offsetof(GJBaseGameLayer, m_enable22Changes), sizeof(GJBaseGameLayer::m_enable22Changes)},
					{"m_allowStaticRotate", offsetof(GJBaseGameLayer, m_allowStaticRotate), sizeof(GJBaseGameLayer::m_allowStaticRotate)},
					{"m_fixNegativeScale", offsetof(GJBaseGameLayer, m_fixNegativeScale), sizeof(GJBaseGameLayer::m_fixNegativeScale)},
					{"m_startingFromBeginning", offsetof(GJBaseGameLayer, m_startingFromBeginning), sizeof(GJBaseGameLayer::m_startingFromBeginning)},
					{"m_activeSfxTriggers", offsetof(GJBaseGameLayer, m_activeSfxTriggers), sizeof(GJBaseGameLayer::m_activeSfxTriggers)},
					{"m_unk8a0", offsetof(GJBaseGameLayer, m_unk8a0), sizeof(GJBaseGameLayer::m_unk8a0)},
					{"m_hoverNode", offsetof(GJBaseGameLayer, m_hoverNode), sizeof(GJBaseGameLayer::m_hoverNode)},
					{"m_areaTransformNode", offsetof(GJBaseGameLayer, m_areaTransformNode), sizeof(GJBaseGameLayer::m_areaTransformNode)},
					{"m_areaSkewNode", offsetof(GJBaseGameLayer, m_areaSkewNode), sizeof(GJBaseGameLayer::m_areaSkewNode)},
					{"m_areaScaleNode", offsetof(GJBaseGameLayer, m_areaScaleNode), sizeof(GJBaseGameLayer::m_areaScaleNode)},
					{"m_areaRotateNode", offsetof(GJBaseGameLayer, m_areaRotateNode), sizeof(GJBaseGameLayer::m_areaRotateNode)},
					{"m_areaTransformNode2", offsetof(GJBaseGameLayer, m_areaTransformNode2), sizeof(GJBaseGameLayer::m_areaTransformNode2)},
					{"m_obb2", offsetof(GJBaseGameLayer, m_obb2), sizeof(GJBaseGameLayer::m_obb2)},
					{"m_spawnRemapTriggers", offsetof(GJBaseGameLayer, m_spawnRemapTriggers), sizeof(GJBaseGameLayer::m_spawnRemapTriggers)},
					{"m_uiObjectPositions", offsetof(GJBaseGameLayer, m_uiObjectPositions), sizeof(GJBaseGameLayer::m_uiObjectPositions)},
					{"m_effectManager", offsetof(GJBaseGameLayer, m_effectManager), sizeof(GJBaseGameLayer::m_effectManager)},
					{"m_gameBlendingLayerT5", offsetof(GJBaseGameLayer, m_gameBlendingLayerT5), sizeof(GJBaseGameLayer::m_gameBlendingLayerT5)},
					{"m_fireBlendingLayerT5", offsetof(GJBaseGameLayer, m_fireBlendingLayerT5), sizeof(GJBaseGameLayer::m_fireBlendingLayerT5)},
					{"m_pixelBlendingLayerT5", offsetof(GJBaseGameLayer, m_pixelBlendingLayerT5), sizeof(GJBaseGameLayer::m_pixelBlendingLayerT5)},
					{"m_particleBlendingLayerT5", offsetof(GJBaseGameLayer, m_particleBlendingLayerT5), sizeof(GJBaseGameLayer::m_particleBlendingLayerT5)},
					{"m_game2BlendingLayerT5", offsetof(GJBaseGameLayer, m_game2BlendingLayerT5), sizeof(GJBaseGameLayer::m_game2BlendingLayerT5)},
					{"m_gameLayerT4", offsetof(GJBaseGameLayer, m_gameLayerT4), sizeof(GJBaseGameLayer::m_gameLayerT4)},
					{"m_gameBlendingLayerT4", offsetof(GJBaseGameLayer, m_gameBlendingLayerT4), sizeof(GJBaseGameLayer::m_gameBlendingLayerT4)},
					{"m_glowLayerT4", offsetof(GJBaseGameLayer, m_glowLayerT4), sizeof(GJBaseGameLayer::m_glowLayerT4)},
					{"m_specialLayerT4", offsetof(GJBaseGameLayer, m_specialLayerT4), sizeof(GJBaseGameLayer::m_specialLayerT4)},
					{"m_textLayerT4", offsetof(GJBaseGameLayer, m_textLayerT4), sizeof(GJBaseGameLayer::m_textLayerT4)},
					{"m_textBlendingLayerT4", offsetof(GJBaseGameLayer, m_textBlendingLayerT4), sizeof(GJBaseGameLayer::m_textBlendingLayerT4)},
					{"m_fireLayerT4", offsetof(GJBaseGameLayer, m_fireLayerT4), sizeof(GJBaseGameLayer::m_fireLayerT4)},
					{"m_fireBlendingLayerT4", offsetof(GJBaseGameLayer, m_fireBlendingLayerT4), sizeof(GJBaseGameLayer::m_fireBlendingLayerT4)},
					{"m_pixelLayerT4", offsetof(GJBaseGameLayer, m_pixelLayerT4), sizeof(GJBaseGameLayer::m_pixelLayerT4)},
					{"m_pixelBlendingLayerT4", offsetof(GJBaseGameLayer, m_pixelBlendingLayerT4), sizeof(GJBaseGameLayer::m_pixelBlendingLayerT4)},
					{"m_particleLayerT4", offsetof(GJBaseGameLayer, m_particleLayerT4), sizeof(GJBaseGameLayer::m_particleLayerT4)},
					{"m_particleBlendingLayerT4", offsetof(GJBaseGameLayer, m_particleBlendingLayerT4), sizeof(GJBaseGameLayer::m_particleBlendingLayerT4)},
					{"m_game2LayerT4", offsetof(GJBaseGameLayer, m_game2LayerT4), sizeof(GJBaseGameLayer::m_game2LayerT4)},
					{"m_game2BlendingLayerT4", offsetof(GJBaseGameLayer, m_game2BlendingLayerT4), sizeof(GJBaseGameLayer::m_game2BlendingLayerT4)},
					{"m_gameLayerT3", offsetof(GJBaseGameLayer, m_gameLayerT3), sizeof(GJBaseGameLayer::m_gameLayerT3)},
					{"m_gameBlendingLayerT3", offsetof(GJBaseGameLayer, m_gameBlendingLayerT3), sizeof(GJBaseGameLayer::m_gameBlendingLayerT3)},
					{"m_glowLayerT3", offsetof(GJBaseGameLayer, m_glowLayerT3), sizeof(GJBaseGameLayer::m_glowLayerT3)},
					{"m_specialLayerT3", offsetof(GJBaseGameLayer, m_specialLayerT3), sizeof(GJBaseGameLayer::m_specialLayerT3)},
					{"m_textLayerT3", offsetof(GJBaseGameLayer, m_textLayerT3), sizeof(GJBaseGameLayer::m_textLayerT3)},
					{"m_textBlendingLayerT3", offsetof(GJBaseGameLayer, m_textBlendingLayerT3), sizeof(GJBaseGameLayer::m_textBlendingLayerT3)},
					{"m_fireLayerT3", offsetof(GJBaseGameLayer, m_fireLayerT3), sizeof(GJBaseGameLayer::m_fireLayerT3)},
					{"m_fireBlendingLayerT3", offsetof(GJBaseGameLayer, m_fireBlendingLayerT3), sizeof(GJBaseGameLayer::m_fireBlendingLayerT3)},
					{"m_pixelLayerT3", offsetof(GJBaseGameLayer, m_pixelLayerT3), sizeof(GJBaseGameLayer::m_pixelLayerT3)},
					{"m_pixelBlendingLayerT3", offsetof(GJBaseGameLayer, m_pixelBlendingLayerT3), sizeof(GJBaseGameLayer::m_pixelBlendingLayerT3)},
					{"m_particleLayerT3", offsetof(GJBaseGameLayer, m_particleLayerT3), sizeof(GJBaseGameLayer::m_particleLayerT3)},
					{"m_particleBlendingLayerT3", offsetof(GJBaseGameLayer, m_particleBlendingLayerT3), sizeof(GJBaseGameLayer::m_particleBlendingLayerT3)},
					{"m_game2LayerT3", offsetof(GJBaseGameLayer, m_game2LayerT3), sizeof(GJBaseGameLayer::m_game2LayerT3)},
					{"m_game2BlendingLayerT3", offsetof(GJBaseGameLayer, m_game2BlendingLayerT3), sizeof(GJBaseGameLayer::m_game2BlendingLayerT3)},
					{"m_gameLayerT2", offsetof(GJBaseGameLayer, m_gameLayerT2), sizeof(GJBaseGameLayer::m_gameLayerT2)},
					{"m_gameBlendingLayerT2", offsetof(GJBaseGameLayer, m_gameBlendingLayerT2), sizeof(GJBaseGameLayer::m_gameBlendingLayerT2)},
					{"m_glowLayerT2", offsetof(GJBaseGameLayer, m_glowLayerT2), sizeof(GJBaseGameLayer::m_glowLayerT2)},
					{"m_specialLayerT2", offsetof(GJBaseGameLayer, m_specialLayerT2), sizeof(GJBaseGameLayer::m_specialLayerT2)},
					{"m_textLayerT2", offsetof(GJBaseGameLayer, m_textLayerT2), sizeof(GJBaseGameLayer::m_textLayerT2)},
					{"m_textBlendingLayerT2", offsetof(GJBaseGameLayer, m_textBlendingLayerT2), sizeof(GJBaseGameLayer::m_textBlendingLayerT2)},
					{"m_fireLayerT2", offsetof(GJBaseGameLayer, m_fireLayerT2), sizeof(GJBaseGameLayer::m_fireLayerT2)},
					{"m_fireBlendingLayerT2", offsetof(GJBaseGameLayer, m_fireBlendingLayerT2), sizeof(GJBaseGameLayer::m_fireBlendingLayerT2)},
					{"m_pixelLayerT2", offsetof(GJBaseGameLayer, m_pixelLayerT2), sizeof(GJBaseGameLayer::m_pixelLayerT2)},
					{"m_pixelBlendingLayerT2", offsetof(GJBaseGameLayer, m_pixelBlendingLayerT2), sizeof(GJBaseGameLayer::m_pixelBlendingLayerT2)},
					{"m_particleLayerT2", offsetof(GJBaseGameLayer, m_particleLayerT2), sizeof(GJBaseGameLayer::m_particleLayerT2)},
					{"m_particleBlendingLayerT2", offsetof(GJBaseGameLayer, m_particleBlendingLayerT2), sizeof(GJBaseGameLayer::m_particleBlendingLayerT2)},
					{"m_game2LayerT2", offsetof(GJBaseGameLayer, m_game2LayerT2), sizeof(GJBaseGameLayer::m_game2LayerT2)},
					{"m_game2BlendingLayerT2", offsetof(GJBaseGameLayer, m_game2BlendingLayerT2), sizeof(GJBaseGameLayer::m_game2BlendingLayerT2)},
					{"m_gameLayerT1", offsetof(GJBaseGameLayer, m_gameLayerT1), sizeof(GJBaseGameLayer::m_gameLayerT1)},
					{"m_gameBlendingLayerT1", offsetof(GJBaseGameLayer, m_gameBlendingLayerT1), sizeof(GJBaseGameLayer::m_gameBlendingLayerT1)},
					{"m_glowLayerT1", offsetof(GJBaseGameLayer, m_glowLayerT1), sizeof(GJBaseGameLayer::m_glowLayerT1)},
					{"m_specialLayerT1", offsetof(GJBaseGameLayer, m_specialLayerT1), sizeof(GJBaseGameLayer::m_specialLayerT1)},
					{"m_textLayerT1", offsetof(GJBaseGameLayer, m_textLayerT1), sizeof(GJBaseGameLayer::m_textLayerT1)},
					{"m_textBlendingLayerT1", offsetof(GJBaseGameLayer, m_textBlendingLayerT1), sizeof(GJBaseGameLayer::m_textBlendingLayerT1)},
					{"m_fireLayerT1", offsetof(GJBaseGameLayer, m_fireLayerT1), sizeof(GJBaseGameLayer::m_fireLayerT1)},
					{"m_fireBlendingLayerT1", offsetof(GJBaseGameLayer, m_fireBlendingLayerT1), sizeof(GJBaseGameLayer::m_fireBlendingLayerT1)},
					{"m_pixelLayerT1", offsetof(GJBaseGameLayer, m_pixelLayerT1), sizeof(GJBaseGameLayer::m_pixelLayerT1)},
					{"m_pixelBlendingLayerT1", offsetof(GJBaseGameLayer, m_pixelBlendingLayerT1), sizeof(GJBaseGameLayer::m_pixelBlendingLayerT1)},
					{"m_particleLayerT1", offsetof(GJBaseGameLayer, m_particleLayerT1), sizeof(GJBaseGameLayer::m_particleLayerT1)},
					{"m_particleBlendingLayerT1", offsetof(GJBaseGameLayer, m_particleBlendingLayerT1), sizeof(GJBaseGameLayer::m_particleBlendingLayerT1)},
					{"m_game2LayerT1", offsetof(GJBaseGameLayer, m_game2LayerT1), sizeof(GJBaseGameLayer::m_game2LayerT1)},
					{"m_game2BlendingLayerT1", offsetof(GJBaseGameLayer, m_game2BlendingLayerT1), sizeof(GJBaseGameLayer::m_game2BlendingLayerT1)},
					{"m_game2LayerB0", offsetof(GJBaseGameLayer, m_game2LayerB0), sizeof(GJBaseGameLayer::m_game2LayerB0)},
					{"m_gameBlendingLayerB0", offsetof(GJBaseGameLayer, m_gameBlendingLayerB0), sizeof(GJBaseGameLayer::m_gameBlendingLayerB0)},
					{"m_fireBlendingLayerB0", offsetof(GJBaseGameLayer, m_fireBlendingLayerB0), sizeof(GJBaseGameLayer::m_fireBlendingLayerB0)},
					{"m_pixelBlendingLayerB0", offsetof(GJBaseGameLayer, m_pixelBlendingLayerB0), sizeof(GJBaseGameLayer::m_pixelBlendingLayerB0)},
					{"m_particleBlendingLayerB0", offsetof(GJBaseGameLayer, m_particleBlendingLayerB0), sizeof(GJBaseGameLayer::m_particleBlendingLayerB0)},
					{"m_game2BlendingLayerB0", offsetof(GJBaseGameLayer, m_game2BlendingLayerB0), sizeof(GJBaseGameLayer::m_game2BlendingLayerB0)},
					{"m_gameLayerB1", offsetof(GJBaseGameLayer, m_gameLayerB1), sizeof(GJBaseGameLayer::m_gameLayerB1)},
					{"m_gameBlendingLayerB1", offsetof(GJBaseGameLayer, m_gameBlendingLayerB1), sizeof(GJBaseGameLayer::m_gameBlendingLayerB1)},
					{"m_glowLayerB1", offsetof(GJBaseGameLayer, m_glowLayerB1), sizeof(GJBaseGameLayer::m_glowLayerB1)},
					{"m_specialLayerB1", offsetof(GJBaseGameLayer, m_specialLayerB1), sizeof(GJBaseGameLayer::m_specialLayerB1)},
					{"m_textLayerB1", offsetof(GJBaseGameLayer, m_textLayerB1), sizeof(GJBaseGameLayer::m_textLayerB1)},
					{"m_textBlendingLayerB1", offsetof(GJBaseGameLayer, m_textBlendingLayerB1), sizeof(GJBaseGameLayer::m_textBlendingLayerB1)},
					{"m_fireLayerB1", offsetof(GJBaseGameLayer, m_fireLayerB1), sizeof(GJBaseGameLayer::m_fireLayerB1)},
					{"m_fireBlendingLayerB1", offsetof(GJBaseGameLayer, m_fireBlendingLayerB1), sizeof(GJBaseGameLayer::m_fireBlendingLayerB1)},
					{"m_pixelLayerB1", offsetof(GJBaseGameLayer, m_pixelLayerB1), sizeof(GJBaseGameLayer::m_pixelLayerB1)},
					{"m_pixelBlendingLayerB1", offsetof(GJBaseGameLayer, m_pixelBlendingLayerB1), sizeof(GJBaseGameLayer::m_pixelBlendingLayerB1)},
					{"m_particleLayerB1", offsetof(GJBaseGameLayer, m_particleLayerB1), sizeof(GJBaseGameLayer::m_particleLayerB1)},
					{"m_particleBlendingLayerB1", offsetof(GJBaseGameLayer, m_particleBlendingLayerB1), sizeof(GJBaseGameLayer::m_particleBlendingLayerB1)},
					{"m_game2LayerB1", offsetof(GJBaseGameLayer, m_game2LayerB1), sizeof(GJBaseGameLayer::m_game2LayerB1)},
					{"m_game2BlendingLayerB1", offsetof(GJBaseGameLayer, m_game2BlendingLayerB1), sizeof(GJBaseGameLayer::m_game2BlendingLayerB1)},
					{"m_gameLayerB2", offsetof(GJBaseGameLayer, m_gameLayerB2), sizeof(GJBaseGameLayer::m_gameLayerB2)},
					{"m_gameBlendingLayerB2", offsetof(GJBaseGameLayer, m_gameBlendingLayerB2), sizeof(GJBaseGameLayer::m_gameBlendingLayerB2)},
					{"m_glowLayerB2", offsetof(GJBaseGameLayer, m_glowLayerB2), sizeof(GJBaseGameLayer::m_glowLayerB2)},
					{"m_specialLayerB2", offsetof(GJBaseGameLayer, m_specialLayerB2), sizeof(GJBaseGameLayer::m_specialLayerB2)},
					{"m_textLayerB2", offsetof(GJBaseGameLayer, m_textLayerB2), sizeof(GJBaseGameLayer::m_textLayerB2)},
					{"m_textBlendingLayerB2", offsetof(GJBaseGameLayer, m_textBlendingLayerB2), sizeof(GJBaseGameLayer::m_textBlendingLayerB2)},
					{"m_fireLayerB2", offsetof(GJBaseGameLayer, m_fireLayerB2), sizeof(GJBaseGameLayer::m_fireLayerB2)},
					{"m_fireBlendingLayerB2", offsetof(GJBaseGameLayer, m_fireBlendingLayerB2), sizeof(GJBaseGameLayer::m_fireBlendingLayerB2)},
					{"m_pixelLayerB2", offsetof(GJBaseGameLayer, m_pixelLayerB2), sizeof(GJBaseGameLayer::m_pixelLayerB2)},
					{"m_pixelBlendingLayerB2", offsetof(GJBaseGameLayer, m_pixelBlendingLayerB2), sizeof(GJBaseGameLayer::m_pixelBlendingLayerB2)},
					{"m_particleLayerB2", offsetof(GJBaseGameLayer, m_particleLayerB2), sizeof(GJBaseGameLayer::m_particleLayerB2)},
					{"m_particleBlendingLayerB2", offsetof(GJBaseGameLayer, m_particleBlendingLayerB2), sizeof(GJBaseGameLayer::m_particleBlendingLayerB2)},
					{"m_game2LayerB2", offsetof(GJBaseGameLayer, m_game2LayerB2), sizeof(GJBaseGameLayer::m_game2LayerB2)},
					{"m_game2BlendingLayerB2", offsetof(GJBaseGameLayer, m_game2BlendingLayerB2), sizeof(GJBaseGameLayer::m_game2BlendingLayerB2)},
					{"m_gameLayerB3", offsetof(GJBaseGameLayer, m_gameLayerB3), sizeof(GJBaseGameLayer::m_gameLayerB3)},
					{"m_gameBlendingLayerB3", offsetof(GJBaseGameLayer, m_gameBlendingLayerB3), sizeof(GJBaseGameLayer::m_gameBlendingLayerB3)},
					{"m_glowLayerB3", offsetof(GJBaseGameLayer, m_glowLayerB3), sizeof(GJBaseGameLayer::m_glowLayerB3)},
					{"m_specialLayerB3", offsetof(GJBaseGameLayer, m_specialLayerB3), sizeof(GJBaseGameLayer::m_specialLayerB3)},
					{"m_textLayerB3", offsetof(GJBaseGameLayer, m_textLayerB3), sizeof(GJBaseGameLayer::m_textLayerB3)},
					{"m_textBlendingLayerB3", offsetof(GJBaseGameLayer, m_textBlendingLayerB3), sizeof(GJBaseGameLayer::m_textBlendingLayerB3)},
					{"m_fireLayerB3", offsetof(GJBaseGameLayer, m_fireLayerB3), sizeof(GJBaseGameLayer::m_fireLayerB3)},
					{"m_fireBlendingLayerB3", offsetof(GJBaseGameLayer, m_fireBlendingLayerB3), sizeof(GJBaseGameLayer::m_fireBlendingLayerB3)},
					{"m_pixelLayerB3", offsetof(GJBaseGameLayer, m_pixelLayerB3), sizeof(GJBaseGameLayer::m_pixelLayerB3)},
					{"m_pixelBlendingLayerB3", offsetof(GJBaseGameLayer, m_pixelBlendingLayerB3), sizeof(GJBaseGameLayer::m_pixelBlendingLayerB3)},
					{"m_particleLayerB3", offsetof(GJBaseGameLayer, m_particleLayerB3), sizeof(GJBaseGameLayer::m_particleLayerB3)},
					{"m_particleBlendingLayerB3", offsetof(GJBaseGameLayer, m_particleBlendingLayerB3), sizeof(GJBaseGameLayer::m_particleBlendingLayerB3)},
					{"m_game2LayerB3", offsetof(GJBaseGameLayer, m_game2LayerB3), sizeof(GJBaseGameLayer::m_game2LayerB3)},
					{"m_game2BlendingLayerB3", offsetof(GJBaseGameLayer, m_game2BlendingLayerB3), sizeof(GJBaseGameLayer::m_game2BlendingLayerB3)},
					{"m_gameLayerB4", offsetof(GJBaseGameLayer, m_gameLayerB4), sizeof(GJBaseGameLayer::m_gameLayerB4)},
					{"m_gameBlendingLayerB4", offsetof(GJBaseGameLayer, m_gameBlendingLayerB4), sizeof(GJBaseGameLayer::m_gameBlendingLayerB4)},
					{"m_glowLayerB4", offsetof(GJBaseGameLayer, m_glowLayerB4), sizeof(GJBaseGameLayer::m_glowLayerB4)},
					{"m_specialLayerB4", offsetof(GJBaseGameLayer, m_specialLayerB4), sizeof(GJBaseGameLayer::m_specialLayerB4)},
					{"m_textLayerB4", offsetof(GJBaseGameLayer, m_textLayerB4), sizeof(GJBaseGameLayer::m_textLayerB4)},
					{"m_textBlendingLayerB4", offsetof(GJBaseGameLayer, m_textBlendingLayerB4), sizeof(GJBaseGameLayer::m_textBlendingLayerB4)},
					{"m_fireLayerB4", offsetof(GJBaseGameLayer, m_fireLayerB4), sizeof(GJBaseGameLayer::m_fireLayerB4)},
					{"m_fireBlendingLayerB4", offsetof(GJBaseGameLayer, m_fireBlendingLayerB4), sizeof(GJBaseGameLayer::m_fireBlendingLayerB4)},
					{"m_pixelLayerB4", offsetof(GJBaseGameLayer, m_pixelLayerB4), sizeof(GJBaseGameLayer::m_pixelLayerB4)},
					{"m_pixelBlendingLayerB4", offsetof(GJBaseGameLayer, m_pixelBlendingLayerB4), sizeof(GJBaseGameLayer::m_pixelBlendingLayerB4)},
					{"m_particleLayerB4", offsetof(GJBaseGameLayer, m_particleLayerB4), sizeof(GJBaseGameLayer::m_particleLayerB4)},
					{"m_particleBlendingLayerB4", offsetof(GJBaseGameLayer, m_particleBlendingLayerB4), sizeof(GJBaseGameLayer::m_particleBlendingLayerB4)},
					{"m_game2LayerB4", offsetof(GJBaseGameLayer, m_game2LayerB4), sizeof(GJBaseGameLayer::m_game2LayerB4)},
					{"m_game2BlendingLayerB4", offsetof(GJBaseGameLayer, m_game2BlendingLayerB4), sizeof(GJBaseGameLayer::m_game2BlendingLayerB4)},
					{"m_gameLayerB5", offsetof(GJBaseGameLayer, m_gameLayerB5), sizeof(GJBaseGameLayer::m_gameLayerB5)},
					{"m_gameBlendingLayerB5", offsetof(GJBaseGameLayer, m_gameBlendingLayerB5), sizeof(GJBaseGameLayer::m_gameBlendingLayerB5)},
					{"m_glowLayerB5", offsetof(GJBaseGameLayer, m_glowLayerB5), sizeof(GJBaseGameLayer::m_glowLayerB5)},
					{"m_specialLayerB5", offsetof(GJBaseGameLayer, m_specialLayerB5), sizeof(GJBaseGameLayer::m_specialLayerB5)},
					{"m_textLayerB5", offsetof(GJBaseGameLayer, m_textLayerB5), sizeof(GJBaseGameLayer::m_textLayerB5)},
					{"m_textBlendingLayerB5", offsetof(GJBaseGameLayer, m_textBlendingLayerB5), sizeof(GJBaseGameLayer::m_textBlendingLayerB5)},
					{"m_fireLayerB5", offsetof(GJBaseGameLayer, m_fireLayerB5), sizeof(GJBaseGameLayer::m_fireLayerB5)},
					{"m_fireBlendingLayerB5", offsetof(GJBaseGameLayer, m_fireBlendingLayerB5), sizeof(GJBaseGameLayer::m_fireBlendingLayerB5)},
					{"m_pixelLayerB5", offsetof(GJBaseGameLayer, m_pixelLayerB5), sizeof(GJBaseGameLayer::m_pixelLayerB5)},
					{"m_pixelBlendingLayerB5", offsetof(GJBaseGameLayer, m_pixelBlendingLayerB5), sizeof(GJBaseGameLayer::m_pixelBlendingLayerB5)},
					{"m_particleLayerB5", offsetof(GJBaseGameLayer, m_particleLayerB5), sizeof(GJBaseGameLayer::m_particleLayerB5)},
					{"m_particleBlendingLayerB5", offsetof(GJBaseGameLayer, m_particleBlendingLayerB5), sizeof(GJBaseGameLayer::m_particleBlendingLayerB5)},
					{"m_game2LayerB5", offsetof(GJBaseGameLayer, m_game2LayerB5), sizeof(GJBaseGameLayer::m_game2LayerB5)},
					{"m_game2BlendingLayerB5", offsetof(GJBaseGameLayer, m_game2BlendingLayerB5), sizeof(GJBaseGameLayer::m_game2BlendingLayerB5)},
					{"m_player1", offsetof(GJBaseGameLayer, m_player1), sizeof(GJBaseGameLayer::m_player1)},
					{"m_player2", offsetof(GJBaseGameLayer, m_player2), sizeof(GJBaseGameLayer::m_player2)},
					{"m_levelSettings", offsetof(GJBaseGameLayer, m_levelSettings), sizeof(GJBaseGameLayer::m_levelSettings)},
					{"m_objects", offsetof(GJBaseGameLayer, m_objects), sizeof(GJBaseGameLayer::m_objects)},
					{"m_collisionBlocks", offsetof(GJBaseGameLayer, m_collisionBlocks), sizeof(GJBaseGameLayer::m_collisionBlocks)},
					{"m_spawnObjectsArray", offsetof(GJBaseGameLayer, m_spawnObjectsArray), sizeof(GJBaseGameLayer::m_spawnObjectsArray)},
					{"m_spawnObjects", offsetof(GJBaseGameLayer, m_spawnObjects), sizeof(GJBaseGameLayer::m_spawnObjects)},
					{"m_unkdd0", offsetof(GJBaseGameLayer, m_unkdd0), sizeof(GJBaseGameLayer::m_unkdd0)},
					{"m_unkdd8", offsetof(GJBaseGameLayer, m_unkdd8), sizeof(GJBaseGameLayer::m_unkdd8)},
					{"m_disabledObjects", offsetof(GJBaseGameLayer, m_disabledObjects), sizeof(GJBaseGameLayer::m_disabledObjects)},
					{"m_unke08", offsetof(GJBaseGameLayer, m_unke08), sizeof(GJBaseGameLayer::m_unke08)},
					{"m_areaObjects", offsetof(GJBaseGameLayer, m_areaObjects), sizeof(GJBaseGameLayer::m_areaObjects)},
					{"m_processedAreaObjects", offsetof(GJBaseGameLayer, m_processedAreaObjects), sizeof(GJBaseGameLayer::m_processedAreaObjects)},
					{"m_visibilityGroups", offsetof(GJBaseGameLayer, m_visibilityGroups), sizeof(GJBaseGameLayer::m_visibilityGroups)},
					{"m_visibleObjects", offsetof(GJBaseGameLayer, m_visibleObjects), sizeof(GJBaseGameLayer::m_visibleObjects)},
					{"m_visibleObjectsCount", offsetof(GJBaseGameLayer, m_visibleObjectsCount), sizeof(GJBaseGameLayer::m_visibleObjectsCount)},
					{"m_visibleObjectsIndex", offsetof(GJBaseGameLayer, m_visibleObjectsIndex), sizeof(GJBaseGameLayer::m_visibleObjectsIndex)},
					{"m_visibleObjects2", offsetof(GJBaseGameLayer, m_visibleObjects2), sizeof(GJBaseGameLayer::m_visibleObjects2)},
					{"m_visibleObjects2Count", offsetof(GJBaseGameLayer, m_visibleObjects2Count), sizeof(GJBaseGameLayer::m_visibleObjects2Count)},
					{"m_visibleObjects2Index", offsetof(GJBaseGameLayer, m_visibleObjects2Index), sizeof(GJBaseGameLayer::m_visibleObjects2Index)},
					{"m_unked0", offsetof(GJBaseGameLayer, m_unked0), sizeof(GJBaseGameLayer::m_unked0)},
					{"m_disabledObjectsCount", offsetof(GJBaseGameLayer, m_disabledObjectsCount), sizeof(GJBaseGameLayer::m_disabledObjectsCount)},
					{"m_unked8", offsetof(GJBaseGameLayer, m_unked8), sizeof(GJBaseGameLayer::m_unked8)},
					{"m_areaObjectsCount", offsetof(GJBaseGameLayer, m_areaObjectsCount), sizeof(GJBaseGameLayer::m_areaObjectsCount)},
					{"m_processedAreaObjectsCount", offsetof(GJBaseGameLayer, m_processedAreaObjectsCount), sizeof(GJBaseGameLayer::m_processedAreaObjectsCount)},
					{"m_unkee4", offsetof(GJBaseGameLayer, m_unkee4), sizeof(GJBaseGameLayer::m_unkee4)},
					{"m_disabledObjectsIndex", offsetof(GJBaseGameLayer, m_disabledObjectsIndex), sizeof(GJBaseGameLayer::m_disabledObjectsIndex)},
					{"m_unkeec", offsetof(GJBaseGameLayer, m_unkeec), sizeof(GJBaseGameLayer::m_unkeec)},
					{"m_areaObjectsIndex", offsetof(GJBaseGameLayer, m_areaObjectsIndex), sizeof(GJBaseGameLayer::m_areaObjectsIndex)},
					{"m_processedAreaObjectsIndex", offsetof(GJBaseGameLayer, m_processedAreaObjectsIndex), sizeof(GJBaseGameLayer::m_processedAreaObjectsIndex)},
					{"m_groupDict", offsetof(GJBaseGameLayer, m_groupDict), sizeof(GJBaseGameLayer::m_groupDict)},
					{"m_staticGroupDict", offsetof(GJBaseGameLayer, m_staticGroupDict), sizeof(GJBaseGameLayer::m_staticGroupDict)},
					{"m_optimizedGroupDict", offsetof(GJBaseGameLayer, m_optimizedGroupDict), sizeof(GJBaseGameLayer::m_optimizedGroupDict)},
					{"m_groups", offsetof(GJBaseGameLayer, m_groups), sizeof(GJBaseGameLayer::m_groups)},
					{"m_staticGroups", offsetof(GJBaseGameLayer, m_staticGroups), sizeof(GJBaseGameLayer::m_staticGroups)},
					{"m_optimizedGroups", offsetof(GJBaseGameLayer, m_optimizedGroups), sizeof(GJBaseGameLayer::m_optimizedGroups)},
					{"m_parentGroupsDict", offsetof(GJBaseGameLayer, m_parentGroupsDict), sizeof(GJBaseGameLayer::m_parentGroupsDict)},
					{"m_parentGroupIDs", offsetof(GJBaseGameLayer, m_parentGroupIDs), sizeof(GJBaseGameLayer::m_parentGroupIDs)},
					{"m_removedParentGroupIDs", offsetof(GJBaseGameLayer, m_removedParentGroupIDs), sizeof(GJBaseGameLayer::m_removedParentGroupIDs)},
					{"m_targetGroupsArray", offsetof(GJBaseGameLayer, m_targetGroupsArray), sizeof(GJBaseGameLayer::m_targetGroupsArray)},
					{"m_targetGroups", offsetof(GJBaseGameLayer, m_targetGroups), sizeof(GJBaseGameLayer::m_targetGroups)},
					{"m_linkedGroupDict", offsetof(GJBaseGameLayer, m_linkedGroupDict), sizeof(GJBaseGameLayer::m_linkedGroupDict)},
					{"m_lastUsedLinkedID", offsetof(GJBaseGameLayer, m_lastUsedLinkedID), sizeof(GJBaseGameLayer::m_lastUsedLinkedID)},
					{"m_objectParent", offsetof(GJBaseGameLayer, m_objectParent), sizeof(GJBaseGameLayer::m_objectParent)},
					{"m_inShaderParent", offsetof(GJBaseGameLayer, m_inShaderParent), sizeof(GJBaseGameLayer::m_inShaderParent)},
					{"m_aboveShaderParent", offsetof(GJBaseGameLayer, m_aboveShaderParent), sizeof(GJBaseGameLayer::m_aboveShaderParent)},
					{"m_objectLayer", offsetof(GJBaseGameLayer, m_objectLayer), sizeof(GJBaseGameLayer::m_objectLayer)},
					{"m_inShaderObjectLayer", offsetof(GJBaseGameLayer, m_inShaderObjectLayer), sizeof(GJBaseGameLayer::m_inShaderObjectLayer)},
					{"m_aboveShaderObjectLayer", offsetof(GJBaseGameLayer, m_aboveShaderObjectLayer), sizeof(GJBaseGameLayer::m_aboveShaderObjectLayer)},
					{"m_background", offsetof(GJBaseGameLayer, m_background), sizeof(GJBaseGameLayer::m_background)},
					{"m_unk1000", offsetof(GJBaseGameLayer, m_unk1000), sizeof(GJBaseGameLayer::m_unk1000)},
					{"m_groundLayer", offsetof(GJBaseGameLayer, m_groundLayer), sizeof(GJBaseGameLayer::m_groundLayer)},
					{"m_groundLayer2", offsetof(GJBaseGameLayer, m_groundLayer2), sizeof(GJBaseGameLayer::m_groundLayer2)},
					{"m_middleground", offsetof(GJBaseGameLayer, m_middleground), sizeof(GJBaseGameLayer::m_middleground)},
					{"m_batchNodes", offsetof(GJBaseGameLayer, m_batchNodes), sizeof(GJBaseGameLayer::m_batchNodes)},
					{"m_objectsToDeactivate", offsetof(GJBaseGameLayer, m_objectsToDeactivate), sizeof(GJBaseGameLayer::m_objectsToDeactivate)},
					{"m_labelObjects", offsetof(GJBaseGameLayer, m_labelObjects), sizeof(GJBaseGameLayer::m_labelObjects)},
					{"m_timeLabelObjects", offsetof(GJBaseGameLayer, m_timeLabelObjects), sizeof(GJBaseGameLayer::m_timeLabelObjects)},
					{"m_spawnTuples", offsetof(GJBaseGameLayer, m_spawnTuples), sizeof(GJBaseGameLayer::m_spawnTuples)},
					{"m_increasedLayerCapacity", offsetof(GJBaseGameLayer, m_increasedLayerCapacity), sizeof(GJBaseGameLayer::m_increasedLayerCapacity)},
					{"m_varianceValues", offsetof(GJBaseGameLayer, m_varianceValues), sizeof(GJBaseGameLayer::m_varianceValues)},
					{"m_destroyObjectValues", offsetof(GJBaseGameLayer, m_destroyObjectValues), sizeof(GJBaseGameLayer::m_destroyObjectValues)},
					{"m_enterEasingValues", offsetof(GJBaseGameLayer, m_enterEasingValues), sizeof(GJBaseGameLayer::m_enterEasingValues)},
					{"m_enterEasingIndices", offsetof(GJBaseGameLayer, m_enterEasingIndices), sizeof(GJBaseGameLayer::m_enterEasingIndices)},
					{"m_enterEasingValuesIndex", offsetof(GJBaseGameLayer, m_enterEasingValuesIndex), sizeof(GJBaseGameLayer::m_enterEasingValuesIndex)},
					{"m_dualTouchTrigger", offsetof(GJBaseGameLayer, m_dualTouchTrigger), sizeof(GJBaseGameLayer::m_dualTouchTrigger)},
					{"m_clicks", offsetof(GJBaseGameLayer, m_clicks), sizeof(GJBaseGameLayer::m_clicks)},
					{"m_attempts", offsetof(GJBaseGameLayer, m_attempts), sizeof(GJBaseGameLayer::m_attempts)},
					{"m_jumping", offsetof(GJBaseGameLayer, m_jumping), sizeof(GJBaseGameLayer::m_jumping)},
					{"m_leftSectionIndex", offsetof(GJBaseGameLayer, m_leftSectionIndex), sizeof(GJBaseGameLayer::m_leftSectionIndex)},
					{"m_rightSectionIndex", offsetof(GJBaseGameLayer, m_rightSectionIndex), sizeof(GJBaseGameLayer::m_rightSectionIndex)},
					{"m_bottomSectionIndex", offsetof(GJBaseGameLayer, m_bottomSectionIndex), sizeof(GJBaseGameLayer::m_bottomSectionIndex)},
					{"m_topSectionIndex", offsetof(GJBaseGameLayer, m_topSectionIndex), sizeof(GJBaseGameLayer::m_topSectionIndex)},
					{"m_isEditor", offsetof(GJBaseGameLayer, m_isEditor), sizeof(GJBaseGameLayer::m_isEditor)},
					{"m_blending", offsetof(GJBaseGameLayer, m_blending), sizeof(GJBaseGameLayer::m_blending)},
					{"m_isPlatformer", offsetof(GJBaseGameLayer, m_isPlatformer), sizeof(GJBaseGameLayer::m_isPlatformer)},
					{"m_player1CollisionBlock", offsetof(GJBaseGameLayer, m_player1CollisionBlock), sizeof(GJBaseGameLayer::m_player1CollisionBlock)},
					{"m_player2CollisionBlock", offsetof(GJBaseGameLayer, m_player2CollisionBlock), sizeof(GJBaseGameLayer::m_player2CollisionBlock)},
					{"m_particleCount", offsetof(GJBaseGameLayer, m_particleCount), sizeof(GJBaseGameLayer::m_particleCount)},
					{"m_customParticleCount", offsetof(GJBaseGameLayer, m_customParticleCount), sizeof(GJBaseGameLayer::m_customParticleCount)},
					{"m_particleSystemLimit", offsetof(GJBaseGameLayer, m_particleSystemLimit), sizeof(GJBaseGameLayer::m_particleSystemLimit)},
					{"m_particlesDict", offsetof(GJBaseGameLayer, m_particlesDict), sizeof(GJBaseGameLayer::m_particlesDict)},
					{"m_customParticles", offsetof(GJBaseGameLayer, m_customParticles), sizeof(GJBaseGameLayer::m_customParticles)},
					{"m_unclaimedParticles", offsetof(GJBaseGameLayer, m_unclaimedParticles), sizeof(GJBaseGameLayer::m_unclaimedParticles)},
					{"m_particleCountToParticleString", offsetof(GJBaseGameLayer, m_particleCountToParticleString), sizeof(GJBaseGameLayer::m_particleCountToParticleString)},
					{"m_claimedParticles", offsetof(GJBaseGameLayer, m_claimedParticles), sizeof(GJBaseGameLayer::m_claimedParticles)},
					{"m_temporaryParticles", offsetof(GJBaseGameLayer, m_temporaryParticles), sizeof(GJBaseGameLayer::m_temporaryParticles)},
					{"m_customParticlesUIDs", offsetof(GJBaseGameLayer, m_customParticlesUIDs), sizeof(GJBaseGameLayer::m_customParticlesUIDs)},
					{"m_gradientLayers", offsetof(GJBaseGameLayer, m_gradientLayers), sizeof(GJBaseGameLayer::m_gradientLayers)},
					{"m_activeGradients", offsetof(GJBaseGameLayer, m_activeGradients), sizeof(GJBaseGameLayer::m_activeGradients)},
					{"m_shaderLayer", offsetof(GJBaseGameLayer, m_shaderLayer), sizeof(GJBaseGameLayer::m_shaderLayer)},
					{"m_objectsDeactivated", offsetof(GJBaseGameLayer, m_objectsDeactivated), sizeof(GJBaseGameLayer::m_objectsDeactivated)},
					{"m_areaObjectsUpdated", offsetof(GJBaseGameLayer, m_areaObjectsUpdated), sizeof(GJBaseGameLayer::m_areaObjectsUpdated)},
					{"m_startPosObject", offsetof(GJBaseGameLayer, m_startPosObject), sizeof(GJBaseGameLayer::m_startPosObject)},
					{"m_useReplay", offsetof(GJBaseGameLayer, m_useReplay), sizeof(GJBaseGameLayer::m_useReplay)},
					{"m_unk3189", offsetof(GJBaseGameLayer, m_unk3189), sizeof(GJBaseGameLayer::m_unk3189)},
					{"m_solidCollisionObjectsCount", offsetof(GJBaseGameLayer, m_solidCollisionObjectsCount), sizeof(GJBaseGameLayer::m_solidCollisionObjectsCount)},
					{"m_solidCollisionObjectsIndex", offsetof(GJBaseGameLayer, m_solidCollisionObjectsIndex), sizeof(GJBaseGameLayer::m_solidCollisionObjectsIndex)},
					{"m_solidCollisionObjects", offsetof(GJBaseGameLayer, m_solidCollisionObjects), sizeof(GJBaseGameLayer::m_solidCollisionObjects)},
					{"m_hazardCollisionObjectsCount", offsetof(GJBaseGameLayer, m_hazardCollisionObjectsCount), sizeof(GJBaseGameLayer::m_hazardCollisionObjectsCount)},
					{"m_hazardCollisionObjectsIndex", offsetof(GJBaseGameLayer, m_hazardCollisionObjectsIndex), sizeof(GJBaseGameLayer::m_hazardCollisionObjectsIndex)},
					{"m_hazardCollisionObjects", offsetof(GJBaseGameLayer, m_hazardCollisionObjects), sizeof(GJBaseGameLayer::m_hazardCollisionObjects)},
					{"m_sequenceTriggers", offsetof(GJBaseGameLayer, m_sequenceTriggers), sizeof(GJBaseGameLayer::m_sequenceTriggers)},
					{"m_isPracticeMode", offsetof(GJBaseGameLayer, m_isPracticeMode), sizeof(GJBaseGameLayer::m_isPracticeMode)},
					{"m_practiceMusicSync", offsetof(GJBaseGameLayer, m_practiceMusicSync), sizeof(GJBaseGameLayer::m_practiceMusicSync)},
					{"m_loadingProgress", offsetof(GJBaseGameLayer, m_loadingProgress), sizeof(GJBaseGameLayer::m_loadingProgress)},
					{"m_flashNode", offsetof(GJBaseGameLayer, m_flashNode), sizeof(GJBaseGameLayer::m_flashNode)},
					{"m_unk31f8", offsetof(GJBaseGameLayer, m_unk31f8), sizeof(GJBaseGameLayer::m_unk31f8)},
					{"m_cameraFlip", offsetof(GJBaseGameLayer, m_cameraFlip), sizeof(GJBaseGameLayer::m_cameraFlip)},
					{"m_cameraWidthOffset", offsetof(GJBaseGameLayer, m_cameraWidthOffset), sizeof(GJBaseGameLayer::m_cameraWidthOffset)},
					{"m_cameraHeightOffset", offsetof(GJBaseGameLayer, m_cameraHeightOffset), sizeof(GJBaseGameLayer::m_cameraHeightOffset)},
					{"m_updateGroundShadows", offsetof(GJBaseGameLayer, m_updateGroundShadows), sizeof(GJBaseGameLayer::m_updateGroundShadows)},
					{"m_collectedItems", offsetof(GJBaseGameLayer, m_collectedItems), sizeof(GJBaseGameLayer::m_collectedItems)},
					{"m_levelLength", offsetof(GJBaseGameLayer, m_levelLength), sizeof(GJBaseGameLayer::m_levelLength)},
					{"m_resetActiveObjects", offsetof(GJBaseGameLayer, m_resetActiveObjects), sizeof(GJBaseGameLayer::m_resetActiveObjects)},
					{"m_skipArtReload", offsetof(GJBaseGameLayer, m_skipArtReload), sizeof(GJBaseGameLayer::m_skipArtReload)},
					{"m_endPortal", offsetof(GJBaseGameLayer, m_endPortal), sizeof(GJBaseGameLayer::m_endPortal)},
					{"m_isTestMode", offsetof(GJBaseGameLayer, m_isTestMode), sizeof(GJBaseGameLayer::m_isTestMode)},
					{"m_freezeStartCamera", offsetof(GJBaseGameLayer, m_freezeStartCamera), sizeof(GJBaseGameLayer::m_freezeStartCamera)},
					{"m_unk322a", offsetof(GJBaseGameLayer, m_unk322a), sizeof(GJBaseGameLayer::m_unk322a)},
					{"m_cameraUnzoomedHeightOffset", offsetof(GJBaseGameLayer, m_cameraUnzoomedHeightOffset), sizeof(GJBaseGameLayer::m_cameraUnzoomedHeightOffset)},
					{"m_targetCameraHeightOffset", offsetof(GJBaseGameLayer, m_targetCameraHeightOffset), sizeof(GJBaseGameLayer::m_targetCameraHeightOffset)},
					{"m_calculateTargetHeightOffset", offsetof(GJBaseGameLayer, m_calculateTargetHeightOffset), sizeof(GJBaseGameLayer::m_calculateTargetHeightOffset)},
					{"m_glitterParticles", offsetof(GJBaseGameLayer, m_glitterParticles), sizeof(GJBaseGameLayer::m_glitterParticles)},
					{"m_staticCameraShake", offsetof(GJBaseGameLayer, m_staticCameraShake), sizeof(GJBaseGameLayer::m_staticCameraShake)},
					{"m_skipCameraShake", offsetof(GJBaseGameLayer, m_skipCameraShake), sizeof(GJBaseGameLayer::m_skipCameraShake)},
					{"m_playerDied", offsetof(GJBaseGameLayer, m_playerDied), sizeof(GJBaseGameLayer::m_playerDied)},
					{"m_extraDelta", offsetof(GJBaseGameLayer, m_extraDelta), sizeof(GJBaseGameLayer::m_extraDelta)},
					{"m_started", offsetof(GJBaseGameLayer, m_started), sizeof(GJBaseGameLayer::m_started)},
					{"m_unk3251", offsetof(GJBaseGameLayer, m_unk3251), sizeof(GJBaseGameLayer::m_unk3251)},
					{"m_cameraWidth", offsetof(GJBaseGameLayer, m_cameraWidth), sizeof(GJBaseGameLayer::m_cameraWidth)},
					{"m_cameraHeight", offsetof(GJBaseGameLayer, m_cameraHeight), sizeof(GJBaseGameLayer::m_cameraHeight)},
					{"m_cameraUnzoomedX", offsetof(GJBaseGameLayer, m_cameraUnzoomedX), sizeof(GJBaseGameLayer::m_cameraUnzoomedX)},
					{"m_halfCameraWidth", offsetof(GJBaseGameLayer, m_halfCameraWidth), sizeof(GJBaseGameLayer::m_halfCameraWidth)},
					{"m_audioEffectsLayer", offsetof(GJBaseGameLayer, m_audioEffectsLayer), sizeof(GJBaseGameLayer::m_audioEffectsLayer)},
					{"m_cameraObb2", offsetof(GJBaseGameLayer, m_cameraObb2), sizeof(GJBaseGameLayer::m_cameraObb2)},
					{"m_activeObjects", offsetof(GJBaseGameLayer, m_activeObjects), sizeof(GJBaseGameLayer::m_activeObjects)},
					{"m_activeObjectsCount", offsetof(GJBaseGameLayer, m_activeObjectsCount), sizeof(GJBaseGameLayer::m_activeObjectsCount)},
					{"m_activeObjectsIndex", offsetof(GJBaseGameLayer, m_activeObjectsIndex), sizeof(GJBaseGameLayer::m_activeObjectsIndex)},
					{"m_lightBGColor", offsetof(GJBaseGameLayer, m_lightBGColor), sizeof(GJBaseGameLayer::m_lightBGColor)},
					{"m_resumeTimer", offsetof(GJBaseGameLayer, m_resumeTimer), sizeof(GJBaseGameLayer::m_resumeTimer)},
					{"m_recordInputs", offsetof(GJBaseGameLayer, m_recordInputs), sizeof(GJBaseGameLayer::m_recordInputs)},
					{"m_unk32a1", offsetof(GJBaseGameLayer, m_unk32a1), sizeof(GJBaseGameLayer::m_unk32a1)},
					{"m_unk32a2", offsetof(GJBaseGameLayer, m_unk32a2), sizeof(GJBaseGameLayer::m_unk32a2)},
					{"m_unk32a3", offsetof(GJBaseGameLayer, m_unk32a3), sizeof(GJBaseGameLayer::m_unk32a3)},
					{"m_unk32a4", offsetof(GJBaseGameLayer, m_unk32a4), sizeof(GJBaseGameLayer::m_unk32a4)},
					{"m_recordString", offsetof(GJBaseGameLayer, m_recordString), sizeof(GJBaseGameLayer::m_recordString)},
					{"m_unk32c8", offsetof(GJBaseGameLayer, m_unk32c8), sizeof(GJBaseGameLayer::m_unk32c8)},
					{"m_unk32d0", offsetof(GJBaseGameLayer, m_unk32d0), sizeof(GJBaseGameLayer::m_unk32d0)},
					{"m_unk32d4", offsetof(GJBaseGameLayer, m_unk32d4), sizeof(GJBaseGameLayer::m_unk32d4)},
					{"m_randomSeed", offsetof(GJBaseGameLayer, m_randomSeed), sizeof(GJBaseGameLayer::m_randomSeed)},
					{"m_unk32e0", offsetof(GJBaseGameLayer, m_unk32e0), sizeof(GJBaseGameLayer::m_unk32e0)},
					{"m_replayRandSeed", offsetof(GJBaseGameLayer, m_replayRandSeed), sizeof(GJBaseGameLayer::m_replayRandSeed)},
					{"m_unk32ec", offsetof(GJBaseGameLayer, m_unk32ec), sizeof(GJBaseGameLayer::m_unk32ec)},
					{"m_currentStep", offsetof(GJBaseGameLayer, m_currentStep), sizeof(GJBaseGameLayer::m_currentStep)},
					{"m_queuedButtons", offsetof(GJBaseGameLayer, m_queuedButtons), sizeof(GJBaseGameLayer::m_queuedButtons)},
					{"m_queuedRecordedButtons", offsetof(GJBaseGameLayer, m_queuedRecordedButtons), sizeof(GJBaseGameLayer::m_queuedRecordedButtons)},
					{"m_unk3330", offsetof(GJBaseGameLayer, m_unk3330), sizeof(GJBaseGameLayer::m_unk3330)},
					{"m_unk3370", offsetof(GJBaseGameLayer, m_unk3370), sizeof(GJBaseGameLayer::m_unk3370)},
					{"m_queuedReplayButtons", offsetof(GJBaseGameLayer, m_queuedReplayButtons), sizeof(GJBaseGameLayer::m_queuedReplayButtons)},
					{"m_unk3390", offsetof(GJBaseGameLayer, m_unk3390), sizeof(GJBaseGameLayer::m_unk3390)},
					{"m_unk3340", offsetof(GJBaseGameLayer, m_unk3340), sizeof(GJBaseGameLayer::m_unk3340)},
					{"m_unk3358", offsetof(GJBaseGameLayer, m_unk3358), sizeof(GJBaseGameLayer::m_unk3358)},
					{"m_queuedRecordedButtonsSize", offsetof(GJBaseGameLayer, m_queuedRecordedButtonsSize), sizeof(GJBaseGameLayer::m_queuedRecordedButtonsSize)},
					{"m_persistentStateString", offsetof(GJBaseGameLayer, m_persistentStateString), sizeof(GJBaseGameLayer::m_persistentStateString)},
					{"m_savedPersistentStateString", offsetof(GJBaseGameLayer, m_savedPersistentStateString), sizeof(GJBaseGameLayer::m_savedPersistentStateString)},
					{"m_savedAttempts", offsetof(GJBaseGameLayer, m_savedAttempts), sizeof(GJBaseGameLayer::m_savedAttempts)},
					{"m_portalIndicators", offsetof(GJBaseGameLayer, m_portalIndicators), sizeof(GJBaseGameLayer::m_portalIndicators)},
					{"m_orbIndicators", offsetof(GJBaseGameLayer, m_orbIndicators), sizeof(GJBaseGameLayer::m_orbIndicators)},
					{"m_indicatorSprites", offsetof(GJBaseGameLayer, m_indicatorSprites), sizeof(GJBaseGameLayer::m_indicatorSprites)},
					{"m_unk3380", offsetof(GJBaseGameLayer, m_unk3380), sizeof(GJBaseGameLayer::m_unk3380)},
					{"m_unk3388", offsetof(GJBaseGameLayer, m_unk3388), sizeof(GJBaseGameLayer::m_unk3388)},
					{"m_unk33a0", offsetof(GJBaseGameLayer, m_unk33a0), sizeof(GJBaseGameLayer::m_unk33a0)},
					{"m_hideGround", offsetof(GJBaseGameLayer, m_hideGround), sizeof(GJBaseGameLayer::m_hideGround)},
					{"m_unk33c0", offsetof(GJBaseGameLayer, m_unk33c0), sizeof(GJBaseGameLayer::m_unk33c0)},
					{"m_objectsToMove", offsetof(GJBaseGameLayer, m_objectsToMove), sizeof(GJBaseGameLayer::m_objectsToMove)},
					{"m_savePositionObjects", offsetof(GJBaseGameLayer, m_savePositionObjects), sizeof(GJBaseGameLayer::m_savePositionObjects)},
					{"m_savePositionValues", offsetof(GJBaseGameLayer, m_savePositionValues), sizeof(GJBaseGameLayer::m_savePositionValues)},
					{"m_keepGroupParents", offsetof(GJBaseGameLayer, m_keepGroupParents), sizeof(GJBaseGameLayer::m_keepGroupParents)},
					{"m_keyframeGroups", offsetof(GJBaseGameLayer, m_keyframeGroups), sizeof(GJBaseGameLayer::m_keyframeGroups)},
					{"m_keyframeGroup", offsetof(GJBaseGameLayer, m_keyframeGroup), sizeof(GJBaseGameLayer::m_keyframeGroup)},
					{"m_uiLayer", offsetof(GJBaseGameLayer, m_uiLayer), sizeof(GJBaseGameLayer::m_uiLayer)},
					{"m_uiObjects", offsetof(GJBaseGameLayer, m_uiObjects), sizeof(GJBaseGameLayer::m_uiObjects)},
					{"m_uiObjectLayers", offsetof(GJBaseGameLayer, m_uiObjectLayers), sizeof(GJBaseGameLayer::m_uiObjectLayers)},
					{"m_uiTriggerUI", offsetof(GJBaseGameLayer, m_uiTriggerUI), sizeof(GJBaseGameLayer::m_uiTriggerUI)},
					{"m_timePlayed", offsetof(GJBaseGameLayer, m_timePlayed), sizeof(GJBaseGameLayer::m_timePlayed)},
					{"m_tickIndex", offsetof(GJBaseGameLayer, m_tickIndex), sizeof(GJBaseGameLayer::m_tickIndex)},
					{"m_clickIndex", offsetof(GJBaseGameLayer, m_clickIndex), sizeof(GJBaseGameLayer::m_clickIndex)},
					{"m_levelEndAnimationStarted", offsetof(GJBaseGameLayer, m_levelEndAnimationStarted), sizeof(GJBaseGameLayer::m_levelEndAnimationStarted)},
					{"m_points", offsetof(GJBaseGameLayer, m_points), sizeof(GJBaseGameLayer::m_points)},
					{"m_pointsString", offsetof(GJBaseGameLayer, m_pointsString), sizeof(GJBaseGameLayer::m_pointsString)},
					{"m_sections", offsetof(GJBaseGameLayer, m_sections), sizeof(GJBaseGameLayer::m_sections)},
					{"m_nonEffectObjects", offsetof(GJBaseGameLayer, m_nonEffectObjects), sizeof(GJBaseGameLayer::m_nonEffectObjects)},
					{"m_collisionBlockSections", offsetof(GJBaseGameLayer, m_collisionBlockSections), sizeof(GJBaseGameLayer::m_collisionBlockSections)},
					{"m_calcNonEffectObjects", offsetof(GJBaseGameLayer, m_calcNonEffectObjects), sizeof(GJBaseGameLayer::m_calcNonEffectObjects)},
					{"m_calcNonEffectObjectsSize", offsetof(GJBaseGameLayer, m_calcNonEffectObjectsSize), sizeof(GJBaseGameLayer::m_calcNonEffectObjectsSize)},
					{"m_calcCollisionBlockObjects", offsetof(GJBaseGameLayer, m_calcCollisionBlockObjects), sizeof(GJBaseGameLayer::m_calcCollisionBlockObjects)},
					{"m_calcCollisionBlockObjectsSize", offsetof(GJBaseGameLayer, m_calcCollisionBlockObjectsSize), sizeof(GJBaseGameLayer::m_calcCollisionBlockObjectsSize)},
					{"m_calcCollisionBlockObjects2", offsetof(GJBaseGameLayer, m_calcCollisionBlockObjects2), sizeof(GJBaseGameLayer::m_calcCollisionBlockObjects2)},
					{"m_calcCollisionBlockObjects2Size", offsetof(GJBaseGameLayer, m_calcCollisionBlockObjects2Size), sizeof(GJBaseGameLayer::m_calcCollisionBlockObjects2Size)},
					{"m_sectionSizes", offsetof(GJBaseGameLayer, m_sectionSizes), sizeof(GJBaseGameLayer::m_sectionSizes)},
					{"m_nonEffectObjectsSizes", offsetof(GJBaseGameLayer, m_nonEffectObjectsSizes), sizeof(GJBaseGameLayer::m_nonEffectObjectsSizes)},
					{"m_collisionBlockSectionSizes", offsetof(GJBaseGameLayer, m_collisionBlockSectionSizes), sizeof(GJBaseGameLayer::m_collisionBlockSectionSizes)},
					{"m_nonEffectObjectsFlags", offsetof(GJBaseGameLayer, m_nonEffectObjectsFlags), sizeof(GJBaseGameLayer::m_nonEffectObjectsFlags)},
					{"m_sectionXFactor", offsetof(GJBaseGameLayer, m_sectionXFactor), sizeof(GJBaseGameLayer::m_sectionXFactor)},
					{"m_sectionYFactor", offsetof(GJBaseGameLayer, m_sectionYFactor), sizeof(GJBaseGameLayer::m_sectionYFactor)},
					{"m_maxGameplayY", offsetof(GJBaseGameLayer, m_maxGameplayY), sizeof(GJBaseGameLayer::m_maxGameplayY)},
					{"m_songTriggerInterval", offsetof(GJBaseGameLayer, m_songTriggerInterval), sizeof(GJBaseGameLayer::m_songTriggerInterval)},
					{"m_stickyGroups", offsetof(GJBaseGameLayer, m_stickyGroups), sizeof(GJBaseGameLayer::m_stickyGroups)},
					{"m_audioVisualizerBG", offsetof(GJBaseGameLayer, m_audioVisualizerBG), sizeof(GJBaseGameLayer::m_audioVisualizerBG)},
					{"m_audioVisualizerSFX", offsetof(GJBaseGameLayer, m_audioVisualizerSFX), sizeof(GJBaseGameLayer::m_audioVisualizerSFX)},
					{"m_showAudioVisualizer", offsetof(GJBaseGameLayer, m_showAudioVisualizer), sizeof(GJBaseGameLayer::m_showAudioVisualizer)},
					{"m_areaMovedCount", offsetof(GJBaseGameLayer, m_areaMovedCount), sizeof(GJBaseGameLayer::m_areaMovedCount)},
					{"m_areaScaledCount", offsetof(GJBaseGameLayer, m_areaScaledCount), sizeof(GJBaseGameLayer::m_areaScaledCount)},
					{"m_areaRotatedCount", offsetof(GJBaseGameLayer, m_areaRotatedCount), sizeof(GJBaseGameLayer::m_areaRotatedCount)},
					{"m_areaColorCount", offsetof(GJBaseGameLayer, m_areaColorCount), sizeof(GJBaseGameLayer::m_areaColorCount)},
					{"m_areaMovedCountTotal", offsetof(GJBaseGameLayer, m_areaMovedCountTotal), sizeof(GJBaseGameLayer::m_areaMovedCountTotal)},
					{"m_areaScaledCountTotal", offsetof(GJBaseGameLayer, m_areaScaledCountTotal), sizeof(GJBaseGameLayer::m_areaScaledCountTotal)},
					{"m_areaRotatedCountTotal", offsetof(GJBaseGameLayer, m_areaRotatedCountTotal), sizeof(GJBaseGameLayer::m_areaRotatedCountTotal)},
					{"m_areaColorCountTotal", offsetof(GJBaseGameLayer, m_areaColorCountTotal), sizeof(GJBaseGameLayer::m_areaColorCountTotal)},
					{"m_movedCount", offsetof(GJBaseGameLayer, m_movedCount), sizeof(GJBaseGameLayer::m_movedCount)},
					{"m_scaledCount", offsetof(GJBaseGameLayer, m_scaledCount), sizeof(GJBaseGameLayer::m_scaledCount)},
					{"m_rotatedCount", offsetof(GJBaseGameLayer, m_rotatedCount), sizeof(GJBaseGameLayer::m_rotatedCount)},
					{"m_followedCount", offsetof(GJBaseGameLayer, m_followedCount), sizeof(GJBaseGameLayer::m_followedCount)},
					{"m_areaMovedCountDisplay", offsetof(GJBaseGameLayer, m_areaMovedCountDisplay), sizeof(GJBaseGameLayer::m_areaMovedCountDisplay)},
					{"m_areaScaledCountDisplay", offsetof(GJBaseGameLayer, m_areaScaledCountDisplay), sizeof(GJBaseGameLayer::m_areaScaledCountDisplay)},
					{"m_areaRotatedCountDisplay", offsetof(GJBaseGameLayer, m_areaRotatedCountDisplay), sizeof(GJBaseGameLayer::m_areaRotatedCountDisplay)},
					{"m_areaColorCountDisplay", offsetof(GJBaseGameLayer, m_areaColorCountDisplay), sizeof(GJBaseGameLayer::m_areaColorCountDisplay)},
					{"m_areaMovedCountTotalDisplay", offsetof(GJBaseGameLayer, m_areaMovedCountTotalDisplay), sizeof(GJBaseGameLayer::m_areaMovedCountTotalDisplay)},
					{"m_areaScaledCountTotalDisplay", offsetof(GJBaseGameLayer, m_areaScaledCountTotalDisplay), sizeof(GJBaseGameLayer::m_areaScaledCountTotalDisplay)},
					{"m_areaRotatedCountTotalDisplay", offsetof(GJBaseGameLayer, m_areaRotatedCountTotalDisplay), sizeof(GJBaseGameLayer::m_areaRotatedCountTotalDisplay)},
					{"m_areaColorCountTotalDisplay", offsetof(GJBaseGameLayer, m_areaColorCountTotalDisplay), sizeof(GJBaseGameLayer::m_areaColorCountTotalDisplay)},
					{"m_movedCountDisplay", offsetof(GJBaseGameLayer, m_movedCountDisplay), sizeof(GJBaseGameLayer::m_movedCountDisplay)},
					{"m_scaledCountDisplay", offsetof(GJBaseGameLayer, m_scaledCountDisplay), sizeof(GJBaseGameLayer::m_scaledCountDisplay)},
					{"m_rotatedCountDisplay", offsetof(GJBaseGameLayer, m_rotatedCountDisplay), sizeof(GJBaseGameLayer::m_rotatedCountDisplay)},
					{"m_followedCountDisplay", offsetof(GJBaseGameLayer, m_followedCountDisplay), sizeof(GJBaseGameLayer::m_followedCountDisplay)},
					{"m_loadingStartPosition", offsetof(GJBaseGameLayer, m_loadingStartPosition), sizeof(GJBaseGameLayer::m_loadingStartPosition)},
					{"m_processingAudioTriggers", offsetof(GJBaseGameLayer, m_processingAudioTriggers), sizeof(GJBaseGameLayer::m_processingAudioTriggers)},
					{"m_audioPaused", offsetof(GJBaseGameLayer, m_audioPaused), sizeof(GJBaseGameLayer::m_audioPaused)},
					{"m_startOptimization", offsetof(GJBaseGameLayer, m_startOptimization), sizeof(GJBaseGameLayer::m_startOptimization)},
					{"m_loadingLayer", offsetof(GJBaseGameLayer, m_loadingLayer), sizeof(GJBaseGameLayer::m_loadingLayer)},
					{"m_debugDrawNode", offsetof(GJBaseGameLayer, m_debugDrawNode), sizeof(GJBaseGameLayer::m_debugDrawNode)},
					{"m_debugDrawPoints", offsetof(GJBaseGameLayer, m_debugDrawPoints), sizeof(GJBaseGameLayer::m_debugDrawPoints)},
					{"m_isDebugDrawEnabled", offsetof(GJBaseGameLayer, m_isDebugDrawEnabled), sizeof(GJBaseGameLayer::m_isDebugDrawEnabled)},
					{"m_disablePlayerHitbox", offsetof(GJBaseGameLayer, m_disablePlayerHitbox), sizeof(GJBaseGameLayer::m_disablePlayerHitbox)},
					{"m_hitboxesOnDeath", offsetof(GJBaseGameLayer, m_hitboxesOnDeath), sizeof(GJBaseGameLayer::m_hitboxesOnDeath)},
					{"m_anticheatSpike", offsetof(GJBaseGameLayer, m_anticheatSpike), sizeof(GJBaseGameLayer::m_anticheatSpike)},
					{"m_timestamp", offsetof(GJBaseGameLayer, m_timestamp), sizeof(GJBaseGameLayer::m_timestamp)},
					{"m_isBetweenSteps", offsetof(GJBaseGameLayer, m_isBetweenSteps), sizeof(GJBaseGameLayer::m_isBetweenSteps)},
					{"m_clickBetweenSteps", offsetof(GJBaseGameLayer, m_clickBetweenSteps), sizeof(GJBaseGameLayer::m_clickBetweenSteps)},
					{"m_clickOnSteps", offsetof(GJBaseGameLayer, m_clickOnSteps), sizeof(GJBaseGameLayer::m_clickOnSteps)}
				};
				struct PLField { size_t off, size; const char* name; };
				static const PLField plfields[] = {
					{offsetof(PlayLayer, m_unk36c8), sizeof(PlayLayer::m_unk36c8), "m_unk36c8"},
					{offsetof(PlayLayer, m_unk36cc), sizeof(PlayLayer::m_unk36cc), "m_unk36cc"},
					{offsetof(PlayLayer, m_unk36cd), sizeof(PlayLayer::m_unk36cd), "m_unk36cd"},
					{offsetof(PlayLayer, m_unk36ce), sizeof(PlayLayer::m_unk36ce), "m_unk36ce"},
					{offsetof(PlayLayer, m_unk36cf), sizeof(PlayLayer::m_unk36cf), "m_unk36cf"},
					{offsetof(PlayLayer, m_damageVerified), sizeof(PlayLayer::m_damageVerified), "m_damageVerified"},
					{offsetof(PlayLayer, m_passedIntegrity), sizeof(PlayLayer::m_passedIntegrity), "m_passedIntegrity"},
					{offsetof(PlayLayer, m_objectsCreated), sizeof(PlayLayer::m_objectsCreated), "m_objectsCreated"},
					{offsetof(PlayLayer, m_unk3768), sizeof(PlayLayer::m_unk3768), "m_unk3768"},
					{offsetof(PlayLayer, m_platformerRestart), sizeof(PlayLayer::m_platformerRestart), "m_platformerRestart"},
					{offsetof(PlayLayer, m_unk376d), sizeof(PlayLayer::m_unk376d), "m_unk376d"},
					{offsetof(PlayLayer, m_isIgnoreDamageEnabled), sizeof(PlayLayer::m_isIgnoreDamageEnabled), "m_isIgnoreDamageEnabled"},
					{offsetof(PlayLayer, m_unk3778), sizeof(PlayLayer::m_unk3778), "m_unk3778"},
					{offsetof(PlayLayer, m_unk377c), sizeof(PlayLayer::m_unk377c), "m_unk377c"},
					{offsetof(PlayLayer, m_unk3780), sizeof(PlayLayer::m_unk3780), "m_unk3780"},
					{offsetof(PlayLayer, m_unk3784), sizeof(PlayLayer::m_unk3784), "m_unk3784"},
					{offsetof(PlayLayer, m_unk3788), sizeof(PlayLayer::m_unk3788), "m_unk3788"},
					{offsetof(PlayLayer, m_unk378c), sizeof(PlayLayer::m_unk378c), "m_unk378c"},
					{offsetof(PlayLayer, m_endChecked), sizeof(PlayLayer::m_endChecked), "m_endChecked"},
					{offsetof(PlayLayer, m_endXPosition), sizeof(PlayLayer::m_endXPosition), "m_endXPosition"},
					{offsetof(PlayLayer, m_unk37b0), sizeof(PlayLayer::m_unk37b0), "m_unk37b0"},
					{offsetof(PlayLayer, m_unk37b1), sizeof(PlayLayer::m_unk37b1), "m_unk37b1"},
					{offsetof(PlayLayer, m_isSilent), sizeof(PlayLayer::m_isSilent), "m_isSilent"},
					{offsetof(PlayLayer, m_unk37cc), sizeof(PlayLayer::m_unk37cc), "m_unk37cc"},
					{offsetof(PlayLayer, m_unk37e0), sizeof(PlayLayer::m_unk37e0), "m_unk37e0"},
					{offsetof(PlayLayer, m_pulseRodIndex), sizeof(PlayLayer::m_pulseRodIndex), "m_pulseRodIndex"},
					{offsetof(PlayLayer, m_maxObjectX), sizeof(PlayLayer::m_maxObjectX), "m_maxObjectX"},
					{offsetof(PlayLayer, m_decimalPercentage), sizeof(PlayLayer::m_decimalPercentage), "m_decimalPercentage"},
					{offsetof(PlayLayer, m_hintShown), sizeof(PlayLayer::m_hintShown), "m_hintShown"},
					{offsetof(PlayLayer, m_progressWidth), sizeof(PlayLayer::m_progressWidth), "m_progressWidth"},
					{offsetof(PlayLayer, m_progressHeight), sizeof(PlayLayer::m_progressHeight), "m_progressHeight"},
					{offsetof(PlayLayer, m_totalGravityEffects), sizeof(PlayLayer::m_totalGravityEffects), "m_totalGravityEffects"},
					{offsetof(PlayLayer, m_activeGravityEffects), sizeof(PlayLayer::m_activeGravityEffects), "m_activeGravityEffects"},
					{offsetof(PlayLayer, m_gravityEffectIndex), sizeof(PlayLayer::m_gravityEffectIndex), "m_gravityEffectIndex"},
					{offsetof(PlayLayer, m_doNot), sizeof(PlayLayer::m_doNot), "m_doNot"},
					{offsetof(PlayLayer, m_unk383c), sizeof(PlayLayer::m_unk383c), "m_unk383c"},
					{offsetof(PlayLayer, m_skipAudioStep), sizeof(PlayLayer::m_skipAudioStep), "m_skipAudioStep"},
					{offsetof(PlayLayer, m_jumps), sizeof(PlayLayer::m_jumps), "m_jumps"},
					{offsetof(PlayLayer, m_hasJumped), sizeof(PlayLayer::m_hasJumped), "m_hasJumped"},
					{offsetof(PlayLayer, m_uncommittedJumps), sizeof(PlayLayer::m_uncommittedJumps), "m_uncommittedJumps"},
					{offsetof(PlayLayer, m_showLeaderboardPercentage), sizeof(PlayLayer::m_showLeaderboardPercentage), "m_showLeaderboardPercentage"},
					{offsetof(PlayLayer, m_hasCompletedLevel), sizeof(PlayLayer::m_hasCompletedLevel), "m_hasCompletedLevel"},
					{offsetof(PlayLayer, m_inResetDelay), sizeof(PlayLayer::m_inResetDelay), "m_inResetDelay"},
					{offsetof(PlayLayer, m_lastAttemptPercent), sizeof(PlayLayer::m_lastAttemptPercent), "m_lastAttemptPercent"},
					{offsetof(PlayLayer, m_endLayerStars), sizeof(PlayLayer::m_endLayerStars), "m_endLayerStars"},
					{offsetof(PlayLayer, m_orbs), sizeof(PlayLayer::m_orbs), "m_orbs"},
					{offsetof(PlayLayer, m_diamonds), sizeof(PlayLayer::m_diamonds), "m_diamonds"},
					{offsetof(PlayLayer, m_secretKey), sizeof(PlayLayer::m_secretKey), "m_secretKey"},
					{offsetof(PlayLayer, m_recordingStopped), sizeof(PlayLayer::m_recordingStopped), "m_recordingStopped"},
					{offsetof(PlayLayer, m_unk38b0), sizeof(PlayLayer::m_unk38b0), "m_unk38b0"},
					{offsetof(PlayLayer, m_unk38b8), sizeof(PlayLayer::m_unk38b8), "m_unk38b8"},
					{offsetof(PlayLayer, m_unk38c0), sizeof(PlayLayer::m_unk38c0), "m_unk38c0"},
					{offsetof(PlayLayer, m_unk38c8), sizeof(PlayLayer::m_unk38c8), "m_unk38c8"},
					{offsetof(PlayLayer, m_unk38cc), sizeof(PlayLayer::m_unk38cc), "m_unk38cc"},
					{offsetof(PlayLayer, m_unk38d0), sizeof(PlayLayer::m_unk38d0), "m_unk38d0"},
					{offsetof(PlayLayer, m_attemptTime), sizeof(PlayLayer::m_attemptTime), "m_attemptTime"},
					{offsetof(PlayLayer, m_bestAttemptTime), sizeof(PlayLayer::m_bestAttemptTime), "m_bestAttemptTime"},
					{offsetof(PlayLayer, m_pauseTime), sizeof(PlayLayer::m_pauseTime), "m_pauseTime"},
					{offsetof(PlayLayer, m_currentTime), sizeof(PlayLayer::m_currentTime), "m_currentTime"},
					{offsetof(PlayLayer, m_pauseDelta), sizeof(PlayLayer::m_pauseDelta), "m_pauseDelta"},
					{offsetof(PlayLayer, m_unk3900), sizeof(PlayLayer::m_unk3900), "m_unk3900"},
					{offsetof(PlayLayer, m_glitterEnabled), sizeof(PlayLayer::m_glitterEnabled), "m_glitterEnabled"},
					{offsetof(PlayLayer, m_bgEffectDisabled), sizeof(PlayLayer::m_bgEffectDisabled), "m_bgEffectDisabled"},
					{offsetof(PlayLayer, m_unk3906), sizeof(PlayLayer::m_unk3906), "m_unk3906"},
					{offsetof(PlayLayer, m_isPaused), sizeof(PlayLayer::m_isPaused), "m_isPaused"},
					{offsetof(PlayLayer, m_disableGravityEffect), sizeof(PlayLayer::m_disableGravityEffect), "m_disableGravityEffect"},
					{offsetof(PlayLayer, m_nextColorKey), sizeof(PlayLayer::m_nextColorKey), "m_nextColorKey"},
					{offsetof(PlayLayer, m_tryPlaceCheckpoint), sizeof(PlayLayer::m_tryPlaceCheckpoint), "m_tryPlaceCheckpoint"},
					{offsetof(PlayLayer, m_musicPrepared), sizeof(PlayLayer::m_musicPrepared), "m_musicPrepared"}
				};
				auto nameAt = [&](size_t off) -> const char* {
					for (auto const& f : lfields)
						if (off >= f.off && off < f.off + f.size) return f.name;
					for (auto const& f : plfields)
						if (off >= f.off && off < f.off + f.size) return f.name;
					return "";
				};
				size_t diffBytes = 0, ranges = 0, i = 0;
				while (i < st.anchorLayerBytes.size()) {
					if (st.anchorLayerBytes[i] == now[i]) { i++; continue; }
					const size_t start = i;
					while (i < st.anchorLayerBytes.size() && st.anchorLayerBytes[i] != now[i]) i++;
					diffBytes += (i - start); ranges++;
					const char* nm = nameAt(start);
					// Containers and pointers legitimately differ; only scalar
					// physics fields are actionable, so name them all and let the
					// list be filtered by hand.
					if (ranges <= 60)
						log::info("    L +0x{:04X}..0x{:04X} ({} bytes) {}",
						          start, i - 1, i - start, nm);
				}
				log::info("Probe 4a-layer: PlayLayer is {} bytes (GJBaseGameLayer prefix {}); "
				          "{} differ across {} range(s)",
				          sizeof(PlayLayer), sizeof(GJBaseGameLayer), diffBytes, ranges);
			}

			log::info("Probe 4b: restored, replaying the same {} steps", st.restoreLen);
			return false;
		}

		finishRestoreTest();
		return false;
	}

	// Push a decision at the current state and take the first branch.
	void solverPushDecision() {
		auto& sv = Solver::get();
		auto* pl = PlayLayer::get();

		Decision d;
		d.step = sv.step;
		if (auto* p = m_player1) {
			d.airPolicy = p->m_isShip || p->m_isBird || p->m_isDart || p->m_isSwing;
		}
		if (!g_config.noSavestates) {
			if (CheckpointObject* cp = pl->createCheckpoint()) { cp->retain(); d.cp = cp; }
		}

		if (auto* pp = m_player1) {
			d.gameModeChangedTime = pp->m_gameModeChangedTime;
			d.unkA29              = pp->m_unkA29;
		}
		d.layerExtraDelta  = m_extraDelta;
		d.layerTimePlayed  = m_timePlayed;
		d.layerTimestamp   = m_timestamp;
		d.layerTickIndex   = m_tickIndex;
		d.layerClickIndex  = m_clickIndex;
		d.layerResumeTimer = m_resumeTimer;
		d.layerJumping     = m_jumping;
		d.enteringHold  = sv.hold;
		d.togglesBefore = d.airPolicy ? sv.togglesUsed : 0;

		// Move ordering. Both branches are still explored (subject to the toggle
		// budget), so this changes only which is tried FIRST.
		if (d.airPolicy) {
			// Continue whatever we were doing. A sustained hold then costs one
			// decision instead of many consecutive lucky flips, and the cheap
			// branch is the simple one.
			//
			// No physics rule here on purpose. The previous attempt preferred
			// "hold when m_isOnGround" to lift off the floor - but in ship mode
			// m_isOnGround is also true against the CEILING, so it held the ship
			// harder into it and best% went backwards, 35.23% -> 33.78%.
			d.choice = sv.hold;
		} else {
			// Cube: no-press is the likelier correct branch (~10% jump rate in
			// the BC dataset), and this policy already clears 35% of the level.
			d.choice = false;
		}
		d.tried = d.choice ? (1u << 1) : (1u << 0);

		sv.stack.push_back(d);
		sv.hold       = d.choice;
		sv.lastBranch = sv.step;
	}

	// The committed prefix, recomputed against the CURRENT stack every time.
	//
	// Decisions older than commitLookbackSteps behind the best progress are
	// frozen: new progress is evidence the prefix works, so re-searching it is
	// waste (measured: 2.9M steps re-deriving a solved cube path). A minimum
	// window is always left mutable so the search can never freeze itself.
	size_t solverCommitFloor() {
		auto& sv = Solver::get();
		if (sv.stack.empty()) return 0;

		// Prefer the last mode transition at or before the frontier. The game
		// tells us where one section ends and the next begins, and that is the
		// right unit to commit: measured, a 1200-step (5 s) window left ~870
		// steps of solved CUBE mutable, and the search burned 3.27M steps there
		// re-deriving it while the toggle budget never once deepened.
		//
		// The transition decision itself stays mutable so the search can still
		// choose its hold state from the exact step the section begins.
		size_t floorIdx = 0;
		bool anchored = false;
		for (size_t i = sv.stack.size(); i-- > 0; ) {
			if (sv.stack[i].modeTransition && sv.stack[i].step <= sv.bestStep) {
				floorIdx = i;
				anchored = true;
				break;
			}
		}

		// Take the LATER of the transition anchor and a sliding window behind the
		// frontier, so the floor keeps advancing WITHIN a long section as
		// progress is made. Anchoring only at the section start left all ~82
		// ship decisions mutable forever, so the toggle budget was spent
		// re-deriving the solved part of the ship instead of extending it.
		(void)anchored;
		if (sv.bestStep > g_config.commitLookbackSteps) {
			const int cutoff = sv.bestStep - g_config.commitLookbackSteps;
			size_t windowIdx = 0;
			while (windowIdx < sv.stack.size() && sv.stack[windowIdx].step < cutoff) windowIdx++;
			floorIdx = std::max(floorIdx, windowIdx);
		}

		// Never freeze the whole stack.
		const size_t maxFloor = sv.stack.size() > 16 ? sv.stack.size() - 16 : 0;
		return std::min(floorIdx, maxFloor);
	}

	// Restore to a decision's captured state. loadFromCheckpoint alone restores
	// position but does not revive a dead player, so this drives practice
	// mode's own respawn, which does both.
	// Restore, then realign by one physics step.
	//
	// MEASURED (Probe 4b): createCheckpoint captures the state after step N, but
	// the practice respawn returns to the state after step N-1 - segment B was
	// offset from segment A by exactly one step, with B[n+1] == A[n] bit for
	// bit on every field. The restore is faithful; it just lands a frame early.
	//
	// Uncorrected, every restore shifts the timeline by a frame, so the search
	// calibrates its input timing to a state one step off from what a clean
	// replay produces. That is why a 20,425-step solution died at step 1222 on
	// verification. One forward step reproduces the captured state exactly.
	void applyExtraState(Solver::PendingExtra const& e) {
		if (auto* p = m_player1) {
			p->m_gameModeChangedTime = e.gameModeChangedTime;
			p->m_unkA29              = e.unkA29;
		}
		m_extraDelta  = e.extraDelta;
		m_timePlayed  = e.timePlayed;
		m_timestamp   = e.timestamp;
		m_tickIndex   = e.tickIndex;
		m_clickIndex  = e.clickIndex;
		m_resumeTimer = e.resumeTimer;
		m_jumping     = e.jumping;
		if (auto* pl2 = PlayLayer::get()) {
			pl2->m_attemptTime     = e.attemptTime;
			pl2->m_bestAttemptTime = e.bestAttemptTime;
			pl2->m_currentTime     = e.currentTime;
			pl2->m_hasJumped       = e.hasJumped;
		}
	}

	void solverRestoreCheckpoint(CheckpointObject* cp, bool realignHold) {
		auto* pl = PlayLayer::get();
		if (!pl || !cp) return;
		if (auto* arr = pl->m_checkpointArray) {
			arr->removeAllObjects();
			arr->addObject(cp);
		}
		auto& ps = ProbeState::get();
		ps.solverRestoring = true;
		pl->resetLevel();
		ps.solverRestoring = false;

		// The checkpoint does NOT store the held-button state, so the respawn
		// always comes back with the button released - and handleButton takes a
		// step to take effect. In ship mode hold controls acceleration on every
		// frame, so the first step after a restore ran with the wrong hold.
		//
		// MEASURED: restores were sound at step 480 (cube, mid-air, input
		// irrelevant) and unsound at 18135 (ship), where segment B differed from
		// A in yVelocity and m_jumpBuffered at the very first row. pushButton /
		// releaseButton act on the player directly, with no queue latency.
		if (auto* p = m_player1) {
			ps.injecting = true;
			if (realignHold) p->pushButton(PlayerButton::Jump);
			else             p->releaseButton(PlayerButton::Jump);
			ps.injecting  = false;
			ps.isHolding  = realignHold;
		}

		applyInput(realignHold);
		GJBaseGameLayer::update(static_cast<float>(kPhysicsDt));

		// Apply the dropped fields AFTER the realign step.
		//
		// Ordering matters and I had it wrong: capture happens at step S, the
		// restore lands at S-1, and the realign advances to S. Writing the
		// step-S values BEFORE the realign meant that step advanced them past S
		// and ran its physics on rolled-back time - which made the PlayerObject
		// delta worse, 24 bytes to 99.
		{
			auto& sv2 = Solver::get();
			if (sv2.pendingExtra.valid) {
				applyExtraState(sv2.pendingExtra);
				sv2.pendingExtra.valid = false;
			}
		}
	}

	void solverRestoreState(Decision& d) {
		if (!d.cp) return;
		Solver::PendingExtra e;
		e.gameModeChangedTime = d.gameModeChangedTime;
		e.unkA29      = d.unkA29;
		e.extraDelta  = d.layerExtraDelta;
		e.timePlayed  = d.layerTimePlayed;
		e.timestamp   = d.layerTimestamp;
		e.tickIndex   = d.layerTickIndex;
		e.clickIndex  = d.layerClickIndex;
		e.resumeTimer = d.layerResumeTimer;
		e.jumping     = d.layerJumping;
		e.valid       = true;
		Solver::get().pendingExtra = e;
		// The input in effect entering this decision is what was applied on the
		// step being redone.
		solverRestoreCheckpoint(d.cp, d.enteringHold);
		Solver::get().restores++;
	}

	// Unwind to the most recent decision with an untried branch and take it.
	// Returns false when the tree is exhausted.
	bool solverBacktrack() {
		auto& sv = Solver::get();
		auto* pl = PlayLayer::get();

		// Escape the thrash: if best% has not improved in a long time, the
		// culprit is far behind the death and exhausting this subtree is futile.
		// Abandon a chunk of the stack so the search resumes much earlier.
		if (g_config.stallLimit > 0 &&
		    sv.deaths - sv.deathsAtBest > static_cast<uint64_t>(g_config.stallLimit)) {
			// Never rewind below the committed prefix. The previous version only
			// held a floor when the stack TOP was an air decision, so once the
			// air decisions were popped the guard stopped applying and the
			// escape tunnelled straight back into the solved cube section.
			const size_t escFloor = solverCommitFloor();
			size_t maxDrop = sv.stack.size() > escFloor
			               ? sv.stack.size() - escFloor : 0;

			// Also prefer not to rewind out of an air section we are working on.
			if (!sv.stack.empty() && sv.stack.back().airPolicy) {
				size_t firstAir = 0;
				while (firstAir < sv.stack.size() && !sv.stack[firstAir].airPolicy) firstAir++;
				if (firstAir < sv.stack.size()) {
					const size_t keep = firstAir + 1; // keep the entry decision itself
					const size_t airDrop = sv.stack.size() > keep ? sv.stack.size() - keep : 0;
					maxDrop = std::min(maxDrop, airDrop);
				}
			}

			const size_t drop = std::min(static_cast<size_t>(g_config.escapeJump), maxDrop);
			for (size_t i = 0; i < drop; i++) {
				if (sv.stack.back().cp) sv.stack.back().cp->release();
				sv.stack.pop_back();
			}
			sv.escapes++;
			sv.deathsAtBest = sv.deaths;

			// Phase A2: the rewind distance in steps AND seconds. Arithmetic says
			// 200 air decisions should be ~800 steps (3.3 s); observation said
			// 0.1-0.2 s (~24-48 steps). Those cannot both be true, so measure it
			// rather than reason about it.
			const int stepTo    = sv.stack.empty() ? 0 : sv.stack.back().step;
			const int deltaSteps = sv.step - stepTo;

			// Phase A3: composition of what is left on the stack.
			size_t airCount = 0;
			for (auto const& e : sv.stack) if (e.airPolicy) airCount++;

			log::info("Solver: stalled {} deaths at {:.2f}% - abandoned {} decisions, "
			          "depth now {} (escape #{})",
			          g_config.stallLimit, sv.bestPct, drop, sv.stack.size(), sv.escapes);
			log::info("  rewind: step {} -> {} = {} steps = {:.3f} s at 240Hz",
			          sv.step, stepTo, deltaSteps, deltaSteps / 240.0);
			log::info("  stack composition: {} air-policy, {} cube-policy of {} total",
			          airCount, sv.stack.size() - airCount, sv.stack.size());
		}

		// Stop at the committed prefix rather than the bottom of the stack.
		// Popping past it is what let the search waste itself re-deriving a
		// cube path it already had.
		const size_t floor = solverCommitFloor();
		sv.commitDepth = floor; // cached for logging
		while (sv.stack.size() > floor) {
			Decision& d = sv.stack.back();

			// The untried branch of an air decision is by construction the one
			// that toggles. Refuse it when the path is already at the toggle
			// budget; the bound rises when the whole tree at this bound is done.
			if (d.tried != 0b11 && d.airPolicy) {
				const bool wouldToggle = (!d.choice) != d.enteringHold;
				// Relative to the committed prefix: toggles spent inside a
				// frozen, already-working prefix must not count against the
				// budget for the part still being solved.
				const int floorToggles = floor < sv.stack.size()
				                       ? sv.stack[floor].togglesBefore : 0;
				const int spent = d.togglesBefore - floorToggles;
				if (wouldToggle && spent >= g_config.toggleBudget) {
					if (d.cp) d.cp->release();
					sv.stack.pop_back();
					continue;
				}
			}

			if (d.tried != 0b11) {
				d.choice = !d.choice;
				d.tried |= d.choice ? (1u << 1) : (1u << 0);
				// Only AIR decisions spend toggle budget. Counting cube flips too
				// meant ~800 cube decisions exhausted the budget before the ship
				// was even reached, so every air toggle was refused and each
				// budget level "exhausted" after a single path.
				sv.togglesUsed = d.togglesBefore +
					((d.airPolicy && d.choice != d.enteringHold) ? 1 : 0);

				if (g_config.noSavestates) {
					// Replay from frame 0 to this decision. Exact by construction:
					// no checkpoint is involved anywhere in the path.
					sv.resyncMacro.assign(sv.macro.begin(),
					                      sv.macro.begin() + std::min<size_t>(d.step, sv.macro.size()));
					sv.resyncTarget       = static_cast<int>(sv.resyncMacro.size());
					sv.resyncing          = true;
					sv.resyncForBacktrack = true;
					sv.resumeHold         = d.choice;
					sv.resumeToggles      = d.togglesBefore +
						((d.airPolicy && d.choice != d.enteringHold) ? 1 : 0);
					sv.resumeBranch       = d.step;
					sv.step               = 0;
					sv.hold               = false;
					sv.replayBacktracks++;
					ProbeState::get().resetPending = true;
					return true;
				}

				const float xBefore = m_player1 ? m_player1->getPositionX() : -1.f;
				solverRestoreState(d);
				const float xAfter = m_player1 ? m_player1->getPositionX() : -1.f;

				// Decisive check that a restore actually moves the player back.
				// Every death previously cost a full run's worth of steps, which
				// is what you see when the restore is being overridden.
				if (sv.restores <= 5) {
					log::info("Solver: restore #{} to step {} - player X {:.1f} -> {:.1f} "
					          "(dead before: {}, after: {})",
					          sv.restores, d.step, xBefore, xAfter,
					          sv.diedAtX, m_player1 && m_player1->m_isDead ? "yes" : "no");
				}
				sv.step = d.step;
				sv.macro.resize(static_cast<size_t>(d.step), 0);
				sv.hold       = d.choice;
				sv.lastBranch = d.step;
				return true;
			}

			if (d.cp) d.cp->release();
			sv.stack.pop_back();
		}
		return false;
	}

	void solverReport(bool force) {
		auto& sv = Solver::get();
		const uint64_t now = probe::nowTicks();
		// Sampled on wall clock, never per tick.
		if (!force && probe::ticksToMicros(now - sv.lastReport) < 2'000'000.0) return;
		sv.lastReport = now;

		const double secs = probe::ticksToMicros(now - sv.startTicks) / 1e6;
		const double stepsPerSec = secs > 0 ? sv.steps / secs : 0.0;

		// The speed multiplier matters for interpreting anything observed by eye:
		// the search runs many times faster than real time, so a rewind of 3.3
		// GAME seconds plays back in a fraction of a wall-clock second. Game time
		// and wall-clock time are not interchangeable here.
		log::info("Solver: best {:.2f}%  depth {}  deaths {}  restores {}  escapes {}  "
		          "steps {}  budget {}  commit {}  resync {}/{}  ({:.0f} steps/s = {:.1f}x real time, {:.0f} restores/s)",
		          sv.bestPct, sv.stack.size(), sv.deaths, sv.restores, sv.escapes, sv.steps,
		          g_config.toggleBudget, sv.commitDepth, sv.resyncs, sv.resyncFailures, stepsPerSec, stepsPerSec / 240.0,
		          secs > 0 ? sv.restores / secs : 0.0);
	}

	// Start re-deriving the current prefix from frame 0 with no savestates.
	void beginResync() {
		auto& sv = Solver::get();
		if (sv.macro.empty() || sv.step <= 0) return;

		sv.resyncMacro.assign(sv.macro.begin(),
		                      sv.macro.begin() + std::min<size_t>(sv.step, sv.macro.size()));

		// Drop every checkpoint BEFORE the level reset, while the objects they
		// reference still exist. Releasing them afterwards frees dangling
		// pointers - which is what crashed the game on the second resync.
		if (auto* pl = PlayLayer::get()) {
			if (auto* arr = pl->m_checkpointArray) arr->removeAllObjects();
		}
		for (auto& d : sv.stack) if (d.cp) d.cp->release();
		sv.stack.clear();
		sv.commitDepth = 0;
		sv.resyncTarget = static_cast<int>(sv.resyncMacro.size());
		sv.resyncing    = true;
		sv.step         = 0;
		sv.hold         = false;
		sv.togglesUsed  = 0;
		sv.prevAirMode  = false;
		sv.prevOnGround = false;
		log::info("Solver: resync - replaying {} committed steps from frame 0 with no "
		          "savestates to validate the prefix", sv.resyncTarget);
		ProbeState::get().resetPending = true;
	}

	// The prefix replayed clean. Adopt it as ground truth and continue from here.
	void finishResync() {
		auto& sv = Solver::get();
		auto* pl = PlayLayer::get();

		if (sv.resyncForBacktrack) {
			// Landed back at the decision, by replay rather than restore. Keep
			// the stack; only the trajectory was re-derived.
			sv.resyncForBacktrack = false;
			sv.resyncing   = false;
			sv.hold        = sv.resumeHold;
			sv.togglesUsed = sv.resumeToggles;
			sv.lastBranch  = sv.resumeBranch;
			sv.macro.resize(static_cast<size_t>(sv.step), 0);
			return;
		}

		// Stack was already released in beginResync, before the reset.
		sv.stack.clear();

		sv.macro         = sv.resyncMacro;
		sv.lastGoodMacro = sv.resyncMacro;
		sv.lastGoodStep  = sv.step;
		sv.lastResyncStep = sv.step;
		sv.commitDepth   = 0;
		sv.resyncing     = false;
		sv.resyncs++;

		// A fresh base decision, captured from a state that is provably
		// reachable in normal play.
		solverPushDecision();

		log::info("Solver: resync #{} OK at step {} ({:.2f}%) - prefix verified clean, "
		          "stack rebased", sv.resyncs, sv.step, pl ? pl->getCurrentPercent() : 0.f);
	}

	// The prefix did NOT replay clean: accumulated restore error made the search
	// build on a trajectory that does not exist in normal play. Fall back to the
	// last prefix that did replay, and re-search forward from there.
	void failResync(int diedAt) {
		auto& sv = Solver::get();
		sv.resyncFailures++;
		ProbeState::get().lastResyncFailStep = diedAt;
		log::error("Solver: resync FAILED - the prefix died at step {} of {} in clean "
		           "replay. Restore drift had corrupted it; falling back to the last "
		           "verified prefix ({} steps).",
		           diedAt, sv.resyncTarget, sv.lastGoodStep);
		log::error("  Probe 4b (F12) will anchor here - this is the exact window where a "
		           "savestate-derived path stops working in normal play.");

		sv.stack.clear(); // released in beginResync, before the reset
		sv.macro       = sv.lastGoodMacro;
		sv.bestPct     = 0.f;   // re-earned from the verified prefix forward
		sv.bestStep    = sv.lastGoodStep;
		sv.commitDepth = 0;

		if (sv.lastGoodStep > 0) {
			sv.resyncMacro  = sv.lastGoodMacro;
			sv.resyncTarget = sv.lastGoodStep;
			sv.resyncing    = true;
			sv.step         = 0;
			sv.hold         = false;
			ProbeState::get().resetPending = true;
		} else {
			sv.resyncing    = false;
			sv.step         = 0;
			sv.hold         = false;
			sv.lastResyncStep = 0;
			ProbeState::get().resetPending = true;
		}
	}

	// One solver step. Returns false to stop the enclosing per-frame loop
	// (a restore moved the player, so stepping again this frame is invalid).
	bool solverStep() {
		auto& sv = Solver::get();
		auto* pl = PlayLayer::get();
		if (!pl || !m_player1) return false;

		// Resync: replay the committed prefix with NO savestates and no
		// branching. This is the validation - if it survives to the target, the
		// prefix provably works in normal play.
		if (sv.resyncing) {
			const size_t idx = static_cast<size_t>(sv.step);
			const bool pressed = idx < sv.resyncMacro.size() && sv.resyncMacro[idx] != 0;
			applyInput(pressed);
			GJBaseGameLayer::update(static_cast<float>(kPhysicsDt));
			sv.step++;
			sv.steps++;

			if (m_player1->m_isDead) { failResync(sv.step); return false; }
			if (ProbeState::get().finished || sv.step >= sv.resyncTarget) finishResync();
			return true;
		}

		// A restore is supposed to revive the player. If GD instead requires
		// its respawn sequence, the search would stall here forever - so say so
		// loudly once rather than spinning silently.
		if (m_player1->m_isDead) {
			static bool warned = false;
			if (!warned) {
				warned = true;
				log::error("Solver: player still dead after a restore. "
				           "loadFromCheckpoint does not revive on its own; the search "
				           "cannot continue without a respawn path. Stopping.");
			}
			solverReport(true);
			solverWriteMacro("partial.txt");
			sv.clear();
			ProbeState::get().mode = Mode::Idle;
			return false;
		}

		// Apply the current hold and advance exactly one physics step.
		applyInput(sv.hold);
		if (sv.macro.size() <= static_cast<size_t>(sv.step)) sv.macro.resize(sv.step + 1, 0);
		sv.macro[sv.step] = sv.hold ? 1 : 0;

		GJBaseGameLayer::update(static_cast<float>(kPhysicsDt));
		sv.step++;
		sv.steps++;

		const float pct = pl->getCurrentPercent();
		if (pct > sv.bestPct) {
			sv.bestPct      = pct;
			sv.deathsAtBest = sv.deaths; // progress: reset the stall counter

			// Record where the frontier reached. The commit floor is derived
			// from this against the CURRENT stack, never stored as an index.
			if (sv.step > sv.bestStep) sv.bestStep = sv.step;

			// Far enough past the last validation: re-derive the prefix cleanly.
			if (g_config.resyncEnabled &&
			    sv.step > sv.lastResyncStep + g_config.resyncIntervalSteps) {
				beginResync();
				return false;
			}
		}
		solverReport(false);

		if (ProbeState::get().finished) {
			solverReport(true);
			log::info("================ SOLVED ================");
			log::info("  {} steps, {} deaths, {} restores, depth {}",
			          sv.step, sv.deaths, sv.restores, sv.stack.size());
			log::info("  {} resyncs ({} failed) - prefix re-derived from frame 0 without "
			          "savestates {} time(s)", sv.resyncs, sv.resyncFailures, sv.resyncs);
			solverWriteMacro("solution.txt");
			log::info("  Verify by replaying from frame 0 - a solution found via "
			          "savestates only counts if it reproduces in a clean run.");
			log::info("========================================");
			sv.clear();
			ProbeState::get().mode = Mode::Idle;
			return false;
		}

		if (m_player1->m_isDead) {
			sv.deaths++;
			sv.diedAtX = m_player1->getPositionX();

			// Phase A1: what mode is this section actually in, and which branch
			// policy is driving it? The ship riding the floor into the first
			// block is exactly what the CUBE policy would produce, because its
			// jump-commitment logic releases the button the instant the player
			// leaves the ground.
			if (sv.deaths <= 20 || sv.deaths % 500 == 0) {
				auto* p = m_player1;
				const bool air = p->m_isShip || p->m_isBird || p->m_isDart || p->m_isSwing;
				log::info("Solver death #{}: step {} X {:.1f} {:.2f}%  policy={}  "
				          "ship={} ufo={} wave={} swing={} ball={} robot={} spider={}  "
				          "onGround={}/{}/{}/{}  yVel={:.3f}  hold={}",
				          sv.deaths, sv.step, sv.diedAtX, pl->getCurrentPercent(),
				          air ? "AIR" : "CUBE",
				          p->m_isShip, p->m_isBird, p->m_isDart, p->m_isSwing,
				          p->m_isBall, p->m_isRobot, p->m_isSpider,
				          p->m_isOnGround, p->m_isOnGround2, p->m_isOnGround3, p->m_isOnGround4,
				          p->m_yVelocity, sv.hold);
			}

			if (!solverBacktrack()) {
				// Every path within the current toggle budget is exhausted.
				// Deepen and restart rather than giving up: this is what makes
				// the search complete as the bound grows.
				if (g_config.toggleBudget < g_config.maxToggleBudget) {
					g_config.toggleBudget++;
					solverReport(true);
					log::info("Solver: exhausted every path with <= {} air toggles above "
					          "the committed prefix (depth {}) at {:.2f}%. Deepening to {}.",
					          g_config.toggleBudget - 1, sv.commitDepth, sv.bestPct,
					          g_config.toggleBudget);

					// Resume from the committed prefix, not from level start:
					// re-deriving a solved prefix is exactly the waste this fixes.
					if (!sv.stack.empty()) {
						Decision& d = sv.stack.back();

						if (g_config.noSavestates) {
							// No checkpoint exists; reposition by replay, exactly as
							// backtracking does. Without this the solver carried on
							// from whatever state the player happened to be in.
							sv.resyncMacro.assign(sv.macro.begin(),
								sv.macro.begin() + std::min<size_t>(d.step, sv.macro.size()));
							sv.resyncTarget       = static_cast<int>(sv.resyncMacro.size());
							sv.resyncing          = true;
							sv.resyncForBacktrack = true;
							sv.resumeHold         = d.choice;
							sv.resumeToggles      = d.togglesBefore;
							sv.resumeBranch       = d.step;
							sv.step               = 0;
							sv.hold               = false;
							ProbeState::get().resetPending = true;
							return false;
						}

						solverRestoreState(d);
						sv.step        = d.step;
						sv.macro.resize(static_cast<size_t>(d.step), 0);
						sv.hold        = d.choice;
						sv.lastBranch  = d.step;
						sv.togglesUsed = d.togglesBefore +
							((d.airPolicy && d.choice != d.enteringHold) ? 1 : 0);
						sv.deathsAtBest = sv.deaths;
						// Reopen the frontier above the floor at the new budget.
						for (auto& e : sv.stack) {
							if (e.airPolicy) e.tried = e.choice ? (1u << 1) : (1u << 0);
						}
					} else {
						const float keepBest = sv.bestPct;
						sv.clear();
						sv.running    = true;
						sv.bestPct    = keepBest;
						sv.startTicks = probe::nowTicks();
						sv.lastReport = sv.startTicks;
						ProbeState::get().resetPending = true;
					}
				} else {
					solverReport(true);
					log::error("Solver: EXHAUSTED at best {:.2f}% up to {} air toggles.",
					           sv.bestPct, g_config.maxToggleBudget);
					sv.clear();
					ProbeState::get().mode = Mode::Idle;
				}
			}
			return false;
		}

		auto* p = m_player1;
		const bool airMode = p->m_isShip || p->m_isBird || p->m_isDart || p->m_isSwing;

		// A mode change is the most timing-critical frame in a section, and
		// nothing otherwise guarantees a decision lands on it. Force one so the
		// search can pick its hold state from the exact step the ship begins.
		if (airMode != sv.prevAirMode) {
			sv.prevAirMode = airMode;
			solverPushDecision();
			if (!sv.stack.empty()) sv.stack.back().modeTransition = true;
			return true;
		}

		if (airMode) {
			// Hold state controls the trajectory continuously, so the action
			// genuinely is per-segment and persists across the interval.
			if (sv.step - sv.lastBranch >= g_config.airBranchInterval) solverPushDecision();
		} else if (sv.hold) {
			// Committed to a jump. Do NOT branch again while still grounded:
			// re-branching every step overwrote the hold after one step, so no
			// press ever lasted long enough to become a jump. Release once the
			// jump has actually launched.
			if (!p->m_isOnGround) {
				sv.hold = false;
				sv.lastBranch = sv.step;
			}
		} else {
			const bool onGround = p->m_isOnGround;
			const bool onRing   = p->m_touchingRings && p->m_touchingRings->count() > 0;

			// Always branch on a landing or an orb - those are the frames where
			// timing actually matters. Between them, branch on an interval
			// rather than every frame.
			const bool landed      = onGround && !sv.prevOnGround;
			const bool intervalDue = onGround &&
				(sv.step - sv.lastBranch >= g_config.groundBranchInterval);

			if (landed || onRing || intervalDue) solverPushDecision();
			sv.prevOnGround = onGround;
		}

		if (sv.stack.size() > static_cast<size_t>(g_config.maxStackDepth)) {
			log::error("Solver: stack depth {} exceeded cap {} (~{} MB of checkpoints). "
			           "Stopping - the branching policy is too fine-grained.",
			           sv.stack.size(), g_config.maxStackDepth,
			           (sv.stack.size() * 22) / 1024);
			solverReport(true);
			solverWriteMacro("partial.txt");
			sv.clear();
			ProbeState::get().mode = Mode::Idle;
			return false;
		}
		return true;
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
			st.solverWantsReset = true;
			st.resetPerAttempt();
			pl->resetLevelFromStart();
			st.solverWantsReset = false;
			return;
		}

		// The solver deliberately runs while the player is dead - that is
		// exactly when it needs to backtrack and respawn. Gating it behind
		// !m_isDead froze the search on the first death.
		const bool solving = st.mode == Mode::Solve && Solver::get().running;
		const bool active  = pl && m_player1 && (solving || (!m_player1->m_isDead && !st.finished));

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
		if (st.mode == Mode::Verify) {
			const size_t idx = static_cast<size_t>(st.stepCounter);
			if (idx >= st.scripted.size()) {
				finishVerify(st.finished, static_cast<int>(idx),
				             pl->getCurrentPercent());
				return;
			}
			const int steps = g_config.physicsFix ? g_config.stepsPerFrame : 1;
			for (int i = 0; i < steps; i++) {
				const size_t k = static_cast<size_t>(st.stepCounter);
				if (k >= st.scripted.size()) break;
				applyInput(st.scripted[k] != 0);
				GJBaseGameLayer::update(g_config.physicsFix
				                        ? static_cast<float>(kPhysicsDt) : dt);
				st.stepCounter++;
				if (st.finished) {
					finishVerify(true, static_cast<int>(st.stepCounter),
					             pl->getCurrentPercent());
					return;
				}
				if (m_player1 && m_player1->m_isDead) {
					finishVerify(false, static_cast<int>(st.stepCounter),
					             pl->getCurrentPercent());
					return;
				}
			}
			return;
		}

		// The solver runs its own loop: as many steps per rendered frame as the
		// budget allows, rather than real-time pace.
		if (st.mode == Mode::Solve) {
			auto& sv = Solver::get();
			const uint64_t frameStart = probe::nowTicks();
			while (sv.running) {
				if (!solverStep()) break;
				if (probe::ticksToMicros(probe::nowTicks() - frameStart) > kSolverFrameBudgetUs) break;
			}
			return;
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

			if (st.mode == Mode::RestoreTest && !driveRestoreTest()) break;

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

		// A solver left running across a level change would search one level
		// using another's macro and checkpoints. Tear it down first, while its
		// checkpoints still reference objects from the level they came from.
		if (Solver::get().running) {
			log::warn("Solver: stopping - level changed while a search was running");
			Solver::get().clear();
		}

		ProbeState::get().resetPerLevel();

		// Namespace this level's files. Sanitised so it is always a safe filename.
		{
			std::string key = level ? std::string(level->m_levelName) : std::string();
			std::string safe;
			for (char c : key) {
				if (std::isalnum(static_cast<unsigned char>(c))) safe.push_back(c);
				else if (c == ' ' || c == '-' || c == '_') safe.push_back('_');
			}
			if (safe.empty()) safe = "level";
			ProbeState::get().levelKey = safe;
		}
		log::info("gd-solver: level '{}' loaded - output files are namespaced as {}_*",
		          ProbeState::get().levelKey, ProbeState::get().levelKey);
		log::info("  F2 = solve/stop, F4 = verify, F5 = record input, F6 = load input, "
		          "F7 = dump trace, F8 = toggle resync, F11 = savestate cost, "
		          "F12 = restore fidelity");
		return true;
	}

	// GD schedules this after a death, and in normal mode it resets the level
	// to the START - overriding whatever we restored. That is why every death
	// cost a full run's worth of steps. While the solver is driving restores
	// itself, GD's automatic reset must not fire.
	void delayedResetLevel() {
		if (Solver::get().running) return;
		PlayLayer::delayedResetLevel();
	}

	void resetLevel() {
		auto& st = ProbeState::get();

		// The only resets during a search are ones the solver asked for: either
		// starting a fresh attempt, or respawning to a chosen checkpoint.
		if (Solver::get().running && !st.solverWantsReset && !st.solverRestoring) return;

		const bool restoring = st.solverRestoring;
		st.solverWantsReset = false;

		PlayLayer::resetLevel();

		// A restore-respawn must not clear the solver's step counter or trace.
		if (restoring) return;

		st.resetPerAttempt();

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
		st.endAnimStep = st.stepCounter;
		st.finished    = true;
		// Phase 0 item 7. The solver's only success path runs through this, so
		// confirm which signal fires and when rather than assuming.
		log::info("COMPLETION: playEndAnimationToPos fired at solver step {} ({:.2f}%)",
		          Solver::get().running ? Solver::get().step : st.stepCounter,
		          getCurrentPercent());
	}

	void levelComplete() {
		PlayLayer::levelComplete();
		auto& st = ProbeState::get();
		st.levelCompleteStep = st.stepCounter;
		log::info("COMPLETION: levelComplete fired at solver step {} (endAnim had "
		          "{}fired first)",
		          Solver::get().running ? Solver::get().step : st.stepCounter,
		          st.endAnimStep >= 0 ? "" : "NOT ");
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
