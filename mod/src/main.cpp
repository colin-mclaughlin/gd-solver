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
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <set>
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

// How long verification keeps stepping past the end of a macro, waiting for the
// level to register completion. Half a second is ample; the observed lag was one
// step.
constexpr int kVerifyGraceSteps = 120;

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

	// Ceiling on how long modes 1/2 may go WITHOUT a decision when the corridor
	// ahead is unchanged. Below airBranchInterval nothing branches; above this,
	// everything does.
	//
	// This knob INTERPOLATES between the two branch modes. At 4 it equals
	// airBranchInterval and mode 1 degenerates to mode 0; at 16 it is mode 1 as
	// shipped. They are not two designs, they are the endpoints of one number.
	//
	// MEASURED, and this is what makes it the live question. With the lastGeoLo/Hi
	// carried-state fix, all three modes on Clubstep:
	//
	//   mode 0   SOLVED, 20,274 deaths
	//   mode 1   STALL 58.41%, exhausts budgets 2..9
	//   mode 2   STALL 58.41%, identical wall
	//
	// Same map, same playerH, same hitbox - so the wall is NOT the map lying about
	// clearance, because mode 0 walks past it. And it is not the commit floor or
	// the budget: at the stall the log reads `budget 9  commit 820(w7680)`, leaving
	// ~960 of ~1780 decisions mutable with nine toggles allowed. The search has the
	// freedom to release earlier and exhausts every path it can EXPRESS.
	//
	// So the constraint is expressiveness: through a smooth up-then-down corridor
	// the lookahead window does not change, no decision is offered, and this is the
	// only thing that forces one. Observed under F9: the ship holds too long,
	// releases too late, and is too high entering the cube portal.
	//
	// Undocumented and never swept until now. B cycles it.
	int  airBranchIntervalMax = 16;

	// A corridor narrower than this many player heights is "tight" and keeps the
	// full branch cadence. In player heights rather than units so it means the
	// same thing for every hitbox: 6 is a corridor with under three body-widths
	// of slack either side.
	double geomTightHeights = 6.0;

	// 0 = branch every airBranchInterval steps. 1 = branch only where the corridor
	// ahead CHANGES. 2 = as 1, but full cadence in tight corridors.
	//
	// Default 0. See the MEASURED block at geometryWantsBranch: mode 1 is what
	// cleared Clubstep in 59 seconds on 9803757, where mode 0 stalls at 58.43%.
	// Restored after c57f14d deleted it as unreachable - it was unreachable at the
	// default and load-bearing on the hotkey. M cycles it.
	//
	// DEFAULT IS NOW 1, on a full 15-level suite (2026-09-12 12:38, M x1, nothing
	// else pressed) against the same suite on the old default:
	//
	//                        mode 0          mode 1
	//   Base After Base      STALL 69.09%    318 deaths / 3s    <- first ever solve
	//   Clubstep             20274 / 178s    5805 / 61s
	//   xStep                 7292 /  63s    2901 / 28s
	//   Cant Let Go           1434 /  13s     516 /  5s
	//   Electroman            4053 /  37s    2378 / 25s
	//   Jumper                2216 /  20s     948 /  9s
	//   Time Machine          1379 /  13s     770 /  8s
	//   Electrodynamix         (1m51s)       4892 / 55s
	//   Clutterfunk           3743 /  34s    STALL 55.65%       <- the one regression
	//   ...every other level faster or equal, no exceptions.
	//
	// 14 of 15 either way, but mode 1 solves the level that had NEVER solved and is
	// 2-3.5x faster on everything else. The trade is Base After Base for
	// Clutterfunk, and Clutterfunk is recoverable by cycling M to 0.
	//
	// The default moves rather than staying at 0 with the result recorded in a
	// commit message, because that is exactly how this mechanism was lost: 9803757
	// reads "Clubstep clears - first demon!" describing an M-pressed run while the
	// default stayed 0, so eleven days later c57f14d deleted it as unreachable.
	// A tested configuration that is not the default configuration is a trap.
	//
	// Mode 2 is NOT the fix for Clutterfunk - measured, same 55.65% stall in the
	// same fixed point. Nor is escapeArchiveRestart: A alone reaches 58.98% where
	// default SOLVES, collapsing the commit floor to 0 and running the budget to 15.
	int  geomBranchMode = 1;

	// Restore lastGeoLo/Hi on a reposition instead of letting it carry across.
	//
	// DEFAULT ON as of this commit. Measured both ways over the full 15-level
	// suite, same binary, one keypress apart:
	//
	//                       OFF (a02573c)     ON
	//   board               14/15             14/15
	//   Clutterfunk         STALL 55.65%      2,051 solve
	//   Clubstep            5,805 solve       STALL 58.41%
	//   Base After Base     318               303
	//
	// 10 of 15 levels faster ON, 3 slower, and the 13 levels both solve total
	// 16,572 deaths OFF against 16,575 ON - the same work, redistributed.
	//
	// THE BOARD DOES NOT DECIDE THIS. Correctness does. With it OFF the branch rule
	// compares against state left behind by whatever path was explored last, so the
	// children of a node depend on history rather than on the node:
	//
	//   MEASURED on the committed binary: 0.47%-4.75% of branch-rule evaluations
	//   return a different answer than they would with correctly-restored state -
	//   1,727 of 36,388 on Clubstep, 1,049 of 38,608 on Clutterfunk. With this ON
	//   the same counter reads 0/58,394, so those divergences are real and not an
	//   artefact of the instrument.
	//
	//   CONSEQUENCE, measured: with it OFF Clutterfunk triggered exhaustion ZERO
	//   times across the whole level, budget pinned at 1 through 2,867 deaths. With
	//   it ON it exhausts at 55.65% and deepens. "Exhausted every path with <= N
	//   air toggles" is false when the set of paths is not fixed, so budget
	//   deepening had no completeness argument behind it.
	//
	// Solutions found with it OFF were still real - Clubstep's macro F4-verified
	// from frame 0, zero diverging steps. The problem is not the answers; it is that
	// nothing measured on a history-dependent tree can be trusted as evidence.
	//
	// F toggles it.
	bool geoStateRestore = true;

	// Restore prevAirMode / prevOnGround on a reposition.
	//
	// THE DEFECT: those two remember what mode the player was in, and whether it
	// was on the ground, at the previous step. The solver forces a decision
	// whenever either CHANGES - a mode transition is the most timing-critical
	// frame in a section, and a landing is where jump timing matters. Neither is
	// put back when the search rewinds, so the first step after a rewind compares
	// against whatever the ABANDONED path left behind. That can invent a forced
	// decision that does not belong to the node, or miss one that does.
	//
	// Same class as lastGeoLo/Hi: the children of a node depend on where the
	// search came from rather than on the node. Milder - it can only fire on the
	// first step after a reposition, and corrects itself on the next.
	//
	// DEFAULT ON as of this commit. MEASURED, full suite, one keypress apart:
	//
	//   15/15 either way. Clubstep 5,930 -> 5,558 (-6%). The other fourteen levels
	//   BIT-IDENTICAL. Nothing regressed, so the fix is free.
	//
	// modeStaleDiverged in the report line measures EXPOSURE, not residual: it
	// compares the stale value against the decision's recorded one at reposition
	// time, and restoring fixes the consequence without changing that comparison -
	// the next exploration still ends somewhere with a different context. So it does
	// NOT drop to zero with this on, and it was wrong to expect that. Measured
	// exposure is 63-212 repositions per level, 2.8%-54%.
	//
	// Why so little behavioural change for so much exposure is NOT understood. The
	// plausible account - untested - is that a spurious forced decision costs
	// nothing under continue-first ordering and its first branch reproduces the same
	// trajectory, so it only bites where the search is deep enough to backtrack into
	// it. Clubstep, the deepest, is the only level that moved. 4 toggles it.
	bool modeStateRestore = true;

	// 4b: kill a branch whose reachable band, propagated from the player's ACTUAL
	// vertical velocity, intersects no live space at the horizon.
	//
	// MEASURED, and this is the whole case for it. Reachable band as a fraction of
	// the naive terminal-rate cone, propagated over the motion census from vy = 0:
	//
	//    10 steps  7%     40 steps  26%     120 steps  65%     231 steps  82%
	//
	// At 20 steps it pins a ship to 9 units inside a 247-unit corridor. At the
	// steer's 300-unit lookahead (231 steps) the band is 612-729 units - wider than
	// any corridor, so nothing would ever be prunable. Velocity knowledge lives at
	// the DECISION horizon, not the steering one, which is why this is ~30 steps
	// and not tied to geomLookaheadFixedX.
	//
	// V toggles it. Default off until it has a before/after suite.
	// The steer projects the altitude before asking the map anything:
	//
	//     y += lead * vy;          // geomLeadHold = 8.0, vy = m_yVelocity
	//
	// That is dimensionally wrong. MEASURED from the motion census, dy per step =
	// vy * 0.2279 (median over 917k transitions; 1/0.2279 = 4.389, the same scale
	// Probe 16 cross-checked at 4.4444). A climbing normal ship sits at vy ~ 7.9,
	// so the current expression projects
	//
	//     8 * 7.9  = 63 world units
	//
	// against a TRUE eight-step travel of 8 * 1.800 = 14.4. It overshoots by
	// exactly 1/0.2279, and 63 units is a quarter of the 247-unit mean corridor,
	// applied upward on every climbing air decision of every level.
	//
	// No reading of "8" produces 63: eight steps is 14.4, and projecting all the
	// way to the 300-unit lookahead (231 steps) would be 415. The af008c1 values
	// (8 ship/wave, 4 mini, 2 UFO/swing) were fitted, and mini getting HALF of
	// ship's lead while measurably climbing FASTER (2.118 vs 1.800) is the tell.
	//
	// With this ON, `lead` means STEPS and the conversion is applied, so the
	// projection is where the player will actually be. Default OFF until measured.
	// P toggles it.
	// ABLATION: silence the steer entirely, leaving map ordering, liveness, the
	// commit floor and everything else untouched. It answers one question that
	// decides the ordering - how much is the steer WORTH?
	//
	// It is asked because the steer's constants are mutually compensating and two
	// separate attempts to correct one of them have now failed in opposite
	// directions (velocity 4c's lookahead/reach pair; the lead term's missing
	// vy->dy conversion, which is dimensionally right and costs a level). Replacing
	// the whole construction is the largest build on the list, so its urgency
	// should be measured rather than assumed:
	//
	//   board collapses without the steer  -> it is load-bearing, replace it first
	//   board barely moves                 -> the tangle is low-stakes, and the
	//                                         value is elsewhere
	//
	// J toggles it.
	// Aim at the live window's centre CLAMPED to what the player can physically
	// reach, instead of pre-projecting the altitude with a fitted lead term.
	//
	// This replaces the whole compensating set at once rather than one term at a
	// time, which has now failed twice: velocity 4c's lookahead/reach pair (whose
	// own comment concluded "they have to be fixed together") and the lead term's
	// missing vy->dy conversion, which is dimensionally correct and costs a level
	// because geomClearanceBand and the geomLead values were fitted around it.
	//
	// With this ON the lead projection is NOT applied - the reach cone subsumes it -
	// and the target becomes clamp(window centre, [y+dLo, y+dHi]) from the measured
	// envelope at the player's actual vy.
	//
	// A CLAMP, not an intersection, on purpose. MEASURED: velocity information
	// dissipates past ~120 steps (band is 65% of the naive cone at 120, 82% at 231)
	// and the lookahead is 231 steps, so an intersection would never constrain and
	// the change would be inert. A clamp degrades gracefully - wide at long
	// horizons, binding at short ones - and its whole job is to stop the steer
	// demanding an altitude the player cannot get to, which is what produces
	// "holds too long, arrives too high".
	//
	// ABLATION that justifies replacing rather than deleting: with the steer
	// silenced the board is 12/15, losing Theory of Everything and Electroman - but
	// xStep 2176->1542 and Electrodynamix 4261->2953 both IMPROVE without it.
	// Load-bearing and harmful at once. Q toggles it.
	bool geomReachClamp = false;

	bool geomSteerSilent = false;

	bool geomLeadInSteps = false;

	// dy per step per unit of m_yVelocity. MEASURED, motion census, event-free
	// rows with >= 3 samples: 0.22787 ship, 0.22771 ship mini, 0.22896 ufo,
	// 0.22857 ufo mini - flat across mode and size, as a unit conversion should be.
	double geomVyToDy = 0.2279;

	bool velocityPrune = false;
	int  velocityPruneSteps = 30;

	// Safety factor on the measured envelope. The census is a SAMPLE, so the true
	// extremes can exceed the observed ones; widening is the safe direction because
	// a wider band prunes LESS. Not a tuning knob - lowering it trades soundness
	// for pruning and should not be done without a completeness argument.
	double velocityPruneMargin = 1.10;

	// How long a discrete tap holds the button. Only needs to outlast the
	// measured one-step injection latency so the press registers as an edge;
	// holding longer does nothing in UFO or swing and would only crowd the next
	// tap.
	int tapLengthSteps = 2;

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

	// Replace the escape heuristic's blind rewind with a return to an archived
	// cell. A toggles it; default off so the 14-level baseline is untouched.
	//
	// What the rewind does today: pop `escapeJump` decisions off the stack and
	// resume. It has no idea what is back there. MEASURED on Theory of
	// Everything, route+urgency build: 19 escapes, best%% frozen at 78.90%% for
	// fifty seconds, the ladder escalating 240 -> 3840 while the search rewound
	// 1285 steps into a region it had already refuted. The rewind pops
	// decisions and FORGETS what it learned, so it re-derives the same limit
	// cycle from a slightly different starting depth every time.
	//
	// This is the detachment failure Go-Explore names, and an archive of
	// diverse promising cells is exactly the memory that fixes it. The DFS's own
	// traversal fills the archive as a side effect; on a stall we select a cell,
	// restore to it, and resume the DFS from there.
	//
	// Chosen over a transposition table for a reason that is about failure
	// modes, not power: a bad archive entry is SOFT - a poor representative gets
	// replaced when a better one arrives - where a bad prune is PERMANENT, and
	// deletes a viable state with no way to notice. Pruning also only stops the
	// search REPEATING itself; it does nothing to redirect it, and this log
	// shows a search that repeats and wanders. The table stays on the shelf and
	// composes on top of this later.
	bool escapeArchiveRestart = false;

	// Let tap state go STALE across a reposition instead of restoring it.
	//
	// RESTORED AS A BISECT, not as a recommendation. Default false, which is
	// bit-for-bit the current baseline.
	//
	// The history, which is the whole reason this is back:
	//
	//   5555e42   introduced at true.  Theory of Everything 78.90%.
	//   9803757   CLUBSTEP CLEARS - first demon.  Still true.
	//   6f7562d   flipped to false.
	//   c57f14d   deleted.
	//
	// 6f7562d justified the flip on two levels. Theory of Everything came out
	// equal. Clutterfunk was a NULL TEST and that commit says so in those words -
	// it reports `free 0T`, generating no Tap gate evaluations at all, so it
	// cannot exercise this flag in either position. CLUBSTEP WAS NEVER RE-RUN,
	// and Clubstep is the level in the suite with the heavy UFO section and the
	// level now stalling in one.
	//
	// The mechanism, from 6f7562d's own comment. decisionCost charges Tap as
	// `choice ? 1 : 0`, and the gate asks what the ALTERNATIVE costs, so which
	// branch is free depends on which way the decision was pushed:
	//
	//   pushed choice=false (no tap)  ->  alternative is tap,    costs 1, GATED
	//   pushed choice=true  (tap)     ->  alternative is no-tap, costs 0, FREE
	//
	// Stale tapRemaining means the countdown never fires, sv.hold outlives the
	// tap, and the next Tap decision is pushed HELD - so its alternative is free.
	// Measured at ~8% of UFO branches escaping the toggle budget.
	//
	// Read it as a release valve on a budget that charges every flap 1 while
	// charging a ship climb of any length 1, not as a clever trick. Exempting Tap
	// outright was tried and REFUTED (tapBudgetExempt: Theory of Everything
	// stalled at 33.24% against a 19s clear), so the valve being partial matters.
	//
	// 6f7562d ended "Kept as a flag rather than deleted so a regression on a level
	// with heavy UFO or swing can be bisected against it. Delete once the 14-level
	// suite is green." The suite has never been green - Base After Base has failed
	// throughout - and c57f14d deleted it anyway, removing the bisect that note
	// existed to preserve, for exactly the case that then occurred.
	//
	// MEASURED AND CONTROLLED, 2026-09-12, same build, same session:
	//
	//   Y on    Clubstep SOLVED at 20,274 deaths - and F4-verified from frame 0,
	//           zero diverging steps, the first verified demon on this lineage.
	//   Y off   Clubstep stuck at 71.75% after 27,430 deaths, 35% longer than the
	//           solve took. Not stopped early: run to exhaustion of patience.
	//
	// So the default is now true, and 6f7562d's flip is reverted with the control
	// run it never had.
	//
	// THE MECHANISM ABOVE IS WRONG, and the control is what proves it. If stale
	// tap state worked by making Tap alternatives free, `free ...T` would rise
	// with the flag on. It falls:
	//
	//   Y on     865 free taps of 54,083 gate evaluations   1.6%
	//   Y off   5979 free taps of 96,814 gate evaluations   6.2%
	//
	// What the logs do show: the control spent 31 of its 57 escapes in the ladder's
	// runaway state - window 7680, anchor released, dropping 671 decisions and
	// ~7450 steps on every escape with the floor creeping by exactly 1 each time -
	// while the solving run never left window 240. The flag is not buying budget.
	// It changes move ordering enough to keep the frontier moving, which keeps the
	// escape ladder out of its fixed point.
	//
	// Read that as: this flag is masking an escape-ladder defect, not fixing one.
	// It earns its default by the controlled run above and by nothing else, and it
	// should be revisited once the steer target is fixed - see docs/STEER_TARGET.md.
	//
	// Y toggles it.
	bool tapStateSurvivesRestore = true;

	// Cells held by the solver's archive.
	//
	// Deliberately well below GoExplore's 8000. Each entry holds a checkpoint
	// (~22 KB), and the DFS already keeps one per stack entry - ~3000 on a long
	// level. Probe 5 measured restore cost climbing 43 ms -> 165 ms at 1000 live
	// states, so an archive sized like GoExplore's would risk slowing down every
	// restore on levels that currently work, which is the one regression this
	// change must not cause. 2000 cells is ~44 MB.
	int  solverArchiveCap = 2000;

	// Cells nearer the start than this fraction of best progress are never
	// selected. Restarting at 4%% when the frontier is at 78%% is a valid move
	// the weighting would occasionally make, and it throws away a minute of work.
	double archiveMinPctFrac = 0.35;

	// Never restore directly to an AIR decision: restore the nearest CUBE
	// ancestor (whose checkpoint is exact) and replay the macro forward through
	// the air section.
	//
	// This was retired in 6257b89 on the grounds that the direct-load restore is
	// exact in every mode, which it is - measured on Theory of Everything itself,
	// UFO SOUND and AIR 3/3. MEASURED CONSEQUENCE ANYWAY: Theory of Everything
	// went 78.90% -> 32.49% in that commit, and every search-tuning constant
	// across it is byte-identical, so the hybrid was supplying something beyond a
	// workaround for inexact air restores. Plan 13.13.
	//
	// Note what Probe 4b actually proves: restoring and re-running reproduces
	// what running produced - SELF-CONSISTENCY. It does not prove the restored
	// state equals the state forward simulation reaches. Replay-forward cannot be
	// wrong about that by construction, which is the leading explanation for a
	// gap no amount of soundness testing has closed.
	// Now OFF by default. It was reinstated only to reproduce 30fe2b7's stale
	// tap state (13.15); with tap ordering supplying that effect deliberately it
	// is pure overhead. MEASURED, same session: Time Machine and Clutterfunk both
	// searched a BIT-IDENTICAL tree with it on and off - same deaths, same depth,
	// same solution length - while Clutterfunk went from 169,516 simulated steps
	// to 3,953,558 for that identical search, 34s -> 3m05s. Air decisions carry
	// exact checkpoints again, so a backtrack into air is O(1) instead of a
	// replay from the nearest cube ancestor.
	bool hybridRestore = false;





	// "Falling" threshold, in units of gravity-relative fall speed (positive =
	// moving the way gravity pulls). 0.0 means any downward motion counts.
	//
	// A knob, not a tuned constant: the risk with this rule is over-tapping. A
	// UFO spends most of each cycle falling, so a sign-only test could fire at
	// nearly every decision, spend the whole toggle budget in three decisions
	// and gate everything after. The tapFirst counters in the report line
	// measure that directly - if the fraction is near 1.0, raise this.
	double tapNeedFallSpeed = 0.0;

	// Probe 7: read the level's own geometry.
	//
	// Every search change this session aimed at a target I had INFERRED from the
	// cell census, and the census only records where the solver went - it cannot
	// show where the level is open. Four of those inferences were wrong. This
	// reads the objects directly, so the corridor map is measured.
	//
	// GD classifies its own objects: GameObject::getType() returns a
	// GameObjectType - Solid=0, Hazard=2, Decoration=7, Breakable=21, Slope=25,
	// AnimatedHazard=47, and every portal/pad/ring named individually. So there
	// is no objectID table to maintain and no heuristic to get wrong; the game
	// is the authority on what a block is. getObjectRect() gives an axis-aligned
	// hitbox (getOrientedBox() exists for rotated objects if that ever matters).
	//
	// Bucketed into the SAME 60x30 bands as the cell census, so "where the level
	// is open" and "where the solver has been" overlay directly.
	//
	// Reading only. Nothing here influences a search decision - that comes later
	// and starts as move ORDERING, never pruning, because a misclassified object
	// used for pruning deletes a viable route and nothing reports it.
	// Probe 8: which open space can still reach the end of the level.
	//
	// A fake corridor is a pocket of open space that looks identical to the real
	// route and then terminates. MEASURED at ToE's 78.90% wall: floors at y 930,
	// 1020, 1110, 1200 make three 60-unit corridors, and at x 20417 two 44x85
	// hazards (objID 88) fill the bottom and middle ones while the top is empty.
	// The solver has occupied y32-33 and y35-36 tens of thousands of times and
	// y38-39 never - it cannot tell the three apart, because from inside they
	// are identical until the frame it dies.
	//
	// Geometry can tell them apart. Flood fill backwards from the level end
	// through open cells; anything that cannot reach the end is a dead pocket.
	//
	// The fill allows free movement between adjacent open cells, which no game
	// mode can actually do. That is deliberate and it is the SAFE direction: it
	// over-estimates what is reachable, so it under-reports dead pockets. A cell
	// it marks dead is dead under any physics. A cell it marks live may still be
	// unreachable in practice - which is why this orders moves rather than
	// pruning them.
	bool deadEndProbe = true;

	// Slice width for the interval map. 30 units is the level's own block grid,
	// which is as fine as the geometry itself.
	int geomSliceX = 30;

	// The height a gap must have to count as passable. Deliberately the SMALLEST
	// player hitbox (mini), so the map is permissive and can never reject a gap
	// some mode would fit through. Per-mode height needs portal positions to
	// know which mode is active at a given x - a clean follow-up, not this pass.
	int geomPlayerHeight = 15;


	// Use the map for move ORDERING: at an air decision, try first whichever
	// branch heads toward space that still has a forward route.
	//
	// Ordering, never pruning. A wrong map costs search time and cannot delete a
	// route, because the physics simulation stays the authority on what kills
	// the player. That is the whole reason this is ordering and the dead-cell
	// prune was a mistake.
	//
	// It only fires when the player is NOT already lined up with a live
	// interval, so on an ordinary stretch - one open column, or several live
	// ones - it does nothing and the search behaves exactly as it does today.
	//
	// This is also the corrected form of tapOrderByNeed, which lost 32.49% ->
	// 27.72%. That rule keyed on m_yVelocity and encoded "higher is better",
	// which is false in a corridor whose correct route is the bottom one. This
	// keys on which channel actually continues, so it is right in both
	// directions by construction.
	bool geometryOrdering = true;

	// Probe 10: measure the ceiling the PLAYER actually has, not the one the
	// objects imply.
	//
	// The map's roof is `highest solid object in the whole level + 120`, one
	// global number applied at every x (geometryBuildMap, m.yHi). That is wrong
	// in a way that changed the outcome of a level.
	//
	// Base After Base, x 17970: the wall has a hole at y 225-315 with the cube
	// portal in it, solid at y 315-375, and the map calls y 375+ OPEN and LIVE -
	// the top band is open at 426 of 428 x bands, so the map believes flying
	// along the roof is a route to the finish. Geometrically that is TRUE. The
	// space is really in the level. The player simply cannot get to it: a ship
	// section imposes a ceiling that is not made of objects, and the roof space
	// sits above it. The solver flew up, clearance steering re-centred it into
	// the phantom corridor on every decision, and it never came back down to the
	// portal. With the map off the same level solves in 15s.
	//
	// GD does not expose this as a level constant. m_maxGameplayY is the only
	// global bound and it reads 2790 against level content around 500, so it is
	// a no-op; the real ship ceiling is camera-relative and moves. So measure it
	// instead of guessing at a field.
	//
	// Ceiling contact is unambiguous and needs no game knowledge: the player is
	// RISING, then vertical velocity is forced to zero, and no solid object is
	// there to have caused it. Recording the y where that happens, per x band,
	// gives the reachable roof as a function of x, from real play.
	//
	// Passive and read-only. It changes nothing yet - the point is to find out
	// whether the measured ceiling is far below the map's roof, which is the
	// claim, before clamping anything to it.
	// OFF by default, and this one is the worse of the two: an ORDERED std::map
	// lookup per step. Same story - it was measurement scaffolding left running.
	// W turns it on; the live readout on I does not depend on it.
	bool ceilingProbe = false;

	// Probe 12: live readout of every ceiling candidate while the level is
	// played BY HAND.
	//
	// Every ceiling measurement so far came out of the solver, and all of them
	// were contaminated the same way: the search was stalled, so samples piled
	// up in one x band against one wall and were overwhelmingly ordinary block
	// collisions rather than the invisible ceiling. Two of my conclusions came
	// from that data and both were wrong. A snapshot from a wedged search is the
	// worst available instrument for this.
	//
	// Flying it by hand fixes all of it at once: every section can be touched
	// deliberately, different portals entered on purpose, and the fields watched
	// moving rather than inferred from two aggregated numbers.
	//
	// Manual play only. It logs, and logging has no business in the stepping
	// path, so it refuses to run while a search is going.
	//
	// I toggles it.
	bool liveCeilingReadout = false;

	// Frames between readout lines. A contact always prints.
	int liveCeilingEvery = 30;

	// Crop what the map offers to the band the player can actually occupy.
	// E toggles it.
	//
	// The map's roof is one global number - tallest solid in the level + 120 -
	// so in an air section it offers hundreds of units of space the player is
	// physically barred from. MEASURED on Base After Base: map roof 630, real
	// ceiling 255 to 375 depending on section, so up to 375 units of phantom
	// sky. At x 17970 that phantom space is a second LIVE interval above the
	// wall, clearance steering re-centred the ship into it on every decision,
	// and the search never came back down to the cube portal three blocks below.
	// The same level with the map off solves in 15s.
	//
	// Where the ceiling comes from, and why not from the level or the camera:
	//
	//   Level data would mean reading portals and assuming which ones get taken,
	//   in what order. Fake portals exist precisely to break that, portals stack
	//   at one x, and a wrong static ceiling can mark a REAL route dead and
	//   delete the solution silently.
	//
	//   The camera looked like the answer and is not. MEASURED: over 4312 ceiling
	//   contacts the camera y read constant at 80 while the real ceiling moved
	//   120 units between sections, so whatever m_objectLayer's y is, it is not
	//   the vertical scroll. m_cameraHeight is a flat 320 throughout.
	//
	// Learning it from contact was the previous attempt and it is level
	// dependent: a section whose correct route never touches the roof never
	// teaches it, and that is exactly the tight section where it would matter.
	// Dropped.
	//
	// GD tracks the anchor itself: GJGameState::m_portalY is the y the camera
	// snapped to when the player ACTUALLY ENTERED a portal. That is live state,
	// set by the entry event, so it carries no assumption about which portals
	// exist, which are fake, what order they are in, or how they stack - a portal
	// that is flown past never sets it. GJGameState::m_isFreeMode covers the 2.2
	// free-fly sections where there is no bound at all.
	//
	// The ceiling is then m_portalY + geomCeilingOffset.
	//
	// Deliberately applied at QUERY time and not baked into liveness. Optimistic
	// liveness can only fail to prune - it can never prune something reachable -
	// so leaving the phantom corridor flagged live costs information and cannot
	// lose a solution. A ceiling baked into the flood could.
	// Back ON, now that the anchor is the entered portal rather than a field that
	// read zero. The previous version computed a ceiling of 135 everywhere, which
	// cropped nearly the whole map and merely switched steering off - Base After
	// Base "solved" in 14s, which was just the map-off result wearing a disguise.
	// DEFAULT OFF, after three attempts that each traded one level for another:
	//
	//   clip to band        Base After Base 17s, Time Machine 21s (from 12s)
	//   target check only   Time Machine 14s, Base After Base 59s
	//   drop wholly-outside Time Machine 8s, Clutterfunk STALLS
	//
	// Clutterfunk names the mechanism, and it is not the cropping. Narrowing the
	// crop from 15113 intervals to 319 left it stalling at the identical 55.65%,
	// while turning the band off entirely solves it in 38s - exactly its
	// historical time. The difference in the reports is the TOGGLE BUDGET: with
	// the band off it deepens to 2 and solves; with it on it never leaves 1, so
	// only single-toggle paths are ever allowed and a zigzag ship corridor needs
	// several. 322k steps for less progress than 158k.
	//
	// So the band changes the search's ordering enough that the budget-1 tree
	// stops exhausting. That is a real interaction and it is not understood, which
	// is exactly why this should not ship on.
	//
	// The ceiling VALUE stays - it is read from the game at 0x764, validated over
	// 608811 steps with zero violations, and it is correct. What is unresolved is
	// how to apply it to the map without disturbing the search. E turns it on.
	// Lower the MAP's roof at build time to the highest ceiling any mode portal
	// in the level can produce. K toggles it.
	//
	// Every previous attempt applied the ceiling at QUERY time, cropping intervals
	// per decision, and each traded one level for another because it perturbed
	// steering. Build time is the right place: liveness simply never routes
	// through phantom space, and nothing is clipped while the search runs.
	//
	// The map's roof today is `highest solid in the level + 120` - 630 on Base
	// After Base, against a real ship ceiling of 390. That 240 units of phantom
	// sky is what the ship rides into the wall chasing.
	//
	// Why using portals here is safe when using them elsewhere was not: this takes
	// the MAXIMUM over every mode portal, which is an upper BOUND. A fake portal
	// that is never entered only makes the bound looser, never tighter. The
	// earlier objection was against using one portal to define an exact band, and
	// it does not apply to a maximum.
	//
	// Conservative by construction: if no mode portal is recognised, the roof is
	// left alone. Lowering it wrongly would delete real space, so the failure mode
	// is "no change" rather than "lost route".
	// 0 off, 1 prefix maximum, 2 most-recent portal. Z cycles.
	//
	// MEASURED: a prefix maximum cannot come back down. Clutterfunk's ship portal
	// at x 8652 caps at 390, its ball portal at x 10796 caps at 330, and
	// max(390,330) is 390 - so the ball section keeps the ship section's roof and
	// 330..390 stays phantom and live for 5000 units.
	//
	// That phantom is upstream of the 55.65% stall, not at it. The commit floor
	// freezes the prefix, so a route flown through space that is not really there
	// gets committed, and the ship section downstream can be unsolvable from the
	// state it arrives in. The old broken flat roof of 344 cut almost exactly that
	// band, which is why a roof that was wrong everywhere else solved this level.
	//
	// Mode 2 tracks the most recent portal instead, which is the only form that
	// can express a section being LOWER than the one before it. Its risk is the
	// one that ruled portals out earlier: a fake portal flown past would set the
	// roof from a band never entered, and if that band is lower, real space gets
	// deleted. The difference now is that MAP ROOF TOO LOW detects exactly that,
	// live, against the ceiling read at 0x764.

	// DEFAULT 2 (MOST RECENT) as of this commit - the first 15/15 in the project.
	//
	// This was previously 0 and written off: it measured 11/14 and the ordering had
	// it CUT, on the grounds that Base After Base solved without it. That
	// measurement was taken BEFORE geoStateRestore, on a build whose search tree was
	// history-dependent - up to 4.75% of branch-rule evaluations diverging from what
	// correctly-restored state gives, and exhaustion never once firing on
	// Clutterfunk. It was a correct mechanism judged by a broken instrument.
	//
	// MEASURED on the current default, full suite, reproduced bit-for-bit twice:
	//
	//   ALL 15 LEVELS SOLVE, Clubstep included (5,930 deaths).
	//   On the 14 both configurations solve: 18,626 -> 13,404 deaths, 28% fewer.
	//   Theory of Everything 4215 -> 1324, Electrodynamix 4261 -> 2263,
	//   Jumper 649 -> 436, Time Machine 691 -> 546, Cycles 490 -> 419.
	//   Worse on three: Stereo Madness 216 -> 350, Back on Track 255 -> 288,
	//   Electroman 1938 -> 2024.
	//
	// Why it matters that this is the DEFAULT and not a hotkey: 9803757 recorded
	// "Clubstep clears - first demon!" from an M-pressed run while its default
	// stayed 0, and eleven days later the mechanism was deleted as unreachable.
	// A tested configuration that is not the default configuration gets lost.
	int geomPortalRoof = 2;

	// Kept for the query-time clamp, now default off - see the measurements above
	// it. The ceiling READ at 0x764 stays valid and is what this roof is
	// calibrated against.
	bool ceilingClamp = false;

	// Read the ceiling straight out of the game instead of deriving it.
	//
	// MEASURED: a float at GJBaseGameLayer + 0x764 holds the ship ceiling exactly,
	// on 8 of 8 contacts across 6 portal heights (480, 540, 630, 540, 450, 390),
	// including the ground-clamped case the arithmetic needed a third constant
	// for. Critically it is CONTINUOUS, not reactive: it reads 480 for the whole
	// of one ship section and flips to 540 the instant the next portal is
	// entered, so it is available at every step before any contact. That is what
	// separates it from m_collidedTopMinY, which also equals the ceiling but only
	// after the player has already hit it.
	//
	// Addressed by offset because it has no name. Every named GJGameState field
	// was printed with its runtime offset and none landed on 0x764; several that
	// should carry real values (m_portalY, m_cameraEdgeValue0..3) read 0, which
	// suggests Geode's layout for the by-value m_gameState is partly wrong rather
	// than that these fields are unused. A raw offset is the honest way to use it
	// until the layout is corrected upstream.
	//
	// Guarded two ways, because an unnamed offset is exactly the thing that rots
	// silently on a game update: the layout check below, and a live cross-check
	// against the arithmetic rule that disagrees loudly rather than quietly.
	bool ceilingFromField = true;

	// Byte offset of that float within GJBaseGameLayer.
	int geomCeilingFieldOffset = 0x764;

	// Layout guard. These two were measured on 2.2081 and are checked once per
	// level; if either moves, the struct has changed and the offset above cannot
	// be trusted, so the arithmetic takes over.
	int geomGuardGameStateOff  = 0x1A8;
	int geomGuardCameraHeightOff = 0x3260;

	// The playable band, in world units. 10 blocks, and the floor is derived from
	// the ceiling rather than measured separately - one constant, not two.
	double geomBandHeight = 300.0;

	// The level's ground surface. The band never sinks below it - a portal low
	// enough that its nominal band would clip through the floor gets the band
	// pushed UP to rest on it instead.
	//
	// MEASURED. A portal at y 225, placed exactly three blocks above the floor,
	// has a nominal band of [60, 360] but a measured ceiling of 390 - which is
	// [90, 390], the same band resting on 90. It also retro-explains Base After
	// Base, where the ceiling measured 390 with no object behind it.
	//
	// 90 is a game constant rather than a level one: the player is 30 tall and
	// sits at y 105 at rest, so its underside is at 90.
	double geomGroundY = 90.0;

	// The CEILING is snapped to this grid - one whole block.
	//
	// MEASURED over a custom level built with portals at deliberately chosen
	// heights, two of them nudged off the grid in the editor (one step = 2 units):
	//
	//   portal 345           -> 480 predicted 480   exact
	//   portal 315           -> 450 predicted 450   exact
	//   portal 311  (-2 st)  -> 450 predicted 446   rounded UP
	//   portal 293  (+4 st)  -> 420 predicted 428   rounded DOWN
	//   portal 225           -> 390 predicted 360   the ground clamp, see geomGroundY
	//
	// Confirmed again on a second custom level, 8 portals, ceiling AND floor
	// touched in each: 345, 405, 495, 405, 315 exact; 263 (nudged +4 steps) gave
	// raw 398 rounding DOWN to 390; 251 (nudged -2 steps) gave raw 386 rounding
	// UP to 390. Two nudges bracketing the same block from either side is what
	// makes this the ceiling being snapped rather than the portal - a single
	// nudge could not have separated the two.
	//
	// One editor step is 2 world units.
	//
	// Eliminated by the same runs, all reading 0.00 at every contact:
	// m_cameraHeightOffset, m_targetCameraHeightOffset,
	// m_cameraUnzoomedHeightOffset. getTargetFlyCameraY returns the portal's own
	// y unchanged on every entry, so it is a position echo and not a target.
	double geomCeilingGrid = 30.0;

	// Ceiling height above the entered portal's centre.
	//
	// CONFIRMED by a custom level built for the purpose: entering a ship portal,
	// from anywhere and at any speed, snaps the camera to 3 blocks above the
	// portal and 4 blocks below it. The portal is 3 blocks tall, so from its
	// snapped centre:
	//
	//   ceiling = snap(portalY) + 45 + 90  = snap(portalY) + 135
	//   floor   = ceiling - 300
	//   band    = 300 = 10 blocks, which matches the public figure
	//
	// The arithmetic was never the hard part - getting the portal's y was.
	// GJGameState::m_portalY reads 0 and never populates, so the anchor now comes
	// from the entry event itself: GJBaseGameLayer::playerWillSwitchMode hands
	// over the portal GameObject the player actually touched.
	//
	// That is what makes it assumption-free. A portal flown past never fires the
	// hook, so fake portals, stacked portals and out-of-order portals are not
	// special cases - they simply never set anything.
// SUPERSEDED as a rule by ceilingOffsetForPortal, which reads this same 135
	// out of a 300 band. Kept because Probe 10 still reports against it, and
	// because bandHeightForPortal's default reproduces it exactly for any mode
	// whose band has not been measured yet.
	double geomCeilingOffset = 135.0;

	// How far BEFORE a portal's getPositionX() its cap is allowed to apply.
	//
	// The game re-anchors the band when the player touches the portal, which is
	// 15..31 units before its centre on every case measured, and the slice scan
	// adds up to one slice more on top of that. 60 is two blocks: comfortably
	// past both, and the relaxation takes a MAX so overshooting only leaves a
	// strip too tall rather than deleting anything. 0 restores the old
	// centre-triggered behaviour.
	double geomRoofLeadX = 60.0;

	// x band for the measured ceiling, in world units. Two blocks: fine enough
	// to follow a section, coarse enough to collect several samples per band.
	double ceilingBandX = 60.0;

	// Speak only where the map actually knows something. D cycles it.
	//
	//   0  always      - steer at every air decision (the behaviour until now)
	//   1  discriminating - only where liveness separates something
	//   2  + narrowing   - also where the corridor ahead is much tighter
	//
	// MEASURED, and this is a confirmed causal regression rather than a theory.
	// Base After Base solved in 13s and VERIFIED when the map held 2034
	// intervals. With the current 4199-interval map it stalls at 69.09% for
	// minutes; with geometry ordering off (X) it solves in 15s.
	//
	//   level                  map ON            map OFF
	//   Theory of Everything   19s               stalls 32.49%
	//   Clutterfunk            38s               34s
	//   Base After Base        stalls 69.09%     15s
	//
	// So the map helps on one level and hurts on two, and the one it helps is the
	// one built out of DECEPTIVE geometry - fake corridors, which is exactly what
	// liveness exists to route around.
	//
	// The tell on Base After Base is `dead 0`: its ship section has no dead space
	// anywhere in the map, so liveness has nothing to discriminate there - and
	// steering still fired on 38% of air decisions, choosing directions from a
	// map with no routing information in it and overriding the search's own
	// ordering on every third decision for 7000 deaths.
	//
	// Clearance steering does two jobs and only one of them generalises. Routing
	// between corridors needs the map. Aiming at the middle of a corridor does
	// not, and in an open ship corridor it is positively wrong - the correct ship
	// line very often hugs a wall.
	//
	// Mode 2 exists for where this has to go rather than where it is: a genuinely
	// tight gap in a harder level is a place the map SHOULD speak, and it may not
	// contain any dead space to announce itself with. Narrowing is the other
	// signal that a real choice is coming.
	// REVERTED TO 0. This was built to fix Base After Base by silencing the steer
	// where the map has nothing to discriminate. It did not fix it - both modes
	// stalled at the same 69.09% - and the real cause turned out to be the map's
	// global roof offering phantom sky above the ship ceiling. It was then left
	// on by default for several builds without ever being re-validated, and
	// neither Theory of Everything nor Clutterfunk was run against it. An
	// unmeasured default that changes steering on every level is exactly the kind
	// of thing that quietly corrupts a baseline.
	//
	// Kept as a flag on D, to be tested deliberately against a clean baseline once
	// the ceiling work lands.
	int geomSteerGate = 0;

	// Mode 2: "much tighter" as a fraction of the corridor the player is in now.
	double geomNarrowFrac = 0.5;

	// How far ahead to look for live space, in units. 300 is ten slices, about
	// 230 physics steps - close to the 240-step mutable window, and far enough
	// to see past a fork before reaching it: ToE's corridors split at x 20130
	// and seal at x 20400, so this sees the seal from x 20100.
	// Lookahead as a REACTION TIME in steps, converted to distance with the
	// player's own measured horizontal speed.
	//
	// It used to be a flat 300 units, which silently assumes one speed. At 2x
	// speed the player covers 300 units in half the time, so it gets half the
	// warning; at 3x, a third. Electrodynamix introduces 2x and 3x and stalls
	// three times (34.89%, 73.65%, 94.01% - 67 of its 111 seconds), which is
	// what that would look like.
	//
	// 229 steps is 300 units at 1x, so this is inert on every level that already
	// solves and only changes behaviour where the speed does.
	//
	// Measured rather than read from speed portals on purpose: the player is
	// right here, so its actual dx per step is available and is the truth,
	// including anything we have not modelled. Portals are needed for the map's
	// vertical reach, where the player is NOT there to measure.
	int geomLookaheadSteps = 229;

	// Scale the lookahead with measured speed. OFF: MEASURED, Electrodynamix went
	// from a 1m51s clear to stalling at 92.92%% twice, with dx 1.95 turning the
	// 300-unit lookahead into 447.
	//
	// The reason is a coupling. geomVerticalReach is still 60 units of y per 30
	// units of x, which at 1.5x speed is far too generous - the player has fewer
	// steps to cross those 30 units, so it cannot climb that far - and the fast
	// sections windows are correspondingly too wide. The fixed 300-unit lookahead
	// was COMPENSATING for that: looking a shorter distance ahead made the
	// over-wide windows matter less. Fixing one error alone removed the
	// cancellation.
	//
	// Both constants are distance-based and both are wrong under a speed change,
	// in opposite directions. They have to be fixed together, and reach needs the
	// speed-portal scan because the player is not there to be measured.
	// OFF. MEASURED, three builds:
	//   both off ................. Electrodynamix 1m51s CLEAR
	//   lookahead only ........... 92.92%% stall
	//   lookahead + reach ........ 29.47%% stall
	//
	// The pair does not cancel after all, and the reach half is what does the
	// damage - see geomSpeedReach. Back to the configuration that clears while
	// the climb rate is measured.
	bool geomLookaheadScaleWithSpeed = false;
	int  geomLookaheadFixedX = 300;   // used while the above is off

	// Read the level's speed rather than assuming one. MEASURED from
	// Electrodynamix's own object list: exactly two speed portals in the whole
	// level, objID 202 at x 2353.5 and objID 203 at x 19906.5, and the reported
	// dx per step was 1.61 before the second and 1.95 after - which is Fast and
	// Faster to three decimals. Both are GameObjectType 20, the same type as the
	// level's 208 colour triggers, so objectID is the only usable discriminator.
	bool geomSpeedScan = true;

	// Forward scan. OFF samples ONE slice at x + lookahead, which is what the
	// steer has always done and means a constriction anywhere in between is
	// invisible - at ToE's last ship section the lookahead lands 270 units past
	// the pillar it needs to thread.
	//
	// ON walks every slice from the player to the lookahead, following the
	// corridor it is actually in, and intersects the live intervals as it goes.
	// The running intersection is the set of altitudes that can see straight
	// through; when it would pinch below a player height the scan stops and
	// aims at the last one that fits. That targets the gap the player must
	// thread next, and it needs no constant - the geometry decides where to
	// stop.
	//
	// Deliberately conservative: it ignores that the player can move vertically
	// while crossing, so it never claims a line-of-sight it does not have. It
	// only picks a steering target, never marks anything dead, so being tight
	// cannot produce the false-dead failure that killed the per-speed reach.
	//
	// N toggles it. The prior from this session is that it will NOT help -
	// window propagation delivered the same information at ToE's pillar and was
	// worth 2%, while the escalation ladder was worth 22% - but the two are not
	// the same mechanism and this has never actually been measured.
	bool geomForwardScan = false;

	// Steer only when the gap is about to become UNREACHABLE.
	//
	// The clearance steer asks "am I off-centre?" and answers on a quarter of
	// the window height. That is a position controller: it says where to be and
	// says it constantly, and it cannot know whether the player is physically
	// able to comply, because the same button does opposite things from
	// different vertical velocities.
	//
	// This asks the question a pilot actually asks: given how fast I can climb
	// or fall, and how many steps until that gap, can I still make it if I do
	// nothing? If yes, say nothing. If no, act now.
	//
	// Two properties the measurements say matter. It is VELOCITY AWARE, so it
	// never demands a move the physics will refuse. And it is QUIET by
	// construction - it speaks only at the last moment - which is the single
	// variable that separated every success from every failure this session
	// (36%% steer rate solves; 61%%, 69%%, 93%%, 97.6%% all stall).
	//
	// The speed it divides by is MEASURED during the run, per mode, size and
	// direction, by the same instrumentation that reported ship up 1.800 and
	// down 5.419 on Theory of Everything. No hardcoded physics; until a mode has
	// been observed it falls back to the old clearance rule.
	//
	// U toggles it.
	bool geomUrgencySteer = false;

	// Steer toward the interval the player's OWN corridor leads to, instead of
	// whichever interval happens to contain the projected altitude.
	//
	// The bug this fixes: the steer samples one slice at the lookahead and picks
	// the live interval containing the projected y, with no check that it is
	// reachable from the interval the player is in now. Flying at a wall, the
	// projection lands in a corridor BEHIND that wall and the steer reports
	// "you are fine, you are in live space".
	//
	// MEASURED, both directions in one session: with steering effectively off
	// Theory of Everything stalls at 32.49%% against a 20s clear, so the signal
	// is load-bearing - while Clutterfunk goes 38s to 34s, so it is also wrong
	// often enough to cost time on a level with no deceptive geometry. Right
	// answer, wrong interval.
	//
	// Walks the chain slice by slice, at each step taking the live interval with
	// the largest overlap with the one before it, and steers at the FULL
	// interval the chain arrives at.
	//
	// Full interval, not the running intersection - that distinction is the
	// whole difference from the forward scan, which intersected, produced a
	// narrow tunnel and a proportionally tiny deadband, and fired on 69%% of
	// decisions. Steer rate is the variable that decides whether the map helps
	// (36%% solves; 61%%, 69%%, 93%%, 97.6%% all stall), so the target tightens
	// while the tolerance does not.
	//
	// O toggles it.
	bool geomRouteSteer = false;

	// Act when the required climb rate exceeds this fraction of the fastest the
	// player has been seen to move. Below 1.0 because arriving exactly at the
	// rim with zero margin is not arriving.
	double geomUrgencyFrac = 0.7;


	// Liveness: propagate the live window backward from the end of the level so
	// a gap that leads nowhere can be told from one that leads out. OFF means
	// infinite reach - every free interval connects to every free interval in
	// the next slice, so everything is live and dead-space steering never fires.
	//
	// Switchable because reach has ALWAYS been one fitted constant - 60 per 30
	// units, every mode, every speed, every size, the whole level - and nothing
	// has ever measured what the signal it produces is worth. What survives with
	// it off: exact walls, free intervals and their centres (clearance
	// steering), the player-fits-through test, the ceiling. What dies:
	// fake-corridor routing.
	//
	// The test case is unambiguous. Clutterfunk takes ZERO dead steers and gains
	// entirely from clearance, so it should be unaffected. Theory of Everything
	// uses both. If ToE holds up with this off, the whole reachability model -
	// per-mode reach, (y, vy) propagation, terminal velocity, pads and rings as
	// transitions - is unnecessary and should not be built.
	//
	// CORRECTION to the note above: reach does NOT decide whether a gap is live.
	// The sweep requires two intervals to OVERLAP in y by a player height before
	// they connect at all; reach only widens a gap's window WITHIN its own gap.
	// So with reach infinite the sweep is still a real backward connectivity
	// flood, and fake corridors are still excluded - the bottom corridor is
	// walled off at the pinch and separated from the top by a floor, so nothing
	// coming backward from the end can enter it.
	//
	// What reach actually buys is WINDOW NARROWING: carrying a distant pinch
	// backward. ToE's last ship section is a 300-unit gap where only 90 units
	// lead anywhere, and reach is what propagates that 90.
	//
	// So there are three settings worth measuring, not two:
	//   0 Windowed  - today. Overlap flood + reach-propagated narrowing.
	//   1 Overlap   - infinite reach. Full connectivity flood, NO narrowing and
	//                 no fitted constant anywhere. Fake corridors still routed.
	//   2 AllLive   - liveness off entirely. Clearance only, for reference.
	//
	// L cycles them and rebuilds the map, so one session measures all three.
	//
	// MEASURED on Theory of Everything, all three built from the same level in
	// one session:
	//   WINDOWED  5610 free, 266 dead (4.7%), mean live window 564
	//   OVERLAP   5610 free, 248 dead (4.4%), mean live window 563
	//   ALLLIVE   5610 free,   0 dead (0.0%), mean live window 542
	// and then solved in OVERLAP in 20 SECONDS against a previous best of 21.
	//
	// So reach contributed nothing. 93%% of the dead intervals are found by
	// overlap connectivity alone, the mean window differs by one unit, and the
	// level that depends on liveness most solves at least as fast without it.
	// Fake corridors are excluded because they are walled off at the pinch and
	// separated from the real corridor by a floor, so a strictly backward flood
	// can never enter them - no reach constant is involved in that at all.
	//
	// OVERLAP is now the default and geomVerticalReach is dead weight kept only
	// so WINDOWED stays measurable. What this deletes: the per-mode reach model,
	// (y, vy) propagation, terminal-velocity measurement, a calibration level,
	// and pads/rings as reachability transitions.
	int geomReachMode = 1;

	// Lead term: steer on where the player is HEADING, not where it is, in steps
	// of current vertical velocity.
	//
	// Comparing current y against a window 300 units ahead assumes y will not
	// change over that stretch, which makes the steer a proportional controller
	// on position with no velocity term. On a double integrator - hold
	// accelerates up, release accelerates down - a P controller overshoots, and
	// against a MOVING reference it never catches up at all: tracking a ramp with
	// P alone leaves permanent error.
	//
	// MEASURED: all four remaining stalls are air modes in tunnels that rise and
	// fall - Clutterfunk 56% (mini ship, saw-lined tunnel), Theory of Everything
	// 33% and 77% (UFO, neither a fake corridor), Time Machine's last ship
	// section (up-and-down tunnel through a gravity portal). Not one is a static
	// passage, and not one is a routing failure. That is the signature of lag
	// against a moving target, not of a wrong route.
	//
	// y + lead*vy is where constant velocity puts the player in `lead` steps.
	//
	// Split by MODE CLASS, not tuned per level. A single lead of 8 took Time
	// Machine from 59s to 8 seconds - budget 1, zero escapes, 742 deaths - while
	// roughly doubling deaths on Clutterfunk (4196 -> 7722) and Theory of
	// Everything (7870 -> 15855). That split follows the action model exactly:
	//
	//   Hold modes (ship, wave) accelerate continuously, so vy changes smoothly
	//   and projecting several steps ahead forecasts well. Time Machine's stall
	//   was a rising-and-falling ship tunnel, which is why it benefited most.
	//
	//   Tap modes (UFO, swing) are impulsive: vy jumps at every tap, so sampled
	//   just after one it projects far up and just before one far down. Beyond a
	//   step or two the projection is noise, and steering on noise is what made
	//   ToE's UFO sections worse - its dead-space steers went 254 -> 1071, i.e.
	//   it was being pushed INTO corridors it had been avoiding.
	//
	// Keying on the action model rather than on which levels happen to be under
	// test is the point: mode dynamics generalise, level results do not.
	double geomLeadHold = 8.0;   // ship, wave, normal size
	double geomLeadTap  = 2.0;   // UFO, swing

	// Mini ship and wave move faster and climb at a steeper slope, so the same
	// number of steps of vy projects much further. MEASURED: lead 8 took Time
	// Machine's normal-ship tunnel from 59s to 7s, and left Clutterfunk - whose
	// remaining stall is a MINI ship tunnel - at 1m09s against 49s with no lead
	// at all. Same mode class, opposite result, and the difference between them
	// is size.
	//
	// Shorter rather than longer: a bigger vy over the same horizon overshoots,
	// so the horizon has to come down to compensate.
	double geomLeadMini = 4.0;   // mini ship, mini wave

	// Following the geometry costs no toggle budget; deviating from it still
	// does.
	//
	// The toggle budget was chosen when the solver was blind. It prices every
	// input change identically, so a tap that climbs into the correct corridor
	// costs exactly as much as a random one - which is why ToE's corridors were
	// unreachable until geometry supplied the ordering. What is worth bounding
	// now is not HOW MANY inputs a path uses but HOW FAR it departs from the
	// route the level implies.
	//
	// Setting decisionCost to 0 outright would remove the bound, but the
	// escalation ladder is keyed to it: solverBacktrack stops at the commit
	// floor and returns false, which deepens the budget and reopens the
	// frontier. With nothing to deepen that becomes a spin, so the bound has to
	// stay meaningful rather than be deleted.
	bool geomFreeFollowing = true;

	// How far the player can move vertically while crossing one slice, in units.
	//
	// Used to widen the live window as it propagates backwards: far from a pinch
	// you have room to manoeuvre into it, close to one you must already be
	// lined up. Generous is the SAFE setting for ordering - a window that is too
	// wide simply steers less often, while one that is too narrow steers when it
	// need not. 60 units is two blocks per 30 units of travel.
	// 120 after the Probe 9 sweep. Everything saturates by 120-240 and dead
	// counts barely move past it (ToE 174 -> 172, so the corridors stay
	// flagged), while constrained intervals drop - Clutterfunk 382 -> 356, ToE
	// 262 -> 216. Reach 0 is catastrophic (1627 constrained on Clutterfunk),
	// which is what makes the cap necessary at all.
	int geomVerticalReach = 60;

	// Vertical units per physics step for the window's reach model. 0 falls back
	// to the geomVerticalReach derivation above. 2.118 is the measured mini-ship
	// figure, the largest of the four ship values, so it is the least likely of
	// them to narrow the window past what a ship can actually fly. See the
	// comment at climbPerStep.
	double geomClimbPerStep = 2.118;


	// How far off-centre the player may drift inside the live window before
	// steering fires, as a FRACTION of the window's height.
	//
	// This exists because exact-x taught us something backwards. Making the map
	// accurate made the solver WORSE - Clutterfunk 34s -> stalled at 36.25%, ToE
	// 26s -> stalled at 96.29% - because reach now scales with segment width, so
	// a 500-unit open stretch credits 1000 units of climb and the window widens
	// to fill its whole gap. That is physically right: given 500 units of travel
	// you really can get anywhere.
	//
	// So the window's value was never its accuracy, it was its TIGHTNESS. A
	// pessimistic window approximated "stay away from walls", and stripping the
	// pessimism removed a heuristic that was quietly doing most of the work -
	// which is also why cutting Clutterfunk's steering 7.4%% -> 1.0%% earlier made
	// it four times slower.
	//
	// Two signals were being conflated. LIVENESS - which gaps continue - is
	// sparse, exact, and is what beat ToE's fake corridors. CLEARANCE - stay off
	// the walls - is dense and purely heuristic. This restores clearance
	// deliberately instead of getting it by accident from a wrong model.
	//
	// Relative, not absolute, so it scales: in a 300-unit gap it only fires
	// within ~40 units of a wall, in a 90-unit pillar slot it fires almost at
	// once. 0.5 reproduces the old behaviour exactly (fire only when outside the
	// window), so this is a strict generalisation.
	// 0.25 works but over-fires: Theory of Everything solved in 1m08s with
	// steering on 58.5%% of air decisions, against 26s and 10.7%% on the older
	// grid map. 0.5 is the other endpoint and stalls both levels outright. The
	// useful value is between them and this is the first probe.
	//
	// Tuning against ToE alone would be a mistake: it is the atypical level,
	// the only one in the set with fake corridors (254 dead steers). Clutterfunk
	// took ZERO dead steers and depends on clearance for all of its benefit, so
	// the band is the only thing that matters there. Sweep across levels of
	// different character, not against the one that motivated the feature.
	double geomClearanceBand = 0.25;

	// 0 = the fraction above. 1 = the measured per-decision distance. See
	// steerDeadband. Default 0 so this is inert until it is tested.
	int geomBandMode = 0;

	// 0/0 means the WHOLE level. The dead-end map has always covered everything
	// (slices -1..848 on ToE, x -30..25440 against a 25855-unit level); it was
	// only this raw object listing that was windowed, which made a section look
	// empty when it had simply never been dumped.
	int geomXMin = 0;
	int geomXMax = 0;
	bool geomDumpObjects = true;  // also write the raw per-object list

	// Probe 6: the corridor sweep.
	//
	// Theory of Everything's third fake-corridor section is three two-block gaps
	// separated by one-block floors. MEASURED from the cell census over the last
	// ten x bands before the 78.90% wall: y32 13545 visits, y33 49267, y34 8285,
	// y35 41469, y36 20261, and y37 AND ABOVE EXACTLY ZERO. Against the corridor
	// model - bottom y32-33 | floor y34 | middle y35-36 | floor y37 | top y38-39
	// - the search lives in the bottom and middle and has never once been in the
	// top, which is the one that continues.
	//
	// Two independent claims sit behind a fix, and this probe tests only the
	// first: (1) a UFO that IS in y38-39 through that section survives and goes
	// past 78.90%; (2) the reason it never gets there is that the deciding
	// decision is frozen below the commit floor. Building the search change
	// while (1) is unverified risks solving the wrong problem, so this replays
	// the known 78.90% macro with a climb forced into it and reports how high it
	// reached - no savestates, no restores, no search.
	//
	// The step numbers here are LEVEL-SPECIFIC and that is deliberate: this is a
	// diagnostic, run once and removed. Nothing level-specific goes anywhere
	// near the solver's decisions.
	// Search forward from a verified prefix WITH savestates.
	//
	// That path forced noSavestates because a restore was not trusted to be
	// exact, so every branch replayed from frame 0 - 15,113 steps each on
	// Theory of Everything, about 1.3 branches per second, which is why it was
	// never usable. The distrust is obsolete: Probe 4b reports CUBE 5/5 and AIR
	// 3/3 bit-identical, a full Stereo Madness solve replays with zero diverging
	// steps out of 20,330, and both regression levels searched a BIT-IDENTICAL
	// tree under two different restore mechanisms. With savestates the same
	// search runs at ~100 branches/s.
	//
	// The result is still verified the same way - a macro only counts if it
	// replays clean from frame 0 - so this trades an exactness belt for the
	// braces we already measure.
	bool   verifiedPrefixSavestates = true;

	int    sweepStepBase   = 15100; // first injection step S
	int    sweepStepStride = 100;   // S increments by this
	int    sweepStepCount  = 4;     // how many S values
	int    sweepMaxTaps    = 8;     // k = 0..this
	double sweepCorridorX  = 20160.0; // x band 336, 77.98% - the corridor proper

	// Census of the cells the DFS visits. Read-only: it records bands and counts
	// and nothing reads them back, so it cannot alter a decision the search
	// makes. Its only cost is one hash and one map lookup per step.
	bool cellProbeEnabled = true;

	// Probe 9: measure the vertical motion MODEL, not the trajectory.
	//
	// Everything the map knows is positional. It reads free intervals off the
	// level and asks whether the player is inside a live one, which cannot
	// distinguish the three situations that decide a ship section: needing to
	// climb while already rising, while flat, and while falling. Same position,
	// completely different answers. The measured asymmetry says how much that
	// matters - ship climbs 1.800 units/step and falls 5.986, so it drops 3.3x
	// faster than it can recover.
	//
	// What this records is the one-step transition, which at fixed dt is a
	// FUNCTION:
	//
	//   vy' = f(mode, size, speed, gravity, ground, prevHold, hold, vy)
	//
	// Recording the function rather than trajectories is what removes the need
	// for a custom measurement level. vy is continuous, so any sustained hold
	// walks through the intermediate values on the way - a 20-step hold hands
	// over f at 20 consecutive points, and nothing arrives at vy=+4 without
	// passing +1, +2 and +3. The union over a run is a contiguous BAND, not
	// scattered samples, and iterating f reconstructs any trajectory inside it.
	//
	// It also answers the coverage worry rather than assuming it away: if a
	// section never reaches terminal velocity then terminal velocity is not part
	// of that section's answer, because the player cannot get there either. The
	// band needed is exactly the band traversed. GD's update is vy += a*dt with a
	// clamp, so f is affine with a flat top and terminal reads off the SHAPE of
	// the curve rather than being something we have to sit at.
	//
	// prevHold is in the key because UFO and swing respond to the PRESS EDGE, not
	// the held state - holding a UFO gives one impulse and then gravity. That one
	// field is what lets impulse modes and force modes be measured by the same
	// passive census instead of two different input patterns.
	//
	// Self-validating, and this is the column to read first. f is a function, so
	// one key must yield one vy'. A key that yields two is a key missing a field,
	// and the spread column says so directly instead of us discovering it later
	// through a wrong prune.
	//
	// Passive: one hash and one map lookup per step, the same cost as
	// cellProbeRecord, no allocation past the map's growth, nothing read back and
	// no file I/O. It changes NO search behaviour - using f to bound reachability
	// is a separate later change that this measurement exists to justify or kill.
	//
	// W dumps it; shift is not needed, it also stays collecting.
	// OFF by default. This is a MEASUREMENT, not a feature: it does a hash lookup
	// and insert on every physics step of every level, and it was left switched on
	// after the velocity investigation was parked. W turns it on when wanted.
	bool motionModelEnabled = false;

	// Quantisation of vy for the model key, in GD velocity units. Fine, because
	// this is the input to an affine fit and the clamp point has to be locatable;
	// too coarse and the ramp and the flat top blur into each other, which is the
	// one feature the measurement is for.
	double motionVyBucket = 0.01;

	// Distinct keys kept. A guard against a mode combination we have not
	// anticipated, not an expected limit.
	int motionModelCap = 400000;

	// Ceiling on distinct cells held. ~80 bytes each, so 300k is ~24 MB. Past
	// the cap new cells are counted and dropped rather than stored, which keeps
	// the census honest about having been truncated instead of silently lying.
	int  cellProbeCap     = 300000;

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
	// 1 s. REVERTED from 720 after measurement: a wider default leaves ~3x more
	// decisions mutable at EVERY point in the search, so DFS has far more to
	// exhaust before it can commit and advance. Theory of Everything regressed
	// from 78.90% to 14.91%, stalling early in the level rather than at the
	// corridor. Reach for a distant mistake is the escalation ladder's job; the
	// default should stay tight.
	int commitLookbackSteps = 240;


	int wideningFactor         = 2;
	int maxCommitLookbackSteps = 7680; // 32 s


	// The floor must never collapse to zero. The window is an ABSOLUTE step count
	// behind the frontier, so early in a level a wide window reaches past the
	// start and commits nothing at all - measured: commit 0 with a 7680-step
	// window at 14.91%, after which the search re-derived the whole level and
	// thrashed. At least this fraction of the progress made always stays frozen.
	double minCommittedFraction = 0.5;

	// Give up on a branch that survives this long without a decision point or
	// death, to avoid an unbounded descent.

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








	// Selection rule. Score-ranking is the naive baseline and is EXPECTED to
	// fail on fake-corridor levels: the decoy scores higher early, so ranking by
	// progress drops the correct branch first - which is precisely the greedy
	// commitment that traps the DFS. Bucketed selection keeps a quota per
	// (mode, y band) so both corridors survive. Kept switchable so the
	// prediction can be tested rather than assumed.


	// --- Go-Explore --------------------------------------------------------
	// Cell size. Too fine and the archive explodes with states that are the same
	// situation; too coarse and two genuinely different situations share a cell
	// and one of them is never explored properly. One block is 30 units, which
	// makes these "same block, same rough height, same rough vertical speed".
	// x: the player covers ~1.28 units per step at normal speed (26k units over
	// Stereo Madness's 20,330 steps), so 60 units is ~47 steps of travel - about
	// a third of an episode. Finer than this and the archive is mostly cells
	// that are the same situation one frame apart.
	double goCellX  = 60.0;
	double goCellY  = 30.0;   // one block; platforming precision lives here

	// Vertical velocity is banded on a SIGNED SQUARE ROOT, not linearly.
	// MEASURED range is -15..+15, but a ship's entire flight envelope is
	// |vy| < 2 where small differences decide everything, while a cube jump
	// swings through +-15 where they barely matter. A linear band cannot serve
	// both: at 4.0 the whole ship envelope collapses into two cells, which is
	// precisely the distinction the beam threw away before dying in a ship
	// section. sqrt keeps resolution where control is and compresses the rest.
	double goCellVy = 0.4;








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
	Sweep,        // Probe 6: replay best.txt with a forced climb injected, and
	              // report how high it got - geometry, with no search involved
};

// Which save/restore mechanism the restore test exercises.
enum class RestoreKind {
	Full,        // createCheckpoint + m_checkpointArray + resetLevel (practice respawn)
	PlayerOnly,  // PlayerObject::saveToCheckpoint / loadFromCheckpoint - no level state
	DirectLoad,  // createCheckpoint + PlayLayer::loadFromCheckpoint called DIRECTLY
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

	// Suppresses PlayLayer::destroyPlayer outright. The direct-load restore path
	// needs the player never to die, because loadFromCheckpoint repositions but
	// does not revive - a separate problem that is why this path was abandoned
	// before its FIDELITY was ever measured.
	bool suppressDeath = false;

	// --- Probe 4b: restore fidelity ---
	// Drift-zero at t=0 is necessary but not sufficient. The real question is
	// whether the SAME input from a restored state produces the same future.
	RestoreKind  restoreKind   = RestoreKind::Full;
	// Restore-fidelity sweep: test several anchors in one run and report which
	// game mode each was in. Soundness has only ever been measured at two points
	// (exact in cube at step 480, not exact in ship at 18135), and the hybrid
	// design rests on cube restores being reliably exact. One point is not
	// enough evidence to build on.
	// `mode` is the specific game mode, not just air-vs-cube. Lumping ship, UFO
	// and wave together as "AIR" hid the fact that every air anchor ever tested
	// was ship or wave: no restore had been measured inside a UFO at all until
	// Theory of Everything, and UFO is the one mode where a click adds impulse
	// mid-arc, so its restore behaviour is not implied by ship's.
	struct SweepResult { int anchor; bool air; bool sound; size_t divergeAt; float pct;
	                     const char* mode; };
	std::vector<int>         sweepAnchors;
	size_t                   sweepIndex = 0;
	std::vector<SweepResult> sweepResults;
	bool                     anchorWasAir = false;
	const char*              anchorMode = "CUBE";
	float                    anchorPct = 0.f;

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

	// Fields loadFromCheckpoint does NOT write. Identified by byte-diff, not
	// guessed: they stayed stale through both a bare direct load and a
	// reset-then-load, because nothing in the restore path ever assigns them.
	// Under the practice respawn they do not differ, so the respawn writes them.
	//
	// m_position is a PlayerObject member distinct from the CCNode position -
	// getPositionX() reads EXACT while this differs, which is how a "perfect"
	// boundary check can still diverge on the very next step.
	std::vector<PlayerButtonCommand> aQueuedButtons; // layer input queue at the anchor
	cocos2d::CCPoint aNodePos   = {0.f, 0.f}; // cocos transform at the anchor
	float         aCameraFlip      = 0.f; // mirror transition progress at the anchor
	float         aCameraUnzoomedX = 0.f;
	bool          aUnk322a         = false;
	bool          aUnk3251         = false;
	bool          aHoldingJump  = false; // m_holdingButtons[Jump] at the anchor
	bool          aIsHolding    = false; // our injection bookkeeping at the anchor
	double        aLastJumpTime = 0.0;
	cocos2d::CCPoint aPlayerPos = {0.f, 0.f};
	CheckpointObject* fullCp   = nullptr;
	PlayerCheckpoint* playerCp = nullptr;

	void releaseCheckpoints() {
		// Drop our checkpoint out of GD's array BEFORE releasing it. The restore
		// path swaps it into m_checkpointArray to drive the practice respawn;
		// leaving it there across a level reset means the game holds a pointer to
		// something we freed, and the next cycle's removeAllObjects double-frees
		// it. That crashed the sweep on its third anchor.
		// The direct-load path never puts the object in the array at all, but may
		// leave m_currentCheckpoint pointing at it, so clear that too.
		if (auto* pl = PlayLayer::get()) {
			if (auto* arr = pl->m_checkpointArray) arr->removeAllObjects();
			if (fullCp && pl->m_currentCheckpoint == fullCp) pl->m_currentCheckpoint = nullptr;
		}
		if (fullCp)   { fullCp->release();   fullCp   = nullptr; }
		if (playerCp) { playerCp->release(); playerCp = nullptr; }
	}

	// --- solve-vs-verify divergence (Probe 8) ---------------------------
	//
	// The verify only ever reported where the player DIED. That is not where the
	// path went wrong: a restore can leave the run on a slightly different
	// trajectory that survives for thousands of steps before it finally clips
	// something. Stereo Madness died at step 18855 while every restore the F10
	// sweep sampled was bit-identical - eight samples out of 1466 restores.
	//
	// So keep the trajectory the solver actually flew, and have the clean replay
	// compare against it step by step. The first step that differs is the real
	// defect; the decision at or before it is the restore to suspect.
	std::vector<probe::TraceRow> solvePathTrace;
	std::vector<int>             solveDecisionSteps;

	// The replay's own trajectory, so both can be dumped side by side. Latching
	// only the FIRST divergence was not enough: a single bit that flips and
	// immediately re-converges says something completely different from one that
	// never recovers, and the first version of this could not tell them apart.
	std::vector<probe::TraceRow> verifyPathTrace;
	int  verifyDivergeStep     = -1;  // first step that differs at all
	int  verifyPersistStep     = -1;  // start of the run of differences that never ends
	int  verifyReconvergeStep  = -1;  // where the first divergence healed, if it did
	int  verifyDivergeCount    = 0;   // total differing steps
	bool verifyDivergeLogged   = false;

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
		sweepAnchors.clear();
		sweepIndex = 0;
		sweepResults.clear();
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

void startVerify(bool practiceMode, const char* file = "solution.txt") {
	auto& st = ProbeState::get();
	auto* pl = PlayLayer::get();
	if (!pl) { log::warn("Verify: not in a level"); return; }

	std::string path = levelFilePath(file);
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
	st.verifyDivergeStep    = -1;
	st.verifyPersistStep    = -1;
	st.verifyReconvergeStep = -1;
	st.verifyDivergeCount   = 0;
	st.verifyDivergeLogged  = false;
	st.verifyPathTrace.clear();
	st.verifyPathTrace.reserve(st.scripted.size() + 256);
	log::info("Verify: replaying {} ({} steps) from frame 0, practice mode {}, "
	          "no savestates, no restores. physics_fix={}",
	          file, st.scripted.size(), practiceMode ? "ON" : "OFF", fileFix ? 1 : 0);
	if (!st.solvePathTrace.empty()) {
		log::info("  Comparing against the solver's own trajectory ({} steps). The "
		          "first differing step is where the run actually went wrong - the "
		          "death is only where it finally showed.", st.solvePathTrace.size());
	}
}

// Dump both trajectories side by side so the difference can be read directly
// instead of inferred from one latched step. Raw bits for every numeric field:
// a decimal rendering can compare equal while the doubles differ in the low
// bits, which is exactly the divergence being hunted.
void writeVerifyDiff() {
	auto& st = ProbeState::get();
	if (st.solvePathTrace.empty() || st.verifyPathTrace.empty()) return;

	const std::string path = levelFilePath("verify_diff.csv");
	std::FILE* f = std::fopen(path.c_str(), "wb");
	if (!f) { log::error("  could not write {}", path); return; }

	std::fprintf(f,
		"step,differs,"
		"s_x,v_x,s_y,v_y,s_rot,v_rot,s_speed,v_speed,"
		"s_yvel,v_yvel,s_grav,v_grav,s_flags,v_flags,flag_diff,post_restore,s_pct,v_pct\n");

	const size_t n = std::min(st.solvePathTrace.size(), st.verifyPathTrace.size());
	const uint32_t keep = ~(probe::kInputSourceMask | probe::kSolverOnlyMask);
	for (size_t i = 0; i < n; i++) {
		auto const& a = st.solvePathTrace[i];
		auto const& b = st.verifyPathTrace[i];
		const bool same =
			a.x == b.x && a.y == b.y && a.rotation == b.rotation &&
			a.playerSpeed == b.playerSpeed && a.yVelocity == b.yVelocity &&
			a.gravity == b.gravity && (a.flags & keep) == (b.flags & keep);
		std::fprintf(f,
			"%zu,%d,"
			"%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
			"%.17g,%.17g,%.17g,%.17g,%08X,%08X,%08X,%d,%.4f,%.4f\n",
			i, same ? 0 : 1,
			(double)probe::asFloat(a.x),           (double)probe::asFloat(b.x),
			(double)probe::asFloat(a.y),           (double)probe::asFloat(b.y),
			(double)probe::asFloat(a.rotation),    (double)probe::asFloat(b.rotation),
			(double)probe::asFloat(a.playerSpeed), (double)probe::asFloat(b.playerSpeed),
			probe::asDouble(a.yVelocity),          probe::asDouble(b.yVelocity),
			probe::asDouble(a.gravity),            probe::asDouble(b.gravity),
			a.flags & keep, b.flags & keep, (a.flags ^ b.flags) & keep,
			(a.flags & probe::FlagPostRestore) ? 1 : 0,
			(double)probe::asFloat(a.percent),     (double)probe::asFloat(b.percent));
	}
	std::fclose(f);
	log::info("  Wrote {} rows of solver-vs-replay trajectory to {}", n, path);
}

// Reports where the clean replay departed from the solver's own trajectory.
//
// Shared by BOTH outcomes on purpose. `endStep` is the death step on a failure
// and the final step on a pass; `passed` only changes the wording.
void reportDivergence(int endStep, bool passed) {
	auto& st = ProbeState::get();
	const char* what = passed ? "end of the run" : "death";

	if (st.solvePathTrace.empty()) {
		log::error("  No solver trajectory recorded, so the divergence step is "
		           "unknown. Run F2 to a solve in this session first, then F4.");
		return;
	}
	if (st.verifyDivergeStep < 0) {
		if (!passed) {
			log::error("  ... but the replay NEVER diverged from the solver's own "
			           "trajectory. The path is identical and still dies, so the "
			           "restores are innocent: practice mode or injection timing "
			           "differs between the search and the replay.");
		}
		return;
	}

	const int dv = st.verifyDivergeStep;
	log::error("  FIRST divergence at step {} ({} of {} steps differ in total).",
	           dv, st.verifyDivergeCount, endStep);
	if (st.verifyReconvergeStep >= 0) {
		log::error("  ... and it HEALED at step {}. A divergence that recovers is not "
		           "what killed the run; look at the permanent one.",
		           st.verifyReconvergeStep);
	}

	// Whether the FIRST divergence follows a restore matters even when it heals:
	// a transient that follows a restore is still a restore defect, just one the
	// level geometry forgave.
	const bool firstWasRestore =
		static_cast<size_t>(dv) < st.solvePathTrace.size() &&
		(st.solvePathTrace[dv].flags & probe::FlagPostRestore) != 0;
	log::error("  That first divergence {} the first step after a restore.",
	           firstWasRestore ? "IS" : "is NOT");

	if (st.verifyPersistStep >= 0) {
		const int pv = st.verifyPersistStep;
		int suspect = -1;
		for (int ds : st.solveDecisionSteps) if (ds <= pv && ds > suspect) suspect = ds;
		log::error("  PERMANENT divergence begins at step {}, {} steps ({:.2f} s) before "
		           "the {} - the run never recovered from this one.",
		           pv, endStep - pv, (endStep - pv) / 240.0, what);
		const bool wasRestore =
			static_cast<size_t>(pv) < st.solvePathTrace.size() &&
			(st.solvePathTrace[pv].flags & probe::FlagPostRestore) != 0;
		log::error("  That step {} the first step after a restore.",
		           wasRestore ? "IS" : "is NOT");
		if (!wasRestore) {
			log::error("  So this is NOT a restore-fidelity defect: the solver and the "
			           "replay ran the same input from the same state and got "
			           "different answers.");
		}
		if (suspect >= 0) {
			log::error("  (Nearest decision at or before it: step {}, {} steps earlier "
			           "- decisions are dense, so this alone means little.)",
			           suspect, pv - suspect);
		}
	} else {
		log::error("  Every divergence healed - the two runs agreed at the final step.");
	}
	writeVerifyDiff();
}

void pathAuditReport();   // Probe 17, defined with the map probes below
void expressAuditStep(PlayerObject* p, bool pressed, uint64_t k);  // Probe 20
void bestDecisionDump();  // Probe 21
void expressAuditReport();

void finishVerify(bool completed, int atStep, float pct) {
	auto& st = ProbeState::get();
	auto* plv = PlayLayer::get();
	log::info("================ VERIFICATION (practice {}) ================",
	          plv && plv->m_isPracticeMode ? "ON" : "OFF");
	// Whether the replay passed or died, every step of it was checked
	// against the map, and the disagreement is the point.
	pathAuditReport();
	if (completed) {
		if (st.verifyDivergeStep >= 0) {
			// A pass is not the same as a match. The search flew one trajectory and
			// the macro reproduces a different one; this run survived anyway, which
			// makes it luck rather than correctness. Reporting divergence only on
			// failure hid precisely this case - Time Machine passed while diverging
			// and said nothing about it.
			log::warn("  PASSED, BUT DIVERGED - the macro clears the level, yet the "
			          "replay did not follow the solver's own trajectory.");
			log::info("  {} steps, reached {:.2f}%", atStep, pct);
			log::warn("  A surviving divergence is a restore defect that happened not "
			          "to be fatal here. Treat this as a bug, not a pass.");
			reportDivergence(atStep, true);
		} else {
			log::info("  PASSED - the macro clears the level from frame 0 in normal mode.");
			log::info("  {} steps, reached {:.2f}%", atStep, pct);
			log::info("  Zero diverging steps: the clean replay reproduced the solver's "
			          "trajectory exactly. This is an independently reproducible "
			          "solution, not a savestate artifact.");
		}
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

		reportDivergence(atStep, false);
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

// Fields written back after a direct loadFromCheckpoint.
//
// loadFromCheckpoint assigns only a SUBSET of PlayerObject, leaving the rest at
// whatever value it already held - and a reset beforehand only changes WHICH
// wrong value that is, since nothing assigns the captured one. Enumerating the
// complement two fields at a time found m_lastJumpTime and m_position, then
// m_lastPortalPos and m_isOnGround2; this ends that by writing all of them.
//
// Generated from bindings/2.2081. Pointers, containers, and cosmetic or
// anti-cheat fields (particles, streaks, glow, SeedValueRSV) are excluded - the
// latter re-seed on every access, so forcing them is meaningless and their raw
// bytes always differ.
struct PlayerFieldSpan { size_t off, size; };
static const PlayerFieldSpan kPlayerFields[] = {
	{offsetof(PlayerObject, m_wasTeleported), sizeof(PlayerObject::m_wasTeleported)},
	{offsetof(PlayerObject, m_fixGravityBug), sizeof(PlayerObject::m_fixGravityBug)},
	{offsetof(PlayerObject, m_reverseSync), sizeof(PlayerObject::m_reverseSync)},
	{offsetof(PlayerObject, m_yVelocityBeforeSlope), sizeof(PlayerObject::m_yVelocityBeforeSlope)},
	{offsetof(PlayerObject, m_dashX), sizeof(PlayerObject::m_dashX)},
	{offsetof(PlayerObject, m_dashY), sizeof(PlayerObject::m_dashY)},
	{offsetof(PlayerObject, m_dashAngle), sizeof(PlayerObject::m_dashAngle)},
	{offsetof(PlayerObject, m_dashStartTime), sizeof(PlayerObject::m_dashStartTime)},
	{offsetof(PlayerObject, m_slopeStartTime), sizeof(PlayerObject::m_slopeStartTime)},
	{offsetof(PlayerObject, m_lastCollisionBottom), sizeof(PlayerObject::m_lastCollisionBottom)},
	{offsetof(PlayerObject, m_lastCollisionTop), sizeof(PlayerObject::m_lastCollisionTop)},
	{offsetof(PlayerObject, m_lastCollisionLeft), sizeof(PlayerObject::m_lastCollisionLeft)},
	{offsetof(PlayerObject, m_lastCollisionRight), sizeof(PlayerObject::m_lastCollisionRight)},
	{offsetof(PlayerObject, m_unk50C), sizeof(PlayerObject::m_unk50C)},
	{offsetof(PlayerObject, m_unk510), sizeof(PlayerObject::m_unk510)},
	{offsetof(PlayerObject, m_slopeAngle), sizeof(PlayerObject::m_slopeAngle)},
	{offsetof(PlayerObject, m_slopeSlidingMaybeRotated), sizeof(PlayerObject::m_slopeSlidingMaybeRotated)},
	{offsetof(PlayerObject, m_quickCheckpointMode), sizeof(PlayerObject::m_quickCheckpointMode)},
	{offsetof(PlayerObject, m_maybeSavedPlayerFrame), sizeof(PlayerObject::m_maybeSavedPlayerFrame)},
	{offsetof(PlayerObject, m_scaleXRelated2), sizeof(PlayerObject::m_scaleXRelated2)},
	{offsetof(PlayerObject, m_groundYVelocity), sizeof(PlayerObject::m_groundYVelocity)},
	{offsetof(PlayerObject, m_yVelocityRelated), sizeof(PlayerObject::m_yVelocityRelated)},
	{offsetof(PlayerObject, m_scaleXRelated3), sizeof(PlayerObject::m_scaleXRelated3)},
	{offsetof(PlayerObject, m_scaleXRelated4), sizeof(PlayerObject::m_scaleXRelated4)},
	{offsetof(PlayerObject, m_scaleXRelated5), sizeof(PlayerObject::m_scaleXRelated5)},
	{offsetof(PlayerObject, m_isCollidingWithSlope), sizeof(PlayerObject::m_isCollidingWithSlope)},
	{offsetof(PlayerObject, m_isBallRotating), sizeof(PlayerObject::m_isBallRotating)},
	{offsetof(PlayerObject, m_unk669), sizeof(PlayerObject::m_unk669)},
	{offsetof(PlayerObject, m_collidingWithSlopeId), sizeof(PlayerObject::m_collidingWithSlopeId)},
	{offsetof(PlayerObject, m_slopeFlipGravityRelated), sizeof(PlayerObject::m_slopeFlipGravityRelated)},
	{offsetof(PlayerObject, m_slopeAngleRadians), sizeof(PlayerObject::m_slopeAngleRadians)},
	{offsetof(PlayerObject, m_rotationSpeed), sizeof(PlayerObject::m_rotationSpeed)},
	{offsetof(PlayerObject, m_rotateSpeed), sizeof(PlayerObject::m_rotateSpeed)},
	{offsetof(PlayerObject, m_isRotating), sizeof(PlayerObject::m_isRotating)},
	{offsetof(PlayerObject, m_isBallRotating2), sizeof(PlayerObject::m_isBallRotating2)},
	{offsetof(PlayerObject, m_isHidden), sizeof(PlayerObject::m_isHidden)},
	{offsetof(PlayerObject, m_speedMultiplier), sizeof(PlayerObject::m_speedMultiplier)},
	{offsetof(PlayerObject, m_yStart), sizeof(PlayerObject::m_yStart)},
	{offsetof(PlayerObject, m_gravity), sizeof(PlayerObject::m_gravity)},
	{offsetof(PlayerObject, m_unk648), sizeof(PlayerObject::m_unk648)},
	{offsetof(PlayerObject, m_gameModeChangedTime), sizeof(PlayerObject::m_gameModeChangedTime)},
	{offsetof(PlayerObject, m_padRingRelated), sizeof(PlayerObject::m_padRingRelated)},
	{offsetof(PlayerObject, m_maybeReducedEffects), sizeof(PlayerObject::m_maybeReducedEffects)},
	{offsetof(PlayerObject, m_maybeIsFalling), sizeof(PlayerObject::m_maybeIsFalling)},
	{offsetof(PlayerObject, m_shouldTryPlacingCheckpoint), sizeof(PlayerObject::m_shouldTryPlacingCheckpoint)},
	{offsetof(PlayerObject, m_playEffects), sizeof(PlayerObject::m_playEffects)},
	{offsetof(PlayerObject, m_maybeCanRunIntoBlocks), sizeof(PlayerObject::m_maybeCanRunIntoBlocks)},
	{offsetof(PlayerObject, m_isOnGround3), sizeof(PlayerObject::m_isOnGround3)},
	{offsetof(PlayerObject, m_checkpointTimeout), sizeof(PlayerObject::m_checkpointTimeout)},
	{offsetof(PlayerObject, m_lastCheckpointTime), sizeof(PlayerObject::m_lastCheckpointTime)},
	{offsetof(PlayerObject, m_lastJumpTime), sizeof(PlayerObject::m_lastJumpTime)},
	{offsetof(PlayerObject, m_lastFlipTime), sizeof(PlayerObject::m_lastFlipTime)},
	{offsetof(PlayerObject, m_flashTime), sizeof(PlayerObject::m_flashTime)},
	{offsetof(PlayerObject, m_flashDuration), sizeof(PlayerObject::m_flashDuration)},
	{offsetof(PlayerObject, m_flashDelay), sizeof(PlayerObject::m_flashDelay)},
	{offsetof(PlayerObject, m_lastSpiderFlipTime), sizeof(PlayerObject::m_lastSpiderFlipTime)},
	{offsetof(PlayerObject, m_unkBool5), sizeof(PlayerObject::m_unkBool5)},
	{offsetof(PlayerObject, m_practiceDeathEffect), sizeof(PlayerObject::m_practiceDeathEffect)},
	{offsetof(PlayerObject, m_accelerationOrSpeed), sizeof(PlayerObject::m_accelerationOrSpeed)},
	{offsetof(PlayerObject, m_snapDistance), sizeof(PlayerObject::m_snapDistance)},
	{offsetof(PlayerObject, m_ringJumpRelated), sizeof(PlayerObject::m_ringJumpRelated)},
	{offsetof(PlayerObject, m_onFlyCheckpointTries), sizeof(PlayerObject::m_onFlyCheckpointTries)},
	{offsetof(PlayerObject, m_slopeRotation), sizeof(PlayerObject::m_slopeRotation)},
	{offsetof(PlayerObject, m_currentSlopeYVelocity), sizeof(PlayerObject::m_currentSlopeYVelocity)},
	{offsetof(PlayerObject, m_unk3d0), sizeof(PlayerObject::m_unk3d0)},
	{offsetof(PlayerObject, m_blackOrbRelated), sizeof(PlayerObject::m_blackOrbRelated)},
	{offsetof(PlayerObject, m_unk3e0), sizeof(PlayerObject::m_unk3e0)},
	{offsetof(PlayerObject, m_unk3e1), sizeof(PlayerObject::m_unk3e1)},
	{offsetof(PlayerObject, m_isAccelerating), sizeof(PlayerObject::m_isAccelerating)},
	{offsetof(PlayerObject, m_isCurrentSlopeTop), sizeof(PlayerObject::m_isCurrentSlopeTop)},
	{offsetof(PlayerObject, m_collidedTopMinY), sizeof(PlayerObject::m_collidedTopMinY)},
	{offsetof(PlayerObject, m_collidedBottomMaxY), sizeof(PlayerObject::m_collidedBottomMaxY)},
	{offsetof(PlayerObject, m_collidedLeftMaxX), sizeof(PlayerObject::m_collidedLeftMaxX)},
	{offsetof(PlayerObject, m_collidedRightMinX), sizeof(PlayerObject::m_collidedRightMinX)},
	{offsetof(PlayerObject, m_canPlaceCheckpoint), sizeof(PlayerObject::m_canPlaceCheckpoint)},
	{offsetof(PlayerObject, m_maybeIsColliding), sizeof(PlayerObject::m_maybeIsColliding)},
	{offsetof(PlayerObject, m_jumpBuffered), sizeof(PlayerObject::m_jumpBuffered)},
	{offsetof(PlayerObject, m_stateRingJump), sizeof(PlayerObject::m_stateRingJump)},
	{offsetof(PlayerObject, m_wasJumpBuffered), sizeof(PlayerObject::m_wasJumpBuffered)},
	{offsetof(PlayerObject, m_wasRobotJump), sizeof(PlayerObject::m_wasRobotJump)},
	{offsetof(PlayerObject, m_stateJumpBuffered), sizeof(PlayerObject::m_stateJumpBuffered)},
	{offsetof(PlayerObject, m_stateRingJump2), sizeof(PlayerObject::m_stateRingJump2)},
	{offsetof(PlayerObject, m_touchedRing), sizeof(PlayerObject::m_touchedRing)},
	{offsetof(PlayerObject, m_touchedCustomRing), sizeof(PlayerObject::m_touchedCustomRing)},
	{offsetof(PlayerObject, m_touchedGravityPortal), sizeof(PlayerObject::m_touchedGravityPortal)},
	{offsetof(PlayerObject, m_maybeTouchedBreakableBlock), sizeof(PlayerObject::m_maybeTouchedBreakableBlock)},
	{offsetof(PlayerObject, m_touchedPad), sizeof(PlayerObject::m_touchedPad)},
	{offsetof(PlayerObject, m_yVelocity), sizeof(PlayerObject::m_yVelocity)},
	{offsetof(PlayerObject, m_fallSpeed), sizeof(PlayerObject::m_fallSpeed)},
	{offsetof(PlayerObject, m_isOnSlope), sizeof(PlayerObject::m_isOnSlope)},
	{offsetof(PlayerObject, m_wasOnSlope), sizeof(PlayerObject::m_wasOnSlope)},
	{offsetof(PlayerObject, m_slopeVelocity), sizeof(PlayerObject::m_slopeVelocity)},
	{offsetof(PlayerObject, m_maybeUpsideDownSlope), sizeof(PlayerObject::m_maybeUpsideDownSlope)},
	{offsetof(PlayerObject, m_isShip), sizeof(PlayerObject::m_isShip)},
	{offsetof(PlayerObject, m_isBird), sizeof(PlayerObject::m_isBird)},
	{offsetof(PlayerObject, m_isBall), sizeof(PlayerObject::m_isBall)},
	{offsetof(PlayerObject, m_isDart), sizeof(PlayerObject::m_isDart)},
	{offsetof(PlayerObject, m_isRobot), sizeof(PlayerObject::m_isRobot)},
	{offsetof(PlayerObject, m_isSpider), sizeof(PlayerObject::m_isSpider)},
	{offsetof(PlayerObject, m_isUpsideDown), sizeof(PlayerObject::m_isUpsideDown)},
	{offsetof(PlayerObject, m_isDead), sizeof(PlayerObject::m_isDead)},
	{offsetof(PlayerObject, m_isOnGround), sizeof(PlayerObject::m_isOnGround)},
	{offsetof(PlayerObject, m_isGoingLeft), sizeof(PlayerObject::m_isGoingLeft)},
	{offsetof(PlayerObject, m_isSideways), sizeof(PlayerObject::m_isSideways)},
	{offsetof(PlayerObject, m_isSwing), sizeof(PlayerObject::m_isSwing)},
	{offsetof(PlayerObject, m_reverseRelated), sizeof(PlayerObject::m_reverseRelated)},
	{offsetof(PlayerObject, m_maybeReverseSpeed), sizeof(PlayerObject::m_maybeReverseSpeed)},
	{offsetof(PlayerObject, m_maybeReverseAcceleration), sizeof(PlayerObject::m_maybeReverseAcceleration)},
	{offsetof(PlayerObject, m_xVelocityRelated2), sizeof(PlayerObject::m_xVelocityRelated2)},
	{offsetof(PlayerObject, m_isDashing), sizeof(PlayerObject::m_isDashing)},
	{offsetof(PlayerObject, m_dashFireFrame), sizeof(PlayerObject::m_dashFireFrame)},
	{offsetof(PlayerObject, m_groundObjectMaterial), sizeof(PlayerObject::m_groundObjectMaterial)},
	{offsetof(PlayerObject, m_vehicleSize), sizeof(PlayerObject::m_vehicleSize)},
	{offsetof(PlayerObject, m_playerSpeed), sizeof(PlayerObject::m_playerSpeed)},
	{offsetof(PlayerObject, m_shipRotation), sizeof(PlayerObject::m_shipRotation)},
	{offsetof(PlayerObject, m_lastPortalPos), sizeof(PlayerObject::m_lastPortalPos)},
	{offsetof(PlayerObject, m_unkUnused3), sizeof(PlayerObject::m_unkUnused3)},
	{offsetof(PlayerObject, m_isOnGround2), sizeof(PlayerObject::m_isOnGround2)},
	{offsetof(PlayerObject, m_lastLandTime), sizeof(PlayerObject::m_lastLandTime)},
	{offsetof(PlayerObject, m_platformerVelocityRelated), sizeof(PlayerObject::m_platformerVelocityRelated)},
	{offsetof(PlayerObject, m_maybeIsBoosted), sizeof(PlayerObject::m_maybeIsBoosted)},
	{offsetof(PlayerObject, m_scaleXRelatedTime), sizeof(PlayerObject::m_scaleXRelatedTime)},
	{offsetof(PlayerObject, m_decreaseBoostSlide), sizeof(PlayerObject::m_decreaseBoostSlide)},
	{offsetof(PlayerObject, m_unkA29), sizeof(PlayerObject::m_unkA29)},
	{offsetof(PlayerObject, m_isLocked), sizeof(PlayerObject::m_isLocked)},
	{offsetof(PlayerObject, m_controlsDisabled), sizeof(PlayerObject::m_controlsDisabled)},
	{offsetof(PlayerObject, m_lastGroundedPos), sizeof(PlayerObject::m_lastGroundedPos)},
	{offsetof(PlayerObject, m_hasEverJumped), sizeof(PlayerObject::m_hasEverJumped)},
	{offsetof(PlayerObject, m_hasEverHitRing), sizeof(PlayerObject::m_hasEverHitRing)},
	{offsetof(PlayerObject, m_position), sizeof(PlayerObject::m_position)},
	{offsetof(PlayerObject, m_isSecondPlayer), sizeof(PlayerObject::m_isSecondPlayer)},
	{offsetof(PlayerObject, m_unkA99), sizeof(PlayerObject::m_unkA99)},
	{offsetof(PlayerObject, m_totalTime), sizeof(PlayerObject::m_totalTime)},
	{offsetof(PlayerObject, m_isBeingSpawnedByDualPortal), sizeof(PlayerObject::m_isBeingSpawnedByDualPortal)},
	{offsetof(PlayerObject, m_audioScale), sizeof(PlayerObject::m_audioScale)},
	{offsetof(PlayerObject, m_unkAngle1), sizeof(PlayerObject::m_unkAngle1)},
	{offsetof(PlayerObject, m_yVelocityRelated3), sizeof(PlayerObject::m_yVelocityRelated3)},
	{offsetof(PlayerObject, m_defaultMiniIcon), sizeof(PlayerObject::m_defaultMiniIcon)},
	{offsetof(PlayerObject, m_swapColors), sizeof(PlayerObject::m_swapColors)},
	{offsetof(PlayerObject, m_switchDashFireColor), sizeof(PlayerObject::m_switchDashFireColor)},
	{offsetof(PlayerObject, m_followRelated), sizeof(PlayerObject::m_followRelated)},
	{offsetof(PlayerObject, m_unk838), sizeof(PlayerObject::m_unk838)},
	{offsetof(PlayerObject, m_stateOnGround), sizeof(PlayerObject::m_stateOnGround)},
	{offsetof(PlayerObject, m_stateUnk), sizeof(PlayerObject::m_stateUnk)},
	{offsetof(PlayerObject, m_stateNoStickX), sizeof(PlayerObject::m_stateNoStickX)},
	{offsetof(PlayerObject, m_stateNoStickY), sizeof(PlayerObject::m_stateNoStickY)},
	{offsetof(PlayerObject, m_stateUnk2), sizeof(PlayerObject::m_stateUnk2)},
	{offsetof(PlayerObject, m_stateBoostX), sizeof(PlayerObject::m_stateBoostX)},
	{offsetof(PlayerObject, m_stateBoostY), sizeof(PlayerObject::m_stateBoostY)},
	{offsetof(PlayerObject, m_maybeStateForce2), sizeof(PlayerObject::m_maybeStateForce2)},
	{offsetof(PlayerObject, m_stateScale), sizeof(PlayerObject::m_stateScale)},
	{offsetof(PlayerObject, m_platformerXVelocity), sizeof(PlayerObject::m_platformerXVelocity)},
	{offsetof(PlayerObject, m_holdingRight), sizeof(PlayerObject::m_holdingRight)},
	{offsetof(PlayerObject, m_holdingLeft), sizeof(PlayerObject::m_holdingLeft)},
	{offsetof(PlayerObject, m_leftPressedFirst), sizeof(PlayerObject::m_leftPressedFirst)},
	{offsetof(PlayerObject, m_scaleXRelated), sizeof(PlayerObject::m_scaleXRelated)},
	{offsetof(PlayerObject, m_maybeHasStopped), sizeof(PlayerObject::m_maybeHasStopped)},
	{offsetof(PlayerObject, m_xVelocityRelated), sizeof(PlayerObject::m_xVelocityRelated)},
	{offsetof(PlayerObject, m_maybeGoingCorrectSlopeDirection), sizeof(PlayerObject::m_maybeGoingCorrectSlopeDirection)},
	{offsetof(PlayerObject, m_isSliding), sizeof(PlayerObject::m_isSliding)},
	{offsetof(PlayerObject, m_maybeSlopeForce), sizeof(PlayerObject::m_maybeSlopeForce)},
	{offsetof(PlayerObject, m_isOnIce), sizeof(PlayerObject::m_isOnIce)},
	{offsetof(PlayerObject, m_physDeltaRelated), sizeof(PlayerObject::m_physDeltaRelated)},
	{offsetof(PlayerObject, m_isOnGround4), sizeof(PlayerObject::m_isOnGround4)},
	{offsetof(PlayerObject, m_maybeSlidingTime), sizeof(PlayerObject::m_maybeSlidingTime)},
	{offsetof(PlayerObject, m_maybeSlidingStartTime), sizeof(PlayerObject::m_maybeSlidingStartTime)},
	{offsetof(PlayerObject, m_changedDirectionsTime), sizeof(PlayerObject::m_changedDirectionsTime)},
	{offsetof(PlayerObject, m_slopeEndTime), sizeof(PlayerObject::m_slopeEndTime)},
	{offsetof(PlayerObject, m_isMoving), sizeof(PlayerObject::m_isMoving)},
	{offsetof(PlayerObject, m_platformerMovingLeft), sizeof(PlayerObject::m_platformerMovingLeft)},
	{offsetof(PlayerObject, m_platformerMovingRight), sizeof(PlayerObject::m_platformerMovingRight)},
	{offsetof(PlayerObject, m_isSlidingRight), sizeof(PlayerObject::m_isSlidingRight)},
	{offsetof(PlayerObject, m_maybeChangedDirectionAngle), sizeof(PlayerObject::m_maybeChangedDirectionAngle)},
	{offsetof(PlayerObject, m_unkUnused2), sizeof(PlayerObject::m_unkUnused2)},
	{offsetof(PlayerObject, m_isPlatformer), sizeof(PlayerObject::m_isPlatformer)},
	{offsetof(PlayerObject, m_stateNoAutoJump), sizeof(PlayerObject::m_stateNoAutoJump)},
	{offsetof(PlayerObject, m_stateDartSlide), sizeof(PlayerObject::m_stateDartSlide)},
	{offsetof(PlayerObject, m_stateHitHead), sizeof(PlayerObject::m_stateHitHead)},
	{offsetof(PlayerObject, m_stateFlipGravity), sizeof(PlayerObject::m_stateFlipGravity)},
	{offsetof(PlayerObject, m_gravityMod), sizeof(PlayerObject::m_gravityMod)},
	{offsetof(PlayerObject, m_stateForce), sizeof(PlayerObject::m_stateForce)},
	{offsetof(PlayerObject, m_stateForceVector), sizeof(PlayerObject::m_stateForceVector)},
	{offsetof(PlayerObject, m_affectedByForces), sizeof(PlayerObject::m_affectedByForces)},
	{offsetof(PlayerObject, m_lastMovedTime), sizeof(PlayerObject::m_lastMovedTime)},
	{offsetof(PlayerObject, m_playerSpeedAC), sizeof(PlayerObject::m_playerSpeedAC)},
	{offsetof(PlayerObject, m_fixRobotJump), sizeof(PlayerObject::m_fixRobotJump)},
	{offsetof(PlayerObject, m_inputsLocked), sizeof(PlayerObject::m_inputsLocked)},
	{offsetof(PlayerObject, m_gv0123), sizeof(PlayerObject::m_gv0123)},
	{offsetof(PlayerObject, m_iconRequestID), sizeof(PlayerObject::m_iconRequestID)},
	{offsetof(PlayerObject, m_unkUnused), sizeof(PlayerObject::m_unkUnused)},
	{offsetof(PlayerObject, m_isOutOfBounds), sizeof(PlayerObject::m_isOutOfBounds)},
	{offsetof(PlayerObject, m_fallStartY), sizeof(PlayerObject::m_fallStartY)},
	{offsetof(PlayerObject, m_disablePlayerSqueeze), sizeof(PlayerObject::m_disablePlayerSqueeze)},
	{offsetof(PlayerObject, m_ignoreDamage), sizeof(PlayerObject::m_ignoreDamage)},
	{offsetof(PlayerObject, m_enable22Changes), sizeof(PlayerObject::m_enable22Changes)},
	{offsetof(PlayerObject, m_enableImpulseFix), sizeof(PlayerObject::m_enableImpulseFix)}
};

// Input semantics differ fundamentally by mode, and lumping them together made
// UFO structurally unsolvable in tight sections (plan section 6.3):
//   Ground  cube/ball/spider/robot - act only on contact with a surface or orb
//   Hold    ship/wave              - hold controls the trajectory continuously
//   Tap     UFO/swing              - each PRESS is a discrete event; holding does
//                                    nothing, so a tap under continue/toggle
//                                    semantics costs TWO budget units (press then
//                                    release) instead of one, halving expressiveness
//                                    exactly where it is scarcest.
enum class ModeClass { Ground, Hold, Tap };

const char* modeClassName(int mc) {
	switch (mc) {
		case 0:  return "Ground";
		case 1:  return "Hold";
		case 2:  return "Tap";
		default: return "none";
	}
}

ModeClass classifyMode(PlayerObject* p) {
	if (!p) return ModeClass::Ground;
	if (p->m_isShip || p->m_isDart)  return ModeClass::Hold; // ship, wave
	if (p->m_isBird || p->m_isSwing) return ModeClass::Tap;  // UFO, swing
	return ModeClass::Ground;                                 // cube, ball, spider, robot
}

// The camera band height for the mode a portal switches INTO, keyed by the
// portal's object ID.
//
// MEASURED in a custom level: ship 300 (10 blocks), UFO 300 (10 blocks), ball
// 240 (8 blocks). Wave, swing, robot and spider are still ASSUMED and
// deliberately loose - a band that is too high only fails to remove phantom
// space, while one that is too low DELETES reachable space, so the unknown case
// errs on 300.
//
// This is the single source for the band, and it has to be, because it existed
// twice and the two copies drifted the moment ball was measured at 240. The map
// roof took the measured value; ceilingForPortal hardcoded g_config's ship band
// in its ground clamp and had no way to know the mode at all. The result was a
// derived ceiling of 390 for every ground-clamped ball section when the truth is
// 330 - exactly 60 units too high, on 7 portals across 4 levels, and it is the
// sole content of all 18 ceiling cross-check warnings ever logged.
double bandHeightForPortal(int objID) {
	switch (objID) {
		case 47:  return 240.0;                    // ball, measured
		case 13:  return 300.0;                    // ship, measured
		case 111: return 300.0;                    // UFO, measured
		default:  return g_config.geomBandHeight;  // assumed, and loose
	}
}

// How far the ceiling sits ABOVE the entered portal's y, before the grid snap.
//
// NOT the constant geomCeilingOffset (135). That number is correct for a 300
// band and wrong for every other one. MEASURED against 0x764 on three levels:
//
//   ToE  6928 UFO  posY  345 -> 480   offset 135
//   ToE 13768 ship posY  285 -> 420   offset 135
//   ToE 16648 UFO  posY 1095 -> 1230  offset 135
//   ToE 24720 ship posY  915 -> 1050  offset 135
//   ToE 11344 ball posY  285 -> 390   offset 105
//   ToE 23548 ball posY  915 -> 1020  offset 105
//
// The clincher is xStep, which puts a ball portal (x 18388) and a ship portal
// (x 23008) at the SAME y and reads 420 and 450 - a 30-unit split at one
// height, which is exactly (300 - 240) / 2 and cannot be explained by anything
// but the band height.
//
// So the band is not anchored by its ceiling at all. It is CENTRED on
// posY - 15, whatever the mode, and only its height changes:
//
//   ceiling = posY - 15 + bandHeight/2
//   floor   = posY - 15 - bandHeight/2
//
// which reproduces the original ship derivation (45 above the portal's snapped
// centre plus 90 clear above its top = 135) as the bandHeight == 300 case,
// rather than contradicting it.
//
// This error was invisible because it is EXACTLY 30 on a ball and the
// cross-check in ceilingUpdate fired on `d > 30.0`. It missed by nothing.
// Portals that END a wall-limited section rather than starting one. Cube is
// the only one confirmed: it is not wall-limited, so nothing should cap the
// roof after it until the next real mode portal.
//
// Deliberately NOT a guess-list. Every ID added to the roof scan by inference
// has cost a level - gravity portals were added because a 390 ceiling happened
// to sit near one, and that raised Base After Base's roof from 386 to 630.
// This one is here because Probe 17 measured the deletion it causes.
bool isRoofReleasePortal(int objID) {
	return objID == 12;
}

double ceilingOffsetForPortal(int objID) {
	return bandHeightForPortal(objID) * 0.5 - 15.0;
}

// The coarse bucket a state falls into: "somewhere like here", not "exactly
// here". Lifted out of goCellKey so the archive and the census that measures
// whether the archive is worth building cannot drift apart - the same reason
// RestoreState came out of Decision and atDecisionPoint came out of the beam.
struct CellCoords {
	int32_t  x, y, vy;
	uint32_t mode;
	uint32_t flags;
};

// Budget cost of taking `choice` at this decision.
//   Hold : a toggle (changing the hold state) costs 1; continuing is free.
//   Tap  : a tap costs 1; not tapping is free. Crucially a tap is ONE unit, not
//          the two it cost when taps were expressed as toggle-on plus toggle-off.
// Everything a restore must put back, in one place.
//
// This lives apart from Decision because the DFS and the beam both restore, and
// two copies of a restore path is how this file kept acquiring bugs that were
// fixed in one caller and not the other (see solverRepositionTo's history). One
// struct, one capture function, one apply function.
//
// Every field here was added because a measurement proved it was missing, never
// because it looked relevant. See 13.11.
struct RestoreState {
	CheckpointObject* cp = nullptr;

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
	double layerAttemptTime     = 0.0;
	double layerBestAttemptTime = 0.0;
	double layerCurrentTime     = 0.0;
	bool   layerHasJumped       = false;

	// The ACTUAL held-button state at capture, not the input we intended.
	//
	// m_holdingButtons is a gd::map, so it is outside the PlayerObject field
	// table the restore writes back, and the restore used to rebuild it by
	// calling pushButton/releaseButton with `enteringHold`. That is the solver's
	// INTENDED input; the container reflects input that has already been through
	// handleButton's one-step latency. The two are out of phase, so every restore
	// near an input change rebuilt the button state one step wrong.
	//
	// MEASURED: at step 18608 of Stereo Madness - a ship section, where hold
	// controls acceleration on every frame - the clean replay held the button and
	// the restored search did not. yVelocity split -4.308 vs -4.485 on that step
	// and the two trajectories never re-converged; the run died 247 steps later.
	bool holdingJump   = false; // m_holdingButtons[Jump]
	bool probeIsHolding = false; // our own injection bookkeeping

	// GJBaseGameLayer's pending input queue. Not in any checkpoint, and cleared
	// by resetLevel, so without this a restore either loses a command that was
	// in flight or - worse - leaves one from before the rewind to be drained on
	// the first step after, delaying the real input by exactly one step.
	std::vector<PlayerButtonCommand> queuedButtons;

	// The COCOS node transform. PlayerObject::m_position is in the field table
	// and is what physics reads; getPositionX() reads CCNode::m_obPosition,
	// which is inherited from cocos and therefore outside an enumeration of
	// PlayerObject's own members. Nothing restored it.
	//
	// MEASURED (Time Machine, step 13071): after a restore the node position
	// still held the position where the player DIED, ~149 units further along,
	// and then crept back over 46 steps while y, rotation, velocity and every
	// flag stayed bit-identical. Physics was correct throughout - only the node
	// lagged - so the level still cleared, which is exactly why this survived
	// eleven levels unnoticed. It still feeds rendering, the camera and the
	// percent readout, and a restore is supposed to be exact.
	cocos2d::CCPoint nodePos = {0.f, 0.f};

	// Mirror/camera transition state - see Solver::PendingExtra.
	float layerCameraFlip      = 0.f;
	float layerCameraUnzoomedX = 0.f;
	bool  layerUnk322a         = false;
	bool  layerUnk3251         = false;

	// Full PlayerObject image at capture, from which kPlayerFields is written
	// back after loadFromCheckpoint. The direct load assigns only a subset, and
	// a reset beforehand only changes WHICH wrong value the rest hold; this is
	// the half of the restore that makes AIR modes exact (Probe 4b, 8/8).
	//
	// Stored whole rather than packed to the 196 spans so the solver reproduces
	// the validated probe path byte for byte. ~2 KB against a 22 KB checkpoint.
	std::vector<uint8_t> playerBytes;
};


struct Decision;
int decisionCost(Decision const& d, bool choice);
int tapCost(Decision const& d, bool choice);

// Release a checkpoint, first detaching it from anything GD still points at.
//
// The old restore put our checkpoint INTO m_checkpointArray and let the practice
// respawn consume it, so clearing the array was enough. The direct load takes
// the object straight and may leave PlayLayer::m_currentCheckpoint pointing at
// it, so releasing without clearing that leaves a dangling pointer for the next
// reset or level exit to walk into. Checkpoint lifetime has already caused three
// separate crashes here, and every air decision now holds one.
inline void releaseCheckpoint(CheckpointObject*& cp) {
	if (!cp) return;
	if (auto* pl = PlayLayer::get()) {
		if (pl->m_currentCheckpoint == cp) pl->m_currentCheckpoint = nullptr;
	}
	cp->release();
	cp = nullptr;
}

// Probe 17: audit a KNOWN-GOOD path against the map.
//
// Every argument about whether the liveness sweep is too strict has been
// made from the geometry, and the geometry is exactly what is in question.
// A solved macro settles it without any of that: the trajectory physically
// happened, so anywhere the map disagrees with it, the MAP is wrong.
// Probe 21: the decisions the search ACTUALLY placed along its best path.
//
// Probe 20 tried to answer this by re-deriving decision points with a shadow of
// the branch rule. It got the phase wrong and produced byte-identical output for
// mode 0 and mode 1, which is impossible - so it was measuring nothing. The
// solver knows where its decisions are; read them instead of reconstructing.
//
// Snapshotted on each new best, so a stall leaves the decision sequence of the
// furthest path behind. Diffed offline against a known-good macro's toggles,
// this says whether the search could have expressed the winning input - and if
// it could, what the steer was telling it to do instead.
// The steer's internals from its most recent evaluation, so a decision can
// record WHY the steer said what it said - not just what it said.
//
// MEASURED need: on Clubstep's stall the search had a decision at the exact
// step the winning path releases (11933) and 6 toggles spent against a budget
// of 9, so it could afford the move and simply was not told to make it. The
// steer is silent across the whole 11891-11949 release window and says CLIMB
// at 11874-11886. Knowing it was silent is not enough to fix it; we need the
// target and the deadband it was silent ABOUT.
struct SteerInfo {
	double yNow = 0, yProj = 0, target = 0, band = 0, lo = 0, hi = 0, lookX = 0;
	int    path = 0;      // which of the three target paths answered
	bool   valid = false;
};

struct Decision {
	int               step      = 0;       // solver step index at capture
	uint8_t           tried     = 0;       // bit0 = tried release, bit1 = tried hold
	bool              choice    = false;   // action currently being explored
	bool              airPolicy = false;   // Hold or Tap (i.e. not Ground)
	ModeClass         modeClass = ModeClass::Ground;

	// Iterative deepening on toggle count, for air sections.
	//
	// Good ship paths have FEW input changes: "hold from ship entry" is one
	// toggle, "hold then release to level off" is two. DFS on raw hold/release
	// reaches those last, having exhausted astronomically many high-toggle
	// sequences first. Bounding the toggle count and raising the bound on
	// exhaustion searches simple paths before complicated ones, and stays
	// complete as the bound grows.
	bool enteringHold  = false; // hold state on arrival, so a toggle is well-defined
	// sv.prevAirMode / sv.prevOnGround on arrival here. Those two gate the FORCED
	// decisions at a mode transition and at a landing, and neither was put back on
	// a reposition - so after a rewind they held whatever the abandoned path left,
	// exactly the defect lastGeoLo/Hi had. See modeStateRestore.
	bool enteringAirMode  = false;
	bool enteringOnGround = false;
	SteerInfo si;            // the steer internals at the moment this was pushed
	double    vyAt = 0.0;    // player vy at the same moment
	// Corridor at the previous decision, on arrival here. Carried state for the
	// mode 1/2 branch rule - see solverRepositionTo. Unread in mode 0.
	double enteringGeoLo = 0.0;
	double enteringGeoHi = 0.0;
	int  togglesBefore = 0;     // air toggles used on the path up to this decision
	int  tapsBefore    = 0;     // taps used on the path up to this decision
	bool modeTransition = false; // pushed because the game mode changed here

	// What the geometry asked for at this decision, if anything. Stored because
	// decisionCost sees only the Decision, long after the player has moved on.
	bool hasSteer    = false;
	bool steerChoice = false;

	RestoreState rs;            // the state entering this decision
};

// Taps spent by this choice. Always 1 for a tap, independent of the flag - the
// flag decides which budget it is charged to, not what it costs.
int tapCost(Decision const& d, bool choice) {
	return (d.modeClass == ModeClass::Tap && choice) ? 1 : 0;
}

// Taps permitted above the commit floor. The same number as holds get.
//
// It used to scale with the mutable window - "so the allowance means the same
// thing at any width" - which quietly made widening change what a decision
// COSTS as well as how far back the search may go. Two concepts on one lever,
// and it stayed hidden while widening was slow, because the scaling is inert
// at the default window.
//


int decisionCost(Decision const& d, bool choice) {
	// Free if it is what the map asked for.
	if (g_config.geomFreeFollowing && d.hasSteer && choice == d.steerChoice) return 0;
	switch (d.modeClass) {
		// A tap costs 1. Charging per RHYTHM CHANGE instead (cheap bursts,
		// expensive isolated taps) was tried and regressed Theory of Everything
		// from 78.90% to 32.49% - the first UFO section needs scattered single
		// taps, which doubled in price. Neither model dominates; per-tap is the
		// one that demonstrably clears more.
		case ModeClass::Tap:   return choice ? 1 : 0;
		case ModeClass::Hold:   return (choice != d.enteringHold) ? 1 : 0;
		case ModeClass::Ground: default: return 0;
	}
}

// One refusal by the toggle-budget gate.
//
// The gate is inside solverBacktrack, which is the stepping path, so this
// RECORDS and never logs - the vector is reserved up front and never grows
// there. Dumped to a file at deepening and on stop, where I/O is allowed.
//
// Why it exists: with hybridRestore ON the current build still exhausts budget 3
// in Theory of Everything's UFO corridor and 30fe2b7 does not, while
// decisionCost, this gate, togglesBefore, move ordering and isDecisionPoint are
// all byte-identical between them. Diffing the two pop sequences locates the
// first real difference instead of inferring it. Plan 13.13.
struct BudgetPop {
	int step, togglesBefore, floorToggles, spent, budget, floorIdx, stackSize;
	int mode;
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
	int                   tapsUsed     = 0;
	size_t                commitDepth  = 0; // cached for logging; see solverCommitFloor()
	int                   lookbackSteps = 0;   // current (possibly widened) window
	// How far back the LAST escape actually reached. The ladder escalates when an
	// escape fails to beat it, so the trigger is the effect rather than a count of
	// escapes - see the stage block in solverStep.
	int                   lastUndone = 0;

	// Probe 19: WHERE the search dies, and what the map was saying there.
	//
	// Every diagnosis so far has come from aggregate counters, which say a level
	// stalled but never where or why. A death histogram localises it: if most
	// deaths land in one cell, the problem is one obstacle, and the map's opinion
	// at that cell is then readable directly rather than inferred.
	std::unordered_map<uint64_t, uint32_t> deathCells;
	std::unordered_map<uint64_t, double>   deathVySum;
	uint64_t              deathsRecorded = 0;
	bool                  anchorReleased = false; // stage 2: transition anchor dropped
	int                   bestStep     = 0; // solver step at which bestPct was reached
	uint64_t              deathsAtBest = 0;
	uint64_t              escapes      = 0;
	uint64_t              geoDeaths    = 0;   // deaths called by the geometry map
	uint64_t              geoSteers    = 0;   // air decisions the map reordered
	uint64_t              geoSteerDead = 0;   // ...because the player was in dead space
	uint64_t              geoSteerWin  = 0;   // ...because it was outside a live window
	uint64_t              geoSteerMute = 0;   // air decisions the gate silenced

	// The ceiling for a portal at this y: offset, then snap the RESULT to the
	// block grid. See geomCeilingGrid for why that order.
	//
	// objID is required, not optional, because the ground clamp below is the
	// whole band: a ball section clamps at 330 and a ship section at 390, and
	// with no objID this silently answered 390 for both.
	static double ceilingForPortal(double portalY, int objID) {
		const double g = std::max(1.0, g_config.geomCeilingGrid);
		const double c = std::round((portalY + ceilingOffsetForPortal(objID)) / g) * g;
		// Rest the band on the ground rather than letting it clip through.
		const double lowest = g_config.geomGroundY + bandHeightForPortal(objID);
		return std::max(c, lowest);
	}

	// The portal the player last actually entered, from playerWillSwitchMode.
	double                entryPortalY    = 0.0;
	double                entryPortalX    = 0.0;
	bool                  haveEntryPortal = false;
	int                   entryPortalMode = -1;   // ModeClass at entry
	int                   entryPortalID   = 0;    // objID, for the band height
	uint64_t              portalEntries   = 0;
	uint64_t              entryStaleDrops = 0;  // records dropped as rewound-past

	// The band of the section the player is currently in. The floor of a band is
	// its ceiling less this, never a second constant - see bandHeightForPortal.
	double entryBand() const { return bandHeightForPortal(entryPortalID); }

	// Probe: does the read field agree with the derived rule? Any disagreement
	// beyond a block means one of them is wrong and we want to hear about it.
	bool                  ceilFieldOk     = false;   // layout guard passed
	uint64_t              ceilDisagreed   = 0;
	double                ceilWorstDelta  = 0.0;

	// The validity test the clamp never had. The player can never be outside its
	// own band; if the computed band excludes the position the player is actually
	// occupying, the band is WRONG, and a wrong band crops real space out of the
	// map - indistinguishable from the level simply being harder.
	//
	// MEASURED: Time Machine solves in 12s with the clamp off and 21s with it on,
	// so something it removes is load-bearing.
	//
	// Suspected flaw: the ceiling describes where the player IS, but it is applied
	// to a slice 300 units AHEAD. Where that lookahead crosses into a section with
	// a different ceiling, the next section is cropped with this section's band.
	// The widest band observed in this level, which is what the map may be
	// cropped against.
	//
	// MEASURED: Clutterfunk is the only level of the three with TWO ship sections
	// at different heights - ceilings 330 and 390 - and it is the only one the
	// crop breaks. With the crop off it solves in 38s, exactly its historical
	// time. The steer consults a slice 300 units ahead, which in a two-section
	// level can belong to the OTHER section, so cropping it with the current
	// band deletes real geometry.
	//
	// Testing against the widest band seen instead is conservative and needs no
	// portal data: space above every ceiling this level has is phantom under all
	// of them, while a lower section's real space stays untouched.

	bool                  roofWarned      = false;
	// Smallest (map roof at the player's x - live ceiling) seen, and where.
	// Negative means the build cropped space the game says the player may
	// legally occupy. Positive and large means the roof is doing nothing.
	double                roofMarginMin   =  1e30;
	double                roofMarginMinX  = 0.0;
	// The min alone is one-sided: it proves the roof never deleted occupied
	// space, and says nothing about a roof sitting so far above the ceiling that
	// it removes nothing. A large mean is that failure.
	double                roofMarginMax   = -1e30;
	double                roofMarginSum   = 0.0;
	uint64_t              roofMarginN     = 0;
	uint64_t              ceilOutside     = 0;
	uint64_t              ceilInside      = 0;
	double                ceilWorstOut    = 0.0;

	// The reachable roof, derived from the game's own portal anchor each step.
	// 1e30 reads as no clamp (ground mode, or free fly).
	double                learnedCeiling  = 1e30;
	uint64_t              ceilingCrops    = 0;

	// Probe 10: the observed ceiling measured against BOTH candidate sources, so
	// one run picks the winner instead of us arguing about it.
	//
	//   portal  = contactY - m_gameState.m_portalY   (needs a derived offset)
	//   camera  = contactY - cameraObb2 rect top     (needs none if it is ~0)
	//
	// The right source is the one whose spread is ~0. If the camera rect wins,
	// geomCeilingOffset stops existing.
	double                ceilOffMin =  1e30, ceilOffMax = -1e30;   // portal
	double                camOffMin  =  1e30, camOffMax  = -1e30;   // camera rect
	uint64_t              ceilOffN   = 0, camOffN = 0;
	double                camTopSeen = 0.0;

	// Probe 11: the field hunt.
	//
	// Three candidates have been eliminated by measurement - m_objectLayer's y
	// (constant 80), m_cameraObb2's rect top (0.5, not world space) and
	// m_gameState.m_portalY (0, never populated). The real ceilings in Base
	// After Base are 255 and 375, so the field we want takes two values 120
	// apart in step with them.
	//
	// Rather than one candidate per run, record contactY MINUS every remaining
	// candidate and report which difference is constant. Constant means that
	// field tracks the ceiling; the constant is then the offset, and zero
	// spread with a near-zero offset means the field IS the ceiling.
	static constexpr int kCeilCand = 14;
	double   candMin[kCeilCand];
	double   candMax[kCeilCand];
	double   candLast[kCeilCand];
	uint64_t candN = 0;

	// Where the clean contacts actually sit. Min and max is what let three
	// distinct values read as two, and a conclusion was drawn from it before
	// the spread was ever looked at.
	std::map<int, uint32_t> ceilHist;
	uint64_t              geoLooks     = 0;   // air decisions the map was consulted on

	// The player's horizontal speed, measured. Held across restores rather than
	// recomputed, because a restore jumps x backwards and the delta across one
	// is meaningless.
	// Probe 9: the previous step's post-update state, which is this step's
	// PRE-update state. Held rather than recomputed because a restore jumps the
	// player and the pair either side of one would not be a transition at all -
	// motionHavePrev is cleared in applyRestoreState for exactly that reason.
	double                motionPrevVy   = 0.0;
	bool                  motionPrevHold = false;
	bool                  motionHavePrev = false;
	bool                  motionPrevGround = false;
	uint8_t               motionPrevMode   = 255;
	bool                  motionPrevPad        = false;
	bool                  motionPrevRing       = false;
	bool                  motionPrevCustomRing = false;

	double                prevX        = 0.0;
	double                dxPerStep    = 1.31;  // 1x; replaced on the first step

	// The largest vertical move the player has actually made in one step, per
	// air mode, in world units - the same units the map's reach is in.
	//
	// geomVerticalReach = 60 per geomSliceX = 30 makes the map's implied climb
	// 2.60 units per step at 1x. Nothing has ever checked that against the game.
	// If the real figure is far higher then the map is not modelling reach at
	// all, it is applying a deliberate tightness that happens to produce useful
	// dead detection - and that changes what a speed correction should even do.
	//
	// Same guard as dxPerStep: only counted on a step that was a real forward
	// step, never across a restore.
	double                prevY        = 0.0;

	// The live interval ahead as it was at the last air decision, so a change
	// can be noticed. See geomBranchMode.
	// Set by the caller before geometrySteer, which is a free function and
	// cannot reach the modify class's members.
	PlayerObject*         steerPlayer  = nullptr;

	// Fastest this mode and size has been seen to move in the given world
	// direction, over every speed column. 0 until something has been observed.
	// The most this mode can move vertically in one step.
	//
	// MEASURED for ship in a purpose-built level (Probe 16), not sampled
	// from whatever the search happened to fly. Two independent readings -
	// the game's own m_yVelocity and slope x the canonical per-step dx -
	// agreed at exactly 4.4444 across ten cells, and the figures held at
	// 0.5x, 1x, 2x, 3x and 4x, so vertical rate is speed-independent and
	// these are physics constants rather than fitted ones.
	//
	// This is what geomUrgencySteer was missing. It was switched off
	// because its only input was the fitted vertical-reach constant that
	// 9803757 dropped as untrustworthy - the mechanism was fine, the number
	// under it was a guess.
	//
	// Modes without a measured figure fall back to the sampled table. They
	// do not appear in any level in the regression set, so there is nothing
	// to measure them against yet.
	double observedClimb(PlayerObject* p, bool up) const {
		if (!p) return 0.0;
		const int mode = p->m_isShip ? 0 : p->m_isDart ? 1 : p->m_isBird ? 2 : 3;
		const int size = p->m_vehicleSize < 0.9f ? 1 : 0;
		const int dir  = up ? 0 : 1;
		if (mode == 0) {
			//                     up      down
			static const double kShip[2][2] = {{1.800, 1.440},   // normal
			                                   {2.118, 1.694}};  // mini
			return kShip[size][dir];
		}
		double v = 0.0;
		for (int sp = 0; sp < 5; sp++) v = std::max(v, maxClimb[mode][size][sp][dir]);
		return v;
	}

	double                lastGeoLo    = 0.0;
	double                lastGeoHi    = 0.0;

	// [mode][size][speed][direction], where mode is 0 ship, 1 wave, 2 UFO,
	// 3 anything else; size is 0 normal, 1 mini; speed is 0..4 slow to fastest;
	// direction is 0 up, 1 down.
	//
	// SPEED IS IN HERE TO FALSIFY AN ASSUMPTION, not because it is believed to
	// matter. The reach model assumes vertical acceleration is per-step and
	// independent of horizontal speed - which would make a 3x ship's path look
	// flatter while its vy is unchanged. If that holds, a mode's up figure is
	// the same across every speed column and the columns can be collapsed. If
	// it does not, the columns differ and the model needs a speed term. Reading
	// one aggregate number could not tell the two apart.
	//
	// Up and down are kept apart because they are not the same number in GD -
	// a ship's climb and its fall are separate accelerations - and the map's
	// backward sweep widens a window in BOTH directions from one figure, so it
	// is currently using the wrong one in at least one of them.
	//
	// The largest single-step move is terminal vertical velocity in the map's
	// own units, which is exactly the constant an (y, vy) propagation needs.
	// Caveat worth remembering when reading it: this is the max OBSERVED, so it
	// is a LOWER bound on capability. It can prove 2.60 is too tight; it cannot
	// prove 2.60 is enough.
	double                maxClimb[4][2][5][2] = {};

	// Probe 16: the vertical ENVELOPE, for the liveness reach model.
	//
	// Keyed on the game's OWN m_playerSpeed, not on observed dx. The first cut
	// bucketed dx and it failed: in free flight an update() call does not
	// reliably cover exactly one physics step, so a two-step sample at slow
	// speed reads dx 2.09 and masquerades as 3x - which is precisely what the
	// first run produced, and why normal-size ship never registered below 3x at
	// all. Knowing the speed independently makes dx a CHECK rather than a key.
	//
	// The reported figure is slope x the canonical per-step dx. Slope is
	// frame-rate free, so it stays correct however many steps a sample spanned,
	// and it was the only column of the first run that survived.
	//
	// [mode][mini][speed][dir], dir 0 = world up. Not cleared with the solver:
	// a measurement session spans several attempts and restarting the level
	// must not discard it.
	double                envMaxSlope[8][2][5][2] = {};
	// Read rather than inferred. m_yVelocity is the game's own vertical
	// velocity, so it sidesteps the whole question of how many physics steps an
	// update() call covered - the slope figure stays as the independent
	// cross-check. m_fallSpeed and m_gravity are the mode's own constants and
	// may hand us terminal velocity directly instead of us measuring toward it.
	double                envMaxVy[8][2][5][2]      = {};
	double                envSumGravity[8][2][5][2] = {};
	double                envSumFall[8][2][5][2]    = {};
	double                envSumSpdMul[8][2][5][2]  = {};
	double                envSumGravMod[8][2][5][2] = {};
	uint64_t              envUpsideN[8][2][5][2]    = {};
	double                envSumDx[8][2][5][2]    = {};
	double                envSumRaw[8][2][5][2]   = {};  // observed m_playerSpeed
	uint64_t              envN[8][2][5][2]        = {};
	uint64_t              envNearN[8][2][5][2]    = {};  // within 5% of max slope
	bool                  envLocked[8][2][5][2]   = {};
	uint64_t              envRejected             = 0;   // repositions
	uint64_t              envStationary           = 0;   // pinned or at rest

	SteerInfo steerInfo;

	struct DecRow {
		int  step; uint8_t mode; bool choice, hasSteer, steerChoice; int togglesBefore;
		double y, vy, yProj, target, band, lo, hi;
		int    path;
	};
	std::vector<DecRow> bestDecisions;

	// Probe 20: is the winning macro EXPRESSIBLE by the branch rule?
	//
	// The one question three suite-level experiments could only infer. Replay a
	// path that is known to clear the level and, at every step where it CHANGES
	// INPUT, ask whether the branch rule would have offered a decision there. A
	// toggle with no decision point is a move the search physically cannot make.
	struct ExprMiss { double x, y, vy, pct; uint64_t step; int elapsed, mode, reason;
	                  double glo, ghi; };
	std::vector<ExprMiss> exprMisses;
	bool                  exprPrevPressed = false, exprHavePrev = false;
	uint64_t              exprLastBranch = 0;
	double                exprGeoLo = 0.0, exprGeoHi = 0.0;
	uint64_t              exprAirSteps = 0, exprToggles = 0;
	uint64_t              exprOffered = 0, exprMissed = 0;
	uint64_t              exprMissCadence = 0, exprMissUnchanged = 0, exprMissNoRead = 0;
	uint64_t              exprOfferPoints = 0;
	uint64_t              pathAuditSteps   = 0;
	uint64_t              pathOffMap       = 0;  // no interval holds the player
	uint64_t              pathDead         = 0;  // in an interval marked dead
	uint64_t              pathOutWindow    = 0;  // live, but outside its window
	double                pathFirstOffX    = -1.0, pathFirstOffY  = 0.0;
	double                pathFirstDeadX   = -1.0, pathFirstDeadY = 0.0;
	double                pathFirstOutX    = -1.0, pathFirstOutY  = 0.0;
	double                pathWorstOff     = 0.0,  pathWorstOffX  = 0.0;
	// WHERE the off-map steps are, in twentieths of the mapped x range. First
	// and worst were not enough: they said the crop's deepest point was at
	// x 28649, and left open whether the bulk of it sat before or after the
	// wall - which is the whole question of whether the crop causes the stall
	// or merely coexists with it.
	uint64_t              pathOffBin[20]   = {};
	double                pathOffBinWorst[20] = {};
	double                envPrevX = 0.0, envPrevY = 0.0;
	bool                  envHavePrev = false;

	// Which speed column a measured dx belongs in: nearest of the five, split
	// at the midpoints.
	static int speedBucket(double dx) {
		if (dx < 1.172) return 0;
		if (dx < 1.456) return 1;
		if (dx < 1.782) return 2;
		if (dx < 2.175) return 3;
		return 4;
	}

	// How often the tap-ordering rule fires, split by gravity direction. The
	// point is to catch over-tapping from the FIRST report line rather than by
	// inferring it from a stalled percentage an hour later.
	uint64_t              tapDecisions = 0;
	uint64_t              tapFirstChosen = 0;
	struct PendingExtra {
		double gameModeChangedTime = 0.0;
		bool   unkA29 = false;
		double extraDelta = 0.0, timePlayed = 0.0, timestamp = 0.0;
		int    tickIndex = 0, clickIndex = 0, resumeTimer = 0;
		bool   jumping = false;
		double attemptTime = 0.0, bestAttemptTime = 0.0, currentTime = 0.0;
		bool   hasJumped = false;

		// The mirror/camera group. GD implements mirror mode as a CAMERA FLIP,
		// and m_cameraFlip is a FLOAT, not a flag - it is the progress of an
		// animated transition (toggleFlipped takes a `noEffects` argument
		// precisely because the default path animates).
		//
		// MEASURED (Time Machine, step 13071, a blue un-mirror portal): a restore
		// landing inside that transition left m_cameraFlip at a different value,
		// and the reported player x then swept smoothly from +146.7 to -150.0
		// over the 95 steps of the transition before snapping back into
		// agreement. y, rotation, velocity, gravity and every flag stayed
		// bit-identical throughout, which is why the level still cleared and why
		// this hid behind eleven passing levels: the flip is presentational, so
		// it corrupts the reported position without corrupting the physics.
		//
		// It only bites when a restore lands INSIDE a transition window, which is
		// why one flip diverged and the level's other flips did not.
		float  cameraFlip = 0.f;
		float  cameraUnzoomedX = 0.f;
		bool   unk322a = false;
		bool   unk3251 = false;

		bool   valid = false;
	};
	PendingExtra          pendingExtra{};

	bool                  resyncing      = false;
	bool                  resyncForBacktrack = false;
	bool                  resumeHold     = false;
	bool                  resumeAirMode  = false;
	bool                  resumeOnGround = false;
	uint64_t              modeStaleEvals = 0, modeStaleDiverged = 0;
	// Shadow of lastGeoLo/Hi that IS restored on reposition, so the branch rule can
	// be evaluated both ways and the difference counted. Measurement only.
	double                shadowGeoLo  = 0.0;
	double                shadowGeoHi  = 0.0;
	uint64_t              brIvSum = 0, brIvN = 0;   // derived branch ceiling
	int                   brIvMin = 1 << 30, brIvMax = 0;
	uint64_t              velPrunes = 0;   // 4b: branches killed by the velocity band
	uint64_t              geoStaleEvals    = 0;  // branch-rule comparisons made
	uint64_t              geoStaleDiverged = 0;  // ...where stale and correct disagree
	double                resumeGeoLo    = 0.0;
	double                resumeGeoHi    = 0.0;
	int                   resumeTap      = 0;
	int                   resumeToggles  = 0;
	int                   resumeTaps     = 0;
	int                   resumeBranch   = 0;
	uint64_t              replayBacktracks = 0;

	// Anchor replay: walking the macro forward from a cube ancestor to reach an
	// air decision that carries no checkpoint of its own.
	bool                  anchorReplaying    = false;
	int                   anchorReplayTarget = 0;
	uint64_t              anchorReplays      = 0;
	bool                  tapping            = false; // rhythm state (persists)
	bool                  resumeTapping      = false;
	int                   tapRemaining       = 0;   // steps left in the current tap
	int                   resyncTarget   = 0;
	int                   lastResyncStep = 0;
	uint64_t              resyncs        = 0;
	uint64_t              resyncFailures = 0;
	std::vector<uint8_t>  resyncMacro;      // prefix being validated
	std::vector<uint8_t>  lastGoodMacro;    // last prefix that replayed clean
	// The furthest-reaching path found so far, kept so a stalled search can be
	// WATCHED rather than inferred from counters. Written to <level>_best.txt.
	std::vector<uint8_t>  bestMacro;
	int                   lastGoodStep   = 0;

	// The trajectory of the path currently on the stack, truncated on every
	// restore exactly as `macro` is. Preallocated and never grown in the
	// stepping path: a realloc there is an unbounded stall, so an overrun stops
	// recording and says so rather than reallocating.
	std::vector<probe::TraceRow> pathTrace;
	bool                         pathTraceFull = false;
	bool                         pendingPostRestore = false;

	// best.txt used to be written ONLY by F9. A stalled run therefore left the
	// file holding whatever an earlier, unrelated run had produced - and F10
	// silently swept anchors across that stale path, reporting results for a
	// section of the level the current search had never reached. Flag the macro
	// dirty on every improvement and flush it from the throttled report.
	bool                         bestMacroDirty = false;

	void clear() {
		// Drop our checkpoints out of GD's reach first. Releasing one while the
		// game still references it leaves a dangling pointer, which is a
		// plausible cause of the crashes on returning to the level screen.
		if (auto* pl = PlayLayer::get()) {
			if (auto* arr = pl->m_checkpointArray) arr->removeAllObjects();
		}
		for (auto& d : stack) releaseCheckpoint(d.rs.cp);
		stack.clear();
		macro.clear();
		running = false;
		hold = false;
		step = 0;
		lastBranch = 0;
		togglesUsed = 0;
		tapsUsed = 0;
		resyncing = false;
		resyncForBacktrack = false;
		replayBacktracks = 0;
		tapping = false;
		resumeTapping = false;
		tapRemaining = 0;
		anchorReplaying    = false;
		anchorReplayTarget = 0;
		anchorReplays      = 0;
		resyncTarget = 0;
		lastResyncStep = 0;
		resyncs = resyncFailures = 0;
		budgetPops.clear();
		budgetPops.reserve(200000);
		budgetPopCount = 0;
		budgetGateEvals = 0;
		budgetGateWouldToggle = 0;
		gateFreeTap = gateFreeHold = 0;
		budgetMaxSpent = -1;
		budgetPopsFull = false;
		resyncMacro.clear();
		lastGoodMacro.clear();
		motionPrevVy   = 0.0;
		motionPrevHold = false;
		motionPrevGround = false;
		motionPrevMode   = 255;
		motionPrevPad = motionPrevRing = motionPrevCustomRing = false;
		motionHavePrev = false;
		bestMacro.clear();
		pathTrace.clear();
		pathTraceFull = false;
		pendingPostRestore = false;
		bestMacroDirty = false;
		lastGoodStep = 0;
		prevAirMode = false;
		prevOnGround = false;
		commitDepth = 0;
		bestStep = 0;
		lookbackSteps = 0;
		lastUndone = 0;
		deathCells.clear();
		deathVySum.clear();
		deathsRecorded = 0;
		anchorReleased = false;
		bestPct = 0.f;
		// escapes MUST reset with the rest. It is not just a counter: the
		// widening gate reads `escapes - escapesAtWiden >= escapesBeforeWidening`,
		// and escapesAtWiden resets here while escapes did not - so the SECOND
		// level solved in a session inherited a nonzero escape count against a
		// zeroed baseline and widened its mutable window on the first stall,
		// however well the search was actually doing.
		//
		// MEASURED: Clutterfunk's first report of a run reads `escapes 25` at 80
		// deaths, inherited from the Time Machine solve before it. It did not
		// bite there only because Clutterfunk never stalled, so the gate never
		// ran. Any level that runs later in a session AND stalls was searching
		// a different shape than the same level run first.
		deaths = restores = steps = escapes = 0;
		geoDeaths = 0;
		geoSteers = geoLooks = 0;
		geoSteerDead = geoSteerWin = geoSteerMute = 0;
		learnedCeiling  = 1e30;
		// roofWarned is per-LEVEL and was never cleared, so one warning anywhere in
		// a session permanently silenced the check for every level after it.
		roofWarned      = false;
		roofMarginMin   =  1e30;
		roofMarginMinX  = 0.0;
		roofMarginMax   = -1e30;
		roofMarginSum   = 0.0;
		roofMarginN     = 0;
		ceilingCrops    = 0;
		entryPortalY    = 0.0;
		entryPortalX    = 0.0;
		haveEntryPortal = false;
		entryPortalMode = -1;
		entryPortalID   = 0;
		portalEntries   = 0;
		entryStaleDrops = 0;
		ceilOffMin =  1e30;
		ceilOffMax = -1e30;
		camOffMin  =  1e30;
		camOffMax  = -1e30;
		ceilOffN = camOffN = 0;
		camTopSeen = 0.0;
		for (int i = 0; i < kCeilCand; i++) {
			candMin[i]  =  1e30;
			candMax[i]  = -1e30;
			candLast[i] = 0.0;
		}
		candN = 0;
		ceilHist.clear();
		prevX = 0.0;
		dxPerStep = 1.31;
		prevY = 0.0;
		lastGeoLo = lastGeoHi = 0.0;
		resumeGeoLo = resumeGeoHi = 0.0;
		resumeAirMode = resumeOnGround = false;
		modeStaleEvals = modeStaleDiverged = 0;
		shadowGeoLo = shadowGeoHi = 0.0;
		brIvSum = brIvN = 0; brIvMin = 1 << 30; brIvMax = 0;
		velPrunes = 0;
		geoStaleEvals = geoStaleDiverged = 0;
		std::memset(maxClimb, 0, sizeof(maxClimb));
		tapDecisions = tapFirstChosen = 0;
	}

	// Reserved once in clear(); the stepping path only ever push_back()s into
	// the reserved capacity and stops recording when it is full.
	std::vector<BudgetPop> budgetPops;
	uint64_t               budgetPopCount = 0;
	uint64_t               budgetGateEvals = 0;        // gate reached at all
	uint64_t               budgetGateWouldToggle = 0;  // ...and the branch costs a toggle

	// Gate evaluations whose alternative was FREE, split by mode class. This is
	// the number the stale-tap question turns on: a free alternative is a
	// decision the budget cannot refuse, so it is the budget leaking. Split
	// because the leak is specific to Tap's cost asymmetry - Hold charges for a
	// CHANGE and is symmetric, Tap charges for a TAP and is not.
	uint64_t               gateFreeTap  = 0;
	uint64_t               gateFreeHold = 0;
	int                    budgetMaxSpent = -1;        // highest `spent` ever seen
	bool                   budgetPopsFull = false;

	static Solver& get() { static Solver s; return s; }
};


// ---------------------------------------------------------------------------
// Go-Explore: archive of cells, return-then-explore
// ---------------------------------------------------------------------------
//
// Ecoffet et al., "First Return, Then Explore" (Nature 2021). The algorithm
// exists to fix two named failures, both of which this project has now measured:
//
//   DETACHMENT - the searcher forgets promising frontiers it already reached.
//     The DFS's Theory of Everything stall is exactly this: escapes 41/42/43
//     were identical, 18,557 deaths re-deriving one subtree, because the escape
//     heuristic drops decisions and their `tried` state along with them.
//
//   DERAILMENT - the searcher explores WHILE returning, so it never reliably
//     gets back to the promising state. The beam pruning its way through a ship
//     section and losing the thread is a fair description of this.
//
// The fix for both is structural rather than heuristic: keep an archive of
// states, RETURN to one with no exploration at all (a savestate restore), and
// only then explore. The paper notes returning by state load rather than replay
// is ~45x faster, which is the same substrate argument this project is built on.
//
// Unlike the beam there is no synchronised frontier, so there is nothing that
// can collapse: the archive only ever grows or improves.

struct GoEntry {
	RestoreState rs;                // how to get back here, with no exploration
	std::vector<uint8_t> macro;     // inputs from frame 0 to this cell

	float    pct    = 0.f;
	int      step   = 0;
	bool     hold   = false;        // input state on arrival, for the restore
	uint32_t chosen = 0;            // times selected as an exploration start
	uint32_t seen   = 0;            // times any trajectory reached this cell
};

// One node of the local DFS an episode runs inside a cell.
//
// Deliberately separate from Decision: that one carries the global solver's
// commit floor, toggle budget, escape bookkeeping and mode anchors, none of
// which apply to a bounded local search whose job is to explore one
// neighbourhood and then hand control back to the archive.
// The solver's own cell archive, filled by the DFS's traversal and read by the
// escape path. Separate from GoExplore's so the two searches cannot corrupt
// each other's memory, but deliberately the SAME cell function and the same
// entry shape - if the cell abstraction is right for one it is right for both,
// and having two would mean two things to get wrong.
// Probe 10: the ceiling the player actually has, per x band. See ceilingProbe.
struct CeilingProbe {
	struct Band {
		double maxY    = -1e30;  // highest the player ever got here, any mode
		double contactY = 1e30;  // lowest y at which it was STOPPED while rising
		uint32_t reached = 0, contacts = 0;
	};
	std::map<int32_t, Band> bands;
	uint64_t samples = 0;

	// The rule we are actually trying to find.
	//
	// Public sources say the playable band is a fixed 10 blocks - 300 units -
	// and that a ship's roof cannot be raised. That matches the observed
	// behaviour: the same height, anchored wherever the section puts it, which
	// is why it moves with the portal rather than with the objects.
	//
	// If that is right then `contactY - cameraY` is CONSTANT across every
	// contact in every section, and one number turns the map's global roof into
	// a correct per-section one. If it is not constant, the anchoring is
	// something else and the spread here says how much else.
	double offMin =  1e30, offMax = -1e30;   // contactY - cameraY
	double camHMin =  1e30, camHMax = -1e30; // m_cameraHeight
	double ceilMin =  1e30, ceilMax = -1e30; // absolute contact y
	uint64_t contacts = 0;

	void clear() {
		bands.clear();
		samples = 0;
		offMin = camHMin = ceilMin =  1e30;
		offMax = camHMax = ceilMax = -1e30;
		contacts = 0;
	}
	static CeilingProbe& get() { static CeilingProbe c; return c; }
};

// Probe 9: the measured one-step vertical transition. See motionModelEnabled.
struct MotionModel {
	struct Row {
		// The key, kept unpacked so the dump can print it - the map is keyed by
		// a hash and the fields are not otherwise recoverable.
		int32_t  vyb   = 0;
		uint8_t  mode  = 0, size = 0, speed = 0, flags = 0;

		double   vyNextMin =  1e30;  // the two extremes this key ever produced.
		double   vyNextMax = -1e30;  // any spread means the key is incomplete
		double   dySum     = 0.0;    // mean world-units moved, for the ramp
		uint32_t count     = 0;
	};

	std::unordered_map<uint64_t, Row> rows;
	uint64_t recorded = 0;
	uint64_t capped   = 0;

	void clear() { rows.clear(); recorded = capped = 0; }
	static MotionModel& get() { static MotionModel m; return m; }
};

struct SolverArchive {
	std::unordered_map<uint64_t, GoEntry> cells;
	std::vector<uint64_t> keys;

	uint64_t restarts    = 0;   // escapes served by a return instead of a rewind
	uint64_t restartFail = 0;   // ...and escapes that fell back to the rewind
	uint64_t newCells    = 0;
	uint64_t evicted     = 0;

	// Own RNG, seeded fixed: a run that misbehaves once and never again is not
	// debuggable. xorshift64*, so the sequence is ours and not the CRT's.
	uint64_t rng = 0x9E3779B97F4A7C15ull;
	uint64_t next() {
		rng ^= rng >> 12; rng ^= rng << 25; rng ^= rng >> 27;
		return rng * 2685821657736338717ull;
	}

	void clear() {
		for (auto& kv : cells) releaseCheckpoint(kv.second.rs.cp);
		cells.clear();
		keys.clear();
		restarts = restartFail = newCells = evicted = 0;
		rng = 0x9E3779B97F4A7C15ull;
	}

	static SolverArchive& get() { static SolverArchive a; return a; }
};

struct GoDecision {
	RestoreState rs;
	uint8_t tried        = 0;      // bit0 = tried release, bit1 = tried hold
	bool    enteringHold = false;  // input that produced rs, for the restore
	int     step         = 0;
	int     macroLen     = 0;      // curMacro length on arrival, for truncation
};


// A read-only census of the cells the DFS actually visits.
//
// It exists to settle one question BEFORE anything is built on the answer:
// when the search stalls in a fake corridor, did it ever touch the other
// corridors at all? An archive can only ever return a search to somewhere it
// has already been. If every cell around the stall sits at a single height,
// then an archive-backed restart has nowhere else to send it and would hand
// back the same corridor forever - so it should not be built.
//
// Deliberately stores NO checkpoint. Capturing one is only ~12 us (Probe 5) and
// would be affordable, but it would add thousands of live checkpoints to the
// DFS and so perturb the very restore cost that is being measured separately.
// What is stored is bands and counts, and nothing in the solver reads any of it
// back, so this cannot change a single decision the search takes.
//
// Caveat worth knowing when reading a dump: reported x is corrupted for the ~95
// steps of a mirror-portal camera flip (see Solver::PendingExtra::cameraFlip),
// so a handful of wildly out-of-range x bands around a flip are an artifact of
// the reporting, not places the player went.
// Probe 6 state. One replay per (S, k) variant, back to back at solver speed.
struct Sweep {
	static Sweep& get() { static Sweep s; return s; }

	std::vector<uint8_t> base;   // best.txt as loaded
	std::vector<uint8_t> macro;  // base with this variant's climb injected

	struct Result {
		int   S, k, endStep, maxY;
		float pct;
		bool  died, cleared;
	};
	std::vector<Result> results;

	int   variant = 0;
	int   step    = 0;
	float bestPct = 0.f;
	int   maxY    = -9999;   // highest y band seen at x >= sweepCorridorX
	bool  running = false;

	int variantCount() const {
		return g_config.sweepStepCount * (g_config.sweepMaxTaps + 1);
	}
	int variantS() const {
		return g_config.sweepStepBase +
		       (variant / (g_config.sweepMaxTaps + 1)) * g_config.sweepStepStride;
	}
	int variantK() const { return variant % (g_config.sweepMaxTaps + 1); }

	void clear() {
		base.clear(); macro.clear(); results.clear();
		variant = 0; step = 0; bestPct = 0.f; maxY = -9999; running = false;
	}
};

// Build the current variant's macro: k taps from step S, each a 2-step press
// followed by a 2-step release, on the same 4-step cadence the solver branches
// on. Everything outside the injected window is the 78.90% macro untouched.
void sweepBuildMacro() {
	auto& sw = Sweep::get();
	sw.macro = sw.base;
	const int S = sw.variantS();
	const int k = sw.variantK();
	for (int i = 0; i < k; i++) {
		const size_t a = static_cast<size_t>(S + i * 4);
		if (a + 3 >= sw.macro.size()) break;
		sw.macro[a] = 1; sw.macro[a + 1] = 1;
		sw.macro[a + 2] = 0; sw.macro[a + 3] = 0;
	}
}

struct CellProbe {
	struct Cell {
		int32_t  xb = 0, yb = 0, vyb = 0;
		uint32_t mode = 0, flags = 0;
		uint64_t seen = 0;       // steps that landed in this cell
		float    firstPct = 0.f;
		int      firstStep = 0;
	};
	std::unordered_map<uint64_t, Cell> cells;
	uint64_t recorded = 0;   // steps censused
	uint64_t capped   = 0;   // new cells dropped because the map was full
	uint64_t lastDump = 0;   // ticks, for throttling the file write

	void clear() { cells.clear(); recorded = capped = 0; lastDump = 0; }
	static CellProbe& get() { static CellProbe c; return c; }
};

// Probe 9: write the measured motion model out, plus the two summaries that
// decide whether it can be trusted. File I/O, so never from the stepping path.
//
// Read the SPREAD column first. vy' = f(key) is a function; a nonzero spread on
// any row means that key is missing a field and the model is wrong, and no
// amount of curve fitting downstream will repair it. Read COVERAGE second: the
// vy band actually observed per configuration is the band inside which the
// model can answer, and outside which it must not be asked.
void motionModelDump(const char* why) {
	auto& mm = MotionModel::get();
	if (!g_config.motionModelEnabled || mm.rows.empty()) return;

	const std::string path =
		probe::outputPath(ProbeState::get().levelKey + "_motion.csv");
	std::FILE* f = std::fopen(path.c_str(), "w");
	if (!f) { log::error("Probe 9: cannot write {}", path); return; }

	std::fprintf(f, "# motion model (%s): vy' = f(mode,size,speed,grav,ground,"
	                "prevHold,hold,isAccel,jumpBuf,event,vy)\n", why);
	std::fprintf(f, "# READ THE SPREAD COLUMN FIRST. vy is a function of the "
	                "key, so a nonzero spread means an axis is still missing.\n");
	std::fprintf(f, "# %llu transitions recorded, %llu distinct keys, %llu capped, "
	                "vy bucket %.4f\n",
	             static_cast<unsigned long long>(mm.recorded),
	             static_cast<unsigned long long>(mm.rows.size()),
	             static_cast<unsigned long long>(mm.capped),
	             g_config.motionVyBucket);
	std::fprintf(f, "mode,size,speed,upsideDown,onGround,prevHold,hold,"
	                "isAccel,jumpBuf,event,"
	                "vy,vyNextMin,vyNextMax,spread,dyMean,count\n");

	// Per-configuration coverage and consistency, accumulated while writing.
	struct Cov {
		double vyLo = 1e30, vyHi = -1e30;
		double worstSpread = 0.0;
		uint64_t rows = 0, conflicts = 0, samples = 0;
	};
	std::map<uint32_t, Cov> cov;

	static const char* kMode[8] = {"cube", "ship", "wave", "ufo", "ball",
	                               "robot", "spider", "swing"};

	// Sorted so the CSV reads as curves rather than hash order - the ramp is
	// only legible if vy is monotonic within a configuration.
	std::vector<MotionModel::Row> rows;
	rows.reserve(mm.rows.size());
	for (auto const& kv : mm.rows) rows.push_back(kv.second);
	std::sort(rows.begin(), rows.end(),
		[](MotionModel::Row const& a, MotionModel::Row const& b) {
			if (a.mode  != b.mode)  return a.mode  < b.mode;
			if (a.size  != b.size)  return a.size  < b.size;
			if (a.speed != b.speed) return a.speed < b.speed;
			if (a.flags != b.flags) return a.flags < b.flags;
			return a.vyb < b.vyb;
		});

	for (auto const& r : rows) {
		const double vy      = r.vyb * g_config.motionVyBucket;
		const double spread  = r.vyNextMax - r.vyNextMin;
		const double dyMean  = r.count ? r.dySum / r.count : 0.0;
		std::fprintf(f, "%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.4f,%.6f,%.6f,%.6f,%.4f,%u\n",
		             kMode[r.mode & 7], r.size, r.speed,
		             (r.flags & 1) ? 1 : 0, (r.flags & 8) ? 1 : 0,
		             (r.flags & 2) ? 1 : 0, (r.flags & 4) ? 1 : 0,
		             (r.flags & 16) ? 1 : 0, (r.flags & 32) ? 1 : 0, (r.flags & 64) ? 1 : 0,
		             vy, r.vyNextMin, r.vyNextMax, spread, dyMean, r.count);

		const uint32_t ck = (static_cast<uint32_t>(r.mode) << 16)
		                  | (static_cast<uint32_t>(r.size) << 8)
		                  | r.speed;
		auto& c = cov[ck];
		c.vyLo = std::min(c.vyLo, vy);
		c.vyHi = std::max(c.vyHi, vy);
		c.worstSpread = std::max(c.worstSpread, spread);
		// The tolerance is the BUCKET WIDTH, not zero, and getting this wrong
		// made the first run look catastrophic. vy' is roughly vy + a, so any
		// bucket holding two distinct vy values produces up to one bucket of
		// spread by construction. Measured on Base After Base: p50 0.003, p90
		// 0.009 against a 0.01 bucket - all quantisation, no disagreement.
		//
		// Real disagreements sit far above it and were all one thing: rows with
		// vyNextMax exactly 0, i.e. the floor and ceiling CLAMP, where contact
		// overrides the physics. In free air the spread is exactly 0.000000.
		const double tol = 1.5 * g_config.motionVyBucket;
		if (spread > tol) c.conflicts++;
		c.rows++;
		c.samples += r.count;
	}
	std::fclose(f);

	log::info("Probe 9: motion model -> {}  |  {} transitions, {} keys",
	          path, mm.recorded, mm.rows.size());
	log::info("  configuration      vy band observed        keys   samples  conflicts");
	for (auto const& kv : cov) {
		auto const& c = kv.second;
		log::info("  {:<6} {} {}x   {:+7.3f} .. {:+7.3f}   {:>6}  {:>8}  {:>6}{}",
		          kMode[(kv.first >> 16) & 7],
		          ((kv.first >> 8) & 0xff) ? "mini" : "    ",
		          (kv.first & 0xff),
		          c.vyLo, c.vyHi, c.rows, c.samples, c.conflicts,
		          c.conflicts ? "  <-- KEY IS INCOMPLETE" : "");
	}
	log::info("  Conflicts must be 0. A nonzero count means one key produced two "
	          "different vy', so the key is missing a field and the model cannot "
	          "be used to bound anything until that field is found.");
}

// Write the census out. Called at stalls and at the end of a search, never
// from the hot loop - this formats and does file I/O. Touches no player state,
// so the key handler can call it too.
void cellProbeDump(const char* why) {
	auto& cp = CellProbe::get();
	auto& sv = Solver::get();
	if (!g_config.cellProbeEnabled || cp.cells.empty()) return;

	// Collapse to x band -> y band -> visits. The y SPREAD inside the bands
	// around the stall is the entire answer being sought: several occupied
	// heights means the search did touch more than one corridor and an archive
	// would have somewhere else to send it; one height means it never did, and
	// a restart could only ever hand back the same corridor.
	std::map<int32_t, std::map<int32_t, uint64_t>> byX;
	std::map<int32_t, float> minPct;
	for (auto const& kv : cp.cells) {
		auto const& e = kv.second;
		byX[e.xb][e.yb] += e.seen;
		auto it = minPct.find(e.xb);
		if (it == minPct.end() || e.firstPct < it->second) minPct[e.xb] = e.firstPct;
	}

	const std::string path = levelFilePath("cells.txt");
	std::FILE* f = std::fopen(path.c_str(), "wb");
	if (!f) return;
	std::fprintf(f, "# gd-solver cell census (%s)\n", why);
	std::fprintf(f, "# best %.2f%%  step %d  deaths %llu  escapes %llu\n",
	             sv.bestPct, sv.step,
	             static_cast<unsigned long long>(sv.deaths),
	             static_cast<unsigned long long>(sv.escapes));
	std::fprintf(f, "# cells %zu  steps censused %llu  dropped at cap %llu\n",
	             cp.cells.size(),
	             static_cast<unsigned long long>(cp.recorded),
	             static_cast<unsigned long long>(cp.capped));
	std::fprintf(f, "# cell size: x %.0f units, y %.0f units, vy %.2f (signed sqrt)\n",
	             g_config.goCellX, g_config.goCellY, g_config.goCellVy);
	std::fprintf(f,
	    "#\n"
	    "# One row per x band. `heights` is the number of DISTINCT y bands the\n"
	    "# search ever occupied there, counting cells it entered once and then\n"
	    "# abandoned. Across a fork, heights=1 means the search never saw the\n"
	    "# alternative and an archive has nothing to offer it; heights>1 means\n"
	    "# it did see one, and a restart has somewhere real to send it.\n"
	    "#\n"
	    "# ybands are band(visits); one band is %.0f units of height.\n"
	    "#\n", g_config.goCellY);
	std::fprintf(f, "# %-7s %-9s %-7s %-8s %s\n",
	             "xband", "x", "pct", "heights", "ybands(visits)");
	for (auto const& xk : byX) {
		auto const& ys = xk.second;
		const auto pit = minPct.find(xk.first);
		std::fprintf(f, "%-9d %-9.0f %-7.2f %-8zu ",
		             xk.first,
		             static_cast<double>(xk.first) * g_config.goCellX,
		             pit == minPct.end() ? 0.f : pit->second,
		             ys.size());
		int n = 0;
		for (auto const& yk : ys) {
			if (n++ == 16) { std::fprintf(f, "..."); break; }
			std::fprintf(f, "%d(%llu) ", yk.first,
			             static_cast<unsigned long long>(yk.second));
		}
		std::fputc('\n', f);
	}
	std::fclose(f);
	log::info("Solver: cell census ({}) -> {}  |  {} cells over {} x bands from {} "
	          "steps{}", why, path, cp.cells.size(), byX.size(), cp.recorded,
	          cp.capped ? fmt::format(" ({} dropped at cap - census truncated)",
	                                  cp.capped) : std::string());
}

// Write the furthest-reaching path, so a stall can be replayed and watched at
// normal speed instead of diagnosed from death counts.
void solverWriteBestMacro();

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

// Write the furthest-reaching path the search has found. When the solver stalls,
// the counters say WHERE it stops but never WHY - this lets the best attempt be
// replayed and watched at normal speed instead of inferred from death counts.
// Probe 7. One pass over the level's objects, bucketed into census bands.
void geometryDump() {
	auto* gl = GJBaseGameLayer::get();
	if (!gl || !gl->m_objects) { log::error("Probe 7: no level objects"); return; }

	const double xb = std::max(1.0, g_config.goCellX);
	const double yb = std::max(1.0, g_config.goCellY);

	// band -> what occupies it. Hazard wins the display: a band that is both is
	// lethal, and that is the fact that matters to a route.
	std::map<std::pair<int,int>, char> grid;
	std::map<int, int> typeCount;
	int scanned = 0, inRange = 0;

	std::string objPath = levelFilePath("geometry_objects.txt");
	std::FILE* fo = g_config.geomDumpObjects ? std::fopen(objPath.c_str(), "wb") : nullptr;
	if (fo) std::fprintf(fo, "# %-9s %-9s %-9s %-9s %-6s %-6s %s\n",
	                     "x", "y", "w", "h", "type", "objID", "class");

	auto* arr = gl->m_objects;
	for (unsigned int i = 0; i < arr->count(); i++) {
		auto* o = static_cast<GameObject*>(arr->objectAtIndex(i));
		if (!o) continue;
		scanned++;

		const int t = static_cast<int>(o->getType());
		typeCount[t]++;

		// Everything except pure decoration is written to the object listing:
		// portals, pads and rings are invisible to the map today and are exactly
		// what it needs next - speed for the vertical reach, mode for per-mode
		// player height and ground. Only solid and hazard enter the grid.
		if (fo && t != static_cast<int>(GameObjectType::Decoration)) {
			const cocos2d::CCRect rr = o->getObjectRect();
			std::fprintf(fo, "%-11.2f %-9.2f %-9.2f %-9.2f %-6d %-6d %s\n",
			             rr.getMinX(), rr.getMinY(), rr.size.width, rr.size.height,
			             t, o->m_objectID, "other");
		}

		const bool solid  = t == static_cast<int>(GameObjectType::Solid) ||
		                    t == static_cast<int>(GameObjectType::Slope) ||
		                    t == static_cast<int>(GameObjectType::Breakable);
		const bool hazard = t == static_cast<int>(GameObjectType::Hazard) ||
		                    t == static_cast<int>(GameObjectType::AnimatedHazard);
		if (!solid && !hazard) continue;

		const cocos2d::CCRect r = o->getObjectRect();
		const bool wholeLevel = g_config.geomXMin == 0 && g_config.geomXMax == 0;
		if (!wholeLevel &&
		    (r.getMaxX() < g_config.geomXMin || r.getMinX() > g_config.geomXMax)) continue;
		inRange++;

		if (fo) std::fprintf(fo, "%-11.2f %-9.2f %-9.2f %-9.2f %-6d %-6d %s\n",
		                     r.getMinX(), r.getMinY(), r.size.width, r.size.height,
		                     t, o->m_objectID, hazard ? "HAZARD" : "solid");

		// Half-open on the far edge. A floor occupying [930, 960) is entirely
		// band 31, but floor(960/30) = 32 marked band 32 solid as well, so every
		// floor ate the bottom band of the corridor above it and the map showed
		// ToE's 2-block corridors as 1 band with 2-band floors. MEASURED against
		// the raw objects: floors at y 930/1020/1110/1200 are 30 tall, corridors
		// are 960-1020, 1050-1110, 1140-1200 - 60 units, exactly two blocks.
		const double eps = 1e-6;
		const int x0 = static_cast<int>(std::floor(r.getMinX() / xb));
		const int x1 = static_cast<int>(std::floor((r.getMaxX() - eps) / xb));
		const int y0 = static_cast<int>(std::floor(r.getMinY() / yb));
		const int y1 = static_cast<int>(std::floor((r.getMaxY() - eps) / yb));
		for (int x = x0; x <= x1; x++) {
			for (int y = y0; y <= y1; y++) {
				char& c = grid[{x, y}];
				if (hazard || c == 0) c = hazard ? 'H' : 'S';
			}
		}
	}
	if (fo) std::fclose(fo);

	if (grid.empty()) {
		log::warn("Probe 7: {} objects scanned, none solid or hazardous in x {}..{}",
		          scanned, g_config.geomXMin, g_config.geomXMax);
		return;
	}

	int xlo = INT_MAX, xhi = INT_MIN, ylo = INT_MAX, yhi = INT_MIN;
	for (auto const& kv : grid) {
		xlo = std::min(xlo, kv.first.first);  xhi = std::max(xhi, kv.first.first);
		ylo = std::min(ylo, kv.first.second); yhi = std::max(yhi, kv.first.second);
	}

	const std::string path = levelFilePath("geometry.txt");
	std::FILE* f = std::fopen(path.c_str(), "wb");
	if (!f) { log::error("Probe 7: cannot write {}", path); return; }

	std::fprintf(f, "# gd-solver level geometry (Probe 7)\n");
	std::fprintf(f, "# %d objects scanned, %d solid/hazard in x %d..%d\n",
	             scanned, inRange, g_config.geomXMin, g_config.geomXMax);
	std::fprintf(f, "# bands: x %.0f units, y %.0f units - the SAME bands as the cell "
	                "census, so the two overlay\n", xb, yb);
	std::fprintf(f, "# S = solid, H = hazard, . = OPEN (a route can pass through)\n");
	std::fprintf(f, "# y bands %d..%d increase upward; each row is one x band\n#\n", ylo, yhi);

	std::fprintf(f, "# %-7s %-9s %-7s ", "xband", "x", "pct");
	for (int y = yhi; y >= ylo; y--) std::fprintf(f, "%d", std::abs(y) % 10);
	std::fprintf(f, "  (y%d down to y%d)\n", yhi, ylo);

	auto* pl = PlayLayer::get();
	const double len = (pl && pl->m_levelLength > 1.f) ? pl->m_levelLength : 0.0;
	for (int x = xlo; x <= xhi; x++) {
		const double xu = x * xb;
		std::fprintf(f, "%-9d %-9.0f %-7.2f ", x, xu,
		             len > 0.0 ? (xu / len) * 100.0 : 0.0);
		for (int y = yhi; y >= ylo; y--) {
			auto it = grid.find({x, y});
			std::fputc(it == grid.end() ? '.' : it->second, f);
		}
		std::fputc('\n', f);
	}
	std::fclose(f);

	log::info("Probe 7: {} objects scanned, {} solid/hazard in x {}..{}. "
	          "x bands {}..{}, y bands {}..{}.",
	          scanned, inRange, g_config.geomXMin, g_config.geomXMax,
	          xlo, xhi, ylo, yhi);
	std::string types;
	for (auto const& kv : typeCount) {
		if (!types.empty()) types += " ";
		types += std::to_string(kv.first) + ":" + std::to_string(kv.second);
	}
	log::info("  object types present (type:count) - {}", types);
	log::info("  map -> {}", path);
	if (g_config.geomDumpObjects) log::info("  raw objects -> {}", objPath);
}

// The level's geometry as FREE INTERVALS over EXACT x segments.
//
// Neither axis is on a grid now. y was freed first: bucketing it into 30-unit
// rows made a 7.2-tall spike block 30 units, which blanketed the level in false
// hazard and collapsed Theory of Everything to 6.27%.
//
// x was still sliced every 30 units, and that had the same shape of error in
// miniature: everything overlapping a slice blocked that slice's full width, so
// an object spanning x 24885..24915 blocked both 24870..24900 and 24900..24930
// across its whole y extent. Over-blocking narrows windows, and narrow windows
// are what make the solver steer.
//
// The fix needs no approximation at all. An object blocks a y range over
// [minX, maxX], so the set of blocking objects changes ONLY at an object edge.
// Sort every edge; between two consecutive edges the blocked set is identical by
// construction, so one segment represents that stretch exactly.
//
// Segments have variable width, so the vertical reach scales with width rather
// than being a flat per-slice constant: geomVerticalReach is still expressed per
// geomSliceX units of travel, which keeps 60 meaning what it always meant.
// GD's five speeds, in world units per 1/240 physics step.
//
// The two the solver has actually seen are confirmed against the player's own
// motion: Electrodynamix reported dx 1.61 through its objID-202 section and
// 1.95 through its objID-203 section, matching Fast and Faster here. The other
// three are the same well-known series and are checked the same way the moment
// a level uses one - solverReport prints measured dx beside the map's expected
// dx, so a wrong entry shows up as a mismatch rather than as a silent stall.
constexpr double kSpeedSlow    = 251.16 / 240.0;   // 1.0465
constexpr double kSpeedNormal  = 311.58 / 240.0;   // 1.2983
constexpr double kSpeedFast    = 387.42 / 240.0;   // 1.6143
constexpr double kSpeedFaster  = 468.00 / 240.0;   // 1.9500
constexpr double kSpeedFastest = 576.00 / 240.0;   // 2.4000

// Speed portals. objectID is the discriminator: these share GameObjectType 20
// with colour and pulse triggers, so the type says nothing.
double speedForPortalID(int objID) {
	switch (objID) {
		case 200:  return kSpeedSlow;
		case 201:  return kSpeedNormal;
		case 202:  return kSpeedFast;
		case 203:  return kSpeedFaster;
		case 1334: return kSpeedFastest;
		default:   return 0.0;   // not a speed portal
	}
}

const char* geomReachModeName(int mode) {
	switch (mode) {
		case 1:  return "OVERLAP (infinite, no narrowing)";
		case 2:  return "ALLLIVE (liveness off)";
		default: return "WINDOWED";
	}
}

double speedForStartSetting(int startSpeed) {
	switch (startSpeed) {
		case 1:  return kSpeedSlow;      // Speed::Slow
		case 2:  return kSpeedFast;      // Speed::Fast
		case 3:  return kSpeedFaster;    // Speed::Faster
		case 4:  return kSpeedFastest;   // Speed::Fastest
		default: return kSpeedNormal;    // Speed::Normal
	}
}

struct GeoMap {
	static GeoMap& get() { static GeoMap m; return m; }

	struct Span { float lo, hi; uint8_t type; };      // 1 solid, 2 hazard
	// `lo..hi` is the open gap. `wlo..whi` is the part of it from which the end
	// of the level is still reachable - the LIVE WINDOW. A boolean was not
	// enough: at ToE's last ship section the gap at x 24870 is 300 units and the
	// only way through is 90, and marking the whole 300 live meant a ship at
	// y 760 was "in live space" while already committed to the pillar.
	struct Free { float lo, hi, wlo, whi; bool live; };

	std::vector<double> xs;        // segment boundaries; segment i is [xs[i], xs[i+1])
	std::vector<std::vector<Span>> blocked;
	std::vector<std::vector<Free>> freeSpans;

	// Horizontal units per physics step in each segment, from the level's start
	// speed and its speed portals. The map is built before the run, so unlike
	// the lookahead this cannot be measured off the player - it has to be read.
	std::vector<float> segSpeed;

	// The roof the build actually applied at each segment - what freeSpans was
	// cropped against. It used to be a local in geometryBuildMap, consumed and
	// thrown away, so at run time nothing could ask "what did the roof delete
	// here?" and the only available check was against yHi, the top of the whole
	// map. With a per-x roof that check can never fire: Time Machine's yHi is
	// 810 while the roof holds the back half at 390.
	std::vector<float> segRoof;
	// Slices holding an object that can OVERRIDE velocity: jump pads, jump rings,
	// gravity pads/rings, and every portal. Marked per slice, which over-marks
	// (an orb inside a slice flags the whole slice) - the safe direction.
	//
	// This exists for the velocity prune's soundness rule. MEASURED: the motion
	// census is consistent to 0.36% in free flight but the exceptions are all
	// velocity overrides, so the reachable band is only a true bound where no
	// override can fire. See velocityUnreachable.
	std::vector<uint8_t> segEvent;
	double yLo = 0.0, yHi = 0.0;
	double playerH = 15.0;         // smallest hitbox height: permissive
	int  freeCount = 0, deadCount = 0;

	// What the ROOF cost, measured at build time over EVERY slice - not just the
	// ones the player reached. roofMargin can only ever check the paths actually
	// walked, and on a stalled level the region past the stall is exactly where a
	// bad crop would hide: a roof that deletes required space causes the stall
	// that stops us from measuring it.
	int    roofKilledSlices = 0;   // slices the roof emptied that were not empty
	int    roofRemovedIvls  = 0;   // intervals the roof removed in total
	double roofFirstKillX   = 0.0; // leftmost emptied slice, or 0
	double roofSpaceRemoved = 0.0; // world units of free height the roof deleted
	bool valid = false;

	int segments() const { return static_cast<int>(xs.size()) - 1; }

	// Which segment contains x, or -1 outside the mapped range.
	int segIndex(double x) const {
		if (!valid || xs.size() < 2 || x < xs.front() || x >= xs.back()) return -1;
		return static_cast<int>(std::upper_bound(xs.begin(), xs.end(), x) - xs.begin()) - 1;
	}

	// True only for OPEN space with no forward route. Blocked space returns
	// false: the simulation already handles walls, and claiming them here would
	// confuse "there is no route" with "there is a wall".
	bool isDead(double x, double y) const {
		const int si = segIndex(x);
		if (si < 0) return false;
		for (auto const& f : freeSpans[si])
			if (y >= f.lo && y <= f.hi) return !f.live;
		return false;
	}

	// Units per step at x, or the 1x value outside the mapped range.
	double speedAt(double x) const {
		const int si = segIndex(x);
		if (si < 0 || si >= static_cast<int>(segSpeed.size())) return kSpeedNormal;
		return segSpeed[si];
	}

	void clear() {
		xs.clear(); blocked.clear(); freeSpans.clear(); segSpeed.clear();
		freeCount = deadCount = 0; valid = false;
	}
};

bool geometryBuildMap() {
	auto* gl = GJBaseGameLayer::get();
	if (!gl || !gl->m_objects) return false;

	auto& m = GeoMap::get();
	m.clear();
	m.playerH = std::max(1.0, static_cast<double>(g_config.geomPlayerHeight));

	const double eps = 1e-6;

	struct Obj { double x0, x1; float lo, hi; uint8_t type; };
	std::vector<std::pair<double,double>> eventRanges;
	std::vector<Obj> objs;
	std::vector<double> edges;
	// (x, y) of every portal that re-snaps the camera band, for the per-x roof.
	std::vector<std::pair<double, double>> bandPortals;
	std::vector<int> bandPortalID;
	double modePortalMaxY = -1e18;
	double ylo = 1e18, yhi = -1e18;

	// (x, units per step) for every speed portal, collected in the same pass.
	std::vector<std::pair<double, double>> speedPortals;

	auto* arr = gl->m_objects;
	for (unsigned int i = 0; i < arr->count(); i++) {
		auto* o = static_cast<GameObject*>(arr->objectAtIndex(i));
		if (!o) continue;
		const int t = static_cast<int>(o->getType());
		{
			using G = GameObjectType;
			const bool isEvt =
			   t == static_cast<int>(G::YellowJumpPad)  || t == static_cast<int>(G::PinkJumpPad)
			|| t == static_cast<int>(G::GravityPad)     || t == static_cast<int>(G::YellowJumpRing)
			|| t == static_cast<int>(G::PinkJumpRing)   || t == static_cast<int>(G::GravityRing)
			|| t == static_cast<int>(G::GreenRing)      || t == static_cast<int>(G::InverseGravityPortal)
			|| t == static_cast<int>(G::NormalGravityPortal) || t == static_cast<int>(G::ShipPortal)
			|| t == static_cast<int>(G::CubePortal)     || t == static_cast<int>(G::BallPortal)
			|| t == static_cast<int>(G::UfoPortal)      || t == static_cast<int>(G::WavePortal)
			|| t == static_cast<int>(G::RobotPortal)    || t == static_cast<int>(G::SpiderPortal)
			|| t == static_cast<int>(G::RegularSizePortal) || t == static_cast<int>(G::MiniSizePortal)
			|| t == static_cast<int>(G::TeleportPortal) || t == static_cast<int>(G::DualPortal)
			|| t == static_cast<int>(G::SoloPortal)     || t == static_cast<int>(G::InverseMirrorPortal)
			|| t == static_cast<int>(G::NormalMirrorPortal)
			|| t == static_cast<int>(G::SpiderOrb)      || t == static_cast<int>(G::SpiderPad)
			|| t == static_cast<int>(G::TeleportOrb);
			if (isEvt) {
				const cocos2d::CCRect er = o->getObjectRect();
				eventRanges.emplace_back(er.getMinX(), er.getMaxX());
			}
		}

		// Mode portals, for the roof bound. IDs are the ones already established
		// in this file's portal handling; an unrecognised one simply does not
		// contribute, which loosens the bound rather than tightening it.
		switch (o->m_objectID) {
			// Gravity portals are NOT here, and were briefly added on a bad
			// inference that cost Base After Base its fix.
			//
			// MEASURED by logging every change of the live ceiling with its
			// position: Clutterfunk's ceiling changes at x 8672, 10853 and 15914,
			// which are its ship, ball and ship portals - and it passes straight
			// through the gravity portal at x 19576 with no change at all.
			//
			// What actually explained Clutterfunk's 390 was the GROUND CLAMP, not a
			// gravity portal: its ship portal sits at y 152, so the nominal band is
			// [-3, 297], below the ground, and gets pushed up to [90, 390].
			case 13: case 47: case 111: case 660:
			case 745: case 1331: case 1933:
				modePortalMaxY = std::max(modePortalMaxY,
				                          static_cast<double>(o->getPositionY()));
				bandPortals.emplace_back(static_cast<double>(o->getPositionX()),
				                         static_cast<double>(o->getPositionY()));
				bandPortalID.push_back(o->m_objectID);
				break;

			// The cube portal. Collected as a RELEASE rather than ignored.
			//
			// Cube is not wall-limited, so no camera band applies to it - but
			// leaving it out of the scan did not mean 'no cap there', it meant
			// the PREVIOUS section's cap stood for the rest of the level.
			// MEASURED on Clutterfunk: its last ship portal caps at 390 and the
			// cube section that follows inherited it, so the winning macro spent
			// 1004 steps outside the map, up to 213 units above the roof, all of
			// it past x 25056. Probe 17 against the default map put the same
			// path at 4 steps, so that is the whole of the difference.
			case 12:
				bandPortals.emplace_back(static_cast<double>(o->getPositionX()),
				                         static_cast<double>(o->getPositionY()));
				bandPortalID.push_back(o->m_objectID);
				break;
			default: break;
		}

		// Before the solid/hazard filter: portals are neither.
		if (g_config.geomSpeedScan) {
			const double sp = speedForPortalID(o->m_objectID);
			if (sp > 0.0) speedPortals.emplace_back(
				static_cast<double>(o->getPositionX()), sp);
		}

		const bool solid  = t == static_cast<int>(GameObjectType::Solid) ||
		                    t == static_cast<int>(GameObjectType::Slope) ||
		                    t == static_cast<int>(GameObjectType::Breakable);
		const bool hazard = t == static_cast<int>(GameObjectType::Hazard) ||
		                    t == static_cast<int>(GameObjectType::AnimatedHazard);
		if (!solid && !hazard) continue;

		const cocos2d::CCRect r = o->getObjectRect();
		Obj ob;
		ob.x0 = r.getMinX(); ob.x1 = r.getMaxX();
		ob.lo = r.getMinY(); ob.hi = r.getMaxY();
		ob.type = static_cast<uint8_t>(hazard ? 2 : 1);
		if (ob.x1 - ob.x0 < eps) continue;
		objs.push_back(ob);
		edges.push_back(ob.x0);
		edges.push_back(ob.x1);
		ylo = std::min(ylo, static_cast<double>(ob.lo));
		yhi = std::max(yhi, static_cast<double>(ob.hi));
	}
	if (objs.empty()) return false;

	std::sort(edges.begin(), edges.end());
	// Collapse edges closer together than a hundredth of a unit: they would make
	// degenerate segments whose scaled reach is zero.
	std::vector<double> xs;
	for (double e : edges)
		if (xs.empty() || e - xs.back() > 0.01) xs.push_back(e);
	if (xs.size() < 2) return false;

	m.xs = xs;
	const int n = m.segments();
	m.blocked.assign(n, {});
	m.freeSpans.assign(n, {});
	m.segRoof.assign(n, 0.f);
	m.segEvent.assign(n, 0);
	for (auto const& er : eventRanges) {
		for (int si = 0; si < n; si++) {
			if (m.xs[si + 1] <= er.first) continue;
			if (m.xs[si] >= er.second)    break;
			m.segEvent[static_cast<size_t>(si)] = 1;
		}
	}
	m.roofKilledSlices = 0;
	m.roofRemovedIvls  = 0;
	m.roofFirstKillX   = 0.0;
	m.roofSpaceRemoved = 0.0;
	m.yLo = ylo - 120.0;
	m.yHi = yhi + 120.0;

	// Per-x roof: the highest ceiling any band-setting portal AT OR BEFORE this x
	// can produce.
	//
	// A single global maximum was the first version and it only removes phantom
	// space above the TALLEST band in the level. Any section lower than that keeps
	// its own phantom space live - which is the original Base After Base bug,
	// surviving anywhere the problem section is not the highest one. It worked
	// there only because that section happened to be the tallest.
	//
	// A prefix maximum is still a safe upper bound: at any x the player's band was
	// set by some portal at or before x, so it cannot exceed the largest cap among
	// them. And an unentered fake portal only ever loosens the bound, never
	// tightens it - the same property that made the global version safe.
	// Band height per mode. The 300 everywhere else in this file was measured on
	// SHIP and does not hold for every mode.
	//
	// MEASURED: Clutterfunk's ball section reads a ceiling of 330, which is
	// 90 + 240 - an 8-block band, confirmed independently in a custom level. With
	// a ship-sized 300 the cap there computes to 390 and leaves 330..390 phantom
	// and live.
	//
	// That also explains an accident. The old broken roof was 344, which is within
	// half a block of the correct ball roof of 330 - so it was approximating this
	// by luck, which is why Clutterfunk solved under a roof that was wrong
	// everywhere else.
	//
	// The band itself now lives in bandHeightForPortal, so the map roof and the
	// query-time rule cannot disagree about it again.

	std::vector<double> roofAt(static_cast<size_t>(n), m.yHi);
	if (g_config.geomPortalRoof > 0 && !bandPortals.empty()) {
		std::vector<std::tuple<double, double, int>> bp;
		bp.reserve(bandPortals.size());
		for (size_t i = 0; i < bandPortals.size(); i++)
			bp.emplace_back(bandPortals[i].first, bandPortals[i].second,
			                i < bandPortalID.size() ? bandPortalID[i] : 0);
		std::sort(bp.begin(), bp.end());
		const double g = std::max(1.0, g_config.geomCeilingGrid);
		size_t pi = 0;
		double run = -1e18;
		double lowest = m.yHi, highest = -1e18;
		for (int si = 0; si < n; si++) {
			while (pi < bp.size() && std::get<0>(bp[pi]) <= m.xs[si]) {
				// The running maximum is over CAPS, not raw portal y, so a mode
				// with a shorter band cannot be dragged up by a taller one's
				// position alone.
				const double py  = std::get<1>(bp[pi]);
				const int    pid = std::get<2>(bp[pi]);
				const double bh  = bandHeightForPortal(pid);
				// A release portal has no band: its cap is the whole map, which
				// lets MOST RECENT drop the previous section's cap and makes
				// PREFIX MAX uncapped from there on - each mode's own semantics,
				// applied to 'this section is not wall-limited'.
				double c = m.yHi;
				if (!isRoofReleasePortal(pid)) {
					c = std::round((py + ceilingOffsetForPortal(pid)) / g) * g;
					c = std::max(c, g_config.geomGroundY + bh);  // rest on the ground
				}
				// Mode 1 keeps the running maximum; mode 2 takes the most
				// recent portal's cap, so a lower section can lower the roof.
				run = (g_config.geomPortalRoof >= 2) ? c : std::max(run, c);
				pi++;
			}
			if (run > -1e17)
				roofAt[static_cast<size_t>(si)] = std::min(m.yHi, run);
		}

		// LEAD-IN. The scan above raises the roof at the portal's
		// getPositionX(), but the game re-anchors the band EARLIER - the player
		// touches the portal before its centre. MEASURED against 0x764:
		//
		//   ToE  x 16634 real 1230, roof 420  - portal centre 16665, lag 31
		//   ToE  x  6914 real  480, roof 390  - portal centre  6945, lag 31
		//   xStep x 4590 real  390, roof 330  - portal centre  4605, lag 15
		//
		// 810 units of real space deleted in the first of those, right where
		// ToE's back half begins. Two causes stack: the portal's own half-width,
		// and `<= m.xs[si]` testing the slice's LEFT edge, which defers a portal
		// sitting mid-slice to the following slice.
		//
		// The fix is a backward relaxation, not an earlier trigger. Simply moving
		// the trigger left is only safe when the next cap is HIGHER; where it is
		// lower it would crop the tail of the section being left. Taking the MAX
		// of the two over the lead-in strip is safe in both directions - it can
		// only leave a strip slightly too tall, which fails to prune, and never
		// deletes space the player can occupy.
		if (g_config.geomRoofLeadX > 0.0) {
			for (auto const& t : bp) {
				const double px = std::get<0>(t);
				const double py = std::get<1>(t);
				const int    pid = std::get<2>(t);
				const double bh = bandHeightForPortal(pid);
				double c = m.yHi;
				if (!isRoofReleasePortal(pid)) {
					c = std::round((py + ceilingOffsetForPortal(pid)) / g) * g;
					c = std::min(m.yHi, std::max(c, g_config.geomGroundY + bh));
				}
				const double x0 = px - g_config.geomRoofLeadX;
				for (int si = 0; si < n; si++) {
					if (m.xs[si + 1] <= x0) continue;
					if (m.xs[si] >= px)     break;
					roofAt[static_cast<size_t>(si)] =
						std::max(roofAt[static_cast<size_t>(si)], c);
				}
			}
		}

		for (int si = 0; si < n; si++) {
			lowest  = std::min(lowest,  roofAt[static_cast<size_t>(si)]);
			highest = std::max(highest, roofAt[static_cast<size_t>(si)]);
		}
		log::info("Solver: map roof {:.0f} -> per-x {:.0f}..{:.0f} over {} "
		          "band-setting portals, lead-in {:.0f}. Space above the roof at "
		          "each x is unreachable there.",
		          m.yHi, lowest, highest, bandPortals.size(),
		          g_config.geomRoofLeadX);
	}

	// Each object contributes its y span to every segment it covers.
	for (auto const& ob : objs) {
		// Direct lookup, NOT segIndex: that early-returns -1 while m.valid is
		// still false, which is the whole of the build. MEASURED CONSEQUENCE:
		// every object fell back to segment 0 and was smeared from the level
		// start to its own right edge, the blocked spans merged into one mass,
		// and the map reported 5927 free intervals with ZERO dead - inert, so
		// Clutterfunk fell back to its pre-geometry 34s and ToE walled at 78.90%.
		int a = static_cast<int>(
			std::upper_bound(m.xs.begin(), m.xs.end(), ob.x0) - m.xs.begin()) - 1;
		if (a < 0) a = 0;
		for (int si = a; si < n && m.xs[si] < ob.x1 - eps; si++)
			m.blocked[si].push_back({ob.lo, ob.hi, ob.type});
	}

	for (int si = 0; si < n; si++) {
		auto& spans = m.blocked[si];
		std::sort(spans.begin(), spans.end(),
		          [](GeoMap::Span const& a, GeoMap::Span const& b) { return a.lo < b.lo; });
		std::vector<GeoMap::Span> merged;
		for (auto const& sp : spans) {
			if (!merged.empty() && sp.lo <= merged.back().hi) {
				merged.back().hi = std::max(merged.back().hi, sp.hi);
				if (sp.type == 2) merged.back().type = 2;
			} else merged.push_back(sp);
		}
		m.blocked[si] = merged;

		// Complement, keeping only gaps the player actually fits through.
		std::vector<GeoMap::Free> fr;
		const double roofSi = roofAt[static_cast<size_t>(si)];
		m.segRoof[static_cast<size_t>(si)] = static_cast<float>(roofSi);
		double cur = m.yLo;
		for (auto const& sp : merged) {
			const double top = std::min(static_cast<double>(sp.lo), roofSi);
			if (top - cur >= m.playerH)
				fr.push_back({static_cast<float>(cur), static_cast<float>(top),
				              static_cast<float>(cur), static_cast<float>(top), false});
			cur = std::max(cur, static_cast<double>(sp.hi));
		}
		if (roofSi - cur >= m.playerH)
			fr.push_back({static_cast<float>(cur), static_cast<float>(roofSi),
			              static_cast<float>(cur), static_cast<float>(roofSi), false});

		// The same complement with NO roof, purely to price the roof. Two numbers
		// matter and they answer opposite questions: intervals removed says
		// whether the roof is cutting out space, and a slice emptied outright says
		// the map now claims the level is impossible there - which is never a
		// legitimate answer, since the player has to pass through every slice.
		if (roofSi < m.yHi - 0.5) {
			int    ivlsNoRoof  = 0;
			double spaceNoRoof = 0.0, spaceRoof = 0.0;
			double c2 = m.yLo;
			for (auto const& sp : merged) {
				const double top = static_cast<double>(sp.lo);
				if (top - c2 >= m.playerH) { ivlsNoRoof++; spaceNoRoof += top - c2; }
				c2 = std::max(c2, static_cast<double>(sp.hi));
			}
			if (m.yHi - c2 >= m.playerH) { ivlsNoRoof++; spaceNoRoof += m.yHi - c2; }
			for (auto const& f : fr) spaceRoof += f.hi - f.lo;

			m.roofRemovedIvls += std::max(0, ivlsNoRoof - static_cast<int>(fr.size()));
			m.roofSpaceRemoved += std::max(0.0, spaceNoRoof - spaceRoof);
			if (fr.empty() && ivlsNoRoof > 0) {
				if (m.roofKilledSlices == 0) m.roofFirstKillX = m.xs[si];
				m.roofKilledSlices++;
			}
		}
		m.freeSpans[si] = fr;
	}

	// Speed per segment: the level's start speed until the first portal, then
	// whatever the last portal to the left of the segment set. Portals are
	// sorted because m_objects is in no particular x order.
	{
		double startSpeed = kSpeedNormal;
		if (g_config.geomSpeedScan && gl->m_levelSettings)
			startSpeed = speedForStartSetting(
				static_cast<int>(gl->m_levelSettings->m_startSpeed));

		std::sort(speedPortals.begin(), speedPortals.end());
		m.segSpeed.assign(n, static_cast<float>(startSpeed));
		size_t pi = 0;
		double cur = startSpeed;
		for (int si = 0; si < n; si++) {
			while (pi < speedPortals.size() && speedPortals[pi].first <= m.xs[si]) {
				cur = speedPortals[pi].second;
				pi++;
			}
			m.segSpeed[si] = static_cast<float>(cur);
		}
	}

	// Forward sweep, right to left. x never decreases in GD: a mirror portal
	// flips the CAMERA and leaves world coordinates alone (13.11), and a
	// teleport portal cannot send the player backwards.
	// geomVerticalReach is how far the player can climb over geomSliceX units of
	// x - but that is only true at ONE speed. What is actually constant is the
	// climb per STEP: vy is bounded by the mode, and x speed does not enter it.
	// At 2x the player crosses the same 30 units in 0.80 of the steps and at 3x
	// in 0.67, so a distance-based reach over-credits it by exactly that factor
	// and the map's live windows come out too wide.
	//
	// Calibrate the per-step rate off the 1x meaning of the existing constant,
	// so this is arithmetically inert on every level that runs at one speed and
	// changes behaviour only where the speed does.
	// MEASURED, not fitted. This is the one number the window depends on, and
	// getting it from geomVerticalReach = 60 (which works out at 2.596) is what
	// retired WINDOWED in 9803757 - the mechanism was sound, its input was a
	// guess. Probe 16 measured ship in a purpose-built level: 1.800 up / 1.440
	// down normal, 2.118 / 1.694 mini, identical at 0.5x, 1x, 2x, 3x and 4x, and
	// cross-checked to 0.1% by the game's own m_yVelocity.
	//
	// One value for the whole map is the FIRST CUT, not the end state. The map
	// knows where the portals are, so each section could use its own mode's rate;
	// a cube climbs far faster than a ship and is over-constrained by this. The
	// point of a single value is to find out whether a real corridor helps at all
	// before building the per-section version.
	//
	// Too SMALL narrows the window past the flyable set and deletes routes; too
	// large just fails to narrow. Probe 17 measures which, by replaying a known-
	// good macro and counting how much of it falls outside the window.
	const double climbPerStep = g_config.geomClimbPerStep > 0.0
	                          ? g_config.geomClimbPerStep
	                          : static_cast<double>(g_config.geomVerticalReach) /
	                            std::max(1.0, static_cast<double>(g_config.geomSliceX)) *
	                            kSpeedNormal;

	// AllLive: no flood at all. Every gap live, window is the whole gap, so
	// clearance still has a centre to aim at and liveness contributes nothing.
	if (g_config.geomReachMode == 2) {
		for (auto& fr : m.freeSpans)
			for (auto& f : fr) { f.live = true; f.wlo = f.lo; f.whi = f.hi; }
	} else {

	for (auto& f : m.freeSpans[n - 1]) { f.live = true; f.wlo = f.lo; f.whi = f.hi; }

	for (int si = n - 2; si >= 0; si--) {
		// Reach scales with how many STEPS this segment takes to cross, which is
		// its width divided by the speed in force there.
		const double segDx = kSpeedNormal;
		// Overlap mode: reach is unbounded, so a gap's window is its whole gap
		// and the sweep reduces to pure overlap connectivity.
		const double reach = g_config.geomReachMode == 1
			? 1e18
			: climbPerStep * (m.xs[si + 1] - m.xs[si]) / segDx;
		auto& here = m.freeSpans[si];
		auto const& next = m.freeSpans[si + 1];
		for (auto& f : here) {
			double wlo = 1e18, whi = -1e18;
			for (auto const& g : next) {
				if (!g.live) continue;
				// Clipped to g's OWN gap: the player may move vertically while
				// crossing, but only inside the gap they are in - never through
				// the floor between two gaps. Unclipped, ToE's top corridor
				// window leaked into the middle one and un-flagged both.
				const double glo = std::max(static_cast<double>(g.lo),
				                            static_cast<double>(g.wlo) - reach);
				const double ghi = std::min(static_cast<double>(g.hi),
				                            static_cast<double>(g.whi) + reach);
				const double lo = std::max(static_cast<double>(f.lo), glo);
				const double hi = std::min(static_cast<double>(f.hi), ghi);
				if (hi - lo < m.playerH) continue;
				wlo = std::min(wlo, lo);
				whi = std::max(whi, hi);
			}
			if (whi - wlo >= m.playerH) {
				f.live = true;
				f.wlo  = static_cast<float>(wlo);
				f.whi  = static_cast<float>(whi);
			}
		}
	}

	}   // geomReachMode

	for (auto const& col : m.freeSpans)
		for (auto const& f : col) { m.freeCount++; if (!f.live) m.deadCount++; }

	// What the roof cost, over every slice in the level rather than the ones the
	// player happened to reach. A slice emptied outright is never legitimate -
	// the player must pass through every x, so a slice with no free interval says
	// the map has declared the level impossible there.
	if (g_config.geomPortalRoof > 0) {
		if (m.roofKilledSlices > 0)
			log::error("ROOF EMPTIED {} slices, first at x {:.0f}. The map now says "
			           "the level cannot be passed there at all, which cannot be "
			           "right - the roof is cutting out space that is needed.",
			           m.roofKilledSlices, m.roofFirstKillX);
		log::info("Solver: roof audit - removed {} intervals and {:.0f} units of "
		          "free height across {} slices, {} emptied. Removed height per "
		          "slice averages {:.0f}.",
		          m.roofRemovedIvls, m.roofSpaceRemoved, n, m.roofKilledSlices,
		          n > 0 ? m.roofSpaceRemoved / n : 0.0);
	}

	m.valid = true;
	return true;
}

// Probe 10: what ceiling did the player actually have, and how far below the
// map's roof is it? A large gap is the claim in ceilingProbe, measured.
void ceilingProbeDump(const char* why) {
	auto& cp = CeilingProbe::get();
	if (!g_config.ceilingProbe || cp.bands.empty()) return;

	const std::string path =
		probe::outputPath(ProbeState::get().levelKey + "_ceiling.csv");
	std::FILE* f = std::fopen(path.c_str(), "w");
	if (!f) { log::error("Probe 10: cannot write {}", path); return; }

	auto const& m = GeoMap::get();
	const double roof = m.valid ? m.yHi : 0.0;
	std::fprintf(f, "# measured ceiling (%s). map roof (yHi) = %.1f, one global "
	                "value for every x\n", why, roof);
	std::fprintf(f, "xband,x,maxYReached,contactY,reached,contacts,mapRoof,gap\n");

	double worstGap = 0.0;
	int    worstX   = 0;
	uint64_t contactBands = 0;
	for (auto const& kv : cp.bands) {
		auto const& e = kv.second;
		const double x = kv.first * g_config.ceilingBandX;
		const double ceil = e.contacts ? e.contactY : e.maxY;
		const double gap  = roof - ceil;
		if (e.contacts) {
			contactBands++;
			if (gap > worstGap) { worstGap = gap; worstX = static_cast<int>(x); }
		}
		std::fprintf(f, "%d,%.0f,%.1f,%.1f,%u,%u,%.1f,%.1f\n",
		             kv.first, x, e.maxY,
		             e.contacts ? e.contactY : 0.0,
		             e.reached, e.contacts, roof, gap);
	}
	std::fclose(f);

	{
		auto& sv = Solver::get();
		if (sv.ceilOffN) {
			log::info("Probe 10: ceiling MINUS m_portalY over {} contacts: "
			          "{:.2f} .. {:.2f}  (spread {:.2f}); geomCeilingOffset is {:.2f}",
			          sv.ceilOffN, sv.ceilOffMin, sv.ceilOffMax,
			          sv.ceilOffMax - sv.ceilOffMin, g_config.geomCeilingOffset);
			if (sv.ceilOffMax - sv.ceilOffMin < 1.0)
				log::info("  portal anchor CONSTANT - set geomCeilingOffset to {:.2f}.",
				          0.5 * (sv.ceilOffMin + sv.ceilOffMax));
			else
				log::info("  portal anchor NOT constant - m_portalY is not the whole "
				          "story; the spread is how far off one number would be.");

			if (sv.camOffN) {
				log::info("Probe 10: ceiling MINUS camera rect top over {} contacts: "
				          "{:.2f} .. {:.2f}  (spread {:.2f}), last rect top {:.1f}",
				          sv.camOffN, sv.camOffMin, sv.camOffMax,
				          sv.camOffMax - sv.camOffMin, sv.camTopSeen);
				if (sv.camOffMax - sv.camOffMin < 1.0) {
					const double mid = 0.5 * (sv.camOffMin + sv.camOffMax);
					if (std::abs(mid) < 2.0)
						log::info("  CAMERA RECT IS THE CEILING - offset {:.2f}, i.e. "
						          "nothing left to derive. Clamp straight to it.", mid);
					else
						log::info("  camera rect constant at offset {:.2f} - still one "
						          "number, but a read one rather than a counted one.",
						          mid);
				} else {
					log::info("  camera rect NOT constant either.");
				}
			} else {
				log::info("Probe 10: m_cameraObb2 was null at every contact - the "
				          "camera rect is not readable this way.");
			}

			if (sv.candN) {
				static const char* kCand[Solver::kCeilCand] = {
					"cameraEdge(0)", "cameraEdge(1)", "cameraEdge(2)",
					"cameraEdge(3)", "cameraEdge(4)", "cameraEdge(5)",
					"collidedTopMinY", "collidedBottomMaxY", "maxGameplayY",
					"yStart", "fallStartY", "isOutOfBounds",
					"groundLayer.y", "groundLayer2.y"};
				log::info("Probe 11: contactY MINUS each candidate, over {} contacts. "
				          "The ceiling is 255 and 375 here, so the winner is whichever "
				          "difference is CONSTANT.", sv.candN);
				bool anyWon = false;
				for (int i = 0; i < Solver::kCeilCand; i++) {
					const double spread = sv.candMax[i] - sv.candMin[i];
					const bool   won    = spread < 1.0;
					if (won) anyWon = true;
					log::info("  {:<20} {:+9.2f} .. {:+9.2f}  spread {:8.2f}  last {:9.2f}{}",
					          kCand[i], sv.candMin[i], sv.candMax[i], spread,
					          sv.candLast[i], won ? "   <-- CONSTANT" : "");
				}
				if (!anyWon)
					log::info("  None constant. Every candidate GD exposes is now "
					          "eliminated, and the ceiling is not directly readable.");

				log::info("Probe 11: where the clean contacts sit (y -> count). One "
				          "cluster means one ceiling; a spread of block-spaced values "
				          "means the detector is still letting collisions through.");
				for (auto const& kv : sv.ceilHist)
					log::info("    y {:>6}   {:>6} contacts", kv.first, kv.second);
			}
		} else {
			log::info("Probe 10: no ceiling contact seen, so the offset is "
			          "unconfirmed and the clamp is running on its default.");
		}
	}
	log::info("Probe 10: measured ceiling -> {}", path);
	log::info("  {} x bands, {} with a confirmed ceiling contact, {} samples",
	          cp.bands.size(), contactBands, cp.samples);
	if (contactBands) {
		log::info("  map roof {:.0f}; the largest confirmed gap is {:.0f} units at "
		          "x {} - that much phantom open space above the player.",
		          roof, worstGap, worstX);
		log::info("  A large gap is the bug: the map is offering routes through "
		          "space the player is physically barred from.");
		log::info("  ceiling y ranged {:.1f} .. {:.1f} over {} contacts",
		          cp.ceilMin, cp.ceilMax, cp.contacts);
		log::info("  ceiling MINUS camera y: {:.2f} .. {:.2f}  (spread {:.2f})",
		          cp.offMin, cp.offMax, cp.offMax - cp.offMin);
		log::info("  m_cameraHeight: {:.2f} .. {:.2f}", cp.camHMin, cp.camHMax);
		if (cp.offMax - cp.offMin < 1.0)
			log::info("  CONSTANT offset - the ceiling is camera-anchored and one "
			          "number fixes the map: clamp each x to cameraY + {:.2f}.",
			          0.5 * (cp.offMin + cp.offMax));
		else
			log::info("  NOT constant, so the ceiling is not a fixed camera offset. "
			          "The spread above is how far off a single number would be.");
	} else {
		log::info("  No ceiling contact seen yet. Fly a ship section into the roof "
		          "and dump again, or the measurement has nothing to say.");
	}
}

// Is (x, y) inside a BLOCKED span - i.e. is there a real object here?
//
// The discriminator for ceiling learning: a stop against a block is ordinary
// physics the map already models, a stop against nothing is the invisible
// ceiling that it does not.
bool geometryBlockedAt(double x, double y) {
	auto const& m = GeoMap::get();
	if (!m.valid) return false;
	const int si = m.segIndex(x);
	if (si < 0) return false;
	for (auto const& sp : m.blocked[si])
		if (y >= sp.lo && y <= sp.hi) return true;
	return false;
}

bool geometryWindowAt(double x, double y, double* lo, double* hi);
void reachOffsets(double vy, bool mini, int n, double* dLo, double* dHi);
bool velocityUnreachable(double x, double y, double vy, double dxPerStep,
                         bool mini, double* bandLo, double* bandHi);

// Which way to steer at `(x, y)`: +1 prefer the climbing branch, -1 the falling
// one, 0 leave the existing ordering alone.
//
// 0 is the common answer: if the player is already inside a live interval at the
// lookahead, nothing needs steering.
// How close to the target counts as 'close enough to say nothing'.
//
// Mode 0 (geomClearanceBand): a fraction of the corridor. The code's own
// comments condemn it twice - "A narrower target getting LESS tolerance is
// backwards" and "Three levels wanted three different values, which is
// overfitting rather than tuning". It gives the most slack where precision
// matters least, and its value was chosen because it happened to solve three
// levels on a build where the map roof and the ceiling rule were both wrong.
//
// Mode 1 (MEASURED): the distance the player can actually move in one decision
// interval, from the measured climb rate. Below that the controller cannot
// resolve anything - the player will move that far before the next decision
// regardless - so it is the honest floor for 'close enough', and it does not
// care how wide the corridor happens to be.
//
// For a normal ship that is 1.800 x 4 = 7.2 units against 30 in a 120-unit
// corridor and 71 in a 285-unit one, so the map should speak far more often.
// That is the point: it currently has an opinion on 11% of air decisions and
// every steer ever recorded was a drift correction rather than a warning.
double steerDeadband(double intervalHeight) {
	if (g_config.geomBandMode == 0)
		return std::max(0.0, g_config.geomClearanceBand) * intervalHeight;
	auto* p = Solver::get().steerPlayer;
	const double up = Solver::get().observedClimb(p, true);
	const double dn = Solver::get().observedClimb(p, false);
	const double rate = std::max(up, dn);
	// No measurement for this mode yet: fall back rather than inventing a band.
	if (rate <= 0.0)
		return std::max(0.0, g_config.geomClearanceBand) * intervalHeight;
	return rate * std::max(1, g_config.airBranchInterval);
}
int geometrySteer(double x, double y, double vy, double lead, double lookaheadUnits) {
	auto const& m = GeoMap::get();
	if (!m.valid) return 0;

	// Everything below asks about the player's altitude at the lookahead. Answer
	// with the projected altitude rather than the current one - see
	// geomLeadHold / geomLeadTap. World coordinates throughout: the caller flips the sign
	// for inverted gravity when turning this into hold or tap.
	const double yNow = y;          // before the lead projection: where the
	                                // player IS, which is what decides the
	                                // corridor it is currently in
	if (!g_config.geomReachClamp)
		y += g_config.geomLeadInSteps ? lead * vy * g_config.geomVyToDy
	                              : lead * vy;

	// Clamp a target to the reachable band. Identity when the flag is off.
	auto clampReach = [&](double t) -> double {
		if (!g_config.geomReachClamp) return t;
		auto& sv = Solver::get();
		if (sv.dxPerStep <= 0.0) return t;
		const int n = std::max(1, static_cast<int>(lookaheadUnits / sv.dxPerStep));
		const bool mini = sv.steerPlayer && sv.steerPlayer->m_vehicleSize < 0.9f;
		double dLo = 0.0, dHi = 0.0;
		reachOffsets(vy, mini, n, &dLo, &dHi);
		return std::min(yNow + dHi, std::max(yNow + dLo, t));
	};

	const int si = m.segIndex(x + lookaheadUnits);
	if (si < 0) return 0;

	// Steer ONLY when this altitude lands in dead OPEN space at the lookahead.
	//
	// MEASURED CONSEQUENCE of getting this wrong: the first version steered
	// whenever the player was not inside a LIVE interval, which also fires when
	// the altitude lands inside a BLOCKED span - a floor or platform ahead at the
	// current height. That is constant in normal play, so it fired on 34%% of air
	// decisions (32234/94639), overrode continue-first ordering across the whole
	// level, and Theory of Everything fell 78.90%% -> 57.54%%, flying low into a
	// wall because the nearest live interval near the ground is the ground.
	//
	// Landing in a wall says nothing about which channel continues. Only dead
	// open space does.
	// Forward scan: replace the single sampled slice with the corridor the
	// player is in, walked forward and intersected. See geomForwardScan.
	double scanLo = 0.0, scanHi = 0.0;
	double rawLo  = 0.0, rawHi  = 0.0;
	bool   scanned = false;
	if (g_config.geomForwardScan) {
		const int s0 = m.segIndex(x);
		if (s0 >= 0 && si >= s0) {
			// Start from the live interval the player is actually in.
			for (auto const& f : m.freeSpans[s0]) {
				if (!f.live || y < f.lo || y > f.hi) continue;
				scanLo = f.lo; scanHi = f.hi; scanned = true;
				rawLo  = f.lo; rawHi  = f.hi;   // kept for the deadband
				break;
			}
			for (int t = s0 + 1; scanned && t <= si; t++) {
				double bestLo = 0.0, bestHi = 0.0, bestW = -1.0;
				for (auto const& f : m.freeSpans[t]) {
					if (!f.live) continue;
					const double lo = std::max(scanLo, static_cast<double>(f.lo));
					const double hi = std::min(scanHi, static_cast<double>(f.hi));
					if (hi - lo > bestW) { bestW = hi - lo; bestLo = lo; bestHi = hi; }
				}
				// Pinched out: stop here and steer at the last gap that fits.
				if (bestW < m.playerH) break;
				scanLo = bestLo; scanHi = bestHi;
			}
		}
		if (scanned) {
			const double centre = clampReach( 0.5 * (scanLo + scanHi));
			// Band from the RAW interval the player is in, not from the scan's
			// intersection. MEASURED: scaling it to the intersection fired the
			// steer on 92.6%% of decisions against 36.7%% without the scan, and
			// Clutterfunk stalled at 33.72%% instead of solving in 38s.
			//
			// A narrower target getting LESS tolerance is backwards, and it is
			// the same slack-relative failure recorded below - a 285-unit
			// corridor gives 71 units of deadband, a 90-unit threaded gap gives
			// 22. The target should tighten; the tolerance should not.
			const double band = steerDeadband(rawHi - rawLo);
			{ auto& si = Solver::get().steerInfo;
			  si.yNow = yNow; si.yProj = y; si.target = centre; si.band = band;
			  si.lo = rawLo; si.hi = rawHi; si.lookX = x + lookaheadUnits;
			  si.path = 1; si.valid = true; }
			if (std::abs(y - centre) <= band) return 0;
			Solver::get().geoSteerWin++;
			return y < centre ? 1 : -1;
		}
		// Not in live space at all - fall through to the dead-space logic below.
	}

	// Route steering: follow the player's own corridor forward and aim at what
	// it leads to. See geomRouteSteer.
	if (g_config.geomRouteSteer) {
		const int s0 = m.segIndex(x);
		if (s0 >= 0 && si >= s0) {
			double clo = 0.0, chi = 0.0;
			bool   have = false;
			for (auto const& f : m.freeSpans[s0]) {
				if (!f.live || yNow < f.lo || yNow > f.hi) continue;
				clo = f.lo; chi = f.hi; have = true;
				break;
			}
			if (have) {
				for (int t = s0 + 1; t <= si; t++) {
					double blo = 0.0, bhi = 0.0, bw = -1.0;
					for (auto const& f : m.freeSpans[t]) {
						if (!f.live) continue;
						const double lo = std::max(clo, static_cast<double>(f.lo));
						const double hi = std::min(chi, static_cast<double>(f.hi));
						// Ranked by overlap with the corridor so far, but the
						// TARGET is the whole interval - the corridor may widen
						// or shift, and clipping to the overlap would invent a
						// tunnel that is not there.
						if (hi - lo > bw) { bw = hi - lo; blo = f.lo; bhi = f.hi; }
					}
					// Chain broken: nothing ahead connects. Aim at the last
					// interval that did rather than inventing a target.
					if (bw < m.playerH) break;
					clo = blo; chi = bhi;
				}

				const double centre = clampReach( 0.5 * (clo + chi));
				const double band = steerDeadband(chi - clo);
				{ auto& si = Solver::get().steerInfo;
				  si.yNow = yNow; si.yProj = y; si.target = centre; si.band = band;
				  si.lo = clo; si.hi = chi; si.lookX = x + lookaheadUnits;
				  si.path = 2; si.valid = true; }
				if (std::abs(y - centre) <= band) return 0;

				// Urgency gate. Route alone fired on 78.7%% of decisions and
				// stalled Theory of Everything at 12.74%%: a corridor 300 units
				// ahead can sit 200 units above the player, so aiming at its
				// centre demands a position that cannot be reached this
				// decision - and the demand simply repeats, overriding the
				// search's own ordering everywhere.
				//
				// Three bands, and the third is the one that matters:
				//   plenty of time  -> quiet
				//   act now or lose -> steer
				//   unreachable     -> QUIET. The branch is doomed; jamming the
				//                      ordering cannot save it, and the escape
				//                      ladder exists to back out of exactly
				//                      this. Speaking here is what turned every
				//                      previous controller into a permanent
				//                      override.
				if (g_config.geomUrgencySteer) {
					auto* pp = Solver::get().steerPlayer;
					const bool up = centre > yNow;
					const double need = up ? std::max(0.0, clo - yNow)
					                       : std::max(0.0, yNow - chi);
					const double dxs = std::max(0.01, Solver::get().dxPerStep);
					const double steps = lookaheadUnits / dxs;
					const double vmax = Solver::get().observedClimb(pp, up);
					if (need > 0.0 && vmax > 0.0 && steps >= 1.0) {
						const double required = need / steps;
						if (required < g_config.geomUrgencyFrac * vmax) return 0;
						if (required > vmax) return 0;
					}
				}

				Solver::get().geoSteerWin++;
				return y < centre ? 1 : -1;
			}
			// Not in live space: fall through to the dead-space logic below.
		}
	}

	// The standalone urgency steer is gone. Its gate asked "am I inside ANY
	// live interval at the lookahead?", which is almost always true, so it
	// returned no-opinion every time: geoSteer 0/20194 on Clutterfunk. Urgency
	// is now a GATE on the route target above, which is the question it should
	// have been asking - can I still reach THE INTERVAL MY CORRIDOR LEADS TO.
	// Crop to the band the player can actually occupy. See ceilingClamp.
	//
	// An interval sitting entirely above the learned roof is not a route, it is
	// scenery. One trimmed to below player height is not one either.
	// The clamp flag gates USING the band, not knowing it.
	const bool haveBand = g_config.ceilingClamp &&
	                      Solver::get().learnedCeiling < 1e29;
	const double ceilY  = haveBand ? Solver::get().learnedCeiling : 1e30;
	// The band is bounded below as well - the portal snaps a floor at the same
	// moment it snaps a roof, and space below it is just as unreachable.
	// The floor is the ceiling less the band, not a second measured constant.
	const double floorY = haveBand
	                    ? Solver::get().learnedCeiling - Solver::get().entryBand()
	                    : -1e30;
	// The band constrains the TARGET, not the map.
	//
	// MEASURED, Time Machine: cropping intervals out of the map cost 21s against
	// 12s uncropped - and the mechanism was NOT a wrong band. `out 0/76889` says
	// the player was never once outside it. Removing intervals leaves the
	// projected altitude outside any usable interval more often, which TRIGGERS a
	// steer: the rate went 44% -> 56%. More steering means more overriding of the
	// search's own ordering, which has cost time every time it has happened.
	//
	// So the band refuses an unreachable TARGET instead. Nothing is removed from
	// the map, so no route can be lost, and this can only ever SUPPRESS a steer,
	// never create one - the opposite of the failure above. It still kills the
	// phantom corridor that motivated all of this: Base After Base's target sat at
	// ~440 with a band topping out at 390.
	//
	// Portal spans were the alternative and are worse. Deciding which x range a
	// band covers means reading portals from level data and assuming which of them
	// get taken, which is exactly what fake portals exist to break.
	// Drop an interval only when it lies ENTIRELY outside the band. Never clip one
	// that overlaps.
	//
	// Both halves of this were measured separately and each failed on its own:
	//
	//   CLIPPING to the band shrinks real intervals, so the projected altitude
	//   falls outside a usable interval more often and TRIGGERS a steer - rate
	//   44% -> 56%, Time Machine 12s -> 21s.
	//
	//   Not cropping at all leaves liveness believing the phantom corridor above
	//   the ship ceiling reaches the level end, so the search keeps exploring
	//   there - Base After Base 17s -> 59s, with the old 69.09% stall returning.
	//
	// Dropping only wholly-outside intervals does the useful half of each. The
	// phantom corridor is entirely above the ceiling, so it goes; every real
	// interval overlaps the band, so it survives at full extent and nothing about
	// normal steering changes.
	auto usable = [&](GeoMap::Free const& f, double* lo, double* hi) -> bool {
		*lo = static_cast<double>(f.lo);
		*hi = static_cast<double>(f.hi);
		if (*hi - *lo < m.playerH) return false;
		// A phantom-space filter lived here: drop an interval sitting wholly above the
		// widest ceiling seen, or wholly below the lowest floor. DELETED, MEASURED
		// inert - 0 rejects in 126,140 evaluations across all fifteen levels. It
		// suppressed space the player cannot occupy, and geomPortalRoof now removes
		// that at map-build time instead (roofMargin mean ~700 -> 0-62).
		//
		// It also carried ceilMaxSeen/floorMinSeen: run-wide monotonic accumulators,
		// read by the steer and never restored on a reposition, so the steer's view of
		// the map depended on search history. Same defect class as lastGeoLo/Hi and
		// prevAirMode, resolved by removal rather than by another restore.
		return true;
	};

	auto targetReachable = [&](double t) -> bool {
		if (!haveBand) return true;
		return t >= floorY - 1.0 && t <= ceilY + 1.0;
	};

	// Does the map actually discriminate here? See geomSteerGate.
	if (g_config.geomSteerGate > 0) {
		auto const& fs = m.freeSpans[si];
		bool anyDead   = false;
		int  liveCount = 0;
		for (auto const& f : fs) {
			double ulo = 0.0, uhi = 0.0;
			if (!usable(f, &ulo, &uhi)) continue;   // out of reach: not a route
			if (f.live) liveCount++;
			else        anyDead = true;
		}
		// Something dead means liveness has a route to express. More than one
		// live interval means there is a fork, even if both continue. One live
		// interval and nothing dead means every altitude here leads onward, and
		// any opinion about which one is invented.
		bool discriminates = anyDead || liveCount > 1;

		// Mode 3: speak ONLY when there is exactly one live interval.
		//
		// The inverse of mode 1, and the one that matches what the failures
		// actually look like. Watched at normal speed, both Base After Base and
		// Clutterfunk fail the same way: the ship rides high, the corridor
		// staircases down, and it flies into the wall at the top. With a high
		// interval and a low one both live, the steer locks onto whichever the
		// player is in and re-centres it there on every decision - it is actively
		// holding the ship up.
		//
		// One live interval means there is no choice and the map genuinely knows
		// the answer. Two or more means it has no basis for preferring either, so
		// silence lets release-first ordering try descending on its own.
		if (g_config.geomSteerGate == 3)
			discriminates = liveCount == 1;

		if (!discriminates && g_config.geomSteerGate >= 2) {
			const int s0 = m.segIndex(x);
			if (s0 >= 0) {
				double wNow = 0.0, wAhead = 0.0;
				for (auto const& f : m.freeSpans[s0])
					if (yNow >= f.lo && yNow <= f.hi) { wNow = f.hi - f.lo; break; }
				for (auto const& f : fs)
					if (y >= f.lo && y <= f.hi) { wAhead = f.hi - f.lo; break; }
				// A corridor closing down is a choice arriving, whether or not
				// anything beside it is dead yet.
				if (wNow > 0.0 && wAhead > 0.0
				    && wAhead < g_config.geomNarrowFrac * wNow)
					discriminates = true;
			}
		}

		if (!discriminates) {
			Solver::get().geoSteerMute++;
			return 0;
		}
	}

	auto const& fr = m.freeSpans[si];
	bool inDead = false;
	for (auto const& f : fr) {
		double ulo = 0.0, uhi = 0.0;
		if (!usable(f, &ulo, &uhi)) { Solver::get().ceilingCrops++; continue; }
		if (y < ulo || y > uhi) continue;
		if (f.live) {
			// Aim for the middle of the window rather than merely being inside
			// it. Being outside is subsumed: outside means further than half the
			// height from centre, which any band below 0.5 already catches.
			const double lo = std::max(static_cast<double>(f.wlo), ulo);
			const double hi = std::min(static_cast<double>(f.whi), uhi);
			const double centre = clampReach( 0.5 * (lo + hi));
			// Fraction of the window HEIGHT. Slack-relative was tried and is
			// worse overall: it made narrow windows demand near-perfect centring
			// (a 20-unit gap has 2.5 units of slack, so under a unit of
			// deadband), which took Time Machine to 28s - its fastest ever - and
			// collapsed Theory of Everything to 29.65% with steering on 61% of
			// decisions. Three levels wanted three different values, which is
			// overfitting rather than tuning, so this is back to the only setting
			// that solved all of them.
			const double band = steerDeadband(hi - lo);
			{ auto& si = Solver::get().steerInfo;
			  si.yNow = yNow; si.yProj = y; si.target = centre; si.band = band;
			  si.lo = lo; si.hi = hi; si.lookX = x + lookaheadUnits;
			  si.path = 3; si.valid = true; }
			if (std::abs(y - centre) <= band) return 0;
			// Unreachable target: the player cannot occupy that altitude from
			// here, so steering at it is worse than saying nothing.
			if (!targetReachable(centre)) { Solver::get().ceilingCrops++; return 0; }
			Solver::get().geoSteerWin++;
			return y < centre ? 1 : -1;
		}
		inDead = true;
		break;
	}
	if (!inDead) return 0;             // blocked, or a gap too small: no opinion

	double best = 1e18;
	int dir = 0;
	for (auto const& f : fr) {
		if (!f.live) continue;
		const double c = 0.5 * (static_cast<double>(f.lo) + static_cast<double>(f.hi));
		const double d = std::abs(c - y);
		if (d < best) { best = d; dir = c > y ? 1 : -1; }
	}
	if (dir != 0) Solver::get().geoSteerDead++;
	return dir;
}

// Bounds of the LIVE interval containing `y` at `x`, or false if the altitude
// is blocked, in dead space, or off the map. Used to notice when the corridor
// ahead changes - see geomBranchMode.
// The measured vertical envelope, shared by the 4b prune and the steer's reach
// clamp so the constants exist once.
//
// MEASURED from the motion census (event-free rows, >=3 samples, 917k
// transitions): ship +0.118 / -0.103 per step with |vy| <= 8.00, mini
// +0.137 / -0.122 with |vy| <= 9.42, and dy = vy * 0.2279. Terminal check:
// 8.00 * 0.2279 = 1.823 units/step against Probe 16's independent 1.800.
void reachOffsets(double vy, bool mini, int n, double* dLo, double* dHi) {
	const double aUp = (mini ? 0.137 : 0.118) * g_config.velocityPruneMargin;
	const double aDn = (mini ? 0.122 : 0.103) * g_config.velocityPruneMargin;
	const double vCl = (mini ? 9.42  : 8.00 ) * g_config.velocityPruneMargin;
	double vHi = vy, vLo = vy, yHi = 0.0, yLo = 0.0;
	for (int i = 0; i < n; i++) {
		vHi = std::min( vCl, vHi + aUp);
		vLo = std::max(-vCl, vLo - aDn);
		yHi += vHi * g_config.geomVyToDy;
		yLo += vLo * g_config.geomVyToDy;
	}
	*dLo = yLo; *dHi = yHi;
}

// 4b. Is every reachable altitude dead at the horizon?
//
// The envelope below is MEASURED from the motion census (event-free rows, >=3
// samples, 917k transitions over the 15-level suite):
//
//              accel up/step   accel down/step   |vy| seen
//   ship        +0.118           -0.103            8.00      (P99 / P1)
//   ship mini   +0.137           -0.122            9.42
//
// and dy = vy * 0.2279 (median over the census; 1/0.2279 = 4.389, which is the
// same scale Probe 16 cross-checked at 4.4444). Terminal check: 8.00 * 0.2279 =
// 1.823 units/step against Probe 16's independently measured 1.800.
//
// SOUNDNESS. The census is a true bound only where nothing can override velocity,
// so this refuses to answer when a pad, ring or portal lies inside the horizon -
// those are exactly the transitions the census marks as events, and GeoMap::
// segEvent records where they are. Refusing costs a missed prune; answering
// anyway would delete a reachable branch.
//
// Hold modes only. A UFO tap SETS velocity (measured: a single step moves vy by
// +13.27 from any value), so its reachable band is the whole corridor after one
// tap and a prune would essentially never fire.
bool velocityUnreachable(double x, double y, double vy, double dxPerStep,
                         bool mini, double* bandLo, double* bandHi) {
	auto const& m = GeoMap::get();
	if (!m.valid || dxPerStep <= 0.0) return false;

	const int n = std::max(1, g_config.velocityPruneSteps);
	double dLo = 0.0, dHi = 0.0;
	reachOffsets(vy, mini, n, &dLo, &dHi);
	const double yLo = y + dLo, yHi = y + dHi;
	if (bandLo) *bandLo = yLo;
	if (bandHi) *bandHi = yHi;

	const double hx = x + static_cast<double>(n) * dxPerStep;
	const int s0 = m.segIndex(x), s1 = m.segIndex(hx);
	if (s0 < 0 || s1 < 0) return false;                 // off the map: no opinion

	// Refuse where velocity can be overridden.
	for (int si = s0; si <= s1 && si < static_cast<int>(m.segEvent.size()); si++)
		if (m.segEvent[static_cast<size_t>(si)]) return false;

	for (auto const& f : m.freeSpans[static_cast<size_t>(s1)]) {
		if (!f.live) continue;
		if (static_cast<double>(f.hi) < yLo || static_cast<double>(f.lo) > yHi) continue;
		if (std::min<double>(f.hi, yHi) - std::max<double>(f.lo, yLo) >= m.playerH)
			return false;                                // something live is reachable
	}
	return true;
}

bool geometryWindowAt(double x, double y, double* lo, double* hi) {
	auto const& m = GeoMap::get();
	if (!m.valid) return false;
	const int si = m.segIndex(x);
	if (si < 0) return false;
	for (auto const& f : m.freeSpans[si]) {
		if (y < f.lo || y > f.hi) continue;
		if (!f.live) return false;
		*lo = static_cast<double>(f.lo);
		*hi = static_cast<double>(f.hi);
		return true;
	}
	return false;
}

// Probe 9. What geomVerticalReach costs, measured without running a search.
//
// Reach decides how much of a gap counts as the usable window, which is the only
// thing driving how often geometrySteer fires - and steering is not free.
// MEASURED against the pre-geometry baseline: Clutterfunk 34s -> 45s and Time
// Machine 2m00 -> 2m38, both +32%, on levels with no fake corridors where the
// map has nothing useful to say. That tax is the price of catching ToE's
// corridors, and this finds out how much of it is necessary.
//
// Rebuilding the map is instant, so this tries every value in one keypress
// rather than one rebuild-and-solve per value. The proxy for steering pressure
// is `constrained`: intervals whose window is narrower than the gap, since those
// are the only places geometrySteer can return non-zero.
void geometryReachSweep() {
	if (!GJBaseGameLayer::get()) { log::warn("Probe 9: not in a level"); return; }

	const int saved = g_config.geomVerticalReach;
	const int values[] = {0, 15, 30, 60, 120, 240, 480, 960};

	log::info("Probe 9: geomVerticalReach sweep. `constrained` is intervals whose "
	          "window is narrower than the gap - the only places steering can fire.");
	log::info("  %-8s %-8s %-7s %-12s %-8s", "reach", "free", "dead", "constrained", "meanWin%");

	for (int v : values) {
		g_config.geomVerticalReach = v;
		if (!geometryBuildMap()) { log::warn("  reach {}: map build failed", v); continue; }
		auto const& m = GeoMap::get();

		int constrained = 0;
		double ratioSum = 0.0;
		int live = 0;
		for (auto const& col : m.freeSpans)
			for (auto const& f : col) {
				if (!f.live) continue;
				live++;
				const double gap = static_cast<double>(f.hi) - static_cast<double>(f.lo);
				const double win = static_cast<double>(f.whi) - static_cast<double>(f.wlo);
				if (gap > 0.0) {
					ratioSum += win / gap;
					if (win < gap - 0.5) constrained++;
				}
			}
		log::info("  {:<8} {:<8} {:<7} {:<12} {:.1f}", v, m.freeCount, m.deadCount,
		          constrained, live > 0 ? 100.0 * ratioSum / live : 0.0);
	}

	// Leave the map as the solver expects to find it.
	g_config.geomVerticalReach = saved;
	geometryBuildMap();
	log::info("  restored reach {} and rebuilt.", saved);
}

// Probe 8. Writes the map geometryBuildMap() computes.
//
// Rendered on the cell census's 60x30 bands so the two overlay for reading, by
// SAMPLING the band centre - the map itself is intervals and keeps full
// precision; only this picture is bucketed.
void deadEndDump() {
	auto* pl = PlayLayer::get();
	if (!pl) { log::error("Probe 8: no level"); return; }
	if (!geometryBuildMap()) { log::warn("Probe 8: no solid or hazardous objects"); return; }

	auto& m = GeoMap::get();
	const double yb = std::max(1.0, g_config.goCellY);
	const int ylo = static_cast<int>(std::floor(m.yLo / yb));
	const int yhi = static_cast<int>(std::floor(m.yHi / yb));

	const std::string path = levelFilePath("deadends.txt");
	std::FILE* f = std::fopen(path.c_str(), "wb");
	if (!f) { log::error("Probe 9: cannot write {}", path); return; }

	std::fprintf(f, "# gd-solver dead-end map (Probe 8, free intervals)\n");
	std::fprintf(f, "# %d exact x segments (object edges, variable width); "
	                "player height %.0f\n", m.segments(), m.playerH);
	std::fprintf(f, "# %d free intervals, %d of them DEAD\n", m.freeCount, m.deadCount);
	std::fprintf(f, "# S solid  H hazard  . inside the live window  o open and live but "
	                "OUTSIDE the window  X dead\n");
	std::fprintf(f, "# Blocked spans come from real object hitboxes, so a 7.2-tall spike\n"
	                "# blocks 7.2 units. Picture is sampled at each 30-unit band centre.\n#\n");

	const double len = pl->m_levelLength > 1.f ? pl->m_levelLength : 0.0;
	std::fprintf(f, "# %-7s %-9s %-7s ", "slice", "x", "pct");
	for (int y = yhi; y >= ylo; y--) std::fprintf(f, "%d", std::abs(y) % 10);
	std::fprintf(f, "  (y%d down to y%d)\n", yhi, ylo);

	for (int si = 0; si < m.segments(); si++) {
		const double xu = m.xs[si];
		std::fprintf(f, "%-9d %-9.1f %-7.2f ", si, xu, len > 0.0 ? (xu / len) * 100.0 : 0.0);
		auto const& blk = m.blocked[si];
		auto const& fr  = m.freeSpans[si];
		for (int y = yhi; y >= ylo; y--) {
			const double yc = (y + 0.5) * yb;   // sample the band centre
			char c = 0;
			for (auto const& sp : blk)
				if (yc >= sp.lo && yc <= sp.hi) { c = sp.type == 2 ? 'H' : 'S'; break; }
			if (!c) {
				c = ' ';
				for (auto const& fs : fr)
					if (yc >= fs.lo && yc <= fs.hi) {
						// 'o' is open and live but OUTSIDE the window that gets
						// through - the part a route must not be in.
						c = !fs.live ? 'X' : (yc >= fs.wlo && yc <= fs.whi) ? '.' : 'o';
						break;
					}
				// A gap too short for the player is neither free nor blocked.
				if (c == ' ') c = ':';
			}
			std::fputc(c, f);
		}
		std::fputc('\n', f);
	}
	std::fclose(f);

	log::info("Probe 8: interval map over {} exact x segments (x {:.0f}..{:.0f}), player "
	          "height {:.0f} - {} free intervals, {} DEAD ({:.1f}%).",
	          m.segments(), m.xs.front(), m.xs.back(), m.playerH, m.freeCount, m.deadCount,
	          m.freeCount > 0 ? 100.0 * m.deadCount / m.freeCount : 0.0);
	log::info("  ':' is a gap too short for a {:.0f}-unit player. Blocked spans are real "
	          "hitboxes, not buckets -> {}", m.playerH, path);
}

void sweepWriteResults() {
	auto& sw = Sweep::get();
	const std::string path = levelFilePath("sweep.txt");
	std::FILE* f = std::fopen(path.c_str(), "wb");

	log::info("Probe 6: corridor sweep complete - {} variants.", sw.results.size());
	log::info("  maxY is the highest y band reached at x >= {:.0f}. Corridor model:"
	          " bottom y32-33 | floor y34 | middle y35-36 | floor y37 | TOP y38-39.",
	          g_config.sweepCorridorX);
	log::info("  %-7s %-4s %-8s %-6s %-9s %s", "S", "k", "pct", "maxY", "endStep", "how");

	if (f) {
		std::fprintf(f, "# gd-solver corridor sweep (Probe 6)\n");
		std::fprintf(f, "# maxY = highest y band at x >= %.0f\n", g_config.sweepCorridorX);
		std::fprintf(f, "# corridor model: bottom 32-33 | floor 34 | middle 35-36 | "
		                "floor 37 | TOP 38-39\n");
		std::fprintf(f, "# %-7s %-4s %-8s %-6s %-9s %s\n",
		             "S", "k", "pct", "maxY", "endStep", "how");
	}

	int bestIdx = -1;
	for (size_t i = 0; i < sw.results.size(); i++) {
		auto const& r = sw.results[i];
		const char* how = r.cleared ? "CLEARED" : (r.died ? "died" : "macro end");
		log::info("  {:<7} {:<4} {:<8.2f} {:<6} {:<9} {}",
		          r.S, r.k, r.pct, r.maxY, r.endStep, how);
		if (f) std::fprintf(f, "%-9d %-4d %-8.2f %-6d %-9d %s\n",
		                    r.S, r.k, r.pct, r.maxY, r.endStep, how);
		if (bestIdx < 0 || r.pct > sw.results[bestIdx].pct) bestIdx = static_cast<int>(i);
	}
	if (f) std::fclose(f);

	if (bestIdx >= 0) {
		auto const& b = sw.results[bestIdx];
		log::info("  best: S {} k {} reached {:.2f}% with maxY {} -> {}", b.S, b.k, b.pct,
		          b.maxY,
		          b.maxY >= 38 ? "REACHED THE TOP CORRIDOR (y38+)"
		                       : "never left the bottom/middle - no variant reached y38");
	}
	log::info("  written to {}", path);
}

void solverWriteBudgetPops(const char* why) {
	auto& sv = Solver::get();
	const std::string path = levelFilePath("budgetpops.txt");
	std::FILE* f = std::fopen(path.c_str(), "wb");
	if (!f) return;
	std::fprintf(f, "# gd-solver toggle-budget gate refusals (%s)\n", why);
	std::fprintf(f, "# %llu refusals, %zu recorded%s\n",
	             static_cast<unsigned long long>(sv.budgetPopCount),
	             sv.budgetPops.size(),
	             sv.budgetPopsFull ? " (BUFFER FULL - truncated)" : "");
	std::fprintf(f, "# gate evaluated %llu times, %llu of those on a toggling branch\n",
	             static_cast<unsigned long long>(sv.budgetGateEvals),
	             static_cast<unsigned long long>(sv.budgetGateWouldToggle));
	std::fprintf(f, "# highest `spent` observed: %d (budget %d)\n",
	             sv.budgetMaxSpent, g_config.toggleBudget);
	std::fprintf(f, "# mode: 0=Ground 1=Hold 2=Tap\n");
	std::fprintf(f, "# %-8s %-5s %-8s %-8s %-6s %-7s %-9s %s\n",
	             "step", "mode", "tgBefore", "floorTg", "spent", "budget",
	             "floorIdx", "stack");
	for (auto const& b : sv.budgetPops) {
		std::fprintf(f, "%-9d %-5d %-8d %-8d %-6d %-7d %-9d %d\n",
		             b.step, b.mode, b.togglesBefore, b.floorToggles, b.spent,
		             b.budget, b.floorIdx, b.stackSize);
	}
	std::fclose(f);
	log::info("Solver: budget gate ({}) - {} refusals from {} evaluations ({} on a "
	          "toggling branch), max spent {} against budget {} -> {}",
	          why, sv.budgetPopCount, sv.budgetGateEvals, sv.budgetGateWouldToggle,
	          sv.budgetMaxSpent, g_config.toggleBudget, path);
}

void solverWriteBestMacro() {
	auto& sv = Solver::get();
	if (sv.bestMacro.empty()) return;
	std::string path = levelFilePath("best.txt");
	if (std::FILE* f = std::fopen(path.c_str(), "wb")) {
		std::fprintf(f, "# gd-solver best-progress path, %zu steps, physics_fix=%d, 240Hz\n",
		             sv.bestMacro.size(), g_config.physicsFix ? 1 : 0);
		for (size_t i = 0; i < sv.bestMacro.size(); i++) {
			std::fputc(sv.bestMacro[i] ? '1' : '0', f);
			if ((i + 1) % 80 == 0) std::fputc('\n', f);
		}
		std::fputc('\n', f);
		std::fclose(f);
		log::info("Solver: best path ({} steps, {:.2f}%) written to {} - press F9 to "
		          "watch it at normal speed", sv.bestMacro.size(), sv.bestPct, path);
	}
}

// ---------------------------------------------------------------------------
// Probe 4b: restore fidelity
// ---------------------------------------------------------------------------

const char* restoreKindName(RestoreKind k) {
	switch (k) {
		case RestoreKind::Full:       return "FULL (practice respawn)";
		case RestoreKind::PlayerOnly: return "PLAYER-ONLY (no level state)";
		case RestoreKind::DirectLoad: return "DIRECT loadFromCheckpoint (no respawn)";
	}
	return "?";
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
	} else if (loadMacroFile(levelFilePath("best.txt"), st.scripted, &fix) &&
	           !st.scripted.empty()) {
		// A STALLED level has no solution, which used to make this probe useless
		// on exactly the levels that need it most. The best path is still a real
		// trajectory the search flew, so restore fidelity can be measured along
		// it - including in sections no solved level contains. Theory of
		// Everything's UFO is the first UFO the restore has ever been tested in.
		log::info("Probe 4b: no solution.txt - using best.txt ({} steps). This is the "
		          "furthest the search reached, so anchors sample the section it is "
		          "stuck in.", st.scripted.size());
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
	// Spread anchors across the macro so we sample both cube and air sections.
	st.restoreLen = 400;
	st.sweepAnchors.clear();
	st.sweepIndex = 0;
	st.sweepResults.clear();
	// A recorded divergence beats a blind sweep. The eight spread anchors are for
	// MAPPING where restores are exact; once a verification has localised a real
	// divergence to one step, anchor exactly there instead - the whole point of
	// the layer byte-diff below is to run at a step that is known to be wrong.
	//
	// This is what the verification failure message has always promised ("Probe
	// 4b will now anchor near this step") and never actually did, because the
	// sweep branch was tested first and always won.
	if (st.verifyDivergeStep >= 240 &&
	    st.verifyDivergeStep < static_cast<int>(st.scripted.size()) - 240) {
		st.restoreAnchor = st.verifyDivergeStep;
		st.restoreLen    = 400;
		log::info("Probe 4b: anchoring EXACTLY at step {} - the step a verification "
		          "measured as the first divergence. The PlayLayer byte-diff will "
		          "name whatever layer state the restore fails to reproduce there.",
		          st.restoreAnchor);
	} else {
		const int usable = static_cast<int>(st.scripted.size()) - st.restoreLen - 240;
		if (usable > 1200) {
			for (int i = 1; i <= 8; i++) st.sweepAnchors.push_back(240 + (usable * i) / 9);
		}
	}
	if (!st.sweepAnchors.empty()) {
		st.restoreAnchor = st.sweepAnchors[0];
		log::info("Probe 4b: sweeping {} anchors across the level to map WHERE restores "
		          "are exact, and against which game mode.", st.sweepAnchors.size());
	} else if (st.lastResyncFailStep > 400) {
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
		log::info("  RESTORE IS SOUND - {} steps after restore are bit-identical.  "
		          "[{} at {:.2f}%]", st.segmentA.size(), st.anchorMode, st.anchorPct);
	} else {
		log::error("  RESTORE IS NOT SOUND - diverges {} step(s) after the restore "
		           "point.  [{} at {:.2f}%]", at, st.anchorMode, st.anchorPct);
		std::string base = st.restoreKind == RestoreKind::Full ? "probe4b_full" : "probe4b_playeronly";
		st.segmentA.writeCsv(probe::outputPath(base + "_A.csv"));
		st.segmentB.writeCsv(probe::outputPath(base + "_B.csv"));
		log::error("  segments written to {}_A.csv / _B.csv for diffing", base);
	}
	log::info("===========================================================");

	st.releaseCheckpoints();

	// Sweeping: record this anchor and move to the next.
	if (!st.sweepAnchors.empty()) {
		const bool sound = (at == SIZE_MAX && st.segmentA.size() == st.segmentB.size());
		st.sweepResults.push_back({st.restoreAnchor, st.anchorWasAir, sound, at,
		                           st.anchorPct, st.anchorMode});

		st.sweepIndex++;
		if (st.sweepIndex < st.sweepAnchors.size()) {
			st.restoreAnchor = st.sweepAnchors[st.sweepIndex];
			st.restorePhase  = 0;
			st.segmentA.clear();
			st.segmentB.clear();
			st.haveAnchorRow = false;
			st.resetPending  = true;
			log::info("Probe 4b: next anchor {} ({} of {})",
			          st.restoreAnchor, st.sweepIndex + 1, st.sweepAnchors.size());
			return;
		}

		// Done: the table the hybrid design depends on.
		int cubeTotal = 0, cubeSound = 0, airTotal = 0, airSound = 0;
		log::info("============ Probe 4b: restore fidelity by MODE ============");
		for (auto const& r : st.sweepResults) {
			log::info("  step {:>6}  {:>6.2f}%  {:<6}  {}", r.anchor, r.pct, r.mode,
			          r.sound ? "SOUND" : fmt::format("NOT SOUND (diverges at {})", r.divergeAt));
			if (r.air) { airTotal++;  if (r.sound) airSound++; }
			else       { cubeTotal++; if (r.sound) cubeSound++; }
		}
		log::info("  ----------------------------------------------------------");
		log::info("  CUBE-like: {}/{} sound", cubeSound, cubeTotal);
		log::info("  AIR modes: {}/{} sound", airSound, airTotal);
		if (cubeTotal > 0 && cubeSound == cubeTotal && airSound < airTotal) {
			log::info("  => Restores are exact in cube and not in air. The hybrid design "
			          "is justified: anchor air-mode backtracks on a cube savestate.");
		} else if (cubeTotal > 0 && cubeSound < cubeTotal) {
			log::error("  => Cube restores are NOT reliably exact. The hybrid design would "
			           "be built on a false assumption - anchor on frame 0 instead.");
		}
		log::info("===========================================================");
		st.sweepAnchors.clear();
	}

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

// Probe 17. One step of a verify replay, checked against the map as built.
//
// Classified three ways because they mean different things. OFF-MAP is the
// loudest: the player is somewhere the map has no interval for at all, which
// on a cropped map means the roof deleted space the winning run occupied.
// DEAD means the interval exists but the sweep says the end is unreachable
// from it - a false negative, since the run reached the end from there.
// OUT-OF-WINDOW is the mildest: the space is live but the steering would
// never have aimed at it.
// Probe 20. Expressiveness of a known-good path under the current branch rule.
//
// Evaluates the SAME rule geometryWantsBranch uses, against its own shadow of
// lastBranch/lastGeoLo/Hi, so it reports what the search would have been offered
// rather than what the search happened to do.
//
// A missed toggle is classified by WHY the rule declined:
//   cadence    - fewer than airBranchInterval steps since the last decision
//   unchanged  - the corridor at the lookahead is the same as at the last one
//   noread     - the map had no window there (the rule offers a decision anyway)
void expressAuditStep(PlayerObject* p, bool pressed, uint64_t k) {
	auto& sv = Solver::get();
	auto& m  = GeoMap::get();
	if (!p || p->m_isDead) return;
	const bool toggled = sv.exprHavePrev && (pressed != sv.exprPrevPressed);
	sv.exprPrevPressed = pressed;
	sv.exprHavePrev    = true;

	const ModeClass mc = classifyMode(p);
	if (mc == ModeClass::Ground) { sv.exprLastBranch = k; return; }
	sv.exprAirSteps++;

	const int elapsed = static_cast<int>(k - sv.exprLastBranch);
	int reason = 0;                       // 0 = offered
	bool offer = true;
	double glo = 0.0, ghi = 0.0;
	bool haveWin = false;

	if (elapsed < g_config.airBranchInterval) { offer = false; reason = 1; }
	else if (g_config.geomBranchMode != 0 && m.valid) {
		const double lookX = p->getPositionX() +
			static_cast<double>(g_config.geomLookaheadFixedX);
		haveWin = geometryWindowAt(lookX, p->getPositionY(), &glo, &ghi);
		if (!haveWin) { offer = true; reason = 3; }
		else {
			bool forced = false;
			if (g_config.geomBranchMode >= 3) {
				const bool mini = p->m_vehicleSize < 0.9f;
				const double rate = mini ? 2.118 : 1.800;
				const double slack = (ghi - glo) - m.playerH;
				int c = static_cast<int>(std::floor(std::max(0.0, slack) / rate));
				c = std::max(g_config.airBranchInterval,
				    std::min(g_config.airBranchIntervalMax, c));
				forced = elapsed >= c;
			} else {
				forced = elapsed >= g_config.airBranchIntervalMax;
			}
			if (g_config.geomBranchMode == 2 &&
			    (ghi - glo) < g_config.geomTightHeights * m.playerH) forced = true;
			if (!forced) {
				const bool changed = std::abs(glo - sv.exprGeoLo) > 0.01 ||
				                     std::abs(ghi - sv.exprGeoHi) > 0.01;
				if (!changed) { offer = false; reason = 2; }
			}
		}
	}

	if (offer) {
		sv.exprLastBranch = k;
		sv.exprOfferPoints++;
		if (haveWin) { sv.exprGeoLo = glo; sv.exprGeoHi = ghi; }
	}

	if (toggled) {
		sv.exprToggles++;
		if (offer) sv.exprOffered++;
		else {
			sv.exprMissed++;
			if (reason == 1) sv.exprMissCadence++;
			else if (reason == 2) sv.exprMissUnchanged++;
			else sv.exprMissNoRead++;
			auto* pl = PlayLayer::get();
			if (sv.exprMisses.size() < 20000)
				sv.exprMisses.push_back({p->getPositionX(), p->getPositionY(),
				                         static_cast<double>(p->m_yVelocity),
				                         pl ? pl->getCurrentPercent() : 0.f,
				                         k, elapsed, static_cast<int>(mc), reason,
				                         glo, ghi});
		}
	}
}

// Probe 21. Dump the decisions behind the current best path.
void bestDecisionDump() {
	auto& sv = Solver::get();
	if (sv.bestDecisions.empty()) { log::info("Probe 21: no best path yet."); return; }
	const std::string path =
		probe::outputPath(ProbeState::get().levelKey + "_decisions.csv");
	std::FILE* f = std::fopen(path.c_str(), "w");
	if (!f) { log::error("Probe 21: cannot write {}", path); return; }
	std::fprintf(f, "# Probe 21: decisions on the best path (%.2f%%, %llu steps)\n",
	             sv.bestPct, static_cast<unsigned long long>(sv.bestStep));
	std::fprintf(f, "step,mode,choice,hasSteer,steerChoice,togglesBefore,"
		                "y,vy,yProj,target,band,winLo,winHi,path\n");
	int air = 0;
	for (auto const& d : sv.bestDecisions) {
		if (d.mode != static_cast<uint8_t>(ModeClass::Ground)) air++;
		std::fprintf(f, "%d,%s,%d,%d,%d,%d,%.2f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
		             d.step, modeClassName(d.mode),
		             d.choice ? 1 : 0, d.hasSteer ? 1 : 0, d.steerChoice ? 1 : 0,
		             d.togglesBefore,
		             d.y, d.vy, d.yProj, d.target, d.band, d.lo, d.hi, d.path);
	}
	std::fclose(f);
	int steered = 0;
	for (auto const& d : sv.bestDecisions) if (d.hasSteer) steered++;
	log::info("Probe 21: {} decisions on the best path ({} air), {} with a steer "
	          "({:.1f}%) -> {}",
	          sv.bestDecisions.size(), air, steered,
	          100.0 * steered / static_cast<double>(sv.bestDecisions.size()), path);
}


void expressAuditReport() {
	auto& sv = Solver::get();
	if (!sv.exprToggles) { log::info("Probe 20: no air toggles in the replay."); return; }
	log::info("Probe 20 - is the winning path EXPRESSIBLE? branch mode {}, "
	          "interval {}..{}",
	          g_config.geomBranchMode, g_config.airBranchInterval,
	          g_config.airBranchIntervalMax);
	log::info("  air steps {}   decision points offered {}   one per {:.1f} steps",
	          sv.exprAirSteps, sv.exprOfferPoints,
	          sv.exprOfferPoints ? static_cast<double>(sv.exprAirSteps) / sv.exprOfferPoints : 0.0);
	log::info("  input changes {}   ON a decision point {} ({:.1f}%)   "
	          "MISSED {} ({:.1f}%)",
	          sv.exprToggles, sv.exprOffered,
	          100.0 * sv.exprOffered / sv.exprToggles,
	          sv.exprMissed, 100.0 * sv.exprMissed / sv.exprToggles);
	if (!sv.exprMissed) {
		log::info("  EVERY input change lands on a decision point. The winning path "
		          "is fully expressible under this rule, so the stall is NOT "
		          "granularity.");
		return;
	}
	log::info("  why the rule declined:  cadence {}   corridor unchanged {}   "
	          "no map reading {}",
	          sv.exprMissCadence, sv.exprMissUnchanged, sv.exprMissNoRead);

	// Where they cluster, in 5% bands.
	std::map<int, int> byPct;
	for (auto const& e : sv.exprMisses) byPct[static_cast<int>(e.pct / 5.0) * 5]++;
	std::string hist;
	for (auto const& kv : byPct)
		hist += fmt::format("{}%:{}  ", kv.first, kv.second);
	log::info("  missed toggles by level position:  {}", hist);

	const std::string path = probe::outputPath(ProbeState::get().levelKey + "_express.csv");
	if (std::FILE* f = std::fopen(path.c_str(), "w")) {
		std::fprintf(f, "# Probe 20: input changes in a known-good path with no "
		                "decision point available\n");
		std::fprintf(f, "step,pct,x,y,vy,mode,elapsed,reason,winLo,winHi\n");
		static const char* kR[4] = {"offered","cadence","unchanged","noread"};
		for (auto const& e : sv.exprMisses)
			std::fprintf(f, "%llu,%.2f,%.1f,%.1f,%.3f,%s,%d,%s,%.1f,%.1f\n",
			             static_cast<unsigned long long>(e.step), e.pct, e.x, e.y, e.vy,
			             modeClassName(e.mode), e.elapsed, kR[e.reason & 3],
			             e.glo, e.ghi);
		std::fclose(f);
		log::info("  every missed toggle -> {}", path);
	}
}

void pathAuditStep(PlayerObject* p) {
	auto& m  = GeoMap::get();
	auto& sv = Solver::get();
	if (!m.valid || !p || p->m_isDead) return;
	const double x = p->getPositionX();
	const int si = m.segIndex(x);
	if (si < 0) return;
	const cocos2d::CCRect r = p->getObjectRect();
	const double lo = r.origin.y, hi = r.origin.y + r.size.height;
	sv.pathAuditSteps++;

	// Overlap rather than containment: the hitbox is the player, and the
	// interval holding most of it is the one they are in.
	const GeoMap::Free* best = nullptr;
	double bestOv = 0.0;
	for (auto const& f : m.freeSpans[static_cast<size_t>(si)]) {
		const double ov = std::min(hi, static_cast<double>(f.hi)) -
		                  std::max(lo, static_cast<double>(f.lo));
		if (ov > bestOv) { bestOv = ov; best = &f; }
	}
	if (!best) {
		sv.pathOffMap++;
		if (sv.pathFirstOffX < 0.0) { sv.pathFirstOffX = x; sv.pathFirstOffY = lo; }
		// How far above the roof the player actually was - the number that says
		// what the crop cost.
		const double roof = si < static_cast<int>(m.segRoof.size())
		                  ? static_cast<double>(m.segRoof[static_cast<size_t>(si)])
		                  : m.yHi;
		const double over = lo - roof;
		if (over > sv.pathWorstOff) { sv.pathWorstOff = over; sv.pathWorstOffX = x; }
		if (m.xs.size() > 1) {
			const double span = m.xs.back() - m.xs.front();
			int b = span > 1.0
			      ? static_cast<int>(20.0 * (x - m.xs.front()) / span) : 0;
			b = b < 0 ? 0 : b > 19 ? 19 : b;
			sv.pathOffBin[b]++;
			sv.pathOffBinWorst[b] = std::max(sv.pathOffBinWorst[b], over);
		}
		return;
	}
	if (!best->live) {
		sv.pathDead++;
		if (sv.pathFirstDeadX < 0.0) { sv.pathFirstDeadX = x; sv.pathFirstDeadY = lo; }
		return;
	}
	if (hi < best->wlo || lo > best->whi) {
		sv.pathOutWindow++;
		if (sv.pathFirstOutX < 0.0) { sv.pathFirstOutX = x; sv.pathFirstOutY = lo; }
	}
}

// Probe 19. Where the search actually dies, and what the map said there.
//
// Read it as: if one cell holds a large share of deaths, the stall is one
// obstacle and the map's opinion at that cell is the diagnosis. If deaths are
// spread thin, the search is lost rather than blocked, which is a different
// problem with a different fix.
void deathMapReport(int topN) {
	auto& sv = Solver::get();
	if (sv.deathCells.empty()) { log::info("Probe 19: no deaths recorded."); return; }
	std::vector<std::pair<uint64_t,uint32_t>> v(sv.deathCells.begin(), sv.deathCells.end());
	std::sort(v.begin(), v.end(),
	          [](auto const& a, auto const& b){ return a.second > b.second; });
	auto& m = GeoMap::get();
	log::info("Probe 19: {} deaths across {} cells (one block each). Top {}:",
	          sv.deathsRecorded, sv.deathCells.size(),
	          std::min<size_t>(v.size(), static_cast<size_t>(topN)));
	for (size_t i = 0; i < v.size() && i < static_cast<size_t>(topN); i++) {
		const int xb = static_cast<int>(static_cast<int32_t>(v[i].first >> 32));
		const int yb = static_cast<int>(static_cast<int32_t>(v[i].first & 0xFFFFFFFFu));
		const double x = xb * 30.0, y = yb * 30.0;
		const double share = 100.0 * v[i].second / static_cast<double>(sv.deathsRecorded);
		const double vy = sv.deathVySum[v[i].first] / std::max(1u, v[i].second);
		// What the map believed at that x: the interval holding that y, its live
		// window, and where the steer would have been aiming.
		std::string say = "map: -";
		if (m.valid) {
			const int si = m.segIndex(x + 15.0);
			if (si >= 0) {
				for (auto const& f : m.freeSpans[static_cast<size_t>(si)]) {
					if (y + 30.0 < f.lo || y > f.hi) continue;
					say = fmt::format("gap {:.0f}..{:.0f} window {:.0f}..{:.0f} {} "
					                  "centre {:.0f}",
					                  f.lo, f.hi, f.wlo, f.whi,
					                  f.live ? "live" : "DEAD",
					                  0.5 * (f.wlo + f.whi));
					break;
				}
				if (say == "map: -") say = "NO INTERVAL HOLDS THIS Y";
			}
		}
		log::info("  x {:6.0f}  y {:5.0f}  {:5} deaths ({:4.1f}%)  mean vy {:+6.2f}  {}",
		          x, y, v[i].second, share, vy, say);
	}
}

void pathAuditReport() {
	auto& sv = Solver::get();
	if (!sv.pathAuditSteps) return;
	const double n = static_cast<double>(sv.pathAuditSteps);
	log::info("Probe 17: the winning path against this map - {} steps checked, "
	          "roof mode {}.", sv.pathAuditSteps, g_config.geomPortalRoof);
	log::info("  OFF-MAP {} ({:.2f}%)  DEAD {} ({:.2f}%)  OUT-OF-WINDOW {} ({:.2f}%)",
	          sv.pathOffMap,    100.0 * sv.pathOffMap    / n,
	          sv.pathDead,      100.0 * sv.pathDead      / n,
	          sv.pathOutWindow, 100.0 * sv.pathOutWindow / n);
	if (sv.pathOffMap)
		log::error("  OFF-MAP first at x {:.0f} y {:.0f}; worst {:.0f} units above "
		           "the roof at x {:.0f}. The run was there, so the MAP is wrong.",
		           sv.pathFirstOffX, sv.pathFirstOffY,
		           sv.pathWorstOff, sv.pathWorstOffX);
	if (sv.pathDead)
		log::error("  DEAD first at x {:.0f} y {:.0f}. The run reached the end from "
		           "there, so the liveness sweep has a FALSE NEGATIVE.",
		           sv.pathFirstDeadX, sv.pathFirstDeadY);
	if (sv.pathOutWindow)
		log::warn("  OUT-OF-WINDOW first at x {:.0f} y {:.0f} - live space the "
		          "steering would not have aimed at.",
		          sv.pathFirstOutX, sv.pathFirstOutY);
	if (sv.pathOffMap) {
		auto const& m = GeoMap::get();
		const double x0 = m.xs.empty() ? 0.0 : m.xs.front();
		const double sp = (m.xs.size() > 1 ? m.xs.back() : 0.0) - x0;
		log::info("  where the off-map steps are (x range, count, deepest):");
		for (int b = 0; b < 20; b++) {
			if (!sv.pathOffBin[b]) continue;
			log::info("    x {:6.0f}..{:<6.0f} ({:2.0f}-{:2.0f}%)  {:>6} steps  "
			          "deepest {:.0f} units above the roof",
			          x0 + sp * b / 20.0, x0 + sp * (b + 1) / 20.0,
			          5.0 * b, 5.0 * (b + 1),
			          sv.pathOffBin[b], sv.pathOffBinWorst[b]);
		}
	}
	if (!sv.pathOffMap && !sv.pathDead)
		log::info("  The winning path stays inside live space the whole way. This "
		          "map is not deleting this route - the fault is downstream, in "
		          "the steering and the window, not in liveness.");
}

// Probe 16 readout. Every flown cell is printed with the evidence beside it,
// and the footer names what is still missing, so the table says what is left
// to do rather than leaving it to memory.
void climbEnvelopeDump() {
	auto& sv = Solver::get();
	static const char* kM[8] = {"cube","ship","wave","ufo",
	                            "ball","robot","spider","swing"};
	static const char* kS[5] = {"0.5x","1x","2x","3x","4x"};
	static const double kDx[5] = {kSpeedSlow, kSpeedNormal, kSpeedFast,
	                              kSpeedFaster, kSpeedFastest};
	log::info("Probe 16: vertical envelope. Reported figure is slope x the "
	          "canonical per-step dx, which is frame-rate free. The map's reach "
	          "model assumes {:.3f} for every mode at every speed.",
	          static_cast<double>(g_config.geomVerticalReach) /
	              std::max(1.0, static_cast<double>(g_config.geomSliceX)) *
	              kSpeedNormal);
	log::info("  mode    size  speed   up/step  dn/step | vy up/dn (the game's own "
	          "m_yVelocity) | samples | state - second line is the game's "
	          "constants and the checks on my assumptions");
	int flown = 0, done = 0;
	for (int mo = 0; mo < 8; mo++)
		for (int sz = 0; sz < 2; sz++)
			for (int sp = 0; sp < 5; sp++) {
				const uint64_t nU = sv.envN[mo][sz][sp][0];
				const uint64_t nD = sv.envN[mo][sz][sp][1];
				if (!nU && !nD) continue;
				flown++;
				const bool ok = sv.envLocked[mo][sz][sp][0] && sv.envLocked[mo][sz][sp][1];
				if (ok) done++;
				const double raw = (sv.envSumRaw[mo][sz][sp][0] + sv.envSumRaw[mo][sz][sp][1]) /
				                   static_cast<double>(nU + nD);
				const double mdx = (sv.envSumDx[mo][sz][sp][0] + sv.envSumDx[mo][sz][sp][1]) /
				                   static_cast<double>(nU + nD);
				const uint64_t nAll = nU + nD;
				log::info("  {:<7} {:<5} {:<6}  {:7.3f}  {:7.3f} | vy {:7.3f} {:7.3f} | "
				          "n {:>6}/{:<6} at-max {:>4}/{:<4}  {}",
				          kM[mo], sz ? "mini" : "norm", kS[sp],
				          sv.envMaxSlope[mo][sz][sp][0] * kDx[sp],
				          sv.envMaxSlope[mo][sz][sp][1] * kDx[sp],
				          sv.envMaxVy[mo][sz][sp][0], sv.envMaxVy[mo][sz][sp][1],
				          nU, nD, sv.envNearN[mo][sz][sp][0], sv.envNearN[mo][sz][sp][1],
				          ok ? "done"
				             : !nU ? "NEVER FLEW UP"
				             : !nD ? "NEVER FLEW DOWN"
				             : "thin - hold longer");
				// The game's own constants, and the checks on my guesses. If
				// rawSpeed is not near one of 0.7/0.9/1.1/1.3/1.6 the speed
				// mapping is wrong; if meanDx is not near want the step
				// filter is; if upside is not 0 the cell mixes gravity
				// directions and up/dn mean different things in it.
				log::info("          raw {:.3f}  dx {:.3f}/{:.3f} (span {:.2f} steps)  "
				          "gravity {:.4f}  "
				          "fallSpeed {:.4f}  spdMul {:.4f}  gravMod {:.3f}  "
				          "upside {:.0f}%",
				          raw, mdx, kDx[sp], mdx / kDx[sp],
				          (sv.envSumGravity[mo][sz][sp][0]+sv.envSumGravity[mo][sz][sp][1])/nAll,
				          (sv.envSumFall[mo][sz][sp][0]+sv.envSumFall[mo][sz][sp][1])/nAll,
				          (sv.envSumSpdMul[mo][sz][sp][0]+sv.envSumSpdMul[mo][sz][sp][1])/nAll,
				          (sv.envSumGravMod[mo][sz][sp][0]+sv.envSumGravMod[mo][sz][sp][1])/nAll,
				          100.0*(sv.envUpsideN[mo][sz][sp][0]+sv.envUpsideN[mo][sz][sp][1])/nAll);
			}
	log::info("  {} cells touched, {} complete. {} repositions and {} "
	          "stationary frames skipped. A cell is complete at 150+ samples "
	          "and 30+ at-max per direction - terminal velocity is sustained, "
	          "so a low at-max means the figure is a stray frame.",
	          flown, done, sv.envRejected, sv.envStationary);
	if (!flown)
		log::warn("  Nothing recorded. Probe 16 samples in FREE FLIGHT only - never "
		          "while the solver is running.");
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

	// Watch the best path the search has found, at normal speed. When the solver
	// stalls, the counters say WHERE it stops but not WHY - this replays the
	// furthest-reaching attempt so the failure can actually be seen.
	if (keyPressedEdge(VK_F9)) {
		auto& sv = Solver::get();
		if (sv.running) { solverWriteBestMacro(); }
		startVerify(false, "best.txt");
	}



	// G = dump level geometry (Probe 7). Read-only: no search state is touched,
	// so this is safe to press at any time, including mid-solve.
	if (keyPressedEdge('G')) {
		if (PlayLayer::get()) geometryDump();
		else log::warn("Probe 7: not in a level");
	}

	// L = cycle the reach mode and rebuild the map, so one session can measure
	// windowed against pure overlap against no liveness at all.
	if (keyPressedEdge('L')) {
		g_config.geomReachMode = (g_config.geomReachMode + 1) % 3;
		if (PlayLayer::get() && geometryBuildMap()) {
			auto const& m = GeoMap::get();
			double sumW = 0.0; int liveN = 0;
			for (auto const& fr : m.freeSpans)
				for (auto const& f : fr)
					if (f.live) { sumW += (double)f.whi - (double)f.wlo; liveN++; }
			log::info("Reach {} - map rebuilt: {} free intervals, {} dead ({:.1f}%), "
			          "mean live window {:.0f} units. F2 to solve with this.",
			          geomReachModeName(g_config.geomReachMode),
			          m.freeCount, m.deadCount,
			          m.freeCount > 0 ? 100.0 * m.deadCount / m.freeCount : 0.0,
			          liveN ? sumW / liveN : 0.0);
		} else {
			log::info("Reach {} - not in a level, applies at the next solve.",
			          geomReachModeName(g_config.geomReachMode));
		}
	}

	// K = cycle the clearance deadband. No map rebuild needed - the band is read
	// at steer time - so this takes effect on the next F2.
	//
	// 0.25 is the current value: no steer while the player is within a quarter
	// of the window height of its centre. 0.00 means the trail drives every
	// decision. 0.50 steers only when badly off.
	if (keyPressedEdge('K')) {
		const double next = g_config.geomClearanceBand < 0.01 ? 0.50
		                  : g_config.geomClearanceBand < 0.30 ? 0.00
		                                                      : 0.25;
		g_config.geomClearanceBand = next;
		log::info("Clearance band {:.2f} of window height{} - F2 to solve with this.",
		          g_config.geomClearanceBand,
		          g_config.geomClearanceBand < 0.01 ? " (trail drives every decision)" : "");
	}

	// I = live ceiling readout while you fly (Probe 12).
	if (keyPressedEdge('I')) {
		g_config.liveCeilingReadout = !g_config.liveCeilingReadout;
		log::info("Live ceiling readout: {}{}",
		          g_config.liveCeilingReadout ? "ON" : "off",
		          g_config.liveCeilingReadout
		              ? " - play by hand and fly into the ceiling in each section. "
		                "Every CONTACT line is the roof announcing itself."
		              : "");
	}

	// E = crop the map to the reachable band.
	if (keyPressedEdge('E')) {
		g_config.ceilingClamp = !g_config.ceilingClamp;
		log::info("Reachable-band crop: {}. F2 to solve with this.",
		          g_config.ceilingClamp
		              ? "ON - intervals above the learned ceiling are not routes"
		              : "off - the map's global roof is taken at face value");
	}

	// D = how selective the geometry steer is about speaking.
	if (keyPressedEdge('D')) {
		g_config.geomSteerGate = (g_config.geomSteerGate + 1) % 4;
		static const char* kGate[4] = {
			"ALWAYS - steer at every air decision (the old behaviour)",
			"DISCRIMINATING - only where liveness separates something",
			"DISCRIMINATING + NARROWING - also where the corridor ahead tightens",
			"ONE LIVE ONLY - speak only when there is no choice to make"};
		log::info("Geometry steer gate: {}. F2 to solve with this.",
		          kGate[g_config.geomSteerGate]);
	}

	// X = geometry ordering on/off.
	//
	// Exists to bisect a regression rather than to tune anything: Base After Base
	// solved in 13s and VERIFIED when the map held 2034 intervals, and stalls at
	// 69.09% now that it holds 4199. The map is the one thing known to have
	// changed, and `dead 0` says it is not pruning anything, so if it is the
	// cause it can only be through ORDERING. One run with this off separates
	// those two possibilities.
	if (keyPressedEdge('X')) {
		g_config.geometryOrdering = !g_config.geometryOrdering;
		log::info("Geometry ordering: {}. F2 to solve with this.",
		          g_config.geometryOrdering
		              ? "ON - the map chooses which branch an air decision tries first"
		              : "OFF - release-before-hold everywhere, the map is not consulted");
	}

	// W = dump the measured motion model (Probe 9).
	if (keyPressedEdge('W')) {
		if (!g_config.motionModelEnabled && !g_config.ceilingProbe) {
			g_config.motionModelEnabled = true;
			g_config.ceilingProbe       = true;
			log::info("Probes 9 and 10: collection ON (they cost a map operation per "
			          "step, so they are off by default). W again to dump.");
			return;
		}
		auto& mm = MotionModel::get();
		if (mm.rows.empty()) {
			log::warn("Probe 9: nothing recorded yet - run a solve first (F2), then W. "
			          "Collection is passive and always on.");
		} else {
			motionModelDump("on demand");
		}
		ceilingProbeDump("on demand");
	}



	// Y = let tap state go stale across a reposition (the UFO bisect).
	if (keyPressedEdge('Y')) {
		g_config.tapStateSurvivesRestore = !g_config.tapStateSurvivesRestore;
		log::info("Tap state on a reposition: {}. F2 to solve with this.",
		          g_config.tapStateSurvivesRestore
		              ? "SURVIVES (stale - sv.hold outlives the tap, so the next Tap "
		                "decision is pushed held and its alternative is FREE. This is "
		                "how it was when Clubstep cleared in 9803757)"
		              : "RESTORED (clean, the default - the tap self-releases, so the "
		                "next Tap decision is pushed released and its alternative is "
		                "charged against the toggle budget)");
	}


	// B = ceiling on how long modes 1/2 may go without a decision. At
	// airBranchInterval this collapses mode 1 into mode 0, so the sweep spans both.
	if (keyPressedEdge('B')) {
		int v = g_config.airBranchIntervalMax;
		v = v == 16 ? 12 : v == 12 ? 8 : v == 8 ? 6 : v == 6 ? 4 : 16;
		g_config.airBranchIntervalMax = v;
		log::info("Air branch ceiling: a decision is forced after at most {} steps "
		          "({:.3f}s) when the corridor ahead is unchanged{}. F2 to solve "
		          "with this.",
		          v, v / 240.0,
		          v <= g_config.airBranchInterval
		              ? " - equals airBranchInterval, so mode 1 is now mode 0"
		              : "");
	}


	// F = restore the branch rule's carried state on a reposition.
	if (keyPressedEdge('F')) {
		g_config.geoStateRestore = !g_config.geoStateRestore;
		log::info("Branch-rule carried state: {}. F2 to solve with this.",
		          g_config.geoStateRestore
		              ? "RESTORED on reposition - behaviour CHANGE from a02573c"
		              : "carries across (stale), exactly as a02573c - the default");
	}


	// Q = reach-clamped steer target (replaces the lead term).
	if (keyPressedEdge('Q')) {
		g_config.geomReachClamp = !g_config.geomReachClamp;
		log::info("Steer target: {}. F2 to solve with this.",
		          g_config.geomReachClamp
		              ? "window centre CLAMPED to the measured reachable band, lead "
		                "projection off"
		              : "window centre with the lead projection, as before");
	}

	// J = silence the steer (ablation). Map ordering and liveness stay on.
	if (keyPressedEdge('J')) {
		g_config.geomSteerSilent = !g_config.geomSteerSilent;
		log::info("Steering: {}. F2 to solve with this.",
		          g_config.geomSteerSilent
		              ? "SILENCED - the map still orders moves and marks dead space, "
		                "but never tells a decision which way to go"
		              : "on, as normal");
	}

	// P = lead term in steps, with the measured vy -> dy conversion.
	if (keyPressedEdge('P')) {
		g_config.geomLeadInSteps = !g_config.geomLeadInSteps;
		log::info("Steer lead: {} (ship lead {:.0f} -> projects {:.1f} units at "
		          "terminal climb). F2 to solve with this.",
		          g_config.geomLeadInSteps
		              ? "STEPS, with the measured vy->dy conversion"
		              : "raw lead * m_yVelocity, as before",
		          g_config.geomLeadHold,
		          g_config.geomLeadInSteps
		              ? g_config.geomLeadHold * 7.9 * g_config.geomVyToDy
		              : g_config.geomLeadHold * 7.9);
	}

	// V = velocity reachability prune (4b).
	if (keyPressedEdge('V')) {
		g_config.velocityPrune = !g_config.velocityPrune;
		log::info("Velocity prune: {} (horizon {} steps, margin {:.2f}). "
		          "F2 to solve with this.",
		          g_config.velocityPrune ? "ON - a hold-mode branch whose reachable "
		                                   "band is all dead at the horizon is killed"
		                                 : "OFF",
		          g_config.velocityPruneSteps, g_config.velocityPruneMargin);
	}

	// M = branch on geometry change rather than on a fixed cadence.
	if (keyPressedEdge('M')) {
		g_config.geomBranchMode = (g_config.geomBranchMode + 1) % 4;
		static const char* kName[4] = {
			"every airBranchInterval steps, as before",
			"where the corridor ahead CHANGES",
			"where it CHANGES, but full cadence in tight corridors",
			"where it CHANGES, ceiling DERIVED from slack/climb rate"};
		log::info("Air branching: {} (min {} steps, max {}, tight < {:.0f} units). "
		          "F2 to solve with this.",
		          kName[g_config.geomBranchMode],
		          g_config.airBranchInterval, g_config.airBranchIntervalMax,
		          g_config.geomTightHeights * GeoMap::get().playerH);
	}


	// A = archive restart instead of the blind escape rewind.
	if (keyPressedEdge('A')) {
		g_config.escapeArchiveRestart = !g_config.escapeArchiveRestart;
		log::info("Escape: {}. F2 to solve with this.",
		          g_config.escapeArchiveRestart
		              ? "ARCHIVE RESTART - on a stall, return to a promising archived "
		                "cell and resume from there"
		              : "rewind - drop to the commit floor and escalate the ladder");
	}


	// Z = cap the map roof at the highest portal-derived ceiling.
	if (keyPressedEdge('Z')) {
		g_config.geomPortalRoof = (g_config.geomPortalRoof + 1) % 3;
		static const char* kRoof[3] = {
			"highest solid object + 120, as before",
			"PREFIX MAX - highest cap of any portal at or before each x",
			"MOST RECENT - the cap of the last portal before each x, so a lower "
			"section lowers the roof"};
		log::info("Map roof: {}. F2 to solve with this (the map rebuilds on F2).",
		          kRoof[g_config.geomPortalRoof]);
	}


	// 4 = restore prevAirMode/prevOnGround on a reposition.
	if (keyPressedEdge('4')) {
		g_config.modeStateRestore = !g_config.modeStateRestore;
		log::info("Mode/ground state on a reposition: {}. F2 to solve with this.",
		          g_config.modeStateRestore
		              ? "RESTORED - behaviour CHANGE from cfa8462"
		              : "carries across (stale), exactly as cfa8462 - the default");
	}


	// 3 = the decisions behind the best path (Probe 21). Read-only.
	if (keyPressedEdge('3')) bestDecisionDump();


	// 2 = where the search is dying (Probe 19). Read-only.
	if (keyPressedEdge('2')) deathMapReport(12);

	// T = how the steer decides it is close enough to say nothing.
	if (keyPressedEdge('T')) {
		g_config.geomBandMode = (g_config.geomBandMode + 1) % 2;
		log::info("Steer deadband: {}. F2 to solve with this.",
		          g_config.geomBandMode
		              ? "MEASURED - the distance the player moves in one decision "
		                "interval, so it does not depend on how wide the corridor is"
		              : "fraction of the corridor (geomClearanceBand), as before");
	}



	// N = toggle the forward scan. Read at steer time, so no rebuild.
	if (keyPressedEdge('N')) {
		g_config.geomForwardScan = !g_config.geomForwardScan;
		log::info("Forward scan {} - {}. F2 to solve with this.",
		          g_config.geomForwardScan ? "ON" : "off",
		          g_config.geomForwardScan
		              ? "walks every slice to the lookahead and aims at the gap it must thread"
		              : "samples one slice at the lookahead, as before");
	}



	// O = route steering: follow the corridor the player is actually in.
	if (keyPressedEdge('O')) {
		g_config.geomRouteSteer = !g_config.geomRouteSteer;
		log::info("Steering: {}. F2 to solve with this.",
		          g_config.geomRouteSteer
		              ? "ROUTE - follow this corridor forward and aim at what it leads to"
		              : "clearance - aim at the interval containing the projected altitude");
	}

	// U = urgency steering instead of clearance steering.
	if (keyPressedEdge('U')) {
		g_config.geomUrgencySteer = !g_config.geomUrgencySteer;
		log::info("Urgency gate on the route target: {}. Needs O as well.",
		          g_config.geomUrgencySteer
		              ? "ON - steer only while the corridor is still catchable at "
		                "the measured climb rate, silent when there is time and "
		                "silent when it is already lost"
		              : "off - steer whenever off-centre");
	}


	// R = reach sweep (Probe 9). Read-only; rebuilds the map several times and
	// leaves it as it found it.
	if (keyPressedEdge('R')) geometryReachSweep();

	// S = vertical envelope (Probe 16). Read-only.
	if (keyPressedEdge('S')) climbEnvelopeDump();

	// H = dead-end map (Probe 8). Read-only, same as G.
	if (keyPressedEdge('H')) {
		if (!PlayLayer::get()) log::warn("Probe 8: not in a level");
		else if (!g_config.deadEndProbe) log::warn("Probe 8: disabled in config");
		else deadEndDump();
	}

	// C = corridor sweep (Probe 6). A letter because every F key is taken.
	if (keyPressedEdge('C')) {
		auto& sw = Sweep::get();
		auto& st = ProbeState::get();
		if (sw.running) {
			log::info("Probe 6: stopped by user after {} variants.", sw.results.size());
			if (!sw.results.empty()) sweepWriteResults();
			sw.clear();
			st.mode = Mode::Idle;
		} else if (PlayLayer::get()) {
			bool fix = false;
			sw.clear();
			if (!loadMacroFile(levelFilePath("best.txt"), sw.base, &fix) || sw.base.empty()) {
				log::error("Probe 6: no best.txt to sweep - run a solve first.");
				return;
			}
			g_config.physicsFix = fix;
			PlayLayer::get()->m_isPracticeMode = false;  // geometry, not search
			sweepBuildMacro();
			sw.running = true;
			st.mode    = Mode::Sweep;
			st.resetPending = true;
			log::info("Probe 6: corridor sweep over {} variants - S {}..{} step {}, "
			          "k 0..{} taps. Base macro {} steps. C again to stop.",
			          sw.variantCount(), g_config.sweepStepBase,
			          g_config.sweepStepBase +
			              (g_config.sweepStepCount - 1) * g_config.sweepStepStride,
			          g_config.sweepStepStride, g_config.sweepMaxTaps, sw.base.size());
		}
	}

	if (keyPressedEdge(VK_F8)) {
		g_config.resyncEnabled = !g_config.resyncEnabled;
		log::info("Solver: periodic resync {}", g_config.resyncEnabled ? "ENABLED" : "disabled");
	}

	if (keyPressedEdge(VK_F2)) {
		auto& sv = Solver::get();
		if (sv.running) {
			log::info("Solver: stopped by user at best {:.2f}%", sv.bestPct);
			solverWriteBudgetPops("stopped");
			solverWriteMacro("partial.txt");
			solverWriteBestMacro();
			// Snapshot the census on the way out: a run stopped by hand is the
			// usual way a stall gets inspected, and the map dies with sv.clear().
			cellProbeDump("stopped");
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
			CellProbe::get().clear();
			SolverArchive::get().clear();
			if (g_config.geometryOrdering) {
				if (geometryBuildMap()) {
					auto& m = GeoMap::get();
					log::info("Solver: geometry map built - {} free intervals, {} dead "
					          "({:.1f}%). Ordering {}. Lookahead {} units. Reach {}. "
					          "Speed {:.2f}..{:.2f} dx/step over {} segments.",
					          m.freeCount, m.deadCount,
					          m.freeCount > 0 ? 100.0 * m.deadCount / m.freeCount : 0.0,
					          g_config.geometryOrdering ? "ON" : "off",
					          g_config.geomLookaheadScaleWithSpeed
					              ? g_config.geomLookaheadSteps
					              : g_config.geomLookaheadFixedX,
					          geomReachModeName(g_config.geomReachMode),
					          m.segSpeed.empty() ? 0.0f
					              : *std::min_element(m.segSpeed.begin(), m.segSpeed.end()),
					          m.segSpeed.empty() ? 0.0f
					              : *std::max_element(m.segSpeed.begin(), m.segSpeed.end()),
					          m.segSpeed.size());

					// How much of the mapped band is unreachable sky, and how
					// wide the live windows are because of it. INERT: nothing
					// reads m_maxGameplayY yet.
					//
					// A free interval with no roof runs all the way to yHi, so
					// its centre - which is what clearance steering aims at -
					// sits in empty space far above any route. Counting the
					// intervals whose top IS yHi says how often that happens.
					{
						auto* bgl = GJBaseGameLayer::get();
						const double ceiling = bgl ? static_cast<double>(bgl->m_maxGameplayY) : 0.0;
						int openTop = 0, total = 0;
						double widest = 0.0, sumW = 0.0;
						for (auto const& fr : m.freeSpans)
							for (auto const& f : fr) {
								total++;
								const double w = static_cast<double>(f.whi) -
								                 static_cast<double>(f.wlo);
								if (f.live) { sumW += w; widest = std::max(widest, w); }
								if (f.hi >= m.yHi - 0.01) openTop++;
							}
						log::info("Solver: map band y {:.0f}..{:.0f}, game ceiling "
						          "m_maxGameplayY {:.0f}. {} of {} free intervals reach "
						          "the top of the band. Live window mean {:.0f} widest "
						          "{:.0f} units.",
						          m.yLo, m.yHi, ceiling, openTop, total,
						          total ? sumW / total : 0.0, widest);
					}
				} else {
					log::warn("Solver: geometry map unavailable - geometry ordering and "
					          "pruning both off for this run.");
				}
			}
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
				g_config.noSavestates = !g_config.verifiedPrefixSavestates;
				log::info("Solver: starting from a VERIFIED prefix of {} steps, searching "
				          "forward {}.", sv.resyncTarget,
				          g_config.verifiedPrefixSavestates
				            ? "with savestates (~100 branches/s; restores measured exact "
				              "in every mode, and the macro is still verified from frame 0)"
				            : "WITHOUT savestates - every branch replays from frame 0");
			}
			// Preallocated once, here, so nothing in the stepping path ever
			// reallocates. 64k rows covers every main level with room to spare
			// (Stereo Madness is ~20k steps) at 48 bytes a row, about 3 MB.
			sv.pathTrace.clear();
			sv.pathTrace.reserve(65536);
			sv.pathTraceFull = false;

			log::info("Solver: hybrid restore is {} - air decisions {} their own "
			          "checkpoint. Plan 13.13.",
			          g_config.hybridRestore ? "ON" : "OFF",
			          g_config.hybridRestore ? "do NOT carry" : "carry");
log::info("Solver: steer target is {} (Q toggles).",
			          g_config.geomReachClamp
			              ? "clamped to the measured reachable band"
			              : "the live window centre with a fitted lead projection");
log::info("Solver: steering is {} (J toggles).",
			          g_config.geomSteerSilent ? "SILENCED (ablation)" : "on");
log::info("Solver: steer lead is {} (P toggles).",
			          g_config.geomLeadInSteps
			              ? "in STEPS with the measured vy->dy conversion"
			              : "raw lead * m_yVelocity - projects ~63 units at terminal climb");
log::info("Solver: velocity prune {} (V toggles).",
			          g_config.velocityPrune
			              ? fmt::format("ON, horizon {} steps, margin {:.2f}",
			                            g_config.velocityPruneSteps,
			                            g_config.velocityPruneMargin)
			              : std::string("OFF"));
log::info("Solver: mode/ground state {} (4 toggles).",
			          g_config.modeStateRestore
			              ? "is RESTORED on reposition - BEHAVIOUR CHANGE from cfa8462"
			              : "carries across (stale), exactly as cfa8462");
			log::info("Solver: branch-rule carried state {} (F toggles).",
			          g_config.geoStateRestore
			              ? "is RESTORED on reposition - BEHAVIOUR CHANGE from a02573c"
			              : "carries across (stale), exactly as a02573c");
			log::info("Solver: tap state {} (Y toggles).",
			          g_config.tapStateSurvivesRestore
			              ? "SURVIVES a reposition - stale, as when Clubstep "
			                "cleared in 9803757"
			              : "is RESTORED on a reposition - clean, the current "
			                "default");
			log::info("Solver: starting DFS. air branch interval {} steps, "
			          "release-before-hold ordering. F2 again to stop.",
			          g_config.airBranchInterval);
		}
	}

	if (keyPressedEdge(VK_F11)) runProbe5(1000);
	if (keyPressedEdge(VK_F12)) startRestoreTest(RestoreKind::Full);
	if (keyPressedEdge(VK_F10)) startRestoreTest(RestoreKind::DirectLoad);
	if (keyPressedEdge(VK_F4))  startVerify(false); // normal mode

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
	// Built once and used by three consumers: the probe trace, the solver's path
	// trace, and the verify comparison. They MUST agree field for field or the
	// divergence report compares two different notions of state.
	probe::TraceRow makeTraceRow(bool pressed, int relStep) {
		auto& st = ProbeState::get();
		auto* p  = m_player1;
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
		{
			auto it = p->m_holdingButtons.find(kJumpButton);
			if (it != p->m_holdingButtons.end() && it->second) f |= probe::FlagHoldingJump;
		}
		if (!m_queuedButtons.empty()) f |= probe::FlagQueuedButtons;
		f |= probe::packRingCount(p->m_touchingRings ? p->m_touchingRings->count() : 0);
		row.flags = f;
		return row;
	}

	void recordStep(bool pressed, int relStep) {
		if (!m_player1) return;
		ProbeState::get().trace.push(makeTraceRow(pressed, relStep));
	}

	// A hash of everything that determines the FUTURE of a run, for dedup.
	//
	// The rule is asymmetric, so err toward including state: hashing too much
	// causes UNDER-merging, which costs only efficiency, while hashing too
	// little causes OVER-merging, which silently prunes reachable branches and
	// loses completeness. Two states that hash equal are treated as the same
	// state forever after, so anything left out is a correctness bug, not a
	// tuning issue.
	//
	// What goes in is exactly what a day of restore debugging proved a player
	// consists of: the physics scalars, plus the input state that lives in three
	// other places (m_holdingButtons in a gd::map, m_queuedButtons on the LAYER,
	// the cocos node transform), plus m_cameraFlip, because a mirror transition
	// in flight changes what happens next.
	//
	// What stays OUT is anything time-like. A monotonically increasing value -
	// a step index, an absolute timestamp - makes every state unique and turns
	// the visited set into dead weight that never fires once. That is why the
	// queued buttons contribute only their SEMANTIC content and never their
	// m_timestamp or m_step, and why `percent` is skipped as a pure function of
	// x rather than independent state.
	uint64_t solverStateKey() {
		auto* p = m_player1;
		if (!p) return 0;

		// Physics scalars, as raw bits: a decimal rendering can compare equal
		// while the underlying doubles differ in the low bits.
		struct Core {
			uint32_t x, y, rotation, playerSpeed;
			uint64_t yVelocity, gravity;
			uint32_t flags;
			uint32_t cameraFlip;
			uint32_t nodeX, nodeY;
			uint8_t  holdingJump;
			uint8_t  pad[3];
		} c{};
		c.x           = probe::bits(p->getPositionX());
		c.y           = probe::bits(p->getPositionY());
		c.rotation    = probe::bits(p->getRotation());
		c.playerSpeed = probe::bits(p->m_playerSpeed);
		c.yVelocity   = probe::bits(p->m_yVelocity);
		c.gravity     = probe::bits(p->m_gravity);
		c.cameraFlip  = probe::bits(m_cameraFlip);
		c.nodeX       = probe::bits(p->getPosition().x);
		c.nodeY       = probe::bits(p->getPosition().y);

		uint32_t f = 0;
		if (p->m_isDead)       f |= 1u << 0;
		if (p->m_isOnGround)   f |= 1u << 1;
		if (p->m_isOnGround2)  f |= 1u << 2;
		if (p->m_isOnGround3)  f |= 1u << 3;
		if (p->m_isOnGround4)  f |= 1u << 4;
		if (p->m_jumpBuffered) f |= 1u << 5;
		if (p->m_isUpsideDown) f |= 1u << 6;
		if (p->m_isShip)       f |= 1u << 7;
		if (p->m_isBird)       f |= 1u << 8;
		if (p->m_isBall)       f |= 1u << 9;
		if (p->m_isDart)       f |= 1u << 10;
		if (p->m_isRobot)      f |= 1u << 11;
		if (p->m_isSpider)     f |= 1u << 12;
		if (p->m_isSwing)      f |= 1u << 13;
		if (p->m_isSideways)   f |= 1u << 14;
		c.flags = f;

		{
			auto it = p->m_holdingButtons.find(kJumpButton);
			c.holdingJump = (it != p->m_holdingButtons.end() && it->second) ? 1 : 0;
		}

		uint64_t h = probe::fnv1a(&c, sizeof(c));

		// Pending input, by meaning only. Order matters (the queue is drained in
		// order) but timestamps must not.
		for (auto const& cmd : m_queuedButtons) {
			const uint8_t sem[3] = {
				static_cast<uint8_t>(cmd.m_button),
				static_cast<uint8_t>(cmd.m_isPush ? 1 : 0),
				static_cast<uint8_t>(cmd.m_isPlayer2 ? 1 : 0),
			};
			h = probe::fnv1a(sem, sizeof(sem), h);
		}
		return h;
	}

	// Compare the clean replay against the solver's own trajectory, one step at
	// a time, and latch the first step that differs.
	//
	// The input-source observer bits are masked: they are suppressed while we
	// inject, so they differ by construction even when the physics match. Only
	// physics state is compared, and the first mismatch is reported field by
	// field with both values - a divergence in y alone reads very differently
	// from one in yVelocity alone.
	void compareAgainstSolvePath(bool pressed, size_t k) {
		auto& st = ProbeState::get();
		if (!m_player1) return;

		const probe::TraceRow b = makeTraceRow(pressed, static_cast<int>(k));
		if (st.verifyPathTrace.size() == k &&
		    st.verifyPathTrace.size() < st.verifyPathTrace.capacity())
			st.verifyPathTrace.push_back(b);

		if (st.solvePathTrace.empty() || k >= st.solvePathTrace.size()) return;
		const probe::TraceRow a = st.solvePathTrace[k];   // solver

		const uint32_t keep = ~(probe::kInputSourceMask | probe::kSolverOnlyMask);
		const bool same =
			a.x == b.x && a.y == b.y && a.rotation == b.rotation &&
			a.playerSpeed == b.playerSpeed && a.yVelocity == b.yVelocity &&
			a.gravity == b.gravity && (a.flags & keep) == (b.flags & keep);

		if (same) {
			// A run of differences just ended. Anything before this healed, so
			// it cannot be what killed the run at the end of the level.
			if (st.verifyPersistStep >= 0) {
				if (st.verifyReconvergeStep < 0)
					st.verifyReconvergeStep = static_cast<int>(k);
				st.verifyPersistStep = -1;
			}
			return;
		}

		st.verifyDivergeCount++;
		if (st.verifyPersistStep < 0) st.verifyPersistStep = static_cast<int>(k);
		if (st.verifyDivergeStep < 0) st.verifyDivergeStep = static_cast<int>(k);
		if (st.verifyDivergeLogged) return;
		st.verifyDivergeLogged = true;

		// Logged exactly once, on the single step it happens. Not in the hot
		// path of a search - this runs only during a verification replay.
		log::error("Verify: FIRST DIVERGENCE at step {} - solver vs clean replay:", k);
		if (a.x != b.x)
			log::error("    x           {:.9g}  vs  {:.9g}   (bits {:08X} vs {:08X})",
			           probe::asFloat(a.x), probe::asFloat(b.x), a.x, b.x);
		if (a.y != b.y)
			log::error("    y           {:.9g}  vs  {:.9g}   (bits {:08X} vs {:08X})",
			           probe::asFloat(a.y), probe::asFloat(b.y), a.y, b.y);
		if (a.rotation != b.rotation)
			log::error("    rotation    {:.9g}  vs  {:.9g}",
			           probe::asFloat(a.rotation), probe::asFloat(b.rotation));
		if (a.playerSpeed != b.playerSpeed)
			log::error("    playerSpeed {:.9g}  vs  {:.9g}",
			           probe::asFloat(a.playerSpeed), probe::asFloat(b.playerSpeed));
		if (a.yVelocity != b.yVelocity)
			log::error("    yVelocity   {:.17g}  vs  {:.17g}   (bits {:016X} vs {:016X})",
			           probe::asDouble(a.yVelocity), probe::asDouble(b.yVelocity),
			           a.yVelocity, b.yVelocity);
		if (a.gravity != b.gravity)
			log::error("    gravity     {:.17g}  vs  {:.17g}",
			           probe::asDouble(a.gravity), probe::asDouble(b.gravity));
		if ((a.flags & keep) != (b.flags & keep))
			log::error("    flags       {:08X}  vs  {:08X}   (differing bits {:08X})",
			           a.flags & keep, b.flags & keep,
			           (a.flags ^ b.flags) & keep);
		log::error("    percent     {:.2f}%  vs  {:.2f}%",
		           probe::asFloat(a.percent), probe::asFloat(b.percent));
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
			// The macro must actually REACH the anchor. best.txt is a
			// savestate-derived path that has never been verified from frame 0,
			// so it can die anywhere - and when it did, this probe silently
			// looped forever on a reset instead of saying so, discarding the
			// anchors it had already measured.
			if (m_player1 && m_player1->m_isDead) {
				log::error("Probe 4b: the macro DIED at step {} ({:.2f}%) before reaching "
				           "anchor {}. Everything above this line is still valid; "
				           "anchors beyond it were never measured.",
				           st.stepCounter, pl->getCurrentPercent(), st.restoreAnchor);
				log::error("  A best.txt path is not verified to replay from frame 0. "
				           "Solve the level and use solution.txt for a full sweep.");
				finishRestoreTest();
				return false;
			}
			if (st.stepCounter < st.restoreAnchor) return true;

			// Reached the anchor: capture with BOTH mechanisms so the same run
			// can be replayed against either.
			if (CheckpointObject* cp = pl->createCheckpoint()) { cp->retain(); st.fullCp = cp; }
			if (PlayerCheckpoint* pc = PlayerCheckpoint::create()) {
				pc->retain();
				m_player1->saveToCheckpoint(pc);
				st.playerCp = pc;
			}
			if (auto* pp = m_player1) {
				st.anchorWasAir = pp->m_isShip || pp->m_isBird || pp->m_isDart || pp->m_isSwing;
				st.anchorMode = pp->m_isShip   ? "SHIP"
				              : pp->m_isBird   ? "UFO"
				              : pp->m_isDart   ? "WAVE"
				              : pp->m_isSwing  ? "SWING"
				              : pp->m_isBall   ? "BALL"
				              : pp->m_isRobot  ? "ROBOT"
				              : pp->m_isSpider ? "SPIDER"
				              :                  "CUBE";
			}
			st.anchorPct = pl->getCurrentPercent();
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
				{
					// The REAL button state, not the scripted input for this step -
					// they are one step out of phase through handleButton, and
					// rebuilding the container from the script is what made the
					// solver's restores wrong in ship mode.
					auto it = m_player1->m_holdingButtons.find(kJumpButton);
					st.aHoldingJump = it != m_player1->m_holdingButtons.end() && it->second;
					st.aIsHolding   = st.isHolding;
					st.aQueuedButtons.assign(m_queuedButtons.begin(), m_queuedButtons.end());
					st.aNodePos = m_player1->getPosition();
					st.aCameraFlip      = m_cameraFlip;
					st.aCameraUnzoomedX = m_cameraUnzoomedX;
					st.aUnk322a         = m_unk322a;
					st.aUnk3251         = m_unk3251;
				}
				st.anchorGameModeChangedTime = m_player1->m_gameModeChangedTime;
				st.anchorUnkA29              = m_player1->m_unkA29;
				st.aLastJumpTime             = m_player1->m_lastJumpTime;
				st.aPlayerPos                = m_player1->m_position;
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
			// Built once, for whichever mechanism runs. Previously this lived
			// inside the Full branch, so DirectLoad silently read a stale,
			// invalid struct and restored none of it - making the comparison
			// meaningless rather than merely unfavourable.
			{
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
				e.cameraFlip      = st.aCameraFlip;
				e.cameraUnzoomedX = st.aCameraUnzoomedX;
				e.unk322a         = st.aUnk322a;
				e.unk3251         = st.aUnk3251;
				e.valid       = true;
				Solver::get().pendingExtra = e;
			}

			// Restore and replay the identical input from the anchor.
			if (st.restoreKind == RestoreKind::DirectLoad) {
				// No respawn, no death: reposition by writing the checkpoint back
				// directly. Level state still comes from the full CheckpointObject,
				// so triggers and moving objects are preserved.
				//
				// NOTE: no realign step. The respawn path lands one frame EARLY and
				// needs a forward correction; a direct load lands exactly on the
				// captured state, which is the first evidence that the ship/UFO/wave
				// approximation lives in the respawn rather than in the checkpoint.
				if (st.fullCp) {
					st.suppressDeath = true;

					// Reset to canonical FIRST, then apply the checkpoint directly.
					//
					// The two mechanisms have complementary flaws: the practice
					// respawn wipes everything via resetLevel but then places the
					// player with a fudge (a frame early, and "a set distance behind
					// the icon" in air modes); a bare loadFromCheckpoint places
					// exactly but applies on top of whatever the last 400 steps
					// left dirty - which is why m_lastJumpTime and m_position were
					// still stale.
					//
					// Emptying the checkpoint array makes resetLevel go to level
					// start rather than respawning to a checkpoint, so we get the
					// full wipe with none of the respawn's positioning logic, and
					// then write the exact captured state on top of a clean slate.
					if (auto* arr = pl->m_checkpointArray) arr->removeAllObjects();
					st.solverRestoring = true;
					pl->resetLevel();
					st.solverRestoring = false;

					pl->loadFromCheckpoint(st.fullCp);
					st.suppressDeath = false;

					// Button state FIRST - see solverRestoreState. pushButton and
					// releaseButton run game logic, so they must not follow the
					// snapshot. This probe missed the resulting m_jumpBuffered
					// corruption at all 8 anchors purely because none of them
					// landed on a step with a jump buffered; the solver hit it on
					// its first restore.
					if (auto* p = m_player1) {
						st.injecting = true;
						if (st.aHoldingJump) p->pushButton(PlayerButton::Jump);
						else                 p->releaseButton(PlayerButton::Jump);
						st.injecting = false;
					}

					// Write back everything the restore leaves untouched.
					if (auto* p = m_player1 ; p && !st.anchorBytes.empty()) {
						auto* dst = reinterpret_cast<uint8_t*>(p);
						for (auto const& f : kPlayerFields)
							std::memcpy(dst + f.off, st.anchorBytes.data() + f.off, f.size);
					}

					// The container the field table cannot reach.
					if (auto* p = m_player1) p->m_holdingButtons[kJumpButton] = st.aHoldingJump;
					st.isHolding = st.aIsHolding;
					m_queuedButtons.clear();
					for (auto const& c : st.aQueuedButtons) m_queuedButtons.push_back(c);
					if (auto* p = m_player1) p->setPosition(st.aNodePos);

					// Apply the same extra state the respawn path restores, or this
					// is being compared against a mechanism doing strictly more work.
					auto& sv2 = Solver::get();
					if (sv2.pendingExtra.valid) {
						applyExtraState(sv2.pendingExtra);
						sv2.pendingExtra.valid = false;
					}
				}
			} else if (st.restoreKind == RestoreKind::Full) {
				// The practice-respawn path, exactly as the solver restores.
				// Testing loadFromCheckpoint instead would measure a path the
				// solver never takes.
				const size_t prev = st.restoreAnchor > 0
				                  ? static_cast<size_t>(st.restoreAnchor - 1) : 0;
				const bool realign = prev < st.scripted.size() && st.scripted[prev] != 0;
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

	// Turn a decision's choice into actual button state. Tap modes start a
	// self-releasing countdown; everything else sets a persistent hold.
	void applyDecisionChoice(Decision const& d) {
		auto& sv = Solver::get();
		if (d.modeClass == ModeClass::Tap) {
			sv.tapping      = d.choice;
			sv.tapRemaining = d.choice ? g_config.tapLengthSteps : 0;
			sv.hold         = d.choice;
		} else {
			sv.tapping      = false;
			sv.tapRemaining = 0;
			sv.hold         = d.choice;
		}
	}

	// Capture everything a restore needs. THE single capture path.
	void captureRestoreState(RestoreState& r) {
		auto* pl = PlayLayer::get();
		if (CheckpointObject* cp = pl ? pl->createCheckpoint() : nullptr) {
			cp->retain();
			r.cp = cp;
		}
		if (auto* pp = m_player1) {
			auto const* raw = reinterpret_cast<uint8_t const*>(pp);
			r.playerBytes.assign(raw, raw + sizeof(PlayerObject));
			auto it = pp->m_holdingButtons.find(kJumpButton);
			r.holdingJump    = it != pp->m_holdingButtons.end() && it->second;
			r.probeIsHolding = ProbeState::get().isHolding;
			r.nodePos        = pp->getPosition();
			r.gameModeChangedTime = pp->m_gameModeChangedTime;
			r.unkA29              = pp->m_unkA29;
		}
		r.queuedButtons.assign(m_queuedButtons.begin(), m_queuedButtons.end());
		r.layerExtraDelta  = m_extraDelta;
		r.layerTimePlayed  = m_timePlayed;
		r.layerTimestamp   = m_timestamp;
		r.layerTickIndex   = m_tickIndex;
		r.layerClickIndex  = m_clickIndex;
		r.layerResumeTimer = m_resumeTimer;
		r.layerJumping     = m_jumping;
		r.layerCameraFlip      = m_cameraFlip;
		r.layerCameraUnzoomedX = m_cameraUnzoomedX;
		r.layerUnk322a         = m_unk322a;
		r.layerUnk3251         = m_unk3251;
		if (pl) {
			r.layerAttemptTime     = pl->m_attemptTime;
			r.layerBestAttemptTime = pl->m_bestAttemptTime;
			r.layerCurrentTime     = pl->m_currentTime;
			r.layerHasJumped       = pl->m_hasJumped;
		}
	}

	// Push a decision at the current state and take the first branch.
	void solverPushDecision() {
		auto& sv = Solver::get();
		auto* pl = PlayLayer::get();

		Decision d;
		d.step = sv.step;

		// Gravity-relative fall speed: positive means moving the way gravity
		// pulls, so the test reads the same upright and inverted. m_yVelocity is
		// in world coordinates, so the sign flips with m_isUpsideDown.
		if (auto* p = m_player1) {
			d.modeClass = classifyMode(p);
			d.airPolicy = d.modeClass != ModeClass::Ground;
			const double fall = p->m_isUpsideDown ?  static_cast<double>(p->m_yVelocity)
			                                      : -static_cast<double>(p->m_yVelocity);
		}
		// EVERY decision gets a checkpoint now, air included. Air decisions used
		// to be skipped because restoring to one was unreliable (Probe 4b under
		// the practice respawn: AIR 1/3 bit-identical), so they were reached by
		// replaying forward from the nearest cube ancestor. The direct-load
		// restore is exact in every mode (8/8), so that replay is pure cost:
		// backtracking into a ship or UFO section is now O(1) instead of
		// re-simulating the whole section. The price is ~22 KB per air decision.
		// Under hybridRestore an air decision is never restored to directly, so
		// capturing one is ~22 KB and a createCheckpoint call wasted per decision
		// - and air sections have by far the most decisions.
		const bool needsCheckpoint = !g_config.noSavestates &&
		                             !(g_config.hybridRestore && d.airPolicy);
		if (needsCheckpoint) captureRestoreState(d.rs);
		d.enteringHold  = sv.hold;
		d.enteringAirMode  = sv.prevAirMode;
		// NOT sv.prevOnGround. That is assigned AFTER this push, so reading it here
		// gives the previous ground-policy step, while a step resuming from this
		// decision must compare against the CURRENT one. prevAirMode does not have
		// this problem: it is only written inside the transition branch, which fires
		// whenever it disagrees with the mode, so it always equals the mode at this
		// step by the time a decision is pushed.
		d.enteringOnGround = m_player1 ? m_player1->m_isOnGround : false;
		d.enteringGeoLo = sv.lastGeoLo;
		d.enteringGeoHi = sv.lastGeoHi;
		d.togglesBefore = d.airPolicy ? sv.togglesUsed : 0;
		d.tapsBefore    = d.airPolicy ? sv.tapsUsed    : 0;

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
			// Tap modes have no "continue": a tap self-releases, so sv.hold at
			// push time is carried-over state rather than a sustained action.
			// Ordering by whether a tap is needed is the physical reading of the
			// same decision, and by decisionCost's asymmetry it also makes the
			// no-tap fallback free instead of budget-gated.
			//
			// Deliberately velocity-only. The earlier physics rule that failed
			// keyed on m_isOnGround, which is true against the CEILING in ship
			// mode and so held the ship harder into it; velocity has no such
			// second meaning, and this is confined to Tap so ship is untouched.
			// Geometry first, when it has something to say. Climbing means hold
			// in a Hold mode and a tap in a Tap mode, so one sign drives both.
			int steer = 0;
			if (g_config.geometryOrdering && GeoMap::get().valid) {
				sv.geoLooks++;
				// GD's mini is vehicleSize 0.6 against 1.0 normal; the death log
				// prints the observed value so the threshold can be checked
				// rather than assumed.
				const bool mini = m_player1->m_vehicleSize < 0.9f;
				const double lead =
					(d.modeClass == ModeClass::Tap) ? g_config.geomLeadTap
					: mini                          ? g_config.geomLeadMini
					                                : g_config.geomLeadHold;
				const double lookaheadUnits =
					g_config.geomLookaheadScaleWithSpeed
						? static_cast<double>(g_config.geomLookaheadSteps) * sv.dxPerStep
						: static_cast<double>(g_config.geomLookaheadFixedX);
				sv.steerInfo = SteerInfo{};
				sv.steerInfo.yNow = m_player1->getPositionY();
				sv.steerPlayer = m_player1;
				steer = geometrySteer(m_player1->getPositionX(),
				                      m_player1->getPositionY(),
				                      static_cast<double>(m_player1->m_yVelocity),
				                      lead, lookaheadUnits);
				if (g_config.geomSteerSilent) steer = 0;
				if (steer != 0) sv.geoSteers++;
			}
			d.si   = sv.steerInfo;
			d.vyAt = static_cast<double>(m_player1->m_yVelocity);
			if (steer != 0) {
				d.hasSteer = true;
				// geometrySteer speaks in WORLD y: +1 means get higher. Hold and
				// tap thrust toward the player's own up, which is world-DOWN
				// under inverted gravity - so which branch climbs flips with the
				// gravity direction.
				//
				// MEASURED CONSEQUENCE of assuming hold always climbs: Cycles has
				// a ship stretch alternating normal and reversed gravity portals
				// between spikes. There the steer pushed AWAY from the window,
				// which put the player further outside it, which made it steer
				// again - 27888 steers from 67978 air decisions (41%), against
				// 0.74% on a level it solves, and a hard stall at 58.12%.
				const bool climb = steer > 0;
				d.choice = m_player1->m_isUpsideDown ? !climb : climb;
				d.steerChoice = d.choice;
			} else {
				d.choice = sv.hold;
			}
		} else {
			// Cube: no-press is the likelier correct branch (~10% jump rate in
			// the BC dataset), and this policy already clears 35% of the level.
			d.choice = false;
		}
		d.tried = d.choice ? (1u << 1) : (1u << 0);

		sv.stack.push_back(d);
		applyDecisionChoice(d);
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
		for (size_t i = sv.stack.size(); i-- > 0 && !sv.anchorReleased; ) {
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
		if (sv.lookbackSteps <= 0) sv.lookbackSteps = g_config.commitLookbackSteps;

		// Clamp the window so a committed prefix always survives. Widening is
		// meant to reach further back within the explored region, not to abolish
		// commitment - and an absolute step count does exactly that early in a
		// level, where the window can exceed all progress made.
		// Only clamp a WIDENED window. Applied unconditionally it also bit the
		// default early in a level (bestStep < 2x window), quietly tightening
		// commitment in the first couple of seconds - a behaviour change nobody
		// asked for. The guard exists to stop escalation abolishing commitment,
		// so it should only constrain escalation.
		int window = sv.lookbackSteps;
		if (sv.lookbackSteps > g_config.commitLookbackSteps) {
			const int maxWindow =
				static_cast<int>(sv.bestStep * (1.0 - g_config.minCommittedFraction));
			window = std::max(g_config.commitLookbackSteps, std::min(window, maxWindow));
		}

		if (sv.bestStep > window) {
			const int cutoff = sv.bestStep - window;
			size_t windowIdx = 0;
			while (windowIdx < sv.stack.size() && sv.stack[windowIdx].step < cutoff) windowIdx++;
			floorIdx = std::max(floorIdx, windowIdx);
		}

		// Never freeze the whole stack.
		const size_t maxFloor = sv.stack.size() > 16 ? sv.stack.size() - 16 : 0;
		return std::min(floorIdx, maxFloor);
	}

	// Restore to a decision's captured state.
	//
	// HISTORY, because the shape of this function is not obvious. It used to
	// drive practice mode's own respawn (checkpoint into m_checkpointArray, then
	// resetLevel), which revives a dead player and repositions in one call - but
	// the respawn lands one step EARLY and needed a forward correction step, and
	// in ship/UFO/wave it is approximate BY DESIGN: the GD wiki states air-mode
	// checkpoints are generated "a set distance behind the icon" to account for
	// momentum. Probe 4b measured exactly that: CUBE 5/5 bit-identical, AIR 1/3.
	//
	// The replacement resets to level start (checkpoint array emptied, so the
	// respawn logic is never involved), calls loadFromCheckpoint DIRECTLY, and
	// then writes back the 196 PlayerObject fields the direct load leaves
	// untouched. Exact in every mode: Probe 4b re-run gave CUBE 5/5, AIR 3/3.
	//
	// The reset is retained deliberately. It is what revives the dead player the
	// solver is always backtracking from, and it is what returns LEVEL state -
	// objects, triggers, the effect manager - to canonical before the checkpoint
	// is applied on top. loadFromCheckpoint is sparse over PlayLayer's tracked
	// object lists and no vanilla path ever calls it without a preceding reset;
	// dropping it is a measurable speedup (restore cost scales with level size,
	// 1.06 ms on Stereo Madness against 40.9 ms on Hypersonic) but it has not
	// been validated on a level with triggers or moving objects.
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
		m_cameraFlip      = e.cameraFlip;
		m_cameraUnzoomedX = e.cameraUnzoomedX;
		m_unk322a         = e.unk322a;
		m_unk3251         = e.unk3251;
		if (auto* pl2 = PlayLayer::get()) {
			pl2->m_attemptTime     = e.attemptTime;
			pl2->m_bestAttemptTime = e.bestAttemptTime;
			pl2->m_currentTime     = e.currentTime;
			pl2->m_hasJumped       = e.hasJumped;
		}
	}

	// PROBE ONLY. The practice-respawn restore, kept as the control arm of the
	// F12 sweep so the direct-load path can be measured against the mechanism it
	// replaced. The solver no longer uses it - see solverRestoreState below.
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
		if (auto* p = m_player1) {
			ps.injecting = true;
			if (realignHold) p->pushButton(PlayerButton::Jump);
			else             p->releaseButton(PlayerButton::Jump);
			ps.injecting  = false;
			ps.isHolding  = realignHold;
		}

		// The respawn lands one step EARLY - segment B was offset from segment A
		// by exactly one step, with B[n+1] == A[n] bit for bit. One forward step
		// reproduces the captured state. The direct-load path needs no such
		// correction, which is the cleanest evidence that the air-mode
		// approximation lives in the respawn rather than in the checkpoint.
		applyInput(realignHold);
		GJBaseGameLayer::update(static_cast<float>(kPhysicsDt));

		// Applied AFTER the realign: writing the step-S values BEFORE it meant
		// that step advanced them past S and ran its physics on rolled-back
		// time, which made the PlayerObject delta worse, 24 bytes to 99.
		{
			auto& sv2 = Solver::get();
			if (sv2.pendingExtra.valid) {
				applyExtraState(sv2.pendingExtra);
				sv2.pendingExtra.valid = false;
			}
		}
	}

	void applyRestoreState(RestoreState const& r, bool enteringHold) {
		auto* pl = PlayLayer::get();
		if (!pl || !r.cp) return;
		auto& ps = ProbeState::get();

		// Nothing may die during the restore: the reset and the load both move
		// the player, and a death mid-restore would drag the practice respawn -
		// the approximate mechanism this path exists to avoid - back in.
		ps.suppressDeath = true;

		// Emptying the checkpoint array makes resetLevel go to level START
		// rather than respawning to a checkpoint, so we get the full wipe with
		// none of the respawn's positioning fudge, and revive the dead player.
		if (auto* arr = pl->m_checkpointArray) arr->removeAllObjects();
		ps.solverRestoring = true;
		pl->resetLevel();
		ps.solverRestoring = false;

		pl->loadFromCheckpoint(r.cp);
		ps.suppressDeath = false;

		// The held-button state, which no checkpoint stores. The captured state
		// is the one AFTER the step that ran with `enteringHold` applied, so that
		// is the button state it must come back with.
		//
		// THIS MUST RUN BEFORE THE FIELD WRITE-BACK. pushButton and releaseButton
		// are not passive setters - they execute game logic. Running them after
		// the snapshot was stamped meant releaseButton cleared m_jumpBuffered on
		// every restore that entered on a release, so a jump buffered at capture
		// time was silently dropped. MEASURED: the first divergence between the
		// search and a clean replay of its own solution was exactly that bit, at
		// the very first step after a restore, and the run then flew a subtly
		// wrong trajectory for 18,473 further steps before it hit anything.
		//
		// Calling it first and stamping the snapshot on top gives both halves:
		// m_holdingButtons (a container, so outside the field table) is set by
		// the real call, and every scalar it touches is then overwritten with
		// the captured value.
		if (auto* p = m_player1) {
			ps.injecting = true;
			if (enteringHold) p->pushButton(PlayerButton::Jump);
			else                p->releaseButton(PlayerButton::Jump);
			ps.injecting = false;
			ps.isHolding = enteringHold;
		}

		// The step after a reposition has no valid predecessor: pairing across
		// one would record a "transition" between two unrelated states.
		Solver::get().motionHavePrev = false;

		// Everything the direct load leaves untouched. Authoritative: this is the
		// last thing to write PlayerObject, so nothing above can corrupt it.
		if (auto* p = m_player1; p && !r.playerBytes.empty()) {
			auto* dst = reinterpret_cast<uint8_t*>(p);
			for (auto const& f : kPlayerFields)
				std::memcpy(dst + f.off, r.playerBytes.data() + f.off, f.size);
		}

		// And the containers the field table cannot reach, written directly so no
		// game logic gets a chance to re-derive them from the wrong phase.
		if (auto* p = m_player1) p->m_holdingButtons[kJumpButton] = r.holdingJump;
		ps.isHolding = r.probeIsHolding;

		// The layer's pending input queue. resetLevel cleared it and nothing else
		// puts it back, so restore it to exactly what was in flight at capture.
		m_queuedButtons.clear();
		for (auto const& c : r.queuedButtons) m_queuedButtons.push_back(c);

		// The cocos node transform, which no field table can reach.
		if (auto* p = m_player1) p->setPosition(r.nodePos);

		// Layer and PlayLayer state no checkpoint carries. Applied directly
		// rather than deferred through pendingExtra: the respawn path had to
		// wait for its realign step to finish, and there is no realign step now.
		{
			Solver::PendingExtra e;
			e.gameModeChangedTime = r.gameModeChangedTime;
			e.unkA29          = r.unkA29;
			e.extraDelta      = r.layerExtraDelta;
			e.timePlayed      = r.layerTimePlayed;
			e.timestamp       = r.layerTimestamp;
			e.tickIndex       = r.layerTickIndex;
			e.clickIndex      = r.layerClickIndex;
			e.resumeTimer     = r.layerResumeTimer;
			e.jumping         = r.layerJumping;
			e.attemptTime     = r.layerAttemptTime;
			e.bestAttemptTime = r.layerBestAttemptTime;
			e.currentTime     = r.layerCurrentTime;
			e.hasJumped       = r.layerHasJumped;
			e.cameraFlip      = r.layerCameraFlip;
			e.cameraUnzoomedX = r.layerCameraUnzoomedX;
			e.unk322a         = r.layerUnk322a;
			e.unk3251         = r.layerUnk3251;
			applyExtraState(e);
		}

	}

	// Restore to a decision. Thin wrapper over the shared primitive so the DFS
	// and the beam cannot drift apart the way two hand-written copies did.
	void solverRestoreState(Decision& d) {
		if (!d.rs.cp) return;
		applyRestoreState(d.rs, d.enteringHold);
		Solver::get().restores++;
		Solver::get().pendingPostRestore = true;
	}













	// Has the run reached the next decision point?
	//
	// Deliberately identical to the DFS's branching policy, including both
	// interval constants, and shared by every search that needs it. In Ground
	// modes a decision happens on LANDING or a ring touch - an event, not a
	// clock - because that is the only moment the button can do anything. In air
	// modes the trajectory responds continuously, so decisions fall on a fixed
	// grid instead.
	bool atDecisionPoint(int steps, bool& prevGround) {
		auto* p = m_player1;
		if (!p) return true;
		if (classifyMode(p) != ModeClass::Ground)
			return steps >= g_config.airBranchInterval;

		const bool onGround = p->m_isOnGround;
		const bool onRing   = p->m_touchingRings && p->m_touchingRings->count() > 0;
		const bool landed   = onGround && !prevGround;
		prevGround = onGround;
		return landed || onRing || steps >= g_config.groundBranchInterval;
	}













	// --- Go-Explore ---------------------------------------------------------

	// Which archive cell the CURRENT state falls in.
	//
	// Coarser than solverStateKey on purpose. The key question a cell answers is
	// "have I been somewhere like here", not "have I been exactly here" - exact
	// keys never repeat in continuous air state, which is what made the beam's
	// dedup freeze on Theory of Everything. Position and mode say where you are;
	// vertical velocity is included because arriving at the same point rising
	// and falling are completely different situations, and collapsing them would
	// throw away the distinction that decides whether a gap is clearable.
	// The cell key survived Go-Explore's removal: the escape archive and the
	// cell census both key on it, and they are independent of that searcher.
	// The go* prefix and the goCellX/Y/Vy constants are kept rather than renamed
	// so the history stays greppable.
	uint64_t goCellKey() {
		if (!m_player1) return 0;
		const CellCoords c = goCellCoords();
		return probe::fnv1a(&c, sizeof(c));
	}

	CellCoords goCellCoords() {
		CellCoords c{};
		auto* p = m_player1;
		if (!p) return c;
		c.x    = static_cast<int32_t>(std::floor(p->getPositionX() / std::max(1.0, g_config.goCellX)));
		c.y    = static_cast<int32_t>(std::floor(p->getPositionY() / std::max(1.0, g_config.goCellY)));
		{
			const double v = p->m_yVelocity;
			const double r = v < 0.0 ? -std::sqrt(-v) : std::sqrt(v);
			c.vy = static_cast<int32_t>(std::floor(r / std::max(0.05, g_config.goCellVy)));
		}
		c.mode = static_cast<uint32_t>(classifyMode(p));
		uint32_t f = 0;
		if (p->m_isUpsideDown) f |= 1u << 0;  // gravity direction changes everything
		if (p->m_isOnGround)   f |= 1u << 1;
		c.flags = f;
		return c;
	}


	// Census one step of the DFS. One hash and one map lookup; no checkpoint,
	// no allocation past the map's own growth, and nothing read back.
	// One step of the corridor sweep. Returns false when the variant is over,
	// which is also when the reset for the next one is queued.
	bool sweepStep() {
		auto& sw = Sweep::get();
		auto& st = ProbeState::get();
		auto* pl = PlayLayer::get();
		if (!pl || !m_player1) return false;

		const bool dead    = m_player1->m_isDead;
		const bool ranOut  = sw.step >= static_cast<int>(sw.macro.size());
		if (st.finished || dead || ranOut) {
			Sweep::Result r{};
			r.S       = sw.variantS();
			r.k       = sw.variantK();
			r.endStep = sw.step;
			r.maxY    = sw.maxY;
			r.pct     = sw.bestPct;
			r.died    = dead;
			r.cleared = st.finished;
			sw.results.push_back(r);

			sw.variant++;
			if (sw.variant >= sw.variantCount()) {
				sw.running = false;
				st.mode    = Mode::Idle;
				sweepWriteResults();
				return false;
			}
			sweepBuildMacro();
			sw.step = 0; sw.bestPct = 0.f; sw.maxY = -9999;
			st.resetPending = true;
			return false;
		}

		const bool pressed = sw.macro[static_cast<size_t>(sw.step)] != 0;
		applyInput(pressed);
		GJBaseGameLayer::update(static_cast<float>(kPhysicsDt));
		sw.step++;

		const float pct = pl->getCurrentPercent();
		if (pct > sw.bestPct) sw.bestPct = pct;
		if (m_player1->getPositionX() >= g_config.sweepCorridorX) {
			const int yb = static_cast<int>(
				std::floor(m_player1->getPositionY() / std::max(1.0, g_config.goCellY)));
			if (yb > sw.maxY) sw.maxY = yb;
		}
		return true;
	}

	// The reachable roof for right now, from the game's own portal anchor.
	// Recomputed every step: m_portalY changes the instant a portal is entered,
	// so nothing has to be remembered or forgotten.
	void ceilingUpdate(double vyBefore) {
		auto* p = m_player1;
		auto& sv = Solver::get();
		sv.learnedCeiling = 1e30;
		// The MEASUREMENT must not depend on the clamp. Gating both on one flag
		// meant turning the clamp off silently turned the probe off too, and a
		// whole run came back with "no ceiling contact seen".
		if (!p || p->m_isDead) return;

		// GD says directly whether this mode is bounded, so ask it instead of
		// inferring from the mode class. MEASURED: usingWallLimitedMode() read 1
		// on every ship ceiling contact across two custom levels.
		if (!p->usingWallLimitedMode()) return;
		if (m_gameState.m_isFreeMode) return;

		// Only while the mode still matches the one entered - a restore can land
		// the search before the portal with the entry still recorded.
		// NOT gated on ceilingClamp. The read is one memcpy, and the roof safety
		// net below depends on it - gating both on one flag meant the check could
		// only ever fire when the crop was also on, so a Z-only run reported
		// "0 roof warnings" when it had simply never looked.
		//
		// Preferred: read it. Fallback: derive it.
		const bool canRead = g_config.ceilingFromField && sv.ceilFieldOk;
		double fromField = 0.0;
		if (canRead) {
			float f = 0.f;
			std::memcpy(&f, reinterpret_cast<const uint8_t*>(this)
			                + g_config.geomCeilingFieldOffset, sizeof f);
			fromField = static_cast<double>(f);
		}
		// A restore can land the search BEHIND the portal whose entry is still
		// recorded. playerWillSwitchMode only fires going forward, so nothing
		// un-sets the record when the search rewinds through a portal, and the
		// rule then answers for a section the player is no longer in.
		//
		// MEASURED on Clutterfunk: at x 15765 the record named the ship portal at
		// x 15914 - 149 units AHEAD - while the player was back in the ball
		// section that runs 10796..15868. The ground clamp took the ship band and
		// answered 390 where the field read 330.
		//
		// Being behind your own entry portal is provable rather than guessed, so
		// this drops the record instead of clearing on every restore - the search
		// restores thousands of times a second, and blanket clearing would leave
		// haveDerived false almost always and silence the cross-check entirely.
		// Stepping forward re-enters the portal and the hook re-arms it.
		if (sv.haveEntryPortal &&
		    p->getPositionX() < sv.entryPortalX - g_config.geomRoofLeadX) {
			sv.haveEntryPortal = false;
			sv.entryPortalMode = -1;
			sv.entryPortalID   = 0;
			sv.entryStaleDrops++;
		}

		const bool haveDerived = sv.haveEntryPortal &&
		                         static_cast<int>(classifyMode(p)) == sv.entryPortalMode;
		const double derived = haveDerived
		                     ? Solver::ceilingForPortal(sv.entryPortalY,
		                                                sv.entryPortalID) : 0.0;

		if (canRead && std::isfinite(fromField) && fromField > 0.0) {
			sv.learnedCeiling = fromField;
			// Cross-check. The two were built from the same eight contacts, so a
			// disagreement means the field moved or the rule does not generalise -
			// either way it must be loud, not silent.
			// Not across a portal transition. The two sources are sampled at
			// different instants and genuinely disagree for a few steps either
			// side of a portal, in BOTH directions - measured:
			//
			//   BAB x 17960, portal centre 17985: the record had ALREADY moved to
			//     the cube portal (rule 420) while the field still read the ship
			//     section it had not yet left (390). playerWillSwitchMode fires on
			//     hitbox contact, up to ~31 units early.
			//   Clubstep x 15746, portal centre 15741: the mirror case, the record
			//     promoted and the field still reporting the outgoing section.
			//
			// Both were reported as `delta 30` and `delta 60` disagreements and
			// neither was one - their implied bands came out 240 and 400 for the
			// same objID, which is how you know both were artefacts. Distance
			// rather than a step count because dx per step varies with speed, and
			// geomRoofLeadX because it is the same lead the roof already corrects
			// for.
			const bool nearPortal =
				sv.haveEntryPortal &&
				std::abs(p->getPositionX() - sv.entryPortalX) < g_config.geomRoofLeadX;
			if (haveDerived && !nearPortal) {
				const double d = std::abs(fromField - derived);
				// Was 30.0, and the ball offset error was EXACTLY 30.0, so the
				// guard missed the one defect it existed to catch. Half a block
				// now: the rule predicts every measured contact exactly, so
				// anything approaching a block is worth hearing about.
				if (d > 15.0) {
					sv.ceilDisagreed++;
					if (d > sv.ceilWorstDelta) {
						sv.ceilWorstDelta = d;
						// Both disagreements ever seen at this threshold read
						// `delta exactly 30` - ToE x 24461 and Electroman x 10481 -
						// which is (300 - 240) / 2, the gap between the ship offset
						// and the ball one. In both the portal Y was right and the
						// band was wrong, so the message has to say which portal it
						// thinks it is in. Two very different faults produce the same
						// delta and the old message could not separate them:
						//
						//   a STALE entryPortalID - the search restored to before a
						//     portal it had already entered, so the id names a section
						//     the player is no longer in;
						//   an UNMEASURED band - wave, swing, robot and spider are all
						//     still assumed 300, and until this threshold dropped to 15
						//     a wrong assumption there was completely silent.
						//
						// The implied numbers separate them: invert the rule against
						// the field and the band it asks for is either one we already
						// know (stale id) or one we have never measured.
						const double impliedOff  = fromField - sv.entryPortalY;
						const double impliedBand = 2.0 * (impliedOff + 15.0);
						const bool   atClamp     =
							std::abs(derived - (g_config.geomGroundY + sv.entryBand()))
							< 0.5;
						log::warn("Ceiling: field says {:.1f}, rule says {:.1f} "
						          "(delta {:.1f}) at x {:.0f}. Entry portal objID {} "
						          "y {:.1f}, band {:.0f}, offset {:.0f}; mode {} now "
						          "vs {} at entry. The field implies offset {:.1f} "
						          "and band {:.1f}{}.",
						          fromField, derived, d, p->getPositionX(),
						          sv.entryPortalID, sv.entryPortalY, sv.entryBand(),
						          ceilingOffsetForPortal(sv.entryPortalID),
						          modeClassName(static_cast<int>(classifyMode(p))),
						          modeClassName(sv.entryPortalMode),
						          impliedOff, impliedBand,
						          atClamp
						              ? " - but the rule is sitting on the ground clamp "
						                "here, so the implied pair describes the clamp "
						                "and not the portal"
						              : "");
					}
				}
			}
		} else if (haveDerived) {
			sv.learnedCeiling = derived;
		}

		if (sv.learnedCeiling < 1e29) {
		}

		// Probe 15 lived here: a log::info on every CHANGE of the ground-truth
		// ceiling, to find out which object types re-snap the band. It answered
		// that - gravity portals were added on that evidence and then removed
		// again when they cost Base After Base its roof - and is now deleted.
		//
		// It also had to go. It was a log call in the physics stepping path, which
		// this project forbids, and the reason showed up the moment the search
		// started re-crossing one spot: Clutterfunk logged 1152 CEILING CHANGED
		// lines in a single run, all at x 10784..10796, flapping 390 <-> 330. That
		// is not a fault - two sections meet there and every restore across the
		// boundary re-reads the other one's band. The measurement was right and the
		// logging was wrong.

		// SAFETY NET for the map roof.
		//
		// The roof is derived from portal positions and is only as good as the
		// list of portal IDs that re-snap the band. A missing ID makes the roof
		// too LOW, which silently deletes reachable space - Clutterfunk lost ~46
		// units that way, and the only symptom was a stall. The ceiling read at
		// 0x764 is ground truth, so compare them and say so loudly.
		// Compared PER X, not against yHi. yHi is the top of the whole map, and Z
		// crops per segment, so the old global form could not see a local crop at
		// all - on Time Machine yHi is 810 while the roof pins the back half at
		// 390, and the check sat silent through a 96.88% stall.
		//
		// segRoof already equals yHi everywhere when Z is off, so this subsumes
		// the global check rather than sitting beside it.
		if (sv.learnedCeiling < 1e29) {
			auto const& gm = GeoMap::get();
			const int si = gm.valid ? gm.segIndex(p->getPositionX()) : -1;
			if (si >= 0 && si < static_cast<int>(gm.segRoof.size())) {
				const double roofHere = gm.segRoof[static_cast<size_t>(si)];
				const double margin   = roofHere - sv.learnedCeiling;
				sv.roofMarginMax = std::max(sv.roofMarginMax, margin);
				sv.roofMarginSum += margin;
				sv.roofMarginN++;
				if (margin < sv.roofMarginMin) {
					sv.roofMarginMin  = margin;
					sv.roofMarginMinX = p->getPositionX();
					// Only a NEW WORST prints, so this is bounded and cannot flood the
					// stepping path - the same discipline as ceilWorstDelta above.
					if (margin < -1.0) {
						sv.roofWarned = true;
						log::error("MAP ROOF TOO LOW at x {:.0f}: the real ceiling here "
						           "is {:.0f} but the map roof is {:.0f}, so {:.0f} units "
						           "of reachable space have been deleted here. Either a "
						           "portal that re-snaps the band is missing from the roof "
						           "scan, or MOST RECENT (Z=2) took a lower cap than the "
						           "section actually has.",
						           sv.roofMarginMinX, sv.learnedCeiling, roofHere, -margin);
					}
				}
			}
		}

		// Is the player inside the band we just computed? It always must be.
		if (sv.learnedCeiling < 1e29) {
			const cocos2d::CCRect r = p->getObjectRect();
			const double lo = sv.learnedCeiling - sv.entryBand();
			const double out = std::max(lo - r.origin.y,
			                            (r.origin.y + r.size.height) - sv.learnedCeiling);
			if (out > 1.0) {
				sv.ceilOutside++;
				if (out > sv.ceilWorstOut) {
					sv.ceilWorstOut = out;
					log::warn("Ceiling band WRONG: player y {:.1f}..{:.1f} is {:.1f} "
					          "outside band [{:.1f}, {:.1f}] at x {:.0f} - the clamp is "
					          "cropping space the player is standing in.",
					          r.origin.y, r.origin.y + r.size.height, out,
					          lo, sv.learnedCeiling, p->getPositionX());
				}
			} else {
				sv.ceilInside++;
			}
		}

		// Probe 10: is that offset right? A contact is the roof announcing
		// itself - rising, velocity forced to zero, and no object there to have
		// done it. Recording contactY - m_portalY says whether one constant
		// covers every section.
		const double worldUp = p->m_isUpsideDown ? -vyBefore : vyBefore;
		if (worldUp <= 0.5 || std::abs(p->m_yVelocity) > 1e-6) return;
		const double y = p->getPositionY();

		// Test the player's TOP EDGE, not its centre.
		//
		// This is what contaminated the first measurement. The player collides
		// with its roof, about 15 units above its centre, so a stop against a
		// block at 360 leaves the centre at ~345 - inside no blocked span - and
		// a centre test wrongly calls it an invisible ceiling. MEASURED: the
		// "ceilings" came back as 345 at x 17940 and 375 at x 17880, adjacent
		// bands one block apart, which no camera ceiling does. They were the
		// blocks at y 330 and 360 that the object dump shows there.
		const cocos2d::CCRect pr = p->getObjectRect();
		const double top = pr.origin.y + pr.size.height;
		const double px  = p->getPositionX();
		for (double probe = 0.0; probe <= 8.0; probe += 4.0)
			if (geometryBlockedAt(px, top + probe)) return;

		const double off = y - static_cast<double>(m_gameState.m_portalY);
		sv.ceilOffMin = std::min(sv.ceilOffMin, off);
		sv.ceilOffMax = std::max(sv.ceilOffMax, off);
		sv.ceilOffN++;

		// Probe 11. Everything still unexcluded, measured against the same
		// contact so one run settles it.
		{
			double c[Solver::kCeilCand];
			// getCameraEdgeValue's type selector is undocumented, so sweep it.
			for (int t = 0; t < 6; t++) c[t] = getCameraEdgeValue(t);
			c[6]  = p->m_collidedTopMinY;
			c[7]  = p->m_collidedBottomMaxY;
			c[8]  = m_maxGameplayY;
			c[9]  = p->m_yStart;
			c[10] = p->m_fallStartY;
			c[11] = p->m_isOutOfBounds ? 1.0 : 0.0;
			// GD draws the ship ceiling as a SECOND ground layer - it is the
			// thing you can actually see closing the section in. If its y is the
			// bound then there is nothing left to derive at all.
			c[12] = m_groundLayer  ? m_groundLayer->getPositionY()  : 0.0;
			c[13] = m_groundLayer2 ? m_groundLayer2->getPositionY() : 0.0;
			for (int i = 0; i < Solver::kCeilCand; i++) {
				const double d = y - c[i];
				sv.candMin[i]  = std::min(sv.candMin[i], d);
				sv.candMax[i]  = std::max(sv.candMax[i], d);
				sv.candLast[i] = c[i];
			}
			sv.candN++;
			sv.ceilHist[static_cast<int>(std::lround(y))]++;
		}

		// The camera's own rectangle. If its top edge IS the ceiling then there
		// is nothing left to derive - read it and clamp to it.
		if (m_cameraObb2) {
			const cocos2d::CCRect r = m_cameraObb2->getBoundingRect();
			const double top = r.origin.y + r.size.height;
			sv.camTopSeen = top;
			const double coff = y - top;
			sv.camOffMin = std::min(sv.camOffMin, coff);
			sv.camOffMax = std::max(sv.camOffMax, coff);
			sv.camOffN++;
		}
	}

	// Probe 10: one step of the ceiling measurement.
	//
	// Called from the same one-real-step gate as the motion model, so `vyBefore`
	// really is the velocity entering this step.
	void ceilingProbeRecord(double vyBefore) {
		auto* p = m_player1;
		if (!g_config.ceilingProbe || !p || p->m_isDead) return;
		if (classifyMode(p) == ModeClass::Ground) return;   // air modes only

		auto& cp = CeilingProbe::get();
		const double bx = std::max(1.0, g_config.ceilingBandX);
		const int32_t b = static_cast<int32_t>(std::floor(p->getPositionX() / bx));
		const double y = p->getPositionY();

		auto& e = cp.bands[b];
		cp.samples++;
		if (y > e.maxY) { e.maxY = y; }
		e.reached++;

		// Stopped while rising, in world-up terms so inverted gravity is not
		// counted as a ceiling hit at the floor.
		const double worldUp = p->m_isUpsideDown ? -vyBefore : vyBefore;
		if (worldUp > 0.5 && std::abs(p->m_yVelocity) < 1e-6) {
			if (y < e.contactY) e.contactY = y;
			e.contacts++;

			// The camera, so the anchoring rule can be read off rather than
			// assumed. m_objectLayer scrolls opposite to the camera, so the
			// camera's world y is the negation of its position.
			double camY = 0.0;
			if (m_objectLayer) camY = -m_objectLayer->getPositionY();
			const double off = y - camY;
			cp.offMin  = std::min(cp.offMin,  off);
			cp.offMax  = std::max(cp.offMax,  off);
			cp.camHMin = std::min(cp.camHMin, static_cast<double>(m_cameraHeight));
			cp.camHMax = std::max(cp.camHMax, static_cast<double>(m_cameraHeight));
			cp.ceilMin = std::min(cp.ceilMin, y);
			cp.ceilMax = std::max(cp.ceilMax, y);
			cp.contacts++;
		}
	}

	// Probe 9: one step of the vertical motion model.
	//
	// Keyed on everything that can change the answer. If the spread column in the
	// dump is non-zero, something is missing from exactly this list.
	void motionModelRecord(double vyBefore, bool prevHold, bool hold, double dy,
	                      bool evt) {
		auto* p = m_player1;
		if (!g_config.motionModelEnabled || !p || p->m_isDead) return;

		auto& mm = MotionModel::get();
		const double q = std::max(1e-4, g_config.motionVyBucket);

		MotionModel::Row k;
		k.vyb  = static_cast<int32_t>(std::floor(vyBefore / q));
		// Full mode identity, not just the class: ship and wave are both Hold
		// and do not share a curve.
		k.mode = static_cast<uint8_t>(p->m_isShip ? 1 : p->m_isDart ? 2
		       : p->m_isBird ? 3 : p->m_isBall ? 4 : p->m_isRobot ? 5
		       : p->m_isSpider ? 6 : p->m_isSwing ? 7 : 0);
		k.size  = p->m_vehicleSize < 0.9f ? 1 : 0;
		k.speed = static_cast<uint8_t>(Solver::speedBucket(Solver::get().dxPerStep));
		// m_isAccelerating and m_jumpBuffered are in the key because the decompiled
		// updateJump selects the ship's acceleration COEFFICIENT with them -
		// camila314/gdp gives 0.8 / 1.2 / -1.0 depending on jumpBuffered,
		// isAccelerating, and whether velocity direction matches gravity.
		//
		// MEASURED consequence of leaving them out: over a 15-level census of
		// 1,168,629 transitions and 51,255 keys, 53.65% of keys had NONZERO SPREAD,
		// max 31.0 - so vy was not a function of the key at all. Ship was worst at
		// 70.4%, and the bad rate ROSE with sample count (0% at one sample, 81.5% at
		// 5-19, 82.8% at 20-99), which is a missing axis rather than noise.
		k.flags = static_cast<uint8_t>((p->m_isUpsideDown ? 1 : 0)
		        | (prevHold ? 2 : 0) | (hold ? 4 : 0)
		        | (p->m_isOnGround ? 8 : 0)
		        | (p->m_isAccelerating ? 16 : 0)
		// EVENT bit. Set when something OTHER than the integrator touched the
		// velocity this step: an orb/pad was in contact, the ground state changed,
		// the mode changed, or vy was forced to exactly zero while airborne
		// (ceiling or wall contact).
		   //
		   // m_touchedPad / m_touchedRing / m_touchedCustomRing were added after the
		   // first census with the event bit: m_touchingRings alone left 68 ship-air
		   // free-flight keys inconsistent, and their extremes still sat on +-6.40 -
		   // one of the same fixed pad/orb constants (16.0, 12.8, 10.4, 6.4). Pads fire
		   // on contact without a click and are not in m_touchingRings.
		   //
		   // They are STICKY per attempt, not per step - MEASURED: testing the level
		   // instead of the rising edge marked nearly everything as an event and
		   // collapsed air free-flight from 18,790 keys to 40. Hence the edge test.
		//
		// MARKED rather than dropped, so the claim can be checked instead of
		// assumed. MEASURED beforehand: of the 584 inconsistent air-mode keys, 503
		// (86.1%) have an extreme sitting on an event constant - 0.0 dominates at
		// 450 - and the 81 that do not are 0.43% of all air keys.
		//
		// It is in the KEY, so event and free-flight transitions never share a row
		// and the clean rows carry clean min/max. It is derived partly from AFTER
		// state, so it is not predictable at search time - which is the point: the
		// model predicts free flight, and the geometry map already knows where every
		// orb, pad, floor and ceiling is. Those were never the model's job.
		        | (p->m_jumpBuffered ? 32 : 0)
		        | (evt ? 64 : 0));

		struct Packed { int32_t vyb; uint8_t mode, size, speed, flags; } pk{
			k.vyb, k.mode, k.size, k.speed, k.flags };
		const uint64_t key = probe::fnv1a(&pk, sizeof(pk));
		mm.recorded++;

		const double vyAfter = p->m_yVelocity;
		auto it = mm.rows.find(key);
		if (it == mm.rows.end()) {
			if (mm.rows.size() >= static_cast<size_t>(g_config.motionModelCap)) {
				mm.capped++;
				return;
			}
			k.vyNextMin = k.vyNextMax = vyAfter;
			k.dySum = dy;
			k.count = 1;
			mm.rows.emplace(key, k);
			return;
		}
		auto& r = it->second;
		r.vyNextMin = std::min(r.vyNextMin, vyAfter);
		r.vyNextMax = std::max(r.vyNextMax, vyAfter);
		r.dySum += dy;
		r.count++;
	}

	// Drop the least useful cell, sampled rather than scanned: a full pass over
	// the archive on every insertion at the cap would sit in the stepping path.
	bool solverArchiveEvict() {
		auto& ar = SolverArchive::get();
		if (ar.keys.empty()) return false;

		size_t worstIdx = 0;
		double worstScore = 0.0;
		bool found = false;
		for (int t = 0; t < 32; t++) {
			const size_t i = static_cast<size_t>(ar.next() % ar.keys.size());
			auto it = ar.cells.find(ar.keys[i]);
			if (it == ar.cells.end()) continue;
			// Low progress and heavily-used cells go first. A cell already
			// chosen many times has told us what it has to tell us.
			const double score = it->second.pct - 0.01 * it->second.chosen;
			if (!found || score < worstScore) { worstScore = score; worstIdx = i; found = true; }
		}
		if (!found) return false;

		auto it = ar.cells.find(ar.keys[worstIdx]);
		if (it != ar.cells.end()) {
			releaseCheckpoint(it->second.rs.cp);
			ar.cells.erase(it);
		}
		ar.keys[worstIdx] = ar.keys.back();
		ar.keys.pop_back();
		ar.evicted++;
		return true;
	}

	// Census one step of the DFS into the archive.
	//
	// A checkpoint is captured only for a cell that is NEW. Capture is ~12 us
	// (Probe 5) against a restore's millisecond, but the cost that matters here
	// is memory and the live-checkpoint count, so a cell seen again is a counter
	// bump and nothing else. No "improve the representative" rule either: the
	// DFS reaches a cell by one path and re-capturing on a shorter one would
	// churn checkpoints in the stepping path for a gain the escape cannot use.
	void solverArchiveRecord() {
		if (!g_config.escapeArchiveRestart) return;
		auto& sv = Solver::get();
		auto* p  = m_player1;
		auto* pl = PlayLayer::get();
		if (!p || !pl || p->m_isDead) return;

		auto& ar = SolverArchive::get();
		const uint64_t key = goCellKey();

		auto it = ar.cells.find(key);
		if (it != ar.cells.end()) { it->second.seen++; return; }

		if (ar.cells.size() >= static_cast<size_t>(g_config.solverArchiveCap)
		    && !solverArchiveEvict()) return;

		GoEntry e;
		captureRestoreState(e.rs);
		if (!e.rs.cp) return;
		// The macro is the input sequence from frame 0 to here, which is what
		// makes a restart legitimate: the state we return to is reachable from
		// the start by a sequence we can hand to the verifier. sv.macro is
		// exactly that, truncated to the current step.
		e.macro.assign(sv.macro.begin(),
		               sv.macro.begin() + std::min<size_t>(static_cast<size_t>(sv.step),
		                                                   sv.macro.size()));
		e.pct  = pl->getCurrentPercent();
		e.step = sv.step;
		e.hold = sv.hold;
		e.seen = 1;
		ar.cells.emplace(key, std::move(e));
		ar.keys.push_back(key);
		ar.newCells++;
	}

	// Pick a cell to resume from: promising, and not somewhere we have already
	// spent our time. Tournament rather than a full scan, for the same reason as
	// eviction.
	uint64_t solverArchiveSelect() {
		auto& ar = SolverArchive::get();
		auto& sv = Solver::get();
		if (ar.keys.empty()) return 0;

		// Never restart near the level start when the frontier is far ahead.
		const float minPct = static_cast<float>(sv.bestPct * g_config.archiveMinPctFrac);

		uint64_t bestKey = 0;
		double   bestW   = -1.0;
		// Was g_config.goTournament, borrowed from Go-Explore. Inlined when that
		// searcher was removed rather than given a config field of its own - the
		// escape archive is itself unresolved, and a knob for an unresolved
		// mechanism is how this file got the way it did.
		const int kTournament = 64;
		for (int t = 0; t < kTournament; t++) {
			const uint64_t k = ar.keys[static_cast<size_t>(ar.next() % ar.keys.size())];
			auto it = ar.cells.find(k);
			if (it == ar.cells.end() || !it->second.rs.cp) continue;
			if (it->second.pct < minPct) continue;
			// Under-chosen and under-visited first. This is what makes the
			// restart avoid the stall region without being told where it is:
			// the place the search has been thrashing has an enormous `seen`.
			double w = 1.0 / std::sqrt(1.0 + static_cast<double>(it->second.chosen));
			w       /= std::sqrt(1.0 + static_cast<double>(it->second.seen));
			if (w > bestW) { bestW = w; bestKey = k; }
		}
		return bestKey;
	}

	// Resume the DFS from an archived cell instead of rewinding the stack.
	//
	// The stack is abandoned wholesale rather than truncated. A truncation would
	// be wrong: the entries below a cut belong to the path the DFS walked, and
	// the cell was reached by its own macro, so nothing guarantees they
	// correspond. Clearing and treating the cell as a fresh root is the only
	// consistent choice, and it is sound because the cell carries a frame-0
	// macro - sv.macro stays a real input sequence throughout.
	bool solverArchiveRestartTo() {
		auto& ar = SolverArchive::get();
		auto& sv = Solver::get();

		const uint64_t key = solverArchiveSelect();
		if (!key) return false;
		auto it = ar.cells.find(key);
		if (it == ar.cells.end() || !it->second.rs.cp) return false;

		applyRestoreState(it->second.rs, it->second.hold);

		for (auto& d : sv.stack) releaseCheckpoint(d.rs.cp);
		sv.stack.clear();
		sv.macro       = it->second.macro;
		sv.step        = it->second.step;
		sv.hold        = it->second.hold;
		sv.commitDepth = 0;
		sv.lastBranch  = sv.step;

		// Toggles are counted relative to the commit floor, and the floor is now
		// 0. Carrying the old counts forward would put every freshly pushed
		// decision instantly over budget and gate the entire restarted subtree.
		sv.togglesUsed = 0;
		sv.tapsUsed    = 0;

		sv.restores++;
		it->second.chosen++;
		ar.restarts++;
		return true;
	}

	void cellProbeRecord() {
		if (!g_config.cellProbeEnabled) return;
		auto* p  = m_player1;
		auto* pl = PlayLayer::get();
		if (!p || !pl || p->m_isDead) return;

		auto& cp = CellProbe::get();
		const CellCoords c = goCellCoords();
		const uint64_t key = probe::fnv1a(&c, sizeof(c));
		cp.recorded++;

		auto it = cp.cells.find(key);
		if (it != cp.cells.end()) { it->second.seen++; return; }
		if (cp.cells.size() >= static_cast<size_t>(g_config.cellProbeCap)) { cp.capped++; return; }

		CellProbe::Cell e;
		e.xb = c.x; e.yb = c.y; e.vyb = c.vy;
		e.mode = c.mode; e.flags = c.flags;
		e.seen      = 1;
		e.firstPct  = pl->getCurrentPercent();
		e.firstStep = Solver::get().step;
		cp.cells.emplace(key, e);
	}














	// Put the player back at decision `d`, whatever that takes.
	//
	// THE single place repositioning is decided. Having this logic inline at each
	// call site meant it kept getting fixed in one path and not another: the
	// death gate, then the no-savestate deepening path, then the (now removed)
	// hybrid deepening path all broke the same way.
	//
	// Returns true if repositioning is still in flight (a replay is running and
	// the caller should yield); false if the player is already at `d`.
	bool solverRepositionTo(Decision& d) {
		auto& sv = Solver::get();

		sv.resumeHold    = d.choice;
		sv.resumeTap     = (d.modeClass == ModeClass::Tap && d.choice)
		                 ? g_config.tapLengthSteps : 0;
		sv.resumeTapping = (d.modeClass == ModeClass::Tap) ? d.choice : false;
		sv.resumeToggles = d.togglesBefore + decisionCost(d, d.choice);
		sv.resumeTaps    = d.tapsBefore    + tapCost(d, d.choice);
		sv.resumeBranch  = d.step;
		// The branch rule in mode 1/2 compares the corridor ahead against the
		// corridor at the LAST decision, so lastGeoLo/Hi is carried search state and
		// has to be put back with everything else. Leaving it stale meant a replay
		// could manufacture decision points the original pass never had, so the DFS
		// was not enumerating a fixed tree and could never pop clean to the commit
		// floor - which is the only thing that deepens the toggle budget.
		//
		// MEASURED, Clutterfunk: mode 0 exhausts budget 1 at 455 deaths, deepens to
		// 2 and solves. Mode 1 sat on budget 1 for 5,943 deaths and 13 escapes and
		// stalled at 55.65%, and Clutterfunk has needed budget 2 since 9803757.
		// MEASUREMENT, always on and independent of the flag: how often the stale
		// values actually differ from the correct ones. The geoStateRestore decision
		// was defensible because the rate was measured (0.47%-4.75%) rather than
		// assumed from the bug's shape; this is the same check for the same reason.
		sv.modeStaleEvals++;
		if (sv.prevAirMode != d.enteringAirMode ||
		    sv.prevOnGround != d.enteringOnGround) sv.modeStaleDiverged++;
		sv.resumeAirMode = d.enteringAirMode;
		sv.resumeOnGround = d.enteringOnGround;
		sv.resumeGeoLo   = d.enteringGeoLo;
		sv.resumeGeoHi   = d.enteringGeoHi;
		sv.macro.resize(static_cast<size_t>(d.step), 0);
		if (sv.pathTrace.size() > static_cast<size_t>(d.step))
			sv.pathTrace.resize(static_cast<size_t>(d.step));

		// A usable checkpoint. Exact in every mode now (Probe 4b, 8/8), so this
		// is the path taken for essentially every backtrack.
		if (d.rs.cp && !g_config.noSavestates) {
			solverRestoreState(d);
			sv.step        = d.step;
			sv.hold         = sv.resumeHold;
			if (g_config.modeStateRestore) {
				sv.prevAirMode  = sv.resumeAirMode;
				sv.prevOnGround = sv.resumeOnGround;
			}
			sv.shadowGeoLo  = sv.resumeGeoLo;
			sv.shadowGeoHi  = sv.resumeGeoHi;
			if (g_config.geoStateRestore) {
				sv.lastGeoLo = sv.resumeGeoLo;
				sv.lastGeoHi = sv.resumeGeoHi;
			}
			if (!g_config.tapStateSurvivesRestore) {
				sv.tapping      = sv.resumeTapping;
				sv.tapRemaining = sv.resumeTap;
			}
			sv.togglesUsed  = sv.resumeToggles;
			sv.tapsUsed     = sv.resumeTaps;
			sv.lastBranch   = sv.resumeBranch;
			return false;
		}

		// No checkpoint of its own. Prefer the nearest CUBE ancestor plus a
		// forward replay: bounded by the air section rather than by the whole
		// prefix, and exact by construction because it IS forward simulation.
		if (g_config.hybridRestore && !g_config.noSavestates && !sv.stack.empty()) {
			size_t ai = sv.stack.size() - 1;
			bool found = false;
			while (true) {
				if (!sv.stack[ai].airPolicy && sv.stack[ai].rs.cp) { found = true; break; }
				if (ai == 0) break;
				ai--;
			}
			if (found) {
				Decision& anchor = sv.stack[ai];
				solverRestoreState(anchor);
				sv.step = anchor.step;
				// The trace has to rewind with the player, or the recorded
				// trajectory and the actual one drift apart silently.
				if (sv.pathTrace.size() > static_cast<size_t>(anchor.step))
					sv.pathTrace.resize(static_cast<size_t>(anchor.step));
				if (sv.step < d.step) {
					sv.anchorReplaying    = true;
					sv.anchorReplayTarget = d.step;
					sv.anchorReplays++;
					return true;
				}
				sv.hold         = sv.resumeHold;
				if (g_config.modeStateRestore) {
					sv.prevAirMode  = sv.resumeAirMode;
					sv.prevOnGround = sv.resumeOnGround;
				}
				sv.shadowGeoLo  = sv.resumeGeoLo;
				sv.shadowGeoHi  = sv.resumeGeoHi;
				if (g_config.geoStateRestore) {
					sv.lastGeoLo = sv.resumeGeoLo;
					sv.lastGeoHi = sv.resumeGeoHi;
				}
				if (!g_config.tapStateSurvivesRestore) {
					sv.tapping      = sv.resumeTapping;
					sv.tapRemaining = sv.resumeTap;
				}
				sv.togglesUsed  = sv.resumeToggles;
				sv.tapsUsed     = sv.resumeTaps;
				sv.lastBranch   = sv.resumeBranch;
				return false;
			}
		}

		// Last resort: replay from frame 0 - needs no checkpoint at all, so it is
		// always available and always exact.
		sv.resyncMacro.assign(sv.macro.begin(),
			sv.macro.begin() + std::min<size_t>(d.step, sv.macro.size()));
		sv.resyncTarget       = static_cast<int>(sv.resyncMacro.size());
		sv.resyncing          = true;
		sv.resyncForBacktrack = true;
		sv.step               = 0;
		sv.hold               = false;
		sv.replayBacktracks++;
		ProbeState::get().resetPending = true;
		return true;
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

			// Archive restart, when enabled and the archive has somewhere to go.
			// Taken INSTEAD of the whole rewind-ladder-anchor path: with the
			// stack cleared the commit floor is 0, so widening a mutable window
			// and releasing a transition anchor are both no-ops here. They stay
			// as the fallback for a failed or empty return, which is also what
			// keeps the default behaviour reachable.
			if (g_config.escapeArchiveRestart && solverArchiveRestartTo()) {
				sv.escapes++;
				sv.deathsAtBest = sv.deaths;
				auto& ar = SolverArchive::get();
				log::info("Solver: stalled {} deaths at {:.2f}% - returned to an archived "
				          "cell at step {} ({} cells, {} restarts, {} fallbacks)",
				          g_config.stallLimit, sv.bestPct, sv.step,
				          ar.cells.size(), ar.restarts, ar.restartFail);
				return true;
			}
			if (g_config.escapeArchiveRestart) SolverArchive::get().restartFail++;

			// Never rewind below the committed prefix. The previous version only
			// held a floor when the stack TOP was an air decision, so once the
			// air decisions were popped the guard stopped applying and the
			// escape tunnelled straight back into the solved cube section.
			const size_t escFloor = solverCommitFloor();
			size_t maxDrop = sv.stack.size() > escFloor
			               ? sv.stack.size() - escFloor : 0;

			// Rewind to the floor, and nothing else caps it.
			//
			// escapeJump asked for 200 decisions and the air-section guard asked to
			// stay inside the current air section. Probe 18 logged all 28 escapes on
			// Clutterfunk: the air guard bound ZERO of them, and escapeJump bound
			// none of the first sixteen - the floor was always the smaller number.
			// Once the anchor released it became the cap at exactly 200 and stayed
			// there for twelve escapes. So it was either ignored or it was the thing
			// preventing progress, and never once the right number. The floor is the
			// one quantity that means anything here, so it is now the only one.
			const size_t drop = maxDrop;

			// Probe 18: what the rewind ACTUALLY did, against what it was asked
			// to do. Three quantities claim to control how far back the search
			// may go - escapeJump in decisions, lookbackSteps in steps, and the
			// commit floor as a stack index - and nothing has ever reported
			// which of them bound a given escape. xStep abandons 16, 37, 53,
			// 54, 59, 60, 113, 119, 120, 134, 142 and then 200 decisions, a
			// pattern that tracks depth-against-floor rather than anything to
			// do with where progress stopped. This line settles whether
			// escapeJump is doing any work at all.
			const size_t depthBefore = sv.stack.size();
			const int    stepAtFloor = escFloor < depthBefore
			                         ? sv.stack[escFloor].step : 0;

			for (size_t i = 0; i < drop; i++) {
				releaseCheckpoint(sv.stack.back().rs.cp);
				sv.stack.pop_back();
			}
			sv.escapes++;
			sv.deathsAtBest = sv.deaths;

			int undone = 0;
			{
				const int newStep = sv.stack.empty() ? 0 : sv.stack.back().step;
				// The number that matters: how much ALREADY-SOLVED level this
				// threw away. An escape that lands a few steps behind the best
				// is retrying the hard part; one that lands thousands of steps
				// behind is re-deriving a section it had already cleared.
				undone = sv.bestStep - newStep;
				log::info("Escape #{} at {:.2f}%: dropped {} of {} decisions to the "
				          "floor - depth {}->{}, step {}->{}, undoing {} steps behind "
				          "best. floor idx {} at step {}, window {}, budget {}, anchor {}",
				          sv.escapes, sv.bestPct, drop, depthBefore,
				          depthBefore, sv.stack.size(), sv.bestStep, newStep, undone,
				          escFloor, stepAtFloor, sv.lookbackSteps,
				          g_config.toggleBudget,
				          sv.anchorReleased ? "released" : "held");
			}

			// Escapes that keep bouncing off the commit floor mean the search is
			// boxed in: it has nowhere left to explore inside the window. Widen
			// it so previously-frozen decisions become mutable again. Those have
			// untried branches, so this genuinely enlarges the search space -
			// deterministic DFS then explores paths it could not reach before,
			// rather than re-deriving the same ones.
			// Deepen in the GAPS between widenings - never instead of them.
			//
			// The first version deepened first and skipped the widening whenever it
			// did, so widening could not happen until the budget maxed at 24, i.e.
			// after ~24 escapes and 9600 deaths. MEASURED: Theory of Everything,
			// Time Machine and xStep all stalled with the window pinned at w240
			// while the budget climbed to 16, 23 and 14 - which is the original
			// starvation exactly inverted.
			//
			// Widening keeps its own cadence untouched; deepening only fills the
			// escapes in between. Strictly additive, so neither can starve.
			// ESCALATE ON EFFECT, NOT ON SCHEDULE.
			//
			// One quantity decides everything here: how far back the rewind actually
			// reached, which Probe 18 already reports as `undone`. If an escape did
			// not reach further than the last one, whatever stage we are in is inert
			// and we move to the next immediately.
			//
			// MEASURED, and this is why: on Clutterfunk the old schedule widened the
			// window 240 -> 7680, a 32x increase over five doublings costing three
			// escapes each, while the rewind went 243 -> 483 -> 633 and then FELL to
			// 618. Every escape logged `anchor held`: the transition anchor was
			// binding the whole time, so widening past it could not move the floor by
			// a single step. A schedule cannot see that. `undone` can.
			//
			// The stages in order - widen, release the anchor, raise the budget -
			// replace escapesBeforeWidening, escapesBeforeAnchorRelease and
			// deepenOnStall. The last of those deepened on EVERY stall, which cost
			// Electroman Adventures and Clubstep their solves outright; here the
			// budget rises only once the rewind is genuinely exhausted, which is the
			// deadlock it was reaching for: deepening waits on exhausting the budget-N
			// space, exhausting it needs a rewind that can reach the whole space, and
			// a pinned rewind means the budget can never rise. Clutterfunk sat on
			// budget 1 for 28 escapes and 11,200 deaths.
			if (undone <= sv.lastUndone) {
				if (sv.lookbackSteps < g_config.maxCommitLookbackSteps) {
					const int before = sv.lookbackSteps;
					sv.lookbackSteps = std::min(sv.lookbackSteps * g_config.wideningFactor,
					                            g_config.maxCommitLookbackSteps);
					log::info("Escape reached no further ({} steps) - widening the window "
					          "{} -> {} ({:.1f}s -> {:.1f}s of reach)",
					          undone, before, sv.lookbackSteps,
					          before / 240.0, sv.lookbackSteps / 240.0);
				} else if (!sv.anchorReleased) {
					// The obstacle is not inside this section - it is how we ENTERED
					// it. Releasing the anchor lets the floor move back past the
					// portal, which is the only way to reconsider the approach, or
					// whether to take it at all. A fake portal in a dead-end corridor
					// is otherwise permanently unescapable: taking it scores progress,
					// the anchor freezes behind it, and "do not go here" is
					// unreachable.
					sv.anchorReleased = true;
					log::info("Escape reached no further ({} steps) with the window maxed "
					          "- releasing the mode-transition anchor.", undone);
				}
				// No budget stage here. One was tried and ran away: with the window
				// maxed and the anchor released the floor is already as far back as it
				// goes, so `undone` can only decay by 1 per escape - the search adding
				// one decision - and the trigger is then permanently true. MEASURED:
				// Clubstep took eight consecutive bumps to budget 11, and a budget of
				// 11 over a 7,400-step window is a hopeless subtree that guarantees the
				// next escape fails too. Gating it on budgetMaxSpent did not help
				// either: that is a running maximum over the whole run, so it is
				// trivially satisfied once any branch has ever spent the budget.
				//
				// The budget already deepens on EXHAUSTION, which is the principled
				// trigger. What blocked it was a rewind that could not reach the space
				// it had to enumerate - and the collapse above fixes exactly that, so
				// exhaustion is reachable again. It fired six times in the run that
				// found this.
			}
			sv.lastUndone = undone;

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

			// The counters say the search is stuck; they cannot say whether it is
			// stuck because it never SAW the alternative or because it saw one and
			// walked away. The census can. Throttled - this writes a file.
			{
				auto& cpr = CellProbe::get();
				const uint64_t nowT = probe::nowTicks();
				if (probe::ticksToMicros(nowT - cpr.lastDump) > 30'000'000.0) {
					cpr.lastDump = nowT;
					cellProbeDump("stall");
				}
			}
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
				sv.budgetGateEvals++;

				// There is no separate tap budget. There never was: the branch that
				// would have used one was gated on tapRateBudget, which has been
				// false for the whole life of the file, so tapAllowance was never
				// once called from the search. Taps are charged against the same
				// toggle budget as holds, and steer-matching taps are free under
				// geomFreeFollowing exactly as steer-matching holds are.
				const bool wouldToggle = decisionCost(d, !d.choice) > 0;
				if (wouldToggle) sv.budgetGateWouldToggle++;
				else if (d.modeClass == ModeClass::Tap) sv.gateFreeTap++;
				else                                    sv.gateFreeHold++;
				// Relative to the committed prefix: toggles spent inside a
				// frozen, already-working prefix must not count against the
				// budget for the part still being solved.
				const int floorToggles = floor < sv.stack.size()
				                       ? sv.stack[floor].togglesBefore
				                       : 0;
				const int spent = d.togglesBefore - floorToggles;
				const int budget = g_config.toggleBudget;
				if (spent > sv.budgetMaxSpent) sv.budgetMaxSpent = spent;
				if (wouldToggle && spent >= budget) {
					// Record BEFORE the pop: d is about to be destroyed.
					sv.budgetPopCount++;
					if (sv.budgetPops.size() < sv.budgetPops.capacity()) {
						BudgetPop bp;
						bp.step          = d.step;
						bp.togglesBefore = d.togglesBefore;
						bp.floorToggles  = floorToggles;
						bp.spent         = spent;
						bp.budget        = budget;
						bp.floorIdx      = static_cast<int>(floor);
						bp.stackSize     = static_cast<int>(sv.stack.size());
						bp.mode          = static_cast<int>(d.modeClass);
						sv.budgetPops.push_back(bp);
					} else {
						sv.budgetPopsFull = true;
					}
					releaseCheckpoint(d.rs.cp);
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
				sv.togglesUsed = d.togglesBefore + decisionCost(d, d.choice);
				sv.tapsUsed    = d.tapsBefore    + tapCost(d, d.choice);

				if (solverRepositionTo(d)) return true;
				sv.togglesUsed = sv.resumeToggles;
				sv.tapsUsed    = sv.resumeTaps;
				return true;
			}

			releaseCheckpoint(d.rs.cp);
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

		// Flush the best path here rather than on every improvement: this is
		// already throttled to once every two seconds, and file I/O has no place
		// in the stepping path.
		if (sv.bestMacroDirty) {
			sv.bestMacroDirty = false;
			solverWriteBestMacro();
		}

		const double secs = probe::ticksToMicros(now - sv.startTicks) / 1e6;
		const double stepsPerSec = secs > 0 ? sv.steps / secs : 0.0;

		// The speed multiplier matters for interpreting anything observed by eye:
		// the search runs many times faster than real time, so a rewind of 3.3
		// GAME seconds plays back in a fraction of a wall-clock second. Game time
		// and wall-clock time are not interchangeable here.
		// One step's vertical move per mode, size and direction. Its own line
		// rather than more columns on the report: this is the input to the
		// reachability model, and it only has to be read once per level.
		{
			static const char* kModeName[4] = {"ship", "wave", "ufo ", "othr"};
			static const char* kSpeedName[5] = {"0.5x", "1x", "2x", "3x", "4x"};
			bool any = false;
			for (int mo = 0; mo < 4; mo++)
				for (int sz = 0; sz < 2; sz++) {
					std::string row;
					for (int sp = 0; sp < 5; sp++) {
						const double up = sv.maxClimb[mo][sz][sp][0];
						const double dn = sv.maxClimb[mo][sz][sp][1];
						if (up <= 0.0 && dn <= 0.0) continue;
						row += fmt::format("{} up {:.3f} dn {:.3f}  ",
						                   kSpeedName[sp], up, dn);
					}
					if (row.empty()) continue;
					if (!any) {
						log::info("Solver: climb per step (world units), map assumes "
						          "{:.3f} at every speed:",
						          static_cast<double>(g_config.geomVerticalReach) /
						              std::max(1.0, static_cast<double>(g_config.geomSliceX)) *
						              kSpeedNormal);
						any = true;
					}
					log::info("  {}{}  {}", kModeName[mo], sz ? "-mini" : "     ", row);
				}
		}

		log::info("Solver: best {:.2f}%  depth {}  deaths {}  restores {}  escapes {}  "
		          "steps {}  budget {}  commit {}(w{})  resync {}/{}  cells {}  "
		          "anchorReplays {}  tapFirst {}/{}  geoSteer {}/{} (dead {} win {})  "
		          "archive {}c/{}r/{}f  gate {} free {}T/{}H  mute {}  "
		          "geoStale {}/{}  velPrune {}  brIv {}/{:.1f}/{}  modeStale {}/{}  "
		          "ceil {}/{}c/{}p/{}s out {}/{}  roofMargin {}  "
		          "dx {:.2f}/{:.2f}  vs map {:.2f}  "
		          "({:.0f} steps/s = {:.1f}x real time, {:.0f} restores/s)",
		          sv.bestPct, sv.stack.size(), sv.deaths, sv.restores, sv.escapes, sv.steps,
		          g_config.toggleBudget, sv.commitDepth, sv.lookbackSteps, sv.resyncs, sv.resyncFailures,
		          CellProbe::get().cells.size(), sv.anchorReplays,
		          sv.tapFirstChosen, sv.tapDecisions, sv.geoSteers, sv.geoLooks,
		          sv.geoSteerDead, sv.geoSteerWin,
		          SolverArchive::get().cells.size(), SolverArchive::get().restarts,
		          SolverArchive::get().restartFail,
		          sv.budgetGateEvals, sv.gateFreeTap, sv.gateFreeHold,
		          sv.geoSteerMute,
		          sv.geoStaleDiverged, sv.geoStaleEvals, sv.velPrunes,
		          sv.brIvN ? sv.brIvMin : 0,
		          sv.brIvN ? static_cast<double>(sv.brIvSum) / sv.brIvN : 0.0,
		          sv.brIvMax,
		          sv.modeStaleDiverged, sv.modeStaleEvals,
		          sv.learnedCeiling > 1e29
		              ? std::string("--")
		              : fmt::format("{:.0f}", sv.learnedCeiling),
		          sv.ceilingCrops,
		          sv.portalEntries, sv.entryStaleDrops,
		          sv.ceilOutside, sv.ceilInside,
		          sv.roofMarginMin > 1e29
		              ? std::string("--")
		              : fmt::format("{:.0f}@{:.0f}/{:.0f}/{:.0f}",
		                            sv.roofMarginMin, sv.roofMarginMinX,
		                            sv.roofMarginN ? sv.roofMarginSum / sv.roofMarginN
		                                           : 0.0,
		                            sv.roofMarginMax),
		          sv.dxPerStep,
		          GeoMap::get().valid && m_player1
		              ? GeoMap::get().speedAt(m_player1->getPositionX()) : 0.0,
		          static_cast<double>(g_config.geomVerticalReach) /
		              std::max(1.0, static_cast<double>(g_config.geomSliceX)) * kSpeedNormal,
		          stepsPerSec, stepsPerSec / 240.0,
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
		for (auto& d : sv.stack) releaseCheckpoint(d.rs.cp);
		sv.stack.clear();
		sv.commitDepth = 0;
		sv.resyncTarget = static_cast<int>(sv.resyncMacro.size());
		sv.resyncing    = true;
		sv.step         = 0;
		sv.hold         = false;
		sv.togglesUsed  = 0;
		sv.tapsUsed     = 0;
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
			sv.hold         = sv.resumeHold;
			if (g_config.modeStateRestore) {
				sv.prevAirMode  = sv.resumeAirMode;
				sv.prevOnGround = sv.resumeOnGround;
			}
			sv.shadowGeoLo  = sv.resumeGeoLo;
			sv.shadowGeoHi  = sv.resumeGeoHi;
			if (g_config.geoStateRestore) {
				sv.lastGeoLo = sv.resumeGeoLo;
				sv.lastGeoHi = sv.resumeGeoHi;
			}
			if (!g_config.tapStateSurvivesRestore) {
				sv.tapping      = sv.resumeTapping;
				sv.tapRemaining = sv.resumeTap;
			}
			sv.togglesUsed  = sv.resumeToggles;
			sv.tapsUsed     = sv.resumeTaps;
			sv.lastBranch   = sv.resumeBranch;
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

	// Append the state after the step just simulated, at index sv.step-1.
	//
	// Backtracking rewinds sv.step, so the tail is dropped first - exactly what
	// happens to `macro`. Recording is abandoned rather than reallocating if the
	// buffer fills or the index desyncs, because a wrong-index row would make
	// the divergence report lie about WHERE the paths parted.
	void solverRecordPath(bool pressed) {
		auto& sv = Solver::get();
		if (sv.pathTraceFull || sv.step <= 0) return;
		const size_t idx = static_cast<size_t>(sv.step) - 1;
		if (sv.pathTrace.size() > idx) sv.pathTrace.resize(idx);
		if (sv.pathTrace.size() != idx ||
		    sv.pathTrace.size() >= sv.pathTrace.capacity()) {
			sv.pathTraceFull = true;
			return;
		}
		probe::TraceRow row = makeTraceRow(pressed, static_cast<int>(idx));
		if (sv.pendingPostRestore) {
			row.flags |= probe::FlagPostRestore;
			sv.pendingPostRestore = false;
		}
		sv.pathTrace.push_back(row);
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
			solverRecordPath(pressed);

			if (m_player1->m_isDead) { failResync(sv.step); return false; }
			if (ProbeState::get().finished || sv.step >= sv.resyncTarget) finishResync();
			return true;
		}

		// Anchor replay: walk the macro forward from the cube ancestor to the air
		// decision being backtracked to. Structurally a resync bounded by the air
		// section instead of by the whole prefix.
		if (sv.anchorReplaying) {
			const size_t idx = static_cast<size_t>(sv.step);
			const bool pressed = idx < sv.macro.size() && sv.macro[idx] != 0;
			applyInput(pressed);
			GJBaseGameLayer::update(static_cast<float>(kPhysicsDt));
			sv.step++;
			sv.steps++;
			solverRecordPath(pressed);

			if (m_player1->m_isDead) {
				// The replayed prefix should not die: it is the path that got
				// here. If it does, the macro and the anchor disagree, which is
				// a real defect - fall back to a frame-0 replay rather than
				// leaving a dead player for the next step to trip over.
				log::warn("Solver: anchor replay died at step {} (target {}) - macro "
				          "and anchor disagree. Falling back to a frame-0 replay.",
				          sv.step, sv.anchorReplayTarget);
				sv.anchorReplaying = false;
				sv.resyncMacro.assign(sv.macro.begin(),
					sv.macro.begin() + std::min<size_t>(sv.anchorReplayTarget,
					                                    sv.macro.size()));
				sv.resyncTarget       = static_cast<int>(sv.resyncMacro.size());
				sv.resyncing          = true;
				sv.resyncForBacktrack = true;
				sv.step               = 0;
				sv.hold               = false;
				sv.replayBacktracks++;
				ProbeState::get().resetPending = true;
				return true;
			}

			if (sv.step >= sv.anchorReplayTarget) {
				sv.anchorReplaying = false;
				sv.hold         = sv.resumeHold;
				if (g_config.modeStateRestore) {
					sv.prevAirMode  = sv.resumeAirMode;
					sv.prevOnGround = sv.resumeOnGround;
				}
				sv.shadowGeoLo  = sv.resumeGeoLo;
				sv.shadowGeoHi  = sv.resumeGeoHi;
				if (g_config.geoStateRestore) {
					sv.lastGeoLo = sv.resumeGeoLo;
					sv.lastGeoHi = sv.resumeGeoHi;
				}
				sv.togglesUsed  = sv.resumeToggles;
				sv.tapsUsed     = sv.resumeTaps;
				sv.lastBranch   = sv.resumeBranch;
				sv.macro.resize(static_cast<size_t>(sv.step), 0);

				// tapping/tapRemaining are deliberately NOT restored here, and
				// that is the whole difference between 78.90% and 32.49%.
				//
				// My first port of this block restored them, which reads as the
				// obviously correct thing to do - and it silently changed the
				// search. Restoring resumeTap sets tapRemaining to
				// tapLengthSteps, so sv.hold self-releases after 2 steps.
				// Leaving it stale (almost always 0) means the countdown in
				// solverStep never fires, so sv.hold = resumeHold survives the
				// full 4-step branch interval to the next push.
				//
				// That matters because of decisionCost's asymmetry:
				//   Tap: return choice ? 1 : 0
				// A Tap decision pushed while sv.hold is TRUE gets choice=true,
				// so its untried alternative (no tap) costs ZERO and the toggle
				// budget never gates it. Pushed while hold is false, the
				// alternative is a tap, costs 1, and is refused the moment the
				// budget is spent.
				//
				// MEASURED, same level, same 5243 identical gate refusals up to
				// the split: 30fe2b7 evaluated the gate 22675 times with 1756
				// (7.7%) on a FREE alternative and never exhausted budget 3;
				// this build with the restore in place evaluated 67995 times
				// with 3 free, exhausted budget 3, and stalled. Plan 13.13.
				//
				// This is a bug that happens to help. The principled version is
				// to order Tap decisions by whether a tap is needed - which sets
				// choice=true on purpose and makes the fallback free by the same
				// cost rule - and that is the next change, not this one.
			}
			return true;
		}

		// A restore is supposed to revive the player. If GD instead requires
		// its respawn sequence, the search would stall here forever - so say so
		// loudly once rather than spinning silently.
		if (m_player1->m_isDead) {
			static bool warned = false;
			if (!warned) {
				warned = true;
				log::error("Solver: player still dead after repositioning at step {}. "
				           "Some path reached solverStep without restoring or replaying - "
				           "reposition and step are out of sync. Stopping.", sv.step);
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
		solverRecordPath(sv.hold);

		const float pct = pl->getCurrentPercent();
		if (pct > sv.bestPct) {
			sv.bestPct      = pct;
			sv.bestMacroDirty = true;
			sv.deathsAtBest = sv.deaths; // progress: reset the stall counter

			// Record where the frontier reached. The commit floor is derived
			// from this against the CURRENT stack, never stored as an index.
			if (sv.step > sv.bestStep) sv.bestStep = sv.step;
			sv.bestMacro.assign(sv.macro.begin(),
			                    sv.macro.begin() + std::min<size_t>(sv.step, sv.macro.size()));

			// Probe 21: the decision sequence that produced this best.
			sv.bestDecisions.clear();
			sv.bestDecisions.reserve(sv.stack.size());
			for (auto const& d : sv.stack) {
				if (d.step > static_cast<int>(sv.bestStep)) break;
				sv.bestDecisions.push_back({d.step, static_cast<uint8_t>(d.modeClass),
				                            d.choice, d.hasSteer, d.steerChoice,
				                            d.togglesBefore,
				                            d.si.yNow, d.vyAt, d.si.yProj, d.si.target,
				                            d.si.band, d.si.lo, d.si.hi, d.si.path});
			}

			// Progress: the window was wide enough, so return to the cheap
			// default. A permanently wide window would keep the whole tail of the
			// level mutable and undo the point of committing at all.
			if (sv.lookbackSteps > g_config.commitLookbackSteps || sv.anchorReleased) {
				log::info("Solver: progress at {:.2f}% - resetting escalation (window {} -> {}"
				          "{})", sv.bestPct, sv.lookbackSteps, g_config.commitLookbackSteps,
				          sv.anchorReleased ? ", transition anchor restored" : "");
				sv.lookbackSteps  = g_config.commitLookbackSteps;
				sv.anchorReleased = false;
				sv.lastUndone     = 0;
			}

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
			cellProbeDump("solved");
			motionModelDump("solved");
			ceilingProbeDump("solved");

			// Hand the winning trajectory to the verifier. Done here, outside the
			// stepping path, so the copy costs nothing that matters.
			{
				auto& ps = ProbeState::get();
				if (sv.pathTraceFull) {
					ps.solvePathTrace.clear();
					log::warn("  Path trace overran its buffer - the verify cannot "
					          "report where the paths diverge for this run.");
				} else {
					ps.solvePathTrace = sv.pathTrace;
					ps.solveDecisionSteps.clear();
					ps.solveDecisionSteps.reserve(sv.stack.size());
					for (auto const& d : sv.stack) ps.solveDecisionSteps.push_back(d.step);
					log::info("  Recorded {} steps of the winning trajectory across {} "
					          "decisions. F4 will report the first step where a clean "
					          "replay departs from it.",
					          ps.solvePathTrace.size(), ps.solveDecisionSteps.size());
				}
			}

			log::info("  Verify by replaying from frame 0 - a solution found via "
			          "savestates only counts if it reproduces in a clean run.");
			log::info("========================================");
			sv.clear();
			ProbeState::get().mode = Mode::Idle;
			return false;
		}

		// A cell the map calls dead has no forward route, so continuing to
		// simulate it only postpones the same death by a few hundred steps - and
		// postponing it is what freezes the decision that caused it.
		bool geoDead = false;
		if (g_config.velocityPrune && !m_player1->m_isDead) {
			const ModeClass pmc = classifyMode(m_player1);
			if (pmc == ModeClass::Hold) {
				double bl = 0.0, bh = 0.0;
				if (velocityUnreachable(m_player1->getPositionX(),
				                        m_player1->getPositionY(),
				                        static_cast<double>(m_player1->m_yVelocity),
				                        sv.dxPerStep,
				                        m_player1->m_vehicleSize < 0.9f, &bl, &bh)) {
					geoDead = true;
					sv.velPrunes++;
				}
			}
		}

		if (m_player1->m_isDead || geoDead) {
			sv.deaths++;
			if (geoDead) sv.geoDeaths++;
			sv.diedAtX = m_player1->getPositionX();

			// Phase A1: what mode is this section actually in, and which branch
			// policy is driving it? The ship riding the floor into the first
			// block is exactly what the CUBE policy would produce, because its
			// jump-commitment logic releases the button the instant the player
			// leaves the ground.
			if (sv.deaths <= 20 || sv.deaths % 500 == 0) {
				auto* p = m_player1;
				const bool air = p->m_isShip || p->m_isBird || p->m_isDart || p->m_isSwing;
				log::info("Solver {} #{}: step {} X {:.1f} {:.2f}%  policy={}  "
				          "ship={} ufo={} wave={} swing={} ball={} robot={} spider={}  "
				          "onGround={}/{}/{}/{}  yVel={:.3f}  size={:.2f}  hold={}",
				          geoDead ? "DEAD-CELL" : "death",
				          sv.deaths, sv.step, sv.diedAtX, pl->getCurrentPercent(),
				          air ? "AIR" : "CUBE",
				          p->m_isShip, p->m_isBird, p->m_isDart, p->m_isSwing,
				          p->m_isBall, p->m_isRobot, p->m_isSpider,
				          p->m_isOnGround, p->m_isOnGround2, p->m_isOnGround3, p->m_isOnGround4,
				          p->m_yVelocity, p->m_vehicleSize, sv.hold);
			}

			if (!solverBacktrack()) {
				// Every path within the current toggle budget is exhausted.
				// Deepen and restart rather than giving up: this is what makes
				// the search complete as the bound grows.
				if (g_config.toggleBudget < g_config.maxToggleBudget) {
					g_config.toggleBudget++;
					solverReport(true);
					solverWriteBudgetPops("deepen");
					log::info("Solver: exhausted every path with <= {} air toggles above "
					          "the committed prefix (depth {}) at {:.2f}%. Deepening to {}.",
					          g_config.toggleBudget - 1, sv.commitDepth, sv.bestPct,
					          g_config.toggleBudget);

					// Resume from the committed prefix, not from level start:
					// re-deriving a solved prefix is exactly the waste this fixes.
					if (!sv.stack.empty()) {
						// Same repositioning as backtracking, via the shared path.
						Decision& d = sv.stack.back();
						solverRepositionTo(d);
						sv.togglesUsed  = sv.resumeToggles;
						sv.tapsUsed     = sv.resumeTaps;
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
					cellProbeDump("exhausted");
					sv.clear();
					ProbeState::get().mode = Mode::Idle;
				}
			}
			return false;
		}

		// Horizontal speed, from the player's own motion. Guarded because a
		// restore moves x backwards and a portal can change speed mid-stride:
		// anything outside a sane forward step keeps the previous value rather
		// than poisoning the lookahead for a frame.
		{
			auto& s = Solver::get();
			const double nx = m_player1->getPositionX();
			const double ny = m_player1->getPositionY();
			const double dx = nx - s.prevX;
			if (dx > 0.01 && dx < 20.0) {
				s.dxPerStep = dx;
				// dx being sane means this was one real forward step, so the
				// matching dy is a real one-step vertical move.
				const double rawDy = ny - s.prevY;
				const double dy = std::abs(rawDy);
				if (dy < 200.0) {
					// World up, not player up: the map is in world coordinates,
					// so an inverted-gravity climb belongs in the DOWN bucket.
					const int mode = m_player1->m_isShip ? 0
					               : m_player1->m_isDart ? 1
					               : m_player1->m_isBird ? 2 : 3;
					const int size = m_player1->m_vehicleSize < 0.9f ? 1 : 0;
					const int dir  = rawDy >= 0.0 ? 0 : 1;
					const int spd  = Solver::speedBucket(dx);
					double& slot = s.maxClimb[mode][size][spd][dir];
					slot = std::max(slot, dy);
				}

				// Probe 9, inside this gate deliberately: `dx` being sane is
				// already the proof that this was ONE real forward step rather
				// than the far side of a restore, and a transition recorded
				// across a reposition would be pure noise. sv.hold here is still
				// the input that was applied during this step - the tap countdown
				// that can change it runs further down.
				if (s.motionHavePrev) {
					const uint8_t mmMode = static_cast<uint8_t>(classifyMode(m_player1));
					const bool mmGround = m_player1->m_isOnGround;
					const bool mmEvt =
						      (m_player1->m_touchingRings && m_player1->m_touchingRings->count() > 0)
						   || (m_player1->m_touchedPad       && !s.motionPrevPad)
						   || (m_player1->m_touchedRing      && !s.motionPrevRing)
						   || (m_player1->m_touchedCustomRing && !s.motionPrevCustomRing)
						|| (mmGround != s.motionPrevGround)
						|| (s.motionPrevMode != 255 && mmMode != s.motionPrevMode)
						|| (!mmGround && m_player1->m_yVelocity == 0.0
						    && std::abs(s.motionPrevVy) > 0.01);
					motionModelRecord(s.motionPrevVy, s.motionPrevHold, sv.hold, rawDy, mmEvt);
					ceilingProbeRecord(s.motionPrevVy);
					ceilingUpdate(s.motionPrevVy);
				}
			}
			s.prevX = nx;
			s.prevY = ny;
			// Unconditionally, so the step after a skipped one still pairs
			// against the state it actually started from.
			s.motionPrevVy   = m_player1->m_yVelocity;
			s.motionPrevHold = sv.hold;
			s.motionPrevGround = m_player1->m_isOnGround;
			s.motionPrevMode   = static_cast<uint8_t>(classifyMode(m_player1));
			s.motionPrevPad        = m_player1->m_touchedPad;
			s.motionPrevRing       = m_player1->m_touchedRing;
			s.motionPrevCustomRing = m_player1->m_touchedCustomRing;
			s.motionHavePrev = true;
		}

		// Census where the search has been. Read-only - see CellProbe. Placed
		// after the death check so a dead frame is never counted as a place the
		// player reached, and after the resync early-return so the census covers
		// search steps only, not prefix replay.
		cellProbeRecord();

		// Same placement and the same reasons: after the death check so a dead
		// frame is never recorded as somewhere the player reached, and after the
		// resync early-return so prefix replay does not fill the archive with
		// states the search did not choose.
		solverArchiveRecord();

		auto* p = m_player1;
		const ModeClass mc = classifyMode(p);
		const bool airMode = mc != ModeClass::Ground;

		// A tap is a short press that releases itself. Holding does nothing in
		// UFO or swing, so a persistent hold would just block the next tap.
		if (sv.tapRemaining > 0) {
			sv.tapRemaining--;
			if (sv.tapRemaining == 0) sv.hold = false;
		}

		// A mode change is the most timing-critical frame in a section, and
		// nothing otherwise guarantees a decision lands on it. Force one so the
		// search can pick its hold state from the exact step the ship begins.
		if (airMode != sv.prevAirMode) {
			sv.prevAirMode = airMode;
			solverPushDecision();
			if (!sv.stack.empty()) sv.stack.back().modeTransition = true;
			return true;
		}

		// Has the corridor ahead changed since the last decision? Only consulted
		// between airBranchInterval and airBranchIntervalMax: below the minimum
		// nothing branches, above the maximum everything does.
		//
		// RESTORED. c57f14d deleted this with the note "mode 0 - the default and the
		// only mode ever shipped - returned true immediately, so the whole rule was
		// unreachable". The second half was true and the first half was not: mode 1
		// shipped on a hotkey, and it is what produced the fastest Clubstep clear
		// this project has recorded.
		//
		// MEASURED, and the control arrived eleven days late. Same build - the
		// working tree committed 25 minutes afterwards as 9803757 - same level:
		//
		//   M pressed (mode 1)   Clubstep SOLVED, 6,206 deaths, 59 seconds
		//   default  (mode 0)    Clubstep STALLED at 58.43% after 9,903 deaths
		//
		// 9803757's message reads "Clubstep clears - first demon!" and does not
		// mention the key, so that result has been read as a default-config clear
		// ever since. It was not one.
		//
		// What it does: the fixed cadence pushes a decision every airBranchInterval
		// steps in the air whether or not anything is happening, so a long straight
		// corridor costs the search hundreds of decision points that all have the
		// same answer. Mode 1 reads the free window at the lookahead point and only
		// branches when it has CHANGED since the last decision. It cannot go blind:
		// airBranchIntervalMax forces a decision regardless, and no map reading means
		// no opinion, which falls through to the old cadence.
		//
		// Mode 2 additionally keeps every decision inside a tight corridor, where
		// timing is what matters - suppressing them there is what cost mode 1 the
		// mini ship section.
		//
		// Default 0, so this is inert until M is pressed.
		auto geometryWantsBranch = [&]() -> bool {
			if (g_config.geomBranchMode == 0 || !GeoMap::get().valid) return true;
			const uint64_t elapsed = sv.step - sv.lastBranch;
			const bool derivedIv = g_config.geomBranchMode >= 3;
			if (!derivedIv &&
			    elapsed >= static_cast<uint64_t>(g_config.airBranchIntervalMax)) return true;
			const double lookX = p->getPositionX() +
				(g_config.geomLookaheadScaleWithSpeed
					? static_cast<double>(g_config.geomLookaheadSteps) * sv.dxPerStep
					: static_cast<double>(g_config.geomLookaheadFixedX));
			double glo = 0.0, ghi = 0.0;
			// No reading means no opinion: keep the old cadence rather than
			// suppressing a decision the map cannot vouch for.
			if (!geometryWindowAt(lookX, p->getPositionY(), &glo, &ghi)) return true;

			// MODE 3: derive the decision ceiling from the corridor instead of fixing it.
			//
			// The question a branch interval answers is physical: how long can the
			// player coast before it can no longer correct? With `slack` units of room
			// and a measured climb rate of `rate` units per step, that is slack/rate
			// steps. Tight corridor -> frequent decisions; open corridor -> coast.
			//
			// Rate is Probe 16, measured in a purpose-built level and cross-validated
			// twice (mini/normal 1.1765 against a documented m_vehicleSize 0.85;
			// 8.00 * 0.2279 = 1.823 against the same 1.800).
			//
			// This SUBSUMES mode 2, whose tight-corridor exemption is the same idea as
			// a step function - so geomTightHeights goes with it.
			//
			// Why derived rather than swept: airBranchIntervalMax at 16/12/8/6 is
			// NON-MONOTONIC (8 clears Clubstep's 58.41% wall, 6 does not), and mode 0
			// solves Clubstep while losing Base After Base where mode 1 does the
			// reverse. Two levels want opposite things from one constant and the sweep
			// between them is not monotonic, which is the signature of a quantity that
			// has to be derived.
			if (derivedIv) {
				const bool mini = p->m_vehicleSize < 0.9f;
				const double rate = mini ? 2.118 : 1.800;
				const double slack = (ghi - glo) - GeoMap::get().playerH;
				int ceilSteps = static_cast<int>(std::floor(std::max(0.0, slack) / rate));
				ceilSteps = std::max(g_config.airBranchInterval,
				           std::min(g_config.airBranchIntervalMax, ceilSteps));
				sv.brIvSum += static_cast<uint64_t>(ceilSteps);
				sv.brIvN++;
				if (ceilSteps < sv.brIvMin) sv.brIvMin = ceilSteps;
				if (ceilSteps > sv.brIvMax) sv.brIvMax = ceilSteps;
				if (elapsed >= static_cast<uint64_t>(ceilSteps)) return true;
			}
			// Tight corridor: keep every decision.
			if (g_config.geomBranchMode == 2 &&
			    (ghi - glo) < g_config.geomTightHeights * GeoMap::get().playerH) {
				sv.lastGeoLo = glo; sv.lastGeoHi = ghi;
				sv.shadowGeoLo = glo; sv.shadowGeoHi = ghi;
				return true;
			}
			// MEASUREMENT, not behaviour. `changed` is what the search acts on and is
			// computed exactly as before. `changedShadow` is the same test against
			// state that WAS restored correctly on the last reposition, so the two
			// differ precisely when the stale carry moved a decision point.
			//
			// This exists because the claim "the stale carry manufactures decision
			// points" was reasoned from level outcomes and never counted. If
			// geoStaleDiverged comes back 0, that claim is wrong and geoStateRestore
			// should be deleted rather than committed.
			const bool changed = std::abs(glo - sv.lastGeoLo) > 0.01 ||
			                     std::abs(ghi - sv.lastGeoHi) > 0.01;
			const bool changedShadow = std::abs(glo - sv.shadowGeoLo) > 0.01 ||
			                           std::abs(ghi - sv.shadowGeoHi) > 0.01;
			sv.geoStaleEvals++;
			if (changed != changedShadow) sv.geoStaleDiverged++;
			if (changed)       { sv.lastGeoLo   = glo; sv.lastGeoHi   = ghi; }
			if (changedShadow) { sv.shadowGeoLo = glo; sv.shadowGeoHi = ghi; }
			return changed;
		};

		if (mc == ModeClass::Tap) {
			// Discrete events: decide whether to tap, on the same cadence. No
			// hold persists between decisions.
			if (sv.step - sv.lastBranch >= g_config.airBranchInterval &&
			    geometryWantsBranch()) solverPushDecision();
		} else if (mc == ModeClass::Hold) {
			// Hold state controls the trajectory continuously, so the action
			// genuinely is per-segment and persists across the interval.
			if (sv.step - sv.lastBranch >= g_config.airBranchInterval &&
			    geometryWantsBranch()) solverPushDecision();
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

	// Probe 12: print every ceiling candidate, live, while the level is played by
	// hand. See liveCeilingReadout. Called from update(), so it runs in normal
	// play rather than only inside the solver's step.
	// Which of the five speeds the game says we are in, and its canonical
	// per-step dx. m_playerSpeed's absolute values are not assumed - the index
	// is the nearest of the five known multipliers and the RAW value is
	// reported back, so a wrong guess is visible in the table rather than
	// silently mis-filing every sample.
	static int envSpeedIndex(float ps) {
		static const float kMul[5] = {0.7f, 0.9f, 1.1f, 1.3f, 1.6f};
		int best = 1; float bd = 1e9f;
		for (int i = 0; i < 5; i++) {
			const float d = std::abs(ps - kMul[i]);
			if (d < bd) { bd = d; best = i; }
		}
		return best;
	}
	static double envSpeedDx(int i) {
		static const double kDx[5] = {kSpeedSlow, kSpeedNormal, kSpeedFast,
		                              kSpeedFaster, kSpeedFastest};
		return kDx[i < 0 ? 0 : i > 4 ? 4 : i];
	}

	// Probe 16. Runs in FREE FLIGHT only - same guard as liveCeilingStep - so it
	// never sees a savestate restore and never competes with the solver's own
	// sampler.
	void climbEnvelopeStep() {
		auto* p  = m_player1;
		auto* pl = PlayLayer::get();
		if (!p || !pl || p->m_isDead) return;
		if (Solver::get().running || ProbeState::get().mode != Mode::Idle) return;

		auto& sv = Solver::get();
		const double nx = p->getPositionX();
		const double ny = p->getPositionY();
		if (!sv.envHavePrev) {
			sv.envPrevX = nx; sv.envPrevY = ny; sv.envHavePrev = true; return;
		}
		const double dx    = nx - sv.envPrevX;
		const double rawDy = ny - sv.envPrevY;
		sv.envPrevX = nx; sv.envPrevY = ny;
		if (!(dx > 0.01 && dx < 20.0)) return;   // a reposition, not a step
		const double dy = std::abs(rawDy);
		if (dy >= 200.0) return;                 // a teleport, not motion

		// NOT MOVING is not a direction. Direction comes from the sign of dy,
		// so a ship pinned against the camera ceiling - or resting on the
		// floor - produced dy 0 and filed as an UP sample worth 0.000. On the
		// 3x corridor that was 329 of them, which locked the cell at zero and
		// buried the real figure. A stationary frame says nothing about how
		// far the player CAN move, so it is not a sample at all.
		if (dy < 0.01 || std::abs(p->m_yVelocity) < 0.01) {
			sv.envStationary++; return;
		}

		const int mode = p->m_isShip ? 1 : p->m_isDart ? 2 : p->m_isBird ? 3
		               : p->m_isBall ? 4 : p->m_isRobot ? 5 : p->m_isSpider ? 6
		               : p->m_isSwing ? 7 : 0;
		const int size = p->m_vehicleSize < 0.9f ? 1 : 0;
		const int dir  = rawDy >= 0.0 ? 0 : 1;   // world up, not player up
		const int spd  = envSpeedIndex(p->m_playerSpeed);

		// dx is a DIAGNOSTIC, not a filter. The previous version rejected any
		// sample whose dx was not one step's worth and threw away every last
		// one - 3724 of 3724 - which was the same mistake twice over: free
		// flight runs at 1/120 with the delta hook returning 1/240, so an
		// update() call covers TWO physics steps and no sample could ever
		// match. It did not matter anyway. Both figures recorded here are
		// frame-rate free: m_yVelocity is read at the instant of sampling,
		// and slope is dy/dx with the span cancelling out of both halves.
		// Only an outright reposition is worth discarding.
		const double want  = envSpeedDx(spd);
		const double spanF = dx / want;          // ~ how many steps this covered
		if (spanF < 0.25 || spanF > 8.0) { sv.envRejected++; return; }

		// States where vertical motion is not the mode's own physics. An open
		// corridor has none of these, so rejecting them costs nothing there
		// and stops a stray dash or slope frame from setting the envelope.
		if (p->m_isDashing || p->m_isOnSlope || p->m_isSideways) {
			sv.envRejected++; return;
		}

		const double slope = dy / dx;
		double& sl = sv.envMaxSlope[mode][size][spd][dir];
		// A max alone cannot be trusted: one bad frame sets it forever. Terminal
		// velocity is SUSTAINED, so the count of samples sitting at the top is
		// the evidence that a max is the envelope and not a stray.
		if (slope > sl) { if (slope > sl * 1.05) sv.envNearN[mode][size][spd][dir] = 0; sl = slope; }
		if (slope >= sl * 0.95) sv.envNearN[mode][size][spd][dir]++;
		double& vy = sv.envMaxVy[mode][size][spd][dir];
		vy = std::max(vy, std::abs(p->m_yVelocity));
		sv.envSumDx     [mode][size][spd][dir] += dx;
		sv.envSumRaw    [mode][size][spd][dir] += p->m_playerSpeed;
		sv.envSumGravity[mode][size][spd][dir] += p->m_gravity;
		sv.envSumFall   [mode][size][spd][dir] += p->m_fallSpeed;
		sv.envSumSpdMul [mode][size][spd][dir] += p->m_speedMultiplier;
		sv.envSumGravMod[mode][size][spd][dir] += p->m_gravityMod;
		if (p->m_isUpsideDown) sv.envUpsideN[mode][size][spd][dir]++;
		sv.envN         [mode][size][spd][dir]++;

		// Say so the moment a direction is nailed down, so a corridor can be
		// left as soon as it is done instead of being flown on guesswork and
		// checked afterwards.
		if (!sv.envLocked[mode][size][spd][dir] &&
		    sv.envN[mode][size][spd][dir] >= 150 &&
		    sv.envNearN[mode][size][spd][dir] >= 30) {
			sv.envLocked[mode][size][spd][dir] = true;
			static const char* kM[8] = {"cube","ship","wave","ufo",
			                            "ball","robot","spider","swing"};
			static const char* kS[5] = {"0.5x","1x","2x","3x","4x"};
			const bool both = sv.envLocked[mode][size][spd][0] &&
			                  sv.envLocked[mode][size][spd][1];
			log::info("Probe 16: {} {} {} {} LOCKED at {:.3f} units/step{}",
			          kM[mode], size ? "mini" : "norm", kS[spd],
			          dir ? "DOWN" : "UP", sl * want,
			          both ? " - corridor complete, move on." : "");
		}
	}
	void liveCeilingStep() {
		if (!g_config.liveCeilingReadout) return;
		auto* p  = m_player1;
		auto* pl = PlayLayer::get();
		if (!p || !pl || p->m_isDead) return;
		if (Solver::get().running || ProbeState::get().mode != Mode::Idle) return;

		static const char* kMode[8] = {"cube", "ship", "wave", "ufo",
		                               "ball", "robot", "spider", "swing"};
		const int mi = p->m_isShip ? 1 : p->m_isDart ? 2 : p->m_isBird ? 3
		             : p->m_isBall ? 4 : p->m_isRobot ? 5 : p->m_isSpider ? 6
		             : p->m_isSwing ? 7 : 0;

		static double prevVy = 0.0;
		const double vyBefore = prevVy;
		prevVy = p->m_yVelocity;

		const double y = p->getPositionY();
		const cocos2d::CCRect pr = p->getObjectRect();
		const double top = pr.origin.y + pr.size.height;

		// Stopped while moving up in world terms: the roof announcing itself.
		const double worldUp = p->m_isUpsideDown ? -vyBefore : vyBefore;
		const bool contact = worldUp > 0.5 && std::abs(p->m_yVelocity) < 1e-6;

		// The FLOOR side, which has only ever been derived and never measured:
		// stopped while moving down, with nothing under the player's feet.
		const bool floorHit = worldUp < -0.5 && std::abs(p->m_yVelocity) < 1e-6
		                   && !geometryBlockedAt(p->getPositionX(),
		                                         pr.origin.y - 4.0);
		if (floorHit) {
			auto& sv2 = Solver::get();
			const double bottom = pr.origin.y;
			if (sv2.haveEntryPortal) {
				const double predF = Solver::ceilingForPortal(sv2.entryPortalY,
				                                              sv2.entryPortalID)
				                   - sv2.entryBand();
				log::info("FLOOR   x{:7.0f} bottom {:7.1f}  predicted {:7.1f}  err {:+.1f}"
				          "  [{}{}]  {}",
				          p->getPositionX(), bottom, predF, bottom - predF,
				          kMode[mi], p->m_vehicleSize < 0.9f ? "-mini" : "",
				          std::abs(bottom - predF) < 0.5 ? "PASS" : "MISMATCH");
			} else {
				log::info("FLOOR   x{:7.0f} bottom {:7.1f}  (no portal entry recorded)",
				          p->getPositionX(), bottom);
			}
		}

		static int tick = 0;
		if (!contact && (++tick % std::max(1, g_config.liveCeilingEvery)) != 0) return;

		const double gl1 = m_groundLayer  ? m_groundLayer->getPositionY()  : 0.0;
		const double gl2 = m_groundLayer2 ? m_groundLayer2->getPositionY() : 0.0;

		// The 0x764 candidate on EVERY line, not just contacts. If it only holds
		// the ceiling at the moment of impact it is as reactive as
		// m_collidedTopMinY and no use for prediction; if it holds it continuously
		// through the section, it is the field we want.
		float v764 = 0.f;
		std::memcpy(&v764, reinterpret_cast<const uint8_t*>(this) + 0x764, sizeof v764);

		log::info("{} x{:7.0f} y{:7.1f} top{:7.1f} vy{:+6.2f} {}{}  |  x764 {:8.2f}  collTop {:7.1f} "
		          "collBot {:7.1f}  gl1 {:7.1f} gl2 {:7.1f}  portalY {:7.1f} "
		          "camH {:6.1f} free {} oob {}",
		          contact ? "CONTACT" : "       ",
		          p->getPositionX(), y, top, p->m_yVelocity,
		          kMode[mi], p->m_vehicleSize < 0.9f ? "-mini" : "", v764,
		          p->m_collidedTopMinY, p->m_collidedBottomMaxY, gl1, gl2,
		          m_gameState.m_portalY, m_cameraHeight,
		          m_gameState.m_isFreeMode ? 1 : 0, p->m_isOutOfBounds ? 1 : 0);

		if (contact) {
			// The rule, checked against this contact, in this mode.
			//
			// Every measurement so far has been SHIP. usingWallLimitedMode()
			// returning 1 for a ship says nothing about UFO, wave, ball, spider,
			// swing or any mini variant, and if the offset differs per mode the
			// clamp is wrong everywhere except the one mode it was fitted on.
			// This turns flying the level into the test.
			auto& sv = Solver::get();
			if (sv.haveEntryPortal) {
				const double pred = Solver::ceilingForPortal(sv.entryPortalY,
				                                             sv.entryPortalID);
				const double err  = top - pred;
				log::info("        RULE portal {:.1f} -> predicted {:.1f}, actual {:.1f}"
				          "  err {:+.1f}  [{}{}]  {}",
				          sv.entryPortalY, pred, top, err,
				          kMode[mi], p->m_vehicleSize < 0.9f ? "-mini" : "",
				          std::abs(err) < 0.5 ? "PASS" : "MISMATCH <-- the rule does "
				                                         "not hold for this mode");
			} else {
				log::info("        RULE no portal entry recorded, so nothing to "
				          "predict from - this contact cannot check the rule.");
			}

			// Probe 13: the exhaustive test.
			//
			// Every named candidate has been eliminated, but that only ever tested
			// fields someone thought to enumerate. At a contact the ceiling is
			// KNOWN, so scan the raw bytes of both objects for anything holding it
			// and print the offsets. If a field stores the ceiling this finds it;
			// if nothing does, it is conclusively not stored in either object and
			// the arithmetic is the only way to get it.
			//
			// Read-only, and only at a contact, so it costs nothing in normal play.
			{
				const double want[2] = { top, top - Solver::get().entryBand() };
				static const char* kWhat[2] = { "CEILING", "floor" };
				for (int w = 0; w < 2; w++) {
					int hits = 0;
					auto scan = [&](const char* name, const void* base, size_t bytes) {
						const auto* b = reinterpret_cast<const uint8_t*>(base);
						if (!b) return;
						for (size_t off = 0; off + 4 <= bytes; off += 4) {
							float f;
							std::memcpy(&f, b + off, sizeof f);
							if (std::isfinite(f) && std::abs(f - want[w]) < 0.01) {
								log::info("        SCAN {} float {} +0x{:X} = {:.2f}",
								          kWhat[w], name, off, f);
								hits++;
							}
							if (off + 8 <= bytes && (off % 8) == 0) {
								double d;
								std::memcpy(&d, b + off, sizeof d);
								if (std::isfinite(d) && std::abs(d - want[w]) < 0.01) {
									log::info("        SCAN {} double {} +0x{:X} = {:.2f}",
									          kWhat[w], name, off, d);
									hits++;
								}
							}
						}
					};
					scan("gameLayer", this, 0x5000);
					scan("player",    p,    0x900);
					if (!hits)
						log::info("        SCAN {} {:.1f}: no field in either object "
						          "holds this value.", kWhat[w], want[w]);
				}
			}

			// Everything else named anywhere in this investigation, so one flight
			// tests the lot instead of a build per candidate.
			{
				double obbY = 0.0, obbH = 0.0, obbCY = 0.0;
				if (m_cameraObb2) {
					const cocos2d::CCRect r = m_cameraObb2->getBoundingRect();
					obbY = r.origin.y; obbH = r.size.height;
					obbCY = m_cameraObb2->m_center.y;
				}
				const double objLayerY = m_objectLayer ? m_objectLayer->getPositionY() : 0.0;
				log::info("        obb.y {:8.2f} (top-{:8.2f})  obb.h {:8.2f}  obb.cy "
				          "{:8.2f}   objectLayer.y {:8.2f} (top-{:8.2f})",
				          obbY, top - obbY, obbH, obbCY, objLayerY, top - objLayerY);
				log::info("        maxGameplayY {:8.2f}  sectionYFactor {:8.2f}  "
				          "camUnzoomedX {:8.2f}  halfCamW {:8.2f}",
				          m_maxGameplayY, m_sectionYFactor,
				          m_cameraUnzoomedX, m_halfCameraWidth);
				log::info("        gs.cameraZoom {:8.3f}  gs.cameraAngle {:8.3f}  "
				          "gs.middleGroundOffsetY {:8.2f}",
				          m_gameState.m_cameraZoom, m_gameState.m_cameraAngle,
				          m_gameState.m_middleGroundOffsetY);
				log::info("        player yStart {:8.2f} (top-{:8.2f})  fallStartY "
				          "{:8.2f} (top-{:8.2f})  gravity {:6.3f}",
				          p->m_yStart, top - p->m_yStart,
				          p->m_fallStartY, top - p->m_fallStartY, p->m_gravity);
			}

			// Probe 14: name the field at +0x764.
			//
			// MEASURED: a float at gameLayer +0x764 held the ceiling EXACTLY on
			// all 8 contacts across 6 portal heights - 480, 540, 630, 540, 450,
			// 390, 390, 390 - including the ground-clamped case. That is the
			// field; it just has no name yet, because it sits inside the by-value
			// m_gameState and Geode has most of GJGameState unnamed.
			//
			// Printing each named neighbour's runtime offset identifies it, or
			// proves it is one of the unnamed slots and we address it by offset
			// with the neighbours as a version guard.
			//
			// Prints once, not per contact.
			{
				static bool once = false;
				if (!once) {
					once = true;
					const auto* base = reinterpret_cast<const uint8_t*>(this);
					auto off = [&](const void* f) {
						return static_cast<long>(reinterpret_cast<const uint8_t*>(f) - base);
					};
					log::info("        --- offsets from `this` (target is 0x764) ---");
					log::info("        m_gameState              +0x{:X}", off(&m_gameState));
					log::info("        gs.m_cameraZoom          +0x{:X}", off(&m_gameState.m_cameraZoom));
					log::info("        gs.m_middleGroundOffsetY +0x{:X}", off(&m_gameState.m_middleGroundOffsetY));
					log::info("        gs.m_unkFloat2           +0x{:X}", off(&m_gameState.m_unkFloat2));
					log::info("        gs.m_unkFloat3           +0x{:X}", off(&m_gameState.m_unkFloat3));
					log::info("        gs.m_unkFloat4           +0x{:X}", off(&m_gameState.m_unkFloat4));
					log::info("        gs.m_unkUint1            +0x{:X}", off(&m_gameState.m_unkUint1));
					log::info("        gs.m_portalY             +0x{:X}", off(&m_gameState.m_portalY));
					log::info("        gs.m_unkInt13            +0x{:X}", off(&m_gameState.m_unkInt13));
					log::info("        gs.m_unkFloat5           +0x{:X}", off(&m_gameState.m_unkFloat5));
					log::info("        gs.m_unkFloat6           +0x{:X}", off(&m_gameState.m_unkFloat6));
					log::info("        gs.m_unkFloat7           +0x{:X}", off(&m_gameState.m_unkFloat7));
					log::info("        gs.m_unkFloat8           +0x{:X}", off(&m_gameState.m_unkFloat8));
					log::info("        gs.m_cameraAngle         +0x{:X}", off(&m_gameState.m_cameraAngle));
					log::info("        gs.m_isFreeMode          +0x{:X}", off(&m_gameState.m_isFreeMode));
					// The later half of GJGameState, where 0x764 must actually be:
					// m_gameState starts at 0x1A8 and m_cameraHeight is at 0x3260,
					// so the target is 1468 bytes into the game state.
					auto& gs = m_gameState;
					log::info("        gs.m_targetCameraAngle   +0x{:X}", off(&gs.m_targetCameraAngle));
					log::info("        gs.m_timeWarp            +0x{:X}", off(&gs.m_timeWarp));
					log::info("        gs.m_totalTime           +0x{:X}", off(&gs.m_totalTime));
					log::info("        gs.m_levelTime           +0x{:X}", off(&gs.m_levelTime));
					log::info("        gs.m_currentProgress     +0x{:X}", off(&gs.m_currentProgress));
					log::info("        gs.m_lastActivatedPortal1+0x{:X}", off(&gs.m_lastActivatedPortal1));
					log::info("        gs.m_lastActivatedPortal2+0x{:X}", off(&gs.m_lastActivatedPortal2));
					log::info("        gs.m_cameraPosition      +0x{:X}  = ({:.1f}, {:.1f})",
					          off(&gs.m_cameraPosition), gs.m_cameraPosition.x, gs.m_cameraPosition.y);
					log::info("        gs.m_levelFlipping       +0x{:X}", off(&gs.m_levelFlipping));
					log::info("        gs.m_unkFloat9           +0x{:X}", off(&gs.m_unkFloat9));
					log::info("        gs.m_cameraEdgeValue0    +0x{:X}  = {}", off(&gs.m_cameraEdgeValue0), gs.m_cameraEdgeValue0);
					log::info("        gs.m_cameraEdgeValue1    +0x{:X}  = {}", off(&gs.m_cameraEdgeValue1), gs.m_cameraEdgeValue1);
					log::info("        gs.m_cameraEdgeValue2    +0x{:X}  = {}", off(&gs.m_cameraEdgeValue2), gs.m_cameraEdgeValue2);
					log::info("        gs.m_cameraEdgeValue3    +0x{:X}  = {}", off(&gs.m_cameraEdgeValue3), gs.m_cameraEdgeValue3);
					log::info("        gs.m_unkUint10           +0x{:X}", off(&gs.m_unkUint10));
					log::info("        gs.m_cameraStepDiff      +0x{:X}  = ({:.1f}, {:.1f})",
					          off(&gs.m_cameraStepDiff), gs.m_cameraStepDiff.x, gs.m_cameraStepDiff.y);
					log::info("        gs.m_unkFloat10          +0x{:X}", off(&gs.m_unkFloat10));
					log::info("        gs.m_timeModRelated      +0x{:X}", off(&gs.m_timeModRelated));
					log::info("        gs.m_unkUint13           +0x{:X}", off(&gs.m_unkUint13));
					log::info("        gs.m_unkPoint32          +0x{:X}  = ({:.1f}, {:.1f})",
					          off(&gs.m_unkPoint32), gs.m_unkPoint32.x, gs.m_unkPoint32.y);
					log::info("        gs.m_cameraPosition2     +0x{:X}  = ({:.1f}, {:.1f})",
					          off(&gs.m_cameraPosition2), gs.m_cameraPosition2.x, gs.m_cameraPosition2.y);
					log::info("        gs.m_unkUint14           +0x{:X}", off(&gs.m_unkUint14));
					log::info("        m_cameraHeight           +0x{:X}", off(&m_cameraHeight));
					log::info("        --- and the value at 0x764 right now ---");
					float v764;
					std::memcpy(&v764, base + 0x764, sizeof v764);
					log::info("        *(float*)(this+0x764) = {:.2f}", v764);
				}
			}

			double e[6];
			for (int t = 0; t < 6; t++) e[t] = getCameraEdgeValue(t);
			log::info("        cameraEdge 0..5: {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} "
			          "{:.1f}   <- the ceiling is at y {:.1f} (player top edge)",
			          e[0], e[1], e[2], e[3], e[4], e[5], top);

			// The camera-offset family, which is the last untested group. Printed
			// as the raw value AND as `top - value`, because the question is
			// always whether some difference is constant across sections - that
			// is what a usable anchor looks like, and eyeballing raw numbers has
			// twice led me to the wrong conclusion.
			const double ho  = m_cameraHeightOffset;
			const double tho = m_targetCameraHeightOffset;
			const double uho = m_cameraUnzoomedHeightOffset;
			log::info("        heightOffset {:8.2f} (top-{:8.2f})   target {:8.2f} "
			          "(top-{:8.2f})   unzoomed {:8.2f} (top-{:8.2f})",
			          ho, top - ho, tho, top - tho, uho, top - uho);
			log::info("        camH {:.1f} camW {:.1f} flip {:.2f}  calcTargetHO {}  "
			          "wallLimited {}  flying {}  freeMode {}",
			          m_cameraHeight, m_cameraWidth, m_cameraFlip,
			          m_calculateTargetHeightOffset ? 1 : 0,
			          p->usingWallLimitedMode() ? 1 : 0,
			          p->isFlying() ? 1 : 0,
			          m_gameState.m_isFreeMode ? 1 : 0);
		}
	}

	// The portal the player actually touched. This is the whole anchor: it fires
	// on ENTRY, so a portal that is merely flown past never sets anything and
	// fake or stacked portals need no special handling.
	void playerWillSwitchMode(PlayerObject* player, GameObject* object) {
		GJBaseGameLayer::playerWillSwitchMode(player, object);
		if (!object || !player || player != m_player1) return;

		auto& sv = Solver::get();
		sv.entryPortalY    = object->getPositionY();
		sv.entryPortalX    = object->getPositionX();
		sv.haveEntryPortal = true;
		sv.entryPortalMode = static_cast<int>(classifyMode(player));
		// Recorded because the band depends on the mode entered, and ModeClass is
		// too coarse to carry it - ball and cube are both ModeClass::Ground.
		sv.entryPortalID   = object->m_objectID;
		sv.portalEntries++;

		if (g_config.liveCeilingReadout) {
			// GD's own answer for the same question, for cross-checking ours.
			const double gdTarget = getTargetFlyCameraY(object);
			const double ceil = Solver::ceilingForPortal(sv.entryPortalY,
			                                             sv.entryPortalID);
			log::info("PORTAL objID {} at y {:.1f}  ->  raw {:.1f} snapped to ceiling "
			          "{:.1f}, floor {:.1f} (band {:.0f})",
			          object->m_objectID, sv.entryPortalY,
			          sv.entryPortalY + ceilingOffsetForPortal(sv.entryPortalID),
			          ceil, ceil - sv.entryBand(), sv.entryBand());
			// getTargetFlyCameraY returned exactly the portal y on every entry so
			// far (251->251, 345->345), so it looks like a position echo rather
			// than a camera target. Logged with the derived values beside it in
			// case it diverges on a portal shape we have not tried.
			log::info("        getTargetFlyCameraY {:.1f}  (portalY {:.1f}, so "
			          "target-portalY {:.1f}; predicted ceiling {:.1f})",
			          gdTarget, sv.entryPortalY, gdTarget - sv.entryPortalY, ceil);
		}
	}

	void update(float dt) {
		pollHotkeys();
		liveCeilingStep();
		climbEnvelopeStep();

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
		// Same reason as the searches: a variant that dies is the normal
		// outcome, and the sweep has to keep running to record it and reset.
		const bool sweeping = st.mode == Mode::Sweep && Sweep::get().running;
		const bool active  = pl && m_player1 &&
		                     (solving || sweeping ||
		                      (!m_player1->m_isDead && !st.finished));

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
			// Fresh per replay, so the same macro run against two different maps
			// can be compared directly. Here rather than in startVerify, which is
			// declared above Solver and cannot see it.
			if (st.stepCounter == 0) {
				auto& sv = Solver::get();
				sv.pathAuditSteps = sv.pathOffMap = sv.pathDead = sv.pathOutWindow = 0;
				sv.exprMisses.clear();
				sv.exprPrevPressed = sv.exprHavePrev = false;
				sv.exprLastBranch = 0; sv.exprGeoLo = sv.exprGeoHi = 0.0;
				sv.exprAirSteps = sv.exprToggles = 0;
				sv.exprOffered = sv.exprMissed = sv.exprOfferPoints = 0;
				sv.exprMissCadence = sv.exprMissUnchanged = sv.exprMissNoRead = 0;
				sv.pathFirstOffX = sv.pathFirstDeadX = sv.pathFirstOutX = -1.0;
				sv.pathWorstOff  = sv.pathWorstOffX  = 0.0;
				std::memset(sv.pathOffBin, 0, sizeof(sv.pathOffBin));
				std::memset(sv.pathOffBinWorst, 0, sizeof(sv.pathOffBinWorst));
			}
			const size_t idx = static_cast<size_t>(st.stepCounter);

			// Past the end of the macro: keep stepping with no input for a short
			// grace period before judging.
			//
			// Completion does not always register on the same step it did during
			// the solve - Time Machine's end animation fired one step later on
			// replay, so the macro ran out first and a genuine clear was reported
			// as a death at the final step.
			if (idx >= st.scripted.size()) {
				const size_t over = idx - st.scripted.size();
				if (st.finished) {
					finishVerify(true, static_cast<int>(idx), pl->getCurrentPercent());
					return;
				}
				if (m_player1 && m_player1->m_isDead) {
					finishVerify(false, static_cast<int>(idx), pl->getCurrentPercent());
					return;
				}
				if (over >= static_cast<size_t>(kVerifyGraceSteps)) {
					finishVerify(false, static_cast<int>(idx), pl->getCurrentPercent());
					return;
				}
				GJBaseGameLayer::update(static_cast<float>(kPhysicsDt));
				st.stepCounter++;
				return;
			}
			const int steps = g_config.physicsFix ? g_config.stepsPerFrame : 1;
			for (int i = 0; i < steps; i++) {
				const size_t k = static_cast<size_t>(st.stepCounter);
				if (k >= st.scripted.size()) break;
				const bool pressed = st.scripted[k] != 0;
				applyInput(pressed);
				GJBaseGameLayer::update(g_config.physicsFix
				                        ? static_cast<float>(kPhysicsDt) : dt);
				st.stepCounter++;
				pathAuditStep(m_player1);
				compareAgainstSolvePath(pressed, k);
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

		// Same frame-budget loop as the solver: the sweep is ~36 full replays and
		// would take twenty minutes at real-time pace.
		if (st.mode == Mode::Sweep) {
			auto& sw = Sweep::get();
			const uint64_t frameStart = probe::nowTicks();
			while (sw.running) {
				if (!sweepStep()) break;
				if (probe::ticksToMicros(probe::nowTicks() - frameStart) > kSolverFrameBudgetUs)
					break;
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
		// The layout guard for the unnamed ceiling field. Once per level.
		if (auto* gl = GJBaseGameLayer::get()) {
			const auto* base = reinterpret_cast<const uint8_t*>(gl);
			const long gsOff = static_cast<long>(
				reinterpret_cast<const uint8_t*>(&gl->m_gameState) - base);
			const long chOff = static_cast<long>(
				reinterpret_cast<const uint8_t*>(&gl->m_cameraHeight) - base);
			auto& sv = Solver::get();
			sv.ceilFieldOk = gsOff == g_config.geomGuardGameStateOff &&
			                 chOff == g_config.geomGuardCameraHeightOff;
			if (!sv.ceilFieldOk)
				log::warn("Ceiling field: layout guard FAILED (m_gameState +0x{:X} "
				          "expected +0x{:X}, m_cameraHeight +0x{:X} expected +0x{:X}). "
				          "The struct has changed, so offset 0x{:X} cannot be trusted "
				          "- falling back to the derived rule.",
				          gsOff, g_config.geomGuardGameStateOff,
				          chOff, g_config.geomGuardCameraHeightOff,
				          g_config.geomCeilingFieldOffset);
		}

		// Probe 7 on every level load. It is read-only and touches no search
		// state, and having the map on disk for every level a stall happens on is
		// worth more than the load-time file write costs.
		geometryDump();

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

	// Suppressing the death entirely is what makes a direct loadFromCheckpoint
	// usable: nothing dies, so nothing needs reviving, and the practice respawn -
	// which the wiki says places ship/UFO/wave checkpoints "a set distance behind
	// the icon" - is never involved.
	void destroyPlayer(PlayerObject* player, GameObject* object) {
		if (ProbeState::get().suppressDeath) return;
		// Probe 19. One hash and two map bumps, only while solving. Bucketed at
		// one block so a cell is a place on screen rather than a coordinate.
		if (Solver::get().running && player == m_player1 && !player->m_isDead) {
			auto& sv = Solver::get();
			const int xb = static_cast<int>(std::floor(player->getPositionX() / 30.0));
			const int yb = static_cast<int>(std::floor(player->getPositionY() / 30.0));
			const uint64_t k = (static_cast<uint64_t>(static_cast<uint32_t>(xb)) << 32)
			                 |  static_cast<uint64_t>(static_cast<uint32_t>(yb));
			sv.deathCells[k]++;
			sv.deathVySum[k] += player->m_yVelocity;
			sv.deathsRecorded++;
		}
		PlayLayer::destroyPlayer(player, object);
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
