# GD Solver

**A search that plays Geometry Dash by driving the real game.**

GD Solver is a [Geode](https://geode-sdk.org/) mod that turns a running copy of Geometry Dash
into a deterministic state machine — stepping its physics headlessly at ~30x real time, saving
and restoring exact game states, and systematically searching the resulting tree for an input
sequence that clears the level.

Every input in its output was chosen by search. Nothing is recorded from human play, and no
level knowledge is supplied by hand.

---

## Results

All fifteen of the first main levels, solved end to end — one default configuration for the whole
board, no hand-tuning between levels, no hotkeys pressed.

| # | Level | Deaths | Physics steps | Wall clock | Throughput |
|---|---|---|---|---|---|
| 1 | Stereo Madness | 350 | 28,728 | 3 s | 7,777 steps/s (32x) |
| 2 | Back on Track | 288 | 26,302 | 3 s | 9,013 steps/s (38x) |
| 3 | Polargeist | 459 | 37,493 | 4 s | 8,158 steps/s (34x) |
| 4 | Dry Out | 347 | 29,260 | 4 s | 8,662 steps/s (36x) |
| 5 | Base After Base | 281 | 27,646 | 3 s | 9,243 steps/s (39x) |
| 6 | Can't Let Go | 478 | 43,430 | 5 s | 9,425 steps/s (39x) |
| 7 | Jumper | 436 | 31,204 | 4 s | 7,127 steps/s (30x) |
| 8 | Time Machine | 546 | 35,072 | 6 s | 6,302 steps/s (26x) |
| 9 | Cycles | 419 | 27,725 | 5 s | 6,067 steps/s (25x) |
| 10 | xStep | 2,153 | 77,842 | 19 s | 3,957 steps/s (17x) |
| 11 | Clutterfunk | 2,036 | 128,494 | 20 s | 6,377 steps/s (27x) |
| 12 | Theory of Everything | 1,324 | 50,231 | 13 s | 3,951 steps/s (17x) |
| 13 | Electroman Adventures | 2,024 | 77,302 | 19 s | 4,019 steps/s (17x) |
| 14 | **Clubstep** — *rated Demon* | **5,558** | 163,812 | 52 s | 3,143 steps/s (13x) |
| 15 | Electrodynamix | 2,263 | 80,838 | 22 s | 3,575 steps/s (15x) |
| | **Total** | **18,962** | **865,379** | **~3 min** | |

**Clubstep is a rated Demon** — the first Demon in the main level list. Its solution has been
independently verified: replayed from frame 0 in normal mode, practice off, savestates disabled,
zero diverging steps. *A solution found via savestates is a savestate artifact until it survives
that test.*

The board is reproducible to the death count — the same build on the same levels produces the
same fifteen numbers every run — and physics determinism has been confirmed bit-for-bit across
sessions **eleven days apart**.

---

## Why this is interesting

The Geometry Dash tooling ecosystem is roughly fifty projects, and essentially all of them are
macro, replay, or TAS tools: they play back input a human authored. The distinction this project
cares about is **who decided the input**, and by that measure the prior art is thin.

| Prior work | Approach | Best result | Limitation |
|---|---|---|---|
| **UCSB `geometry-dash-ai`** | DQN / MoE-DQN on pixels | Levels 1–2 of 21, win rate < 100% | RL plateaus; no completeness guarantee |
| **Stanford CS231n (2017)** | CNN + DQN + imitation learning | *"never completed a level"* | — |
| **DashBot 3.0** | Memory-read search | Stereo Madness | Needs a **hand-written portal string per level**; GD 2.11 only; archived |
| **GD-AI / AutoMacro** | (1+1) hill climber | One unrated Insane | The genome *is* a macro; the advanced version never shipped |
| **Pathfinder** *(closest prior art)* | Custom C++ physics reimplementation + randomized hill climbing | Solves classic-era levels in seconds | Simulator supports **up to GD 1.7–1.9**; no robot, spider, swing, duals, triggers, or *anything from 2.2*; search is incomplete by construction |

Pathfinder is the right category and the honest comparison. It is genuinely fast — its game-state
vector makes savestates O(1), which the live game can never match. But every one of its
limitations follows from a single decision: **it reimplements physics instead of driving the
game.** Each mechanic has to be hand-written, so three of the eight game modes and the entire 2.2
feature set remain unimplemented after 243 beta releases.

GD Solver makes the opposite trade, deliberately:

> **Drive the live game and you inherit every mechanic for free, permanently — including
> mechanics that do not exist yet.** The cost is that a branch is slower than truncating a vector.
> This project trades raw speed for total coverage and search completeness.

That trade is what puts a Demon in range. Every Demonlist-tier level uses 2.1+ features, triggers,
duals, and modes a reimplementation would have to add one at a time.

Full landscape research, with sources and verification notes, is in
[`docs/PROJECT_PLAN.md` §3](docs/PROJECT_PLAN.md).

---

## How it works

### The premise

Geometry Dash is a deterministic state machine. Given a state *S* and an input *I* (pressed or not)
for one physics step, the engine produces exactly one *S′*. So a level is a **binary tree**: the
root is the level start, each node has two children, some nodes are death, and one set of nodes is
"reached 100%". A solution is a root-to-goal path that never touches a death node — which, written
out as `(frame, pressed)` pairs, is a macro.

Naively that tree is 2^30000 for a long level. Three things collapse it:

1. **Pruning.** A branch that dies discards its entire subtree, unexplored.
2. **Decision points.** In cube mode, pressing mid-air does nothing. Branching only where input can
   change the outcome turns *two-per-frame* into *two-per-landing*.
3. **Savestates.** Trying an alternative costs a restore, not a replay from level start — so branch
   cost stops scaling with level length.

### Four layers

```
  Layer 4   Search        DFS + iterative deepening on toggle count, geometry-guided ordering
  Layer 3   Savestate     createCheckpoint + full PlayerObject image + container state
  Layer 2   Headless      N physics steps per rendered frame, rendering suppressed
  Layer 1   Determinism   fixed 1/240 dt — if this fails, nothing above it works
```

**Layer 1 — determinism.** `getModifiedDelta` is hooked (calling the original *first*, because it
mutates layer state) and `GJBaseGameLayer::update` is fed our own delta rather than merely
returning one. Returning `1/240` collapses GD's internal sub-stepping to exactly one physics step
per call — the single-tick primitive everything else is built on. Physics dt stays at the true
gameplay value, so a macro found here is valid in ordinary play.

**Layer 2 — headless speed.** Visibility updates are suppressed and N physics steps run per
rendered frame, under a 12 ms per-frame wall-clock budget so the window keeps pumping messages and
the process never goes Not Responding.

**Layer 3 — savestates.** This was the hardest layer, and the finding is worth stating precisely:

> **A byte-exact `PlayerObject` is not a byte-exact player.**

GD spreads player state across at least four places at three levels of the object graph. A correct
restore needs all of it:

| State | Lives in | Why a field table misses it |
|---|---|---|
| 196 scalar fields | `PlayerObject` | `loadFromCheckpoint` assigns only a subset |
| `m_holdingButtons` | a `gd::map` member | a container, not a scalar |
| `m_queuedButtons` | `GJBaseGameLayer` | **a different object entirely** |
| node position | the `CCNode` base | inherited, not `PlayerObject`'s own |
| `m_cameraFlip` | `GJBaseGameLayer` | a `float` — transition progress, not a flag |

The input queue was the bug hiding behind all the others. `handleButton` does not touch the player;
it appends to a layer-level vector that `processQueuedButtons` drains during `update`. That queue is
in no checkpoint and `resetLevel` clears it, so every restore silently discarded the input in
flight and the next press landed one step late. In cube modes a single frame washes out, which is
why ~18,000 cube steps looked clean — but in ship mode, where hold controls acceleration every
frame, it split the trajectory on the spot and never re-converged.

Restores are now **bit-identical in every game mode**, and a full Stereo Madness solve replays from
frame 0 with zero diverging steps out of 20,330.

**Layer 4 — search.** DFS with backtracking over decision points, pruning on death, with several
mechanisms layered on. Each was added because a measurement demanded it:

- **Mode-dependent branching.** The naive `on_ground || touching_ring` rule is cube logic. Applied
  globally it produces a solver that *cannot* solve ship, wave, UFO or swing, because it never
  branches mid-air where those modes need input every frame. Branching policy is per mode class.
- **Toggle semantics.** In air modes the action is "continue vs **toggle**", not "release vs hold".
  Under release-vs-hold a 32-step ship hold requires eight consecutive segments each flipped — the
  very last leaf DFS would reach. Under toggle semantics, "hold from ship entry onward" is *one*
  toggle, and long runs of a single action become the cheapest thing to explore. Same search space,
  better exploration order, completeness preserved.
- **Iterative deepening on toggle count.** The budget starts at 1 and rises on exhaustion, so the
  simplest paths are tried first.
- **Sliding commit floor.** Decisions more than ~240 steps behind the frontier freeze. Without it,
  the search fell back into an already-solved cube prefix and re-searched it exhaustively — 2.9
  million steps with best% frozen.
- **A geometry map (`GeoMap`).** The level is sliced at object edges into variable-width columns;
  each slice stores its blocked spans and the free intervals between them, and a right-to-left
  liveness sweep marks the intervals from which the end of the level is still reachable. The search
  uses it to decide *when* the corridor ahead has changed enough to be worth a decision, and to
  order branches toward the gap it has to thread.
- **The portal roof.** Vehicle portals imply a ceiling the level geometry never draws. Without
  modelling it, the map reported hundreds of units of phantom headroom — measured at 700+ units on
  Clubstep, with 36% of free intervals reaching into space the game would never allow. Closing this
  is what took the board from 14/15 to **15/15**.

---

## Setup

### Requirements

| | |
|---|---|
| OS | Windows — the mod uses Win32 hotkey polling and `psapi` |
| Geometry Dash | **2.2081** |
| [Geode](https://geode-sdk.org/) | **5.8.2** |
| Toolchain | MSVC with C++23, CMake ≥ 3.21 |
| Environment | `GEODE_SDK` pointing at your Geode SDK checkout |

### Build

```sh
cd mod
geode build
```

That compiles, packages `colin.gd-solver.geode`, and installs it into your GD mods directory.

> **Delete `mod/build/` and rebuild clean after any change to `CMakeLists.txt` or `mod.json`.**

### Before running

- **Disable any other mod that hooks `update`** — RLDash and QOLMod in particular. Geode *chains*
  `update` hooks, so another mod's `glReadPixels` or per-tick logging runs inside the stepping path.
  This alone once accounted for apparent nondeterminism (4 distinct trace hashes collapsing to 1
  once removed) and it invalidates every timing measurement.
- Low Detail Mode on; particles, shaders, glow, ship trail and object glow off.
- Hide the attempt counter and progress bar.

### Run

1. Open a level.
2. Press **F2**. The solver takes over and searches.
3. On success it writes `<Level>_solution.txt` to the mod's save directory and logs
   `================ SOLVED ================`.
4. Press **F4** to verify: the macro is replayed from frame 0 in normal mode with practice and
   savestates off. **This is the only result that counts.**

Progress is logged to `<GD>/geode/logs/`, sampled — never per tick.

---

## Controls

**Solver**

| Key | Action |
|---|---|
| `F2` | Start / stop the solver |
| `F4` | Verify the current solution from frame 0 (normal mode, no savestates) |
| `F8` | Toggle periodic resync |
| `F9` | Write and watch the best-so-far path at normal speed |

**Search configuration** — each logs what it switched to; press, then `F2`.

| Key | Toggles |
|---|---|
| `M` | Air branching mode — interval / corridor-change / tight-corridor |
| `B` | Air branch ceiling (`airBranchIntervalMax`) |
| `Z` | Map roof mode — highest-solid / prefix-max / most-recent |
| `V` | Velocity pruning |
| `P` | Steer lead scaling |
| `Q` | Reach-clamped steer target |
| `J` | Mute the steer entirely (ablation) |
| `F` | Branch-rule carried state (`geoStateRestore`) |
| `4` | Mode / ground-state restore on a reposition |
| `Y` | Tap-state survival across a restore |
| `A` | Archive-based escape (Go-Explore style restart) |
| `N` `O` `U` `T` `D` `E` `X` `K` `L` | Steering, forward scan, deadband and reach-mode variants |

**Probes and measurement** — read-only.

| Key | Probe |
|---|---|
| `2` | Death map — where the search is dying |
| `3` | Decision dump for the best path |
| `G` / `H` | Geometry dump / dead-end map |
| `R` | Geometry reachability sweep |
| `S` | Vertical climb envelope |
| `W` | Motion-model census |
| `C` | Corridor sweep |
| `F10` / `F12` | Restore-fidelity tests — direct load / full |
| `F11` | Savestate cost |
| `F1` `F5` `F6` `F7` | Determinism sweeps, input record and replay |

---

## Output

Everything lands in the mod's save directory, namespaced by level.

| File | Contents |
|---|---|
| `<Level>_solution.txt` | The macro — one `0`/`1` per physics step, 80 per line, with a header comment |
| `<Level>_best.txt` | Best path so far, rewritten as progress improves |
| `<Level>_partial.txt` | Written if the search is stopped or hits the depth cap |
| `<Level>_cells.txt` | Cell census from the search |

The macro format is deliberately plain text so a recording can be eyeballed and hand-edited.
**GDR export is not implemented yet** — see Next steps.

---

## Repository layout

```
docs/
  PROJECT_PLAN.md    Research question, landscape survey, verified bindings, Phase 0 results,
                     and every hard-won bug. Read this first.
  MECHANISMS.md      The settled register: every mechanism measured, with the conditions it was
                     measured under, split PROVEN / DISPROVEN / open — plus ordering revisions.
  STEER_TARGET.md    Design for the steer target and the reach model.
mod/
  src/main.cpp          The solver, the geometry map, every probe, and all the hooks (~11.6k lines)
  src/Probe.hpp         Measurement substrate — no allocation, no formatting and no I/O in the
                        stepping path; determinism traces hash raw bit patterns, never text
  src/PlayerFields.inc  The 196 PlayerObject fields a direct checkpoint load leaves untouched
  src/Socket.cpp        Optional localhost progress channel (never called per tick)
```

---

## How this project is run

Three rules shaped the codebase more than any algorithm did.

**1. Measure before concluding.** Reasoning from level outcomes produced confident, wrong claims
repeatedly. A mechanism was deleted as "unreachable" while holding the best Clubstep result in the
project's history. A "regression" was blamed on a code path that turned out to be hardcoded off.
The portal roof — the mechanism that eventually closed the board — sat written off for eleven days
because it had been scored on a build with a state-restoration bug. *A correct mechanism judged by
a broken instrument reads as a failure.*

**2. Results live in the register, not in commit messages.** `docs/MECHANISMS.md` §12 records every
measured outcome **with the conditions it was measured under**. That rule exists because the two
most expensive mistakes in the project were both caused by a tested configuration not being the
default configuration, with the result buried in a commit message. *"Works" without "under what" is
how both of those happened.*

**3. Build the instrument that can fail.** The strongest recurring bug class here is **history
dependence**: search state that the decision path *reads* but the restore path does not *put back*,
so a replay after a rewind manufactures decision points the original pass never had. Three separate
instances were found. The lesson from the third is the sharpest — the counter watching the fix
compared search state against *search state*, which is circular. It measured exposure rather than
residual, and an off-by-one walked clean through a full passing suite. The replacement compares the
restored **character** against the restored **search state** — two different sources — and reads
zero mismatches across 18,962 repositions.

---

## Current limitations

- **Windows only.** Win32 hotkey polling and `psapi`.
- **Pinned to GD 2.2081 / Geode 5.8.2.** Field offsets are version-specific.
- **Cube, ship, ball, UFO and wave are exercised.** Robot, spider, swing and dual sections are not
  yet supported by the branching policy. Robot in particular needs a different action model: jump
  height varies continuously with hold duration, so the action is "hold for N frames", not a binary
  press.
- **Four `bandHeightForPortal` constants are assumed rather than measured** — wave, robot, spider
  and swing. Ship, ball and UFO are measured. These became load-bearing when the roof became
  default-on.
- **Strict completeness is traded away by the escape heuristic.** After 400 deaths with no
  improvement the search abandons part of the stack to force exploration further back. Setting
  `stallLimit = 0` restores pure DFS. This can only cause a *missed* solution, never a false one —
  a macro still has to replay from frame 0 to count.
- **No GDR export yet.** Output is the plain-text step format above.

---

## Next steps

1. **Measure the four unmeasured portal band heights.** They became load-bearing when the roof
   became default-on.
2. **Re-test the steer-lead and reach-clamp variants on the corrected map.** Both were retired on
   arithmetic computed against the *phantom-inflated* corridor, so the reasoning that killed them
   may no longer hold — which is exactly the trap the roof itself fell into.
3. **Robot, spider, swing and dual support** — the three modes Pathfinder never implemented, plus
   duals.
4. **GDR export**, so solutions replay in standard community tooling.
5. **Harder levels**, toward a Demonlist demon.

---

## Credits and licensing

Built on [Geode](https://geode-sdk.org/) and its community bindings.

Prior art was studied and **not copied** — ToastyReplay (no license, all rights reserved),
Pathfinder and `gd-sim` (explicitly not licensed for redistribution), and zBot (restrictive EULA).
Technique was read and understood; every mechanism here is a reimplementation.

peony's writeup *"60tps In 2.2 Is a Lie"* is the source for the determinism fix. camila314's `gdp`
physics decompilations were used to cross-check measured constants — the measured mini/normal ship
ratio of **1.1765** against a documented `m_vehicleSize = 0.85` (`1/0.85 = 1.17647`) is five-digit
agreement with a game constant the solver never reads.
