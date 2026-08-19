# RLDash → GD Solver: Project Plan & Full Context

**Read this entire document before touching code.** It contains hard-won empirical findings, verified binding addresses, corrections to widely-circulated but wrong techniques, and a full account of what was already tried and why it was set aside. Several findings here contradict what you would conclude from reading public repos, and each contradiction cost real debugging time to discover.

### Orientation for whoever picks this up

**One-line summary:** build a Geode mod that drives the *live* Geometry Dash game as a deterministic state machine, stepping physics headlessly at high speed with savestates, and systematically searching for an input sequence that clears the level. Export as a GDR macro.

**If you read nothing else, read these four:**
- **§5.2** — the physics/determinism fix. The widely-copied version of this is wrong and will silently break everything above it.
- **§5.4 + §5.6a** — what a savestate must contain. Vanilla checkpoints omit Y velocity, and a production field list already exists.
- **§7 Phase 0** — all the real risk lives there, and it is a few days of measurement, not weeks of building.
- **§8** — bugs already paid for once. Do not rediscover them.

**Two things will silently destroy the speed premise if carried over from the old code (§9):** `glReadPixels` anywhere in the stepping path (it is a GPU sync stall), and `log::debug` per physics tick (string formatting will dominate runtime at 10,000+ steps/sec). Both exist in the current `main.cpp` and both must go.

**Ground rules specific to this project:**
1. **Verify empirically. Bindings and docs have repeatedly been wrong here** (see §8). Every uncertain flag gets a logging test before anything is built on it.
2. **The author runs the game; you cannot.** Every in-game measurement in Phase 0 must be handed back for a human to run. Write the instrumentation, then ask for the numbers.
3. **Do not copy source from referenced repos.** ToastyReplay has no LICENSE (all rights reserved), Pathfinder states it is not licensed for redistribution, zBot uses a restrictive EULA. Read them to understand technique; reimplement.
4. **Isolate one variable at a time.** This discipline resolved several past dead ends faster.
5. **Push back on this document.** It is a plan, not scripture. If something here looks inefficient, outdated, over-engineered, or plain wrong — say so explicitly and explain why, rather than implementing it because it is written down. Several sections are the *current best guess* and are labelled as such: the DFS-first choice (§6.3), the four-layer split (§6.2), the field list to start from (§5.6a), and the phase ordering (§7) are all open to a better idea. If you see a cleaner or faster approach, propose it before building.
6. **When unsure, search or ask — do not guess.** This project has been burned repeatedly by plausible-sounding assumptions (see §8, and note that even peony's published `getModifiedDelta` snippet is now out of date on the return type). GD modding moves fast and much of what is written online is stale. Concretely:
   - Re-pull `geode-sdk/bindings` and check the current version's `.bro` file rather than trusting any address or signature in §5.1.
   - Web-search for current Geode APIs, GD version changes, and community findings before assuming a technique still applies.
   - If a decision would be expensive to reverse (architecture, data format, search design), ask the author rather than picking.
   - If a claim in §3 about what other tools can or cannot do turns out to be wrong, that materially affects the project's premise — flag it immediately rather than working around it.

**Terminology:** "solver" = the search that finds an input sequence. "macro" = the resulting `(frame, pressed)` list. "savestate" = captured game state that can be restored to. "decision point" = a physics frame where input can change the outcome (`on_ground || touching_ring`).

---

## 1. The research question

**Can AI beat Geometry Dash levels, and how hard a level can it beat?**

Explicitly **not** goals:
- Generalization across levels
- The agent "learning" in a machine-learning sense
- The output being anything other than a working input sequence

What matters is that **AI arrived at the solution, not a human**. A solver that searches for a valid input sequence satisfies this. A macro recorded from human play does not. This distinction is the entire point, and it is what separates this project from the ~50 existing GD bot tools.

The concrete milestone ladder:
1. **Stereo Madness** (proof the stack works)
2. Progressively harder main levels
3. **Any rated Demon** (would be a first, see §3)
4. A Demonlist ("list") demon (would be a significant first)

---

## 2. Project history: what came before and why it was set aside

This project has had three prior phases. All code still exists and should not be deleted, but none of it is the current direction.

### Phase 1: State-based RL (superseded)
A Geode mod read player state (Y position, velocity, alive/dead, level %) and injected jump input over a socket, driving PPO/DQN agents via a Gymnasium env.

**Why it failed:** pure RL on minimal state features converges to a degenerate constant-hold policy. Timing cannot be learned without visual input.

### Phase 2: CV infrastructure (superseded, but infrastructure reused)
Built OpenGL frame capture, dual-socket streaming (state + frames), grayscale 128×128 downsampling. This infrastructure worked and its bug fixes are still relevant (see §8).

### Phase 3: Behavior Cloning + PPO fine-tuning (works, but plateaued)

**What was built and validated:**
- Recorded 4,429 labeled frames across 3 custom "skill" levels (blocks, spikes, combined), 2 takes each, ~10% jump rate
- `SmallCNN`: 4 conv/pool blocks (128→64→32→16→8), flatten, `Linear(4096,128)+ReLU+Dropout(0.3)`, `Linear(128,1)` logit
- BC final metrics: `val_acc 0.946`, `jump_precision 0.638`, `jump_recall 0.925`, `jump_f1 0.755`
- **BC alone cleared 10-11% of Stereo Madness on first live test, a level it never trained on.** This is a real generalization result and stands on its own as evidence, independent of everything after it.
- PPO fine-tuning with `BCFeaturesExtractor` loading the BC backbone, plus a verified weight transplant of BC's decision layer into PPO's `action_net` (`net_arch=dict(pi=[], vf=[64])` makes the policy head a single `Linear(128,2)`, matching BC's discarded `Linear(128,1)` input shape)

**Results across three full runs:**

| Run | Config | Deterministic eval | Notes |
|---|---|---|---|
| v1 | `progress_delta*10`, death `-0.5` | 274/298 steps alternating (~11-12%) | Best RL result. `explained_variance` 0.61, `entropy_loss` → -0.175 |
| v2 | death `-5.0`, jump penalty `-0.1` | 249/276/**49** | Regression. New early deaths at ~1.9% and ~3.59% |
| gated v2 | + decision gating, death `-5.0` | 10.65-10.68%, perfectly alternating | 8.2 hrs. Entropy collapsed to **-0.038**, `explained_variance` **0.15**, `value_loss` **rose** 182→207 |
| gated v1-reward (diag) | + gating, death `-0.5`, 40k steps | (not evaluated) | Entropy already **-0.486** at 40k, `explained_variance` **0.098**, `approx_kl` **0.0042** |

**Why it was set aside:** the 40k diagnostic was designed specifically to test whether the `-5.0` death penalty caused the collapse. It did not. The milder `-0.5` penalty showed the *same collapse signature, slightly delayed*. This rules out death-penalty magnitude and points at gating's reward structure itself (a single gated step can absorb many physics frames, producing per-step rewards observed between 85 and 260, which is a very noisy regression target for the value function).

The RL path is **not proven unsalvageable**. Untried levers remain (see §10). It was set aside because it competes for time with an approach that has a clearer path to a real result.

**Do not delete:** `bc_model.pt` and `ppo_bc_stereo_madness_150k.zip`. The BC generalization result is a genuine, standalone finding.

---

## 3. Landscape research: what actually exists (critical context)

This was researched extensively because the entire premise depends on it. **Do not assume this territory is crowded. It is not.**

### The GD bot ecosystem is ~50 tools and none of them are AI

The definitive community-maintained catalog (originally `CrimsonFork/geometry-dash-macro-mods`, now under the GD Resource Commune on Codeberg) lists roughly 50 tools with detailed feature comparisons: xdBot, Silicate, GatoBot, MegaHack, Eclipse, zBot, yBot, ծbot, ReplayBot, TASBot, uvbot, and many more.

**Every single one is a macro, replay, or TAS tool.** They replay human-authored input. The "recording type" column (Frame / Physics / X-Pos / Time) refers to what playback *synchronizes* against, not what decides the input. Zero automated solvers in the entire catalog.

**TAS** = Tool-Assisted Speedrun. A human uses frame advance, save states, and rewind to construct an input sequence one frame at a time with unlimited thinking time. The tools remove execution skill, not decision-making. Output is human-authored.

### The two search-based attempts, both stalled

**DashBot 3.0 (MCJack123)**, 61 stars, the most prominent:
- Reads process memory. **"the screen is not used as an input at all - the bot plays blind."** Inputs are X position, player mode, and level-finished flag.
- Requires a **hand-specified portal/mode string per level** (Stereo Madness is `'0101'`) because it cannot detect the current game mode. This is human-supplied level knowledge.
- Archived Dec 2025. **Only works on GD 2.11, not 2.2.**
- Headline achievement per its own README: a Twitch highlight of **Stereo Madness** being finished.
- Built-in support only for the 19 main levels.

**GD-AI / AutoMacro (FigmentBoy)**:
- Public repo `FigmentBoy/GD-AI`: single commit, Aug 2020, ~8 stars. It is a **(1+1) hill climber**, not a population GA. `Population(1, ...)`. Elitism keeps the best brain unchanged, mutates one clone.
- Genome is `directions[floor(x)]`, **one boolean per X position unit**. Fitness is `percent²`. Mutation is localized to a window from 300 units before the death point to 100 units after.
- **The genome is literally a macro.** The repo ships `replay.py` whose only job is to replay the saved `replay.json`.
- Uses memory access (`get_x_pos`, `set_x_pos`, `is_dead`, `percent`, `player_kill`). No vision.
- **AutoMacro** is a later, private, more advanced version. Never released. Timeline from Figment's Discord:
  - **Apr 2021**: demo video, Figment says it "can beat most waves in around a second! (other gamemodes are slower)"
  - **May 2022**: community asks if it's coming out. Answer: "No." "Because it sucks."
  - **Dec 2022**: Figment says it is "nowhere near a releasable state," blocked on zBot bugs, will resume after the final zBot release.
  - **Aug 2026 (now)**: zBot is still on `v3.0.0-beta.12`. AutoMacro never shipped. **zBot's source contains zero solver code** (verified by grep for automacro/solver/genetic/brute/search/neural/ai).
- Known to have beaten "WHAT" by UFokinWotM8 (unrated, community-voted Insane, Tiny length) with Practice Mode 0% and Normal Mode 100%. The video description said it would "hopefully be able to beat **list** demons," i.e. that was aspirational, not achieved.

> **Important insight from Figment's quote:** waves are the **fast** case for a solver, other game modes are slower. This is counterintuitive but makes sense: the wave moves at a fixed 45° with no momentum or acceleration, so the reachable set at each frame is a simple interval and it reduces to corridor pathfinding. Cube commits you to a long ballistic arc (deep backtracking). Ship has continuous acceleration (large state space). **This is good news, because top demons are wave-dominated.**

### Vision/ML attempts (all failed or minimal)

**Stanford CS231n 2017 (Ted Li, Sean Rafferty), "Playing Geometry Dash with Convolutional Neural Networks"**:
- CNN on raw pixels feeding both a DQN (double Q, dueling Q, prioritized experience replay) and an imitation-learning classifier
- Trained imitation on levels 2-10, validated on level 1 (same generalization setup as this project's BC phase)
- **"our agent never completed a level when tested on the emulator"**
- Identified the exact approach this project's Phase 3 used as promising future work, citing AlphaGo-style imitation pretraining before DQN, but did not implement it
- Also documented the credit-assignment trap: when an agent jumps too early, negative reward propagates backward and it tends to jump earlier and earlier

**Gustyper/Geometry_Dash_Reinforcement_Learning**: FixMatch semi-supervised CNN for mode classification, multi-head DQN (separate cube and ship heads), frame stacking. Result: **"the agent struggled to beat the game in practice"**, defeated by screen-capture latency (~5 frame desync between decision and execution).

**ThePickleGawd/geometry-dash-ai (UCSB, 5 authors)** — the strongest published result:
- DQN and Mixture-of-Experts DQN on 128×128 grayscale, `FRAME_STACK_SIZE=4`, `PREVIOUS_ACTION=True`
- Config: `DEFAULT_REWARD=0.01`, `JUMP_PUNISHMENT=-0.2`, `DEATH_PUNISHMENT=-10`, `CHECKPOINT_REWARD=0` (disabled), `num_episodes=50000`, `max_steps=5000`
- `RANDOM_SPAWN_PERCENTAGE=0.3` exists in config but `train.py` hardcodes `env.reset(0)`, so it is computed and never used
- **Beat Stereo Madness and Back On Track** (levels 1 and 2 of 21). ~5,000 attempts for SM, ~2,500 for BOT
- Back On Track was initialized from Stereo Madness weights, and they explicitly attribute the halved attempt count to that transfer
- **Win rate is not 100%**; they report "seemingly random mistakes near the end"
- **Reward clipping to [-1, 1] gave "massive performance improvements" in stability**
- Discrete reward every 3% worked best. Continuous time-based rewards failed (never past 15%). NoisyNets failed (never past 10%). Dueling DQN failed. New-state rewards failed.
- They implemented **"load the level from a desired percentage"**
- They cite the Stanford paper but with the wrong title and authors attached, which is an indication of how thin this literature is
- Their DINOv3/Qwen-GRPO variant loads a **frozen** HuggingFace DINOv3 backbone with a thin trainable head. Explicitly labeled "Initial support," no demoed results, not their showcased model.

### Pathfinder (camila314): the closest prior art, and exactly where it stops

**`camila314/pathfinder`** is the single most important project to understand, because it is the same category as this one. Geode mod, GD 2.2081, v1.0.0-beta.243, 380,153 downloads. Mod page: *"Auto-generate macros for levels using simulation! This mod uses a physics simulator under the hood to solve levels in seconds!"*

**Architecture (verified by reading the source):**
- `gd-sim/` is a **custom C++ reimplementation of GD physics**, not the live game. Total size: **2,583 lines.**
- **It does not run the live game's physics at all.** `src/main.cpp` does `ZipUtils::decompressString(m_level->m_levelString, true, 0)` and passes the raw level string to `pathfind(lvlString, ...)`. Geometry Dash serves only as a UI host and a source of level data; every physics tick is computed by the reimplementation. There is also a `subprocess.hpp` used by the debug path to shell out to an external simulator binary.
- **`gd-sim/README.md`, verbatim and complete:**
  > `# Pathfinder`
  > `# Geometry Dash Physics Simulator`
  > `Full support up to 1.7, partial up to 1.9. This code is not currently licensed for redistribution.`
  > `## Notes`
  > `1. Y positions are slightly off because in the real gd, they are offest by 105`

  **GD 1.7 shipped in 2014 and 1.9 in 2015. The simulator's own stated support ceiling is roughly a decade behind current GD (2.2).** It also openly documents a known Y-position inaccuracy.
- `Level::runFrame(bool pressed, float dt = 1/240.)`, fixed 240tps.
- `gameStates` is a `std::vector<Player>`; `rollback(frame)` truncates it. **O(1) savestates for free**, which the live game can never match.
- Output is **GDR**. Ships no bot; you export and replay in xdBot or similar.
- camila314 also maintains **`gdp`: "Full decompilations of Geometry Dash physics functions"**, which is how the simulator achieves accuracy.

**Search algorithm (`src/pathfinder.cpp`) — randomized hill climbing, NOT systematic search:**
```cpp
constexpr int iterations = 300;
for (int i = 0; i < iterations; i++) {
    std::set<uint16_t> inputs;
    for (int i = 0; i < 30; i++) inputs.insert(frame + dist(rng));  // 30 random toggle frames
    int nf = tryInputs(lvl, inputs);
    if (nf > bestFrame) { bestFrame = nf; bestInputs = inputs; }
}
```
300 random input sets per step, keep whichever survives furthest, commit ~2/3 of the way toward it, and on failure roll back progressively (`fail += 5`, escalating `numAway` 1000 → 10000, then full restart from frame 1). **No completeness guarantee.** This is why it is documented to get stuck at specific percentages and emit unfinishable macros.

> ### THE CRITICAL FINDING: Pathfinder's own mod page lists, under the heading **"None of these objects are supported"**:
> **Duals · Upside-down Slopes · Partially Rotated Objects for Cube/UFO/Ball · Robot Mode · Spider Mode · Swing Mode · Any Non-Visual Triggers · Dash Orbs · Teleport Portals · ANYTHING FROM 2.2 · Modifier blocks (D-block, J-block, etc.)**

The source corroborates every item:
- `VehicleType` enum is **only** `{Cube, Ship, Ball, Ufo, Wave}`. **No Robot, Spider, or Swing** — three of the eight game modes are simply absent.
- The object dispatch table in `src/Objects/Object.cpp` is a **hardcoded whitelist of object IDs** covering blocks, hazards, sawblades, 3 pad IDs, 6 orb IDs, vehicle/gravity/size/speed portals, breakable blocks, and slopes. It ends with:
  ```cpp
  // Any block that isnt' defined is ignored
  return {};
  ```
  **Anything outside the whitelist is silently treated as nonexistent.** No triggers. No teleport portals. No duals. No moving objects.
- `gd-sim/README.md`: *"Full support up to 1.7, partial up to 1.9."*

**What this means concretely:** Pathfinder works on static, classic-era geometry in cube/ship/ball/UFO/wave. It cannot touch modern levels. **Every Demonlist demon uses 2.1+ features, triggers, duals, and modes Pathfinder does not implement.** Deadlocked alone is out because it has robot mode. The entire modern demon landscape is beyond it, by construction rather than by tuning.

**Why this validates the live-game architecture.** Every one of Pathfinder's limitations stems from one decision: reimplementing physics instead of driving the real game. Each mechanic must be hand-written, so 2.2 features, triggers, duals, and three game modes remain unimplemented after 243 beta releases. **A live-game solver inherits every mechanic for free, permanently, including future GD updates.** The tradeoff is honest and should be stated plainly: the live game will be *slower per branch* than truncating a vector. This project trades raw speed for total coverage and search completeness.

### The bottom line

**The AI record for beating Geometry Dash levels is level 2 of 21** (UCSB, DQN). **The automated-solver record is Pathfinder**, which is limited to pre-2.2 static geometry with a stochastic, incomplete search. **No AI, search, genetic method, or ML approach has beaten any rated Demon on public record**, and the only tool in the right category structurally cannot attempt a modern one.

The defensible claim for this project is therefore: **a solver that works on levels Pathfinder cannot touch**, via live-game simulation (total mechanic coverage) plus systematic search (completeness). The milestone ladder in §1 is unchanged, but the novelty is now precisely located rather than assumed.

---

## 4. The approach: fundamentals

**Geometry Dash is a deterministic state machine.** At any moment the game has a complete state S (player position, velocity, mode, gravity, plus every moving object and trigger). Given S and an input I (pressed or not), the physics produce exactly one next state S′. Same S and same I always give the same S′.

**Therefore the level is a tree.** Root = level start. Each node = a game state. Each node has two children (press / don't press for one physics step). Some nodes are death. One set of nodes is "reached 100%."

**A solution is a path** from root to a 100% node that never passes through a death node. Written out as `(frame, pressed)` pairs, that path is a macro.

**Search is systematically walking that tree.** Descend a branch; on death, back up to the last choice and take the other branch. Same algorithm as a maze solver or a chess engine.

Naive cost is 2^30000 for a long level. Three things make it tractable:

1. **Pruning.** The instant a branch dies you discard its entire subtree unexplored.
2. **Decision points.** In cube mode, pressing does nothing mid-air. Branch only where input can change the outcome: **`on_ground` OR `touching_ring`**. This turns 2-per-frame into 2-per-landing and is the single biggest lever available.
3. **Savestates.** Without them, testing an alternative branch means replaying from level start. With them, restore the node and continue. Cost per branch goes from "length of level" to "length of segment."

**The property RL lacks: completeness.** If a solution exists and you search exhaustively, you will find it. No plateau, no convergence to hope for, no hyperparameters. This is why the time estimate is bounded in a way RL's was not.

---

## 5. Verified technical foundations

Everything in this section was verified against the actual Geode bindings repo (`geode-sdk/bindings`, `bindings/2.2072/GeometryDash.bro`) or by reading source. **Do not re-verify; do not assume differently.**

### 5.1 Hookability reference (GD 2.2081, the current version)

> **What this table is for, and what it is NOT.**
> Geometry Dash is closed-source with no headers. `geode-sdk/bindings` is a community database of function addresses and class layouts that Geode's codegen turns into usable C++. **You never write these addresses in code** — `$modify` resolves them. The table exists for one purpose: **proving a symbol is hookable.** A listed address means the function exists as a real, callable symbol. `= inline` means the compiler inlined it, there is no address, and **it cannot be hooked at all**. That distinction has already cost this project real debugging time (see §8).
>
> **Do not hardcode any address. Do not pattern-scan. Just use `$modify`.** Addresses change every GD build; if GD updates, re-pull bindings and re-check hookability, nothing else.

| Symbol | Hookable? | Notes |
|---|---|---|
| `GJBaseGameLayer::update(float)` | yes (`0x237850`) | **Use this, not `PlayLayer::update`** |
| `GJBaseGameLayer::getModifiedDelta(float)` | yes (`0x2377b0`) | **Returns `double` in 2.2081, not `float`.** peony's published snippet says `float` — it is out of date |
| `GJBaseGameLayer::processCommands(...)` | yes (`0x239c60`) | **Signature changed** to `(float dt, bool isHalfTick, bool isLastTick)` |
| `GJBaseGameLayer::resetLevelVariables()` | yes (`0x23b040`) | |
| `PlayLayer::updateVisibility(float)` | yes (`0x3af3a0`) | Hook here to suppress rendering |
| `PlayLayer::resetLevel()` | yes (`0x3b8eb0`) | |
| `PlayLayer::resetLevelFromStart()` | yes (`0x3b8d10`) | |
| `PlayLayer::fullReset()` | yes (`0x3b8bf0`) | |
| `PlayLayer::createCheckpoint()` | yes (`0x3b4fc0`) | Returns `CheckpointObject*` |
| `PlayLayer::getCurrentPercent()` | yes (`0x3b3950`) | |
| `PlayLayer::playEndAnimationToPos(CCPoint)` | yes (`0x3aba00`) | Level-complete detection |
| `PlayerObject::pushButton(PlayerButton)` | yes (`0x397f40`) | |
| `PlayerObject::releaseButton(PlayerButton)` | yes (`0x3981d0`) | |
| `GJBaseGameLayer::updateVisibility` | **NO — `= inline`** | Use `PlayLayer::updateVisibility` |
| `GJBaseGameLayer::resetLevel` | **NO — `= inline`** | Use `PlayLayer::resetLevel` |
| `PlayLayer::update` | **NO** | Use `GJBaseGameLayer::update` |
| `PlayLayer::activateEndTrigger` | **NO — inline** | Produces `C2338 static assertion failed`. Use `playEndAnimationToPos` |

`PlayLayer::m_hasCompletedLevel` exists but is useless for detection: it only becomes true once the completion *menu* appears, far too late.

Type note: `PlayerObject::m_gravity` and `PlayerObject::m_yVelocity` are both `double` in 2.2081.

### 5.2 Determinism: the physics fix (CRITICAL, and the public examples are wrong)

Source: peony (developer of Silicate), "60tps In 2.2 Is a Lie", June 2025.

**The naive approach of only hooking `getModifiedDelta` does not work.** peony states directly: *"This is what zBot used to do, and what Mega Hack still does, and it doesn't work."* Time warp physics change when you do that.

**The correct fix has three parts:**

1. Hook `getModifiedDelta`, **call the original first** (it mutates internal `GJBaseGameLayer` state), then return your own delta:
```cpp
float getModifiedDelta(float dt) {
    GJBaseGameLayer::getModifiedDelta(dt);
    float wantedDt = getPhysicsDt() * fminf(m_gameState.m_timeWarp, 1.0f);
    return wantedDt;
}
```
2. Hook `GJBaseGameLayer::update` and **feed your delta into the original call**, not just override the return:
```cpp
void update(float dt) {
    GJBaseGameLayer::update(getPhysicsDt());
}
```
3. For low TPS only: midhook the instruction after `fractionOf240` is computed and force the step count to 1 (it lands in `r11` on Windows), then restore the original dt at function end via `xmm15.f64[0]` so visibility updates work.

```
fractionOf240 = (int)fmaxf(1.0, roundf((float)(dt * 60.0 / fminf(m_gameState.m_timeWarp, 1.0)) * 4.0));
```

**peony states only two 2.2 bots solved this correctly: Silicate and TCBot. Both are closed source.** Implement from this writeup, not from zBot's source.

**For this project:** part 3 matters less because we want *high* TPS, but parts 1 and 2 are mandatory. **Physics dt must stay at the real gameplay value (1/240) so the resulting macro is valid in normal play.** We are not changing the tick rate; we are executing the same ticks faster than wall-clock.

### 5.3 Headless / fast stepping

Pattern (zBot's `physicsbypass.cpp` demonstrates the shape, ~20 lines):
- Hook `PlayLayer::updateVisibility` → early-return when a `disableRender` flag is set
- In the `GJBaseGameLayer::update` hook, run N iterations of `GJBaseGameLayer::update(fixedDelta)` with rendering disabled for all but the last

This is the speed multiplier and is almost certainly the mechanism behind the "press play, it loads for 15 seconds, then plays the level" behavior seen in the AutoMacro video.

### 5.4 Savestates: exactly what vanilla saves and what it misses

`CheckpointObject` members (from bindings):
```
GJGameState        m_gameState;
GJShaderState      m_shaderState;
FMODAudioState     m_audioState;
GameObject*        m_physicalCheckpointObject;
PlayerCheckpoint*  m_player1Checkpoint;
PlayerCheckpoint*  m_player2Checkpoint;
```

`PlayerCheckpoint` members (complete list):
```
cocos2d::CCPoint m_position, m_lastPosition;
int  m_unkInt1;
bool m_isUpsideDown, m_unk7b3;
bool m_isShip, m_isBall, m_isBird, m_isSwing, m_isDart, m_isRobot, m_isSpider;
bool m_isOnGround;
int  m_hasGhostTrail;
std::array<uint8_t,4> m_unkBytes1;
float m_speed;
bool m_isHidden;
```

> **`m_yVelocity` is NOT saved.** This is confirmed from the bindings, not inferred. It is the single field zBot's free-version "practice fix" patches:
> ```cpp
> const std::array memberPairs = { makeMemberPair(&PlayerObject::m_yVelocity), };
> ```
> This omission is why vanilla practice mode desyncs and why every serious bot ships a "practice fix." zBot's `Checkpoint` struct also declares an unused `activatedObjectsCount`, strongly implying the paid version also tracks trigger/object activation state.

**Additional `PlayerObject` fields that are strong savestate candidates** (from bindings, to be validated empirically):
```
m_yVelocity            m_gravity              m_speedMultiplier
m_groundYVelocity      m_yVelocityBeforeSlope m_currentSlopeYVelocity
m_rotationSpeed        m_rotateSpeed          m_isRotating
m_lastJumpTime         m_lastFlipTime         m_lastSpiderFlipTime
m_slopeStartTime       m_slopeAngle           m_isCollidingWithSlope
m_dashX / m_dashY / m_dashAngle / m_dashStartTime
m_lastCollisionBottom / Top / Left / Right
m_jumpBuffered         m_wasJumpBuffered      m_stateRingJump
m_touchedRing          m_touchedGravityPortal m_isAccelerating
m_accelerationOrSpeed  m_snapDistance         m_isOnGround3
```

Note: `m_yVelocity` has a setter with 3-decimal rounding, which is itself a determinism aid:
```cpp
double rounded = (int)velocity;
m_yVelocity = std::round((velocity - rounded) * 1000) / 1000. + rounded;
```

**Savestate strategy:** vanilla checkpoint + a manually maintained field list, extended empirically until restores are bit-identical. This is a measurement-driven process, not something to get right by reading. **However, see §5.6 — a production-tested field list already exists and should be the starting point.**

### 5.5 Decision-point flags (already empirically validated, do not re-verify)

- **`m_isOnGround`** — confirmed correct across all platform elevations, not just level-baseline ground.
- **`m_touchingRings`** — a `CCArray*`, i.e. a **proximity count** (observed 0, 1, 2, 3), not a boolean. Multiple overlapping ring hitboxes are normal level design. Use `count() > 0`. Erring broad is correct: an over-broad decision window costs a slightly-too-frequent branch (harmless); an under-broad one risks skipping the one frame an orb press mattered (fatal).
- **`m_jumpBuffered`** — **rejected.** Standard game-dev meaning is an early-press forgiveness buffer, not a live "can act now" readout. Do not gate on this.

**A latching mechanism for these flags was built and then reverted.** Under the mod's 1-in-4-frame sampling, the concern was that a brief contact could fall entirely between sampled frames. Testing showed latched and instantaneous values agreeing in normal play, and the apparent evidence for latching turned out to be a symptom of the `g_currentAction` bug (§8). It was removed as unjustified complexity. **In the solver this concern disappears entirely**, because the solver steps every physics frame rather than sampling every 4th.

> **Scope warning:** these flags were validated in a **cube-only** context. They are the correct branching signal for cube, ball, and spider, and the *wrong* one for UFO, ship, wave, and swing. See the mode-dependent branching table in §6.3 before building anything on them.

---

### 5.6 ToastyReplay: the best available modern reference

`ToastexGD/ToastyReplay` (v2.2.4, Geode 5.8.1, GD 2.2081, actively developed) is a frame-perfect replay/macro bot for showcasers. **It is not a solver** and is not competing with this project. But it is an actively maintained, open-source Geode mod containing production-tested implementations of three of the four primitives needed here. It is a far better reference than zBot or DashBot.

> **LICENSE WARNING: the repo contains no LICENSE file**, which means all rights reserved by default. Read it to understand the technique and the correct field lists. **Do not copy source.** Reimplement.

**Relevant source paths:**
```
src/core/checkpoint_handler.{hpp,cpp}   savestate capture/restore + drift reconciliation
src/core/checkpoint_system.cpp
src/core/frame_stepper.cpp              frame advance
src/hacks/physicsbypass.cpp             the CORRECT modern physics bypass
src/trajectory/trajectory_physics.{hpp,cpp}   forward physics simulation
src/trajectory/trajectory.cpp
src/format/replay.hpp                   state bundle struct definitions
```

**(a) The definitive savestate field list.** `PlayerStateBundle` = `{PlayerKinematicState motion, PlayerFlagState flags, PlayerEnvironmentState environment}`:

```cpp
struct PlayerKinematicState {
    CCPoint position; float rotation;
    double verticalVelocity;            // the field vanilla checkpoints miss
    double preSlopeVerticalVelocity, horizontalVelocity;
    double dashX, dashY, dashAngle, dashStartTime, slopeStartTime;
    float  fallSpeed, slopeVelocity;
    CCPoint shipRotation, lastPortalPosition, stateForceVector;
};
struct PlayerFlagState {
    bool upsideDown, holdingLeft, holdingRight, platformer, dead;
    bool ship, bird, ball, wave, robot, spider, swing;
    bool sideways, dashing, onSlope, wasOnSlope, onGround, goingLeft;
    bool platformerMovingRight, slidingRight, accelerating;
    bool affectedByForces, jumpBuffered;
    std::array<bool,4> buttonHolds;
};
struct PlayerEnvironmentState {
    double gravity; float gravityMod, playerSpeed, playerSpeedAC;
    double speedMultiplier; float vehicleSize;
    int reverseRelated, stateDartSlide, stateFlipGravity, stateForce;
    bool dualContext, twoPlayerContext, extendedState;
};
```

**Use this as the starting field list rather than deriving one from scratch.** It represents a shipped, tested answer to "what must be saved for a bit-identical restore."

**(b) RNG state matters for determinism.** This was not anticipated. Their anchor carries:
```cpp
struct AnchorRngState { uintptr_t fastRandState; bool locked; uint32_t seed; };
```
GD uses a fast-random source somewhere in gameplay. **If RNG state is not captured and restored, restores will not be deterministic.** Note also `matcool/ReplayBot`'s known issue: *"rob uses a time function somewhere in the physics (why)"*, and a replay that "can randomly die at 26%." Nondeterminism sources are real and must be hunted down in Phase 0.

> **DISPROVEN EMPIRICALLY — see §13.2.** `GJBaseGameLayer::m_randomSeed` takes a different value on every single reset, and ten replays of one input sequence under those ten different seeds produced **byte-identical** physics traces. RNG state does not affect physics and does not need to be captured or restored. Do not build savestate work on this paragraph.

**(c) A ready-made savestate validation method.** `PlayerStateRestorer` exposes exactly the Phase 0 test needed:
```cpp
static float positionalDrift(PlayerObject*, PlayerStateBundle const&);
static float rotationDrift(PlayerObject*, PlayerStateBundle const&);
static float velocityDrift(PlayerObject*, PlayerStateBundle const&);
static bool  needsReconciliation(..., float posTol=0.01f, float rotTol=0.1f, float velTol=0.01f);
```
Capture state, restore, re-measure, assert drift is zero. Build this instrumentation first; it turns "did my savestate work" from a guess into a number.

**(d) Object/trigger activation tracking, the gap flagged in §5.4.** Their trajectory simulator solves it:
```cpp
bool canActivate(PlayerObject*, EffectGameObject*);
void rememberActivation(PlayerObject*, EffectGameObject*);
bool hasActivated(PlayerObject*, EffectGameObject*);
```
Plus `collisionCheckObjects`, `checkSpawnObjects`, `triggerObject`, `ringJump`, `bumpPlayer`, `propellPlayer`, `startDashing`, `teleportPlayer`, `flipGravity`. This is a worked example of simulating physics forward while correctly tracking which objects have already fired.

**(e) Physics bypass done correctly.** Unlike zBot, ToastyReplay implements the equivalent of peony's `fractionOf240` fix via `setExpectedTicksPatchEnabled` / `setExpectedTicks(steps)`, a `TickStepPlanner` that accumulates and plans step counts, and:
```cpp
void consumeSingleTick(GJBaseGameLayer* layer, ReplayEngine& engine) {
    resetSimulation(engine);
    setExpectedTicks(1);
    layer->GJBaseGameLayer::update(fixedDelta(engine));
}
```
`consumeSingleTick` is precisely the "advance exactly one physics tick" primitive the search loop needs. Note the platform split: ARM Mac loops `update(timestep)` per step, while Windows issues a single `update(totalDelta)` with the expected-ticks patch active.

**(f) Macro format landscape.** ToastyReplay's native format is TTR3, but it reads/writes **GDR** and converts from 30+ other bots. GDR is the de-facto interchange format. **If solver output is written as GDR, it can be replayed by ToastyReplay, Silicate, xdBot and others for demos and verification**, which also gives an independent check that a found solution actually works in unmodified gameplay.

## 6. Architecture

### 6.1 The search runs in C++ inside the mod

The existing Python env does one socket round trip per 4 physics frames and tops out around 20 steps/sec. A solver needs millions of physics steps. **IPC would be the entire bottleneck and would erase the benefit of headless stepping.** Keep the socket for logging and progress reporting only.

### 6.2 Four layers, bottom up

**Layer 1 — Determinism.** Fixed dt via the three-part fix in §5.2. Verify by running an identical input sequence 10+ times and confirming bit-identical outcomes. **If this fails, nothing above it works.**

**Layer 2 — Headless speed.** `PlayLayer::updateVisibility` early-return + N physics steps per rendered frame. Physics dt stays 1/240.

**Layer 3 — Savestate.** `createCheckpoint()` + manual field restore starting with `m_yVelocity`. **Measure restore cost early** (practice respawn is known to be slow on object-heavy levels).

**Layer 4 — Search.** Swappable algorithm behind a clean interface.

### 6.3 Search algorithm

**Phase 1: DFS with backtracking.** Prune on death. **Ordering heuristic: try "don't press" before "press"**, since the BC dataset showed a ~10% jump rate, making no-jump far likelier to be correct.

> ### CRITICAL: the branching rule is MODE-DEPENDENT. Do not apply `on_ground || touching_ring` globally.
>
> That rule came from the RL phase, where the scope was cube-only. **Applied to every mode it will silently produce a solver that cannot solve ship, wave, UFO, or swing sections**, because it would never consider branching mid-air where those modes require input every frame.
>
> | Mode | When input can change the outcome | Branch on |
> |---|---|---|
> | **Cube** | Only on ground, or touching an orb/ring | `on_ground \|\| touching_ring` |
> | **Ball** | Only on a surface (press flips gravity), or orb | `on_ground \|\| touching_ring` |
> | **Spider** | Only on floor/ceiling, or orb | `on_ground \|\| touching_ring` |
> | **Robot** | On ground, **but jump height varies with hold duration** | ground contact + hold-length is a *continuous* parameter, not binary |
> | **UFO** | **Any frame** — each press is a fresh jump, mid-air included | every frame (or every press-edge) |
> | **Ship / Wave / Swing** | **Every frame** — hold state continuously controls trajectory | every frame |
>
> Consequences to design for up front:
> - The branching factor is small and sparse in cube/ball/spider, and large and dense in ship/wave/UFO/swing. **The search interface must accept a per-mode branching policy** rather than a single hardcoded predicate.
> - **Robot is the awkward case**: variable jump height means the action is "hold for N frames," not a binary press. Treat N as a bounded enumeration.
> - Pads activate on contact with no input required; orbs require input. Only orbs create a branch.
>
> Note this is precisely where Pathfinder stops: its `VehicleType` enum contains only `{Cube, Ship, Ball, Ufo, Wave}` — no robot, spider, or swing (§3).

**DFS's known weakness:** when the fatal mistake happened long before the death (normal in ship/wave, where a wrong altitude two seconds back dooms you now), DFS backtracks to the *most recent* decision point, which is not the culprit, and exhausts an innocent subtree before backing up far enough.

**Phase 3: beam search.** Keep the top K savestates by progress, expand all, keep top K again. Maintaining K different ways of being at 40% means that when one is doomed, alternatives that made different earlier choices survive. Cost is K live savestates plus a scoring function.

**Wave specialization (later):** wave moves at a fixed 45° with no momentum, so reachable Y positions form an interval. Propagate intervals forward and clip against obstacles instead of branching. This is almost certainly why AutoMacro did "waves in around a second."

**Genetic algorithms are dominated here and should not be used.** DashBot and GD-AI used hill climbing *because they had no savestates and no fast simulation*, making every fitness evaluation a full real-time replay. Given savestates plus headless stepping, GA throws away pruning (a death teaches it nothing about the subtree) and systematic coverage. Pathfinder's randomized hill climbing is the same category and is why it stalls at fixed percentages.

> **Do not over-optimize the algorithm before Phase 0 measurements.** If savestate restore costs 50ms you get ~20 branches/sec and no algorithm saves you. If it costs 0.1ms you get thousands and naive DFS clears Stereo Madness. That single number dominates every algorithmic choice.

---

## 7. Phases

### Phase 0 — Verify primitives (ALL THE REAL RISK LIVES HERE)
Cheap to run, retires the major unknowns. Do not skip or reorder.

1. **Determinism test.** Same input sequence, 10+ runs, bit-identical outcomes (final position, velocity, percent). Fails → fix §5.2 before anything else.
2. **Hunt nondeterminism sources.** Capture/restore `fastRandState` (§5.6b). Investigate the time-based physics function that `matcool/ReplayBot` flagged. Any nondeterminism invalidates the entire search.
3. **Max achievable TPS** with rendering disabled. This sets the whole time budget.
4. **Savestate create/restore cost.** Microseconds vs milliseconds changes the architecture. **This single number dominates every algorithmic choice.**
5. **Build drift instrumentation first** (§5.6c): `positionalDrift` / `rotationDrift` / `velocityDrift`. Capture, restore, re-measure, assert zero. Then extend the field list (starting from §5.6a) until drift is zero across all game modes.
6. **Single-tick primitive.** A verified "advance exactly one physics tick" call, modelled on `consumeSingleTick` (§5.6e).
7. **Level-complete detection** via `playEndAnimationToPos`.

### Phase 1 — Stereo Madness, no savestates
Replay from level start at high TPS. DFS with backtracking at decision points. Short level, cube and ship only. **This is the proof of concept.**

### Phase 2 — Savestate-based search
Swap prefix replay for save/restore. This is what makes level length stop mattering.

### Phase 3 — Scale
Segment decomposition, beam search, wave interval propagation, ship handling. Then progressively harder levels toward a demon.

---

## 8. Hard-won bugs and constraints (carried forward)

These cost real debugging time. Several are non-obvious.

- **`isGameplayActive()` does NOT cover death.** It goes false at the exact moment death happens, which means an edge-detect flag gated behind it can never fire. Use a direct `m_player1->m_isDead` check, decoupled from `isGameplayActive`.
- **`g_currentAction` must reset on level reset AND init.** A global holding the last action is re-asserted every frame. If a controlling process disconnects while the last action was "press," the mod holds jump forever. This produced a completely scrambled trajectory that was initially misdiagnosed as a flag-detection problem. Reset in both `PlayLayer::resetLevel()` and `PlayLayer::init()`.
- **Socket partial writes.** Large frames were silently truncated. A `sendAll()` helper is required.
- **`glReadPixels` needs `glPixelStorei(GL_PACK_ALIGNMENT, 1)`** first, and a non-null buffer.
- **Button-hold precision must be decoupled from decision cadence.** Re-assert the cached action via `handleButton` every `update()` call regardless of how often decisions are made.
- **First-attempt camera pan-in** occurs only on the very first attempt after a level loads. Retries via `resetLevel` do not show it. Warmup counters should reset in `PlayLayer::init` only, never in `resetLevel`.
- **Verify empirically, do not trust bindings or docs at face value.** This project has been bitten repeatedly: `isGameplayActive` was assumed to cover death and did not; `activateEndTrigger` was assumed hookable and is inline; `m_touchingRings` was assumed boolean and is a count; `m_jumpBuffered`'s name means something entirely different from what it suggests. **Every uncertain flag gets a direct logging test before anything is built on it.**

### Recommended GD settings for solver runs
- Low Detail Mode on
- QOLMod: particles, shaders, glow, ship fire trail, object glow all disabled
- Attempt counter and progress bar hidden

---

## 9. Existing code: what to port, what to delete, what to watch for

The RLDash mod is a working Geode mod and its skeleton is a good starting point. **Copy `mod/` (CMake setup, `Socket.*`, hook structure) into the new repo; do not fork the RL repo** (§10).

### Keep

- **Build/CMake setup.** `mod.json` (`id: colin.rldash`, rename for the new project), `geode build` from inside `mod/`. Delete `mod/build/` and rebuild clean after any CMake/`mod.json` change or you get stale-cache errors.
- **The WinSock isolation pattern.** `Socket.hpp/.cpp` deliberately keep `<winsock2.h>` out of any translation unit that includes Geode headers, with `WIN32_LEAN_AND_MEAN` defined project-wide via `target_compile_definitions`. This was needed to avoid `windows.h`/WinSock conflicts. **Preserve this structure**, the conflicts are real and annoying to rediscover.
- **`sendAll()`.** Fixes silent partial writes on the non-blocking socket. Keep the helper.
- **Hook structure knowledge**: `$modify` on `GJBaseGameLayer::update`, `PlayLayer::resetLevel`, `PlayLayer::init`, `PlayLayer::playEndAnimationToPos`.
- **Death detection**: `this->m_player1 != nullptr` + `m_player1->m_isDead`, edge-detected against a previous-frame flag. Deliberately decoupled from `isGameplayActive()` (§8).
- **Input injection**: `this->handleButton(pressed, (int)PlayerButton::Jump, true)` on `GJBaseGameLayer`. Works, verified.

### Delete entirely

- **All frame capture.** `downsampleGrayscale()`, the `glReadPixels` block, `sendLabeledFrame()`, `startFrameServer()`, the 9998 frame socket. The solver does not look at pixels.
  > **`glReadPixels` is a GPU→CPU synchronisation point.** It stalls the pipeline until the GPU drains. Leaving it anywhere in the stepping path would cap the solver at roughly real-time framerates and silently destroy the entire speed premise. It must be gone, not merely disabled behind a flag that could get flipped.
- **`MenuLayer::init` hook.** Writes a debug PPM to a hardcoded `C:/projects/RLDash/test_frame.ppm`. Pure cruft. (Keep only the `startServer()` bootstrap if a logging socket is still wanted, and move it somewhere sensible.)
- **`GetAsyncKeyState` label capture.** BC recording only.
- **`updateProgressbar` hook.** Exists only to log percent.
- **First-attempt warmup** (`g_firstAttemptDone`, `FIRST_ATTEMPT_WARMUP`). That existed to skip the camera pan-in from CNN training data. The solver has no such concern and needs deterministic control from frame zero.

### Rework

- **The `frameCount % 4` throttle must go.** The solver steps *every* physics tick. That throttle existed to match CNN inference cadence.
- **Socket usage becomes logging/progress only** (§6.1). Note two existing quirks that were tolerable for RL and should not be carried into anything on a hot path: `sendAll()` **busy-waits** with `continue` on `WSAEWOULDBLOCK` (a spin loop), and `receiveAction()` reads up to 15 bytes but inspects only `buf[0]`, silently discarding anything queued behind it.
- **`isGameplayActive()` lifecycle gating** needs replacing with explicit solver states (idle / searching / verifying), since the solver drives resets itself rather than reacting to the player.
- **Global state resets.** The existing globals reset in both `resetLevel()` and `init()` (§8, the `g_currentAction` bug). The solver's state machine must follow the same discipline: anything holding cross-frame state resets in **both** places.
- **Naming**: `$modify(MyPlayLayer, GJBaseGameLayer)` is misleadingly named — it modifies `GJBaseGameLayer`, not `PlayLayer`. Rename when porting.
- **`static int frameCount` is a function-local static**, so it never resets across levels. Avoid this pattern for anything the solver depends on.

### The single biggest performance rule

> **No logging in the stepping path.** The current code calls `log::debug` every capture cycle and every `updateProgressbar`. At real-time cadence that is fine. At 10,000+ physics steps per second it is fatal: string formatting and I/O will dominate the entire runtime.
>
> Gate all per-tick logging behind a compile-time or atomic flag that is **off** during search. Report progress by *sampling* (e.g. every N milliseconds of wall clock, or on new-best-percent only), never per tick.

### Python (Phase 3 RL, now paused, stays in the old repo)

`click_recorder.py`, `audit_data.py`, `train_bc.py`, `play_bc.py`, `gd_env.py`, `bc_features_extractor.py`, `train_rl.py`, `continue_rl.py`, `eval_rl.py`, `summarize_run.py`, `test_env.py`. None of this ports.

---

## 10. Open questions and parked items

**Solver:**
- Output format: **GDR is the decided default** (§5.6f). It is the de-facto interchange format, ToastyReplay converts from 30+ bots, and writing GDR means any existing replay bot can independently verify a found solution in unmodified gameplay. Custom format only if GDR proves limiting.
- **Repo: NEW repository. Decided.** Rationale: the RL work is a *finished artifact* with a real standalone result (BC generalizing to an unseen level), and it should stay intact and coherent rather than accumulating a git history of "deleted all the ML." The solver has a different thesis, a different language mix (C++ only vs Python + C++), and a different audience. Two clean repos read better than one confused one. **Copy the `mod/` skeleton, `CMakeLists.txt`, and `Socket.*` across as a starting point; do not fork the whole RLDash repo.**
- Trigger/object activation state in savestates: ToastyReplay's `canActivate`/`rememberActivation`/`hasActivated` (§5.6d) shows the shape of the solution; how much of it is needed here is still open
- Mode-dependent branching policy (§6.3): robot's variable jump height is the least-designed part
- 2.2 game modes (swing etc.), dual sections, size/speed portals, trigger-driven moving obstacles

> ### The rest of this section is OUT OF SCOPE. Do not implement any of it.
> It is recorded so that a future reader knows what was already considered and does not redo the analysis. **Nothing below is a task.**

**RL path, if ever resumed (untried levers, in priority order):**
1. **Reward clipping to [-1, 1]** — independently validated by both the UCSB paper ("massive performance improvements") and the reference repo config. The highest-value untried lever.
2. **`ent_coef`** (SB3 default `0.0`, PPO paper uses `0.01`) — directly targets the observed entropy collapse
3. `share_features_extractor=False` — stops a poorly-fit value function's gradients corrupting BC-validated features
4. `VecNormalize`
5. `freeze_backbone=True`
6. Larger value net (`vf=[128,64]`; `pi=[]` must stay unchanged, the transplant depends on it)
7. Practice checkpoints (UCSB implemented the equivalent as "load from desired percentage")
8. Frame stacking (invalidates `bc_model.pt`, requires full BC retrain)
9. Death-penalty-only isolation test (designed, never run)
10. Mixture-of-Experts for game-mode switching

**Open RL question never resolved:** whether v2's regression came from the jump penalty or the death penalty magnitude. The isolation test was designed but not run.

---

## 13. Phase 0 results (measured, 2026-08-18)

Measured on GD 2.2081 / Geode 5.8.2, Stereo Madness, 120 Hz display. Instrumentation is `mod/src/{main,Probe}.cpp`; raw traces land in the mod's save dir, and Geode writes a full per-session log to `<GD>/geode/logs/`.

### 13.1 Determinism — GATE PASSED
Ten replays of one recorded input sequence produced **one distinct trace hash**, byte-identical including step 0, under all three configs (no fix / §5.2 fix / fix + forced seeds). Phase 0 item 1 is retired.

**Correction to §5.2's framing:** vanilla GD is *already* deterministic here — config (a), with no fix at all, was also 10/10 identical. The fix is not what delivers determinism on a machine with steady vsync. It is still worth keeping for two measured reasons: real frame hitches occur (deltas of 1/35, 1/47, 1/30 observed at attempt start) and vanilla folds those straight into physics; and it yields exactly one physics step per call, which is the granularity a frame-perfect macro needs.

### 13.2 RNG does not affect physics — Phase 0 item 2 retired
`m_randomSeed` differs on every reset. Config (b) does **not** force seeds, so its ten identical runs each ran under a different seed. This is a cleaner result than the forced-seed config could give. `m_replayRandSeed` is 0 throughout. See the correction inline in §5.6b.

### 13.3 Tick structure — most of Phase 0 item 6
`GJBaseGameLayer::update` is called **once per rendered frame** with `dt = 1/refresh`, and GD subdivides that into `refresh/240` sub-steps internally. Returning `1/240` from `getModifiedDelta` collapses this to exactly **one 240 Hz physics step per call** — the single-tick primitive. Running `round(nativeDt / (1/240))` such calls per frame restores real-time pace at full 240 Hz input granularity, and raising that count is the Phase 0 item 3 throughput lever.

**peony's §5.2 part 3 (the `fractionOf240` midhook) is NOT needed.** Returning `1/240` already forces the step count to 1. This was the most build-fragile piece of §5.2 and it can be skipped.

### 13.4 Bindings corrections
- **`GJBaseGameLayer::m_currentStep` is NOT a physics step counter.** It sits in the replay cluster (`m_recordInputs`, `m_recordString`, `m_queuedRecordedButtons`, `m_queuedReplayButtons`) and read **0 for an entire level completion**. It is maintained only while GD's own replay system is active. Do not use it as a macro frame index or savestate key — maintain your own counter, reset per attempt.
- **`handleButton`'s third argument is not "isPlayer1" in any useful sense.** Filtering human jump input on it discarded every press; the game passes `false` for the large majority (observed true×1 / false×10). Injection currently passes `true`; unresolved.
- **`PlayLayer::loadFromCheckpoint(CheckpointObject*)` `0x3b7640`** is the restore primitive missing from §6.2 Layer 3, alongside `removeCheckpoint(bool)` `0x3b7f00` and `m_checkpointArray`.
- **`PlayLayer::levelComplete()` `0x3a7a80`** is hookable — a second completion signal not in the §5.1 table. `playEndAnimationToPos` is **not** virtual; do not declare it so.

### 13.5 Never run RLDash alongside the solver
The old `colin.rldash` mod was installed during the first sweeps. Geode chains `update` hooks, so its `glReadPixels` + per-capture `log::debug` ran inside our stepping path — exactly the two things §9 calls fatal. It alone accounted for config (a)'s apparent nondeterminism (4 distinct hashes → 1 once removed) and for a stale `m_isOnGround4` at step 0. **Any timing measurement (Phase 0 items 3 and 4) is invalid with it loaded.**

### 13.6 Still open
Replay fidelity against the recorded human run is close but not exact (diverged at step 1166/1823, same death point and final percent). Suspected off-by-one in recording phase, since fixed. Items 4, 5 and 7 (savestate cost, drift, completion detection) not yet started.

### 13.7 PHASE 1 COMPLETE — Stereo Madness solved and verified (2026-08-19)

```
SOLVED       20330 steps, 382 deaths, 0 restores, depth 520
VERIFICATION PASSED - clears the level from frame 0 in normal mode,
             practice off, no savestates. Not a savestate artifact.
```

The macro is in `solution.txt`. Every input in it was chosen by search.

**What made the ship section solvable** (it stalled at 35.23% for many runs):
- **Toggle-count iterative deepening** for air modes. Actions are "continue vs
  toggle", not "release vs hold", and the number of toggles is bounded and
  raised on exhaustion. Good ship paths are simple: "hold from entry" is one
  toggle. Plain DFS on raw hold/release reaches those last, after astronomically
  many high-toggle sequences.
- **Sliding commit floor.** Freeze decisions more than ~240 steps behind the
  frontier, anchored at mode transitions, and count toggles RELATIVE to the
  floor so a solved prefix hands its budget back. Without this the search spent
  millions of steps re-deriving an already-solved cube section.

### 13.8 GD's practice checkpoints are approximate BY DESIGN — do not fight this

The single most expensive finding of the project. Restores are **not** bit-exact,
and cannot be made so:

> For ship, UFO and wave, GD generates practice checkpoints **a set distance
> behind the icon** rather than at it, because those modes carry momentum. In
> cube they are placed on contact with a safe surface. The community-documented
> consequence is that practice mode is not 1:1 with real gameplay — which is why
> every serious bot ships a "practice fix".

Evidence accumulated before finding this, all consistent with it:
- Probe 4b: restores bit-identical in cube (step 480), never in ship.
- Restoring **557 named scalar fields** of `PlayerObject` + `PlayLayer` +
  `GJGameState` changed **nothing** — the solve stayed byte-identical
  (20330 steps / 1466 deaths) and verification still failed at the same step.
  The visible state *was* identical; the approximation is not in fields we can
  read.
- A 20,425-step solve found via savestates died at 91.71% on clean replay.

**Do not spend more time extending savestate field lists.** Two real defects were
found and fixed on the way, and both are worth keeping:
- **Restore lands exactly one physics step BEFORE the captured state.** Re-step
  once after restoring to realign. This alone moved verification 5.93% -> 91.71%.
- **`m_gameModeChangedTime` is not restored** by `createCheckpoint`.

### 13.9 The architecture that actually works

**Savestate-free search.** On backtrack, replay from frame 0 to the decision
instead of restoring. Measured **42,000 steps/sec (175x real time)** — *faster*
per step than the savestate path, because no checkpoints are created or held.
Cost is O(prefix) per branch instead of O(1), so it suits short segments.

**Iterative repair.** Verification reports the exact step where a
savestate-derived macro stops being real. Lock everything before it as a verified
prefix, replay that prefix once, then search forward savestate-free. Each round
converts a failure into locked-in progress. This is what closed Stereo Madness:
savestates for the first 89%, savestate-free for the last ~2000 steps.

**Implication for demons:** every demon is ship- and wave-heavy, i.e. mostly the
mode where checkpoints are approximate. The savestate/replay split is not a
Stereo Madness workaround, it is the general design — savestates in cube-like
modes, replay in air modes.

### 13.10 Method note

Three of the four real bugs were found by **mechanical byte-level comparison** of
object memory across a restore, not by reasoning about which field ought to
matter. Every reasoned guess (an offset, the held button, `m_extraDelta`) was
wrong. Probe 4a was Phase 0 item 5, was deferred as speculative, and turned out
to be the tool that resolved this. When a fix produces byte-identical output,
that is the signal to change technique, not to apply more of the same.

---

## 11. Reference material

- **peony, "60tps In 2.2 Is a Lie"** (June 2025) — the authoritative physics/determinism fix. `catflowers.substack.com/p/60tps-in-22-is-a-lie`
- **geode-sdk/bindings**, `bindings/2.2072/GeometryDash.bro` — authoritative class members and addresses
- **ThePickleGawd/geometry-dash-ai** — UCSB DQN/MoE, includes `docs/paper.pdf`. Best published result (levels 1-2)
- **Stanford CS231n 2017 report #605** — `cs231n.stanford.edu/reports/2017/pdfs/605.pdf`
- **FigmentBoy/GD-AI** — the public (1+1) hill climber, 2020
- **camila314/pathfinder** — **the closest prior art and the project to benchmark against.** Custom 2,583-line physics simulator + randomized hill climbing + GDR output. Limited to pre-2.2 static geometry (see §3). *"This code is not currently licensed for redistribution."*
- **camila314/gdp** — "Full decompilations of Geometry Dash physics functions." Valuable as a physics reference even though this project uses the live game.
- **Decoder0007/VBot** — pre-Geode macro bot (recording, playback, FPS bypass, speedhack, frame advance). Its own README: "Fix accuracy it's shit rn." Not relevant beyond confirming the ecosystem is macro tools.
- **zilko/xdBot** — Geode macro/replay bot. Not a solver; it is the *playback* half of the Pathfinder workflow.
- **ToastexGD/ToastyReplay** — **the single most useful code reference for this project.** Modern, actively developed Geode mod (v2.2.4, Geode 5.8.1, GD 2.2081) containing production-tested savestate capture/restore with drift reconciliation, correct physics bypass, single-tick stepping, forward trajectory simulation with object-activation tracking, and GDR format support. See §5.6. **No LICENSE file, so all rights reserved: read and learn, do not copy.**
- **matcool/ReplayBot** — small open-source replay bot. Valuable mainly for its documented nondeterminism findings ("rob uses a time function somewhere in the physics", replays randomly dying at 26%).
- **FigmentBoy/zBot** — practice fix (`m_yVelocity`) and an early physics-bypass pattern. **Superseded as a reference by ToastyReplay: zBot's physics bypass is the outdated/broken approach per peony. License is restrictive (Aseprite-style EULA); do not copy source.**
- **MCJack123/DashBot-3.0** — genetic search, archived, GD 2.11 only
- **Gustyper/Geometry_Dash_Reinforcement_Learning** — vision RL that failed on capture latency
- GD Resource Commune macro/TAS tool comparison table (Codeberg)

---

## 12. Working principles

- **Empirical first.** Changes are tested against real measurements before being kept. Unproven additions get reverted (the flag-latching mechanism is the precedent).
- **Disagree openly.** A plan followed past the point it stops making sense is worse than no plan. Flag inefficiency, staleness, or a better approach as soon as it is visible, with reasoning. Silent compliance with a bad instruction is the failure mode to avoid.
- **Uncertainty is a signal to check, not to guess.** Search, read the current bindings, or ask. Confident-sounding wrong answers have cost this project real time more than once.
- **Isolate one variable at a time.** Several past confusions were resolved faster once this was applied strictly.
- **Diagnostic runs before full runs.** A short run with one variable changed beats an 8-hour run with two.
- **Research by reading code, not descriptions.** Repos were cloned and read directly; claimed capabilities repeatedly differed from actual ones.
- **Never silently overwrite a working baseline.** Distinctly named artifacts for real comparisons.
- **Delete artifacts trained under assumptions later found wrong.** Do not build on a known-bad foundation.
- **Recruiting-timeline-aware scope discipline.** Complexity gets trimmed when it does not demonstrably help.