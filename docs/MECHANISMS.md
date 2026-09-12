# Mechanism audit

Written 2026-09-11, against `mod/src/main.cpp` at 12,061 lines.

The purpose of this document is to make the project legible again. Every
mechanism gets: what it does in plain terms, what its inputs are (a measured
quantity or a chosen constant), what evidence supports it, and a verdict.

Verdicts are one of:

- **KEEP** — has a recorded failure without it.
- **GROUND** — the mechanism is sound, but its constant is fitted and should be
  derived from a measurement instead.
- **COLLAPSE** — real, but duplicated by another mechanism; the two should merge.
- **DELETE** — measured inert, measured harmful, or unreachable.
- **ISOLATE** — no data either way. Needs an ablation run before any verdict.

A note on why this exists: on 2026-09-11 a flag was enabled, tested, and
produced a bit-for-bit identical result, because it was nested inside a second
flag that was off. Neither the author nor the model could see that without
reading the source. That is the failure this document is meant to prevent.

---

## 0. The shape of the problem

- **~100 configuration fields.**
- **Three complete search algorithms**, of which one is in use.
- **37 key bindings**, of which roughly a third toggle things that are off by
  default and have never been isolated.
- **Default config solves 13 of 14 levels.** This is not a rescue. The solver
  works; the debt is in not knowing *which parts* make it work.

---

## 1. Search algorithms

### 1.1 DFS with toggle semantics — F2 — **KEEP**

The live searcher. At each decision point it chooses "keep the current input" or
"flip it". A flip is a *toggle*; a long hold costs one toggle rather than many.

*Evidence:* before toggle semantics the solver stalled permanently at 35.23% on
Stereo Madness — the ship could not express a sustained hold, because every
4-step segment reset to release and "all hold" was the last leaf DFS would ever
reach.

### 1.2 Width-K beam — B — **ISOLATE, likely DELETE**

A parallel frontier of K states advanced in lockstep, with its own ranking,
checkpoint pool, diversity buckets and eviction. Fourteen config fields.

*Evidence:* commit `27cce22` — *"Tried best-first search and other approaches,
none worked, moving on to trying DFS with archive."* It has not been run against
the 14-level board since.

### 1.3 Go-Explore — F3 — **ISOLATE, likely DELETE**

A cell-archive search: discretise state into cells, keep the best per cell,
resample promising cells, explore randomly from them. Eleven config fields
(`goCellX/Y/Vy`, `goEpisodeSteps`, `goMaxDepth`, `goArchiveCap`,
`goProgressBias`, `goTournament`, `goImproveMargin`, `goFrontierPct`, `goSeed`).

*Evidence:* none recorded. No run of it appears in any log reviewed.

> Two of the three algorithms are dormant. They are not harmful — they only run
> when their key is pressed — but they are a large fraction of the file, they
> share `Solver::clear()` and the macro buffers with the DFS, and they make every
> read of the search code ambiguous about which searcher a line serves.

---

## 2. The map

Built once per F2 (and by the G/H probes) before any stepping.

### 2.1 Slices — `geomSliceX = 30` — **KEEP**

The level is cut into vertical strips, with a cut at every object edge. Inside a
strip the geometry is constant, which is what makes everything downstream exact
rather than approximate. Clutterfunk is 2,749 strips.

*Input:* structural (one block). Not a tuning knob.

### 2.2 Free intervals — `geomPlayerHeight = 15` — **KEEP**

The gaps in each strip the player fits through.

*Input:* measured — the smallest hitbox height across modes. Deliberately
permissive.

### 2.3 Liveness, the right-to-left sweep — **KEEP**

Working backwards from the end: a gap is *live* if it overlaps a live gap in the
next strip by at least a player height. Answers "is there still a route to the
end from here?"

*Evidence:* Probe 17 replayed a known-good 22,696-step macro against the map and
found **DEAD = 0** — the sweep never marked a gap dead that the winning run used,
under either roof setting.

*Note:* overlap is **exact, not an approximation**. At a strip boundary the
player occupies a player-height of space and must be legal on both sides; if two
gaps are disjoint there is solid material between them at that x. An earlier
version that let windows leak between gaps un-flagged two of ToE's corridors.

### 2.4 The live window (`wlo`/`whi`) — **KEEP**

Within a live gap, the part that still leads somewhere. A tall gap can be live
with only its lower third usable.

### 2.5 Reach mode — `geomReachMode = 1` (OVERLAP) — **ISOLATE**

Mode 1 sets reach to infinity, so a gap's window expands to its whole gap and
the sweep reduces to pure overlap connectivity. Mode 0 (WINDOWED) narrows the
window by `climbPerStep × slice width / speed`. Mode 2 disables liveness.

*Evidence:* mode 0 was abandoned because `climbPerStep` derived from
`geomVerticalReach = 60` — a fitted constant that `9803757` dropped as
untrustworthy. **That number is now measured** (see 6.1), so the objection that
retired WINDOWED no longer holds. Untested with the real value.

*What WINDOWED would buy:* a player at the top of a tall gap, needing a passage
fifty strips ahead near the bottom, is currently marked live. A reach model asks
whether the descent is possible in the steps available. That is real pruning the
map cannot do today.

### 2.6 The portal roof — `geomPortalRoof = 0` — **KEEP, default off**

Crops each strip at the camera ceiling, because a ship cannot fly above it even
where nothing solid is there. Mode 1 = prefix maximum (only ever rises), mode 2 =
most recent (a lower section lowers the roof).

*Evidence for correctness:* build-time audit reports **0 emptied slices across
all 14 levels in both modes**; `roofMargin` is 0 (tight, never below the real
ceiling) on all 14; Probe 17 puts the winning path at 4 off-map steps, equal to
the probe's own baseline.

*Evidence on value:* **Z is 11/14, default is 13/14.** Its only unique win is
Base After Base, which no other configuration solves. It costs Clutterfunk, ToE
and Clubstep.

*Supporting constants, all measured:* `geomGroundY = 90` (player is 30 tall and
rests at y 105); `geomCeilingGrid = 30` (verified by bracketing nudged portals
from both sides in two custom levels); `bandHeightForPortal` (ship 300, UFO 300,
ball 240, all measured); `ceilingOffsetForPortal` = band/2 − 15 (verified on six
portals and by a ball/ship pair at identical y reading 420 and 450).

`geomRoofLeadX = 60` is a chosen value, but it is a *max* relaxation — too large
only leaves a strip slightly too tall, which fails to prune and never deletes.
Measured lag was 15–31 units.

### 2.7 Speed per segment — `geomSpeedScan = true` — **KEEP**

Reads speed portals so the map knows horizontal units per step in each strip.
Fact, not heuristic.

---

## 3. Steering — how the map talks to the search

### 3.1 Ordering — `geometryOrdering = true` — **KEEP**

At an air decision the map says "try up first" or "try down first". It only
reorders the two branches; it never forces one.

*Evidence:* the solver's progress before geometry existed versus after is the
whole reason the map was built.

### 3.2 The deadband — `geomClearanceBand = 0.25` — **GROUND**

The map says nothing while the player is within 25% of the target interval's
height from its centre.

*Why a deadband is needed:* without one the steer is a bang-bang controller with
no hysteresis — above centre aim down, below centre aim up, every decision. It
would chatter, and because steer-following is free (3.3) the search would chatter
at zero cost while the ship barely moves.

*Why this value is wrong:* the code's own comment says it —

> *"A narrower target getting LESS tolerance is backwards... a 285-unit corridor
> gives 71 units of deadband, a 90-unit threaded gap gives 22. The target should
> tighten; the tolerance should not."*

and

> *"Three levels wanted three different values, which is overfitting rather than
> tuning, so this is back to the only setting that solved all of them."*

Scaling to the corridor gives the most slack where precision matters least. The
value was chosen because it happened to solve three levels, on a build where both
the map roof and the ceiling rule were wrong.

*What it should be:* the distance the player moves in one decision interval.
A ship climbing at 1.800 units/step with decisions every 4 steps covers ~7 units.
Below that, chatter; far above, late reaction. **This is a velocity-derived
quantity, not a tuned fraction.**

*Consequence today:* the map has an opinion on **11% of air decisions**
(`geoSteer 14204/125687`), and the counters show `dead 0, win 14204` — **every
steer in every run was a drift correction.** The map is purely reactive. It
reports that you have already left the window; it never warns that you are about
to.

This is exactly Clutterfunk's wall. At x 16,560 a mini ship rides the ceiling at
y 270–300 having cleared a saw; at x 16,590 the ceiling drops to y 270. A steer
does fire — about 23 steps before the block — far too late to reverse a climb.
The right moment to descend was earlier, while still inside the window, where the
map is silent by construction.

### 3.3 Free following — `geomFreeFollowing = true` — **KEEP**

A toggle that matches the map's suggestion costs **zero** against the budget.

This is the keystone of the whole design. Without it the toggle budget is a
combinatorial bound: N flips over D decisions is ~C(D, N), so with a
200-decision window budget 3 is already ~1.3M combinations and budget 5 is ~2.5
billion. **The budget can only work if the map is right most of the time and
being right is free.**

### 3.4 Steer gate — `geomSteerGate = 0` (ALWAYS) — **ISOLATE**

Modes: 0 speak at every air decision, 1 only where liveness separates something,
2 only when badly off (`geomNarrowFrac = 0.5`).

*Evidence:* none isolated. `geomNarrowFrac` unexamined.

### 3.5 Lookahead — `geomLookaheadSteps = 229` — **GROUND**

How far ahead the map samples for live space, expressed as a reaction time in
steps and converted to distance using the player's measured horizontal speed.

*Provenance:* the 300 was fitted to ToE, whose corridors split at x 20,130 and
seal at x 20,400. 229 steps is 300 units at 1×.

> **CORRECTION (found while deleting):** `geomLookaheadSteps = 229` is **not the
> live value.** `geomLookaheadScaleWithSpeed` is `false`, so the ternary always
> picks `geomLookaheadFixedX = 300` — a flat distance. The speed-scaled version
> was written, documented, and never enabled. Its own comment argues it matters:
> *"At 2x speed the player covers 300 units in half the time, so it gets half the
> warning; at 3x, a third. Electrodynamix introduces 2x and 3x and stalls three
> times (34.89%, 73.65%, 94.01% - 67 of its 111 seconds), which is what that
> would look like."* This moves from GROUND to **ISOLATE** — a built fix, with a
> stated prediction, sitting switched off.

*What it should be:* how far ahead the player can still act, which depends on
climb rate — the measured quantity. This is velocity item 4c.

### 3.6 Predictive steering — `geomForwardScan`, `geomRouteSteer`, `geomUrgencySteer` — all **false** — **ISOLATE**

Three mechanisms that would let the map anticipate rather than react.

**`geomUrgencySteer` is the important one.** It asks *"given my climb rate and
the steps remaining, is this move becoming urgent but still possible?"* and
speaks only in that band. It needs a single max climb rate — no acceleration
model, no turning model.

*Why it is off:* its only input was `observedClimb()`, which read the fitted
vertical-reach constant that `9803757` dropped. **The mechanism was fine; the
number under it was a guess.** `observedClimb()` now returns measured ship values
(see 6.1).

> **DEFECT:** `geomUrgencySteer` is nested inside `if (g_config.geomRouteSteer)`,
> which is off. Enabling it alone does nothing. This was discovered by enabling
> it, running a control, and getting a bit-for-bit identical result. It also has
> two key bindings — `U` (original) and `1` (added in ignorance of `U`). Both are
> inert without `O`.

### 3.7 Mode lead constants — `geomLeadHold = 8`, `geomLeadTap = 2`, `geomLeadMini = 4` — **GROUND**

A velocity lead term added per mode class. Added in `af008c1`, which reported ToE
1m14s → 21s and Time Machine 59s → 7s.

*Input:* fitted per mode class by hand. These are velocity quantities and should
come from the measured envelope.

---

## 4. The cost model

### 4.1 Toggle budget — `toggleBudget = 1`, `maxToggleBudget = 24` — **KEEP**

Bounds the search by *input complexity* rather than depth: iterative deepening on
the number of flips allowed above the commit floor.

*Critical property:* this is combinatorial (see 3.3). It cannot be the mechanism
that flies a complex section — a section needing ten distinct taps is
unreachable by deepening. It is a completeness backstop that only works because
following the map is free.

*Deepening trigger:* on **exhaustion** — every path at ≤N toggles explored.

> **DEFECT — the deadlock.** Deepening waits for exhaustion; exhaustion requires
> enumerating the whole budget-N space above the floor; the rewind cannot reach
> far enough back to finish that enumeration. So on a stalled level the budget
> can never rise. Measured on Clutterfunk under Z: **budget 1 for all 28 escapes
> and ~11,200 deaths.**

### 4.2 `tapAllowance(window)` — **COLLAPSE**

Taps permitted above the floor, scaled by the mutable window so it "means the
same thing at any width". Inert at the default window.

*Problem:* it couples the commitment mechanism to the cost model as a side
effect. Widening the window silently changes what a decision costs. Two concepts
on one lever.

### 4.3 `deepenOnStall` (Q) — **DELETE**

Deepen the budget on a stall instead of waiting for exhaustion — the workaround
for 4.1's deadlock.

*Evidence of harm:* Electroman Adventures stalls at 55.09% and Clubstep at 57.32%
under **Q alone**; both solve on default. It also combines with Z to cost Time
Machine its solve.

*Why:* it fires on *every* stall. The correct trigger is far narrower — only once
the rewind is genuinely exhausted.

---

## 5. Commitment and escape

This is one concept — *how far back may the search go, and when may it go
further* — expressed in three units across seven knobs.

### 5.1 Commit floor — `commitLookbackSteps = 240` — **KEEP**

Decisions more than N steps behind the best progress are frozen as solved.

*Evidence:* measured — a 1200-step window left ~870 steps of solved cube mutable
and the search burned 3.27M steps re-deriving it while the budget never once
deepened.

The floor is the **later** of two anchors: a sliding window `lookbackSteps` behind
the frontier, and a *mode-transition anchor* at the last section boundary at or
before the best step.

### 5.2 Stall trigger — `stallLimit = 400` — **ISOLATE**

Deaths at an unchanged best percentage before an escape fires. `0` restores pure
DFS.

*Input:* chosen. Never varied.

### 5.3 `escapeJump = 200` — **DELETE**

"Pop 200 decisions off the stack."

*Evidence:* Probe 18 logged all 28 escapes on Clutterfunk. **Every escape in
stage 1 was bound by the commit floor, never by `escapeJump`** — actual drops
were 16, 32, 60, 92, 93, 113, 120, 140. Once the anchor was released it became
the cap at exactly 200 and stayed there for twelve escapes.

So it is either ignored or it is the thing preventing progress. It has never once
been the right number.

### 5.4 Widening schedule — `escapesBeforeWidening = 3`, `wideningFactor = 2`, `maxCommitLookbackSteps = 7680` — **COLLAPSE**

After 3 escapes with no progress, double the window.

*Evidence:* measured inert. On Clutterfunk the window went 240 → 480 → 960 →
1920 → 3840 → 7680 — a **32× increase** — while the actual rewind went 243 → 483
→ 633 and then **fell** to 618. Every escape logged `anchor held`: the
transition anchor was binding, so widening past it changed nothing, and the
ladder spent three escapes per doubling on five doublings that moved the floor
zero units.

*What it should be:* escalate **on effect, not on schedule**. Probe 18 already
computes the number ("undoing N steps"). If a widen does not move the floor, it
was inert and the ladder should advance immediately.

### 5.5 Anchor release — `escapesBeforeAnchorRelease = 10` — **COLLAPSE**

Stage 2: drop the transition anchor so the floor can move back past a portal.

*Evidence:* it works — releasing it took the rewind from 625 to 1003 steps
instantly, confirming the anchor was the binding constraint. But at the default
of 10 it arrives roughly **10,400 deaths** into a stall, and runs give up before
then. Cycling it to 1 (key F) fires it immediately and it behaves as designed.

### 5.6 Air-section guard — **DELETE**

A third floor mechanism: do not rewind out of the air section currently being
worked on.

*Evidence:* bound **zero** of 28 escapes in the Probe 18 data.

### 5.7 `escapeArchiveRestart` (A) — **ISOLATE**

An alternative to rewinding: jump to a promising archived cell instead.
Default off, verdict never reached, listed as an open question since `27cce22`.

---

## 6. Measurement — probes

These emit numbers and change no behaviour. They are the reason anything in this
document can be stated as fact.

### 6.1 Probe 16 — vertical envelope — S key

Per (mode, size, speed, direction): max climb from the game's own `m_yVelocity`,
and from slope × canonical per-step dx as an independent cross-check.

**Result — ship, measured in a purpose-built level:**

| | up | down |
|---|---|---|
| normal | **1.800** | **1.440** |
| mini | **2.118** | **1.694** |

Identical at 0.5×, 1×, 2×, 3× and 4×, so **vertical rate is speed-independent**.
The two methods agree at exactly 4.4444 across ten cells. Up/down is 1.25 in both
sizes, mini/normal is 1.1765 in both directions. These are physics constants.

Cube, ball and UFO are unmeasured and fall back to the sampled table. They are
not currently implicated in any failure, so this is deliberate.

### 6.2 Probe 17 — path audit — runs inside F4

Replays a known-good macro against the map and classifies every step as OFF-MAP
(no interval holds the player), DEAD (interval marked unreachable), or
OUT-OF-WINDOW.

*Value:* the trajectory physically happened, so anywhere the map disagrees, the
map is wrong. It settled two questions that argument could not.

### 6.3 Probe 18 — escape anchor — logs per escape

What the rewind actually did against what it was asked to do: which knob bound
the drop, how many steps of solved level it discarded, and the floor, window,
budget and anchor state at that moment. Produced every finding in section 5.

### 6.4 Others

Probe 2 (frame delta), 6 (corridor sweep, C), 7 (geometry dump, G), 8 (dead-end
map, H), 9 (reach sweep, R and motion dump, W), 10 (ceiling offset), 12 (live
ceiling readout, I), 15 (which objects re-snap the band), plus the cell census
and the determinism sweeps on F10–F12.

**Verdict: KEEP all.** They cost nothing when not invoked and they are the only
reason this audit could be written from evidence rather than from reading code.

---

## 7. The graveyard

Off by default, never resolved. Deleted on 2026-09-11: `tapOrderByNeed`,
`tapStateSurvivesRestore`, `tapBudgetExempt`, `tapRateBudget`, `geomSpeedReach`
(five flags, keys T and Y, three startup readouts).

Still present and pending a decision:

| flag | status |
|---|---|
| `geomBranchMode` | branch on geometry change instead of fixed cadence (M) |
| `geometryDeadPrune` | prune dead space outright |
| `ceilingProbe` | a measurement harness |
| `motionModelEnabled` | a measurement harness |
| `beamBestFirst` + 13 beam fields | tried, "none worked" (`27cce22`) |
| 11 `go*` fields | no recorded run |
| `geomLookaheadScaleWithSpeed` | superseded by `geomLookaheadSteps` |
| `geomSteerMargin` | comment says "superseded by geomClearanceBand" |
| `hybridRestore`, `noSavestates`, `resyncEnabled` | restore-path variants |

---

## 8. Verdict summary

**KEEP (7):** DFS + toggle semantics · slices · free intervals · liveness sweep ·
live window · free following · commit floor · toggle budget · map ordering ·
portal roof (default off) · speed scan · all probes.

**GROUND in measured physics (4):** `geomClearanceBand` · `geomLookaheadSteps` ·
`geomLeadHold`/`Tap`/`Mini` · `geomVerticalReach` (done).

**COLLAPSE into one mechanism (3):** widening schedule · anchor release ·
`tapAllowance`.

**DELETE (4):** `escapeJump` · air-section guard · `deepenOnStall` · the
graveyard in §7.

**ISOLATE before deciding (6):** beam · Go-Explore · reach mode ·
`geomSteerGate`/`geomNarrowFrac` · `escapeArchiveRestart` · the predictive steer
trio.

---

## 9. What this implies about the ordering

1. **Delete** — §7 graveyard plus the four measured-inert items. Should shrink
   the file.
2. **Ground the steering constants** — deadband and lookahead from the measured
   climb rate. This is where velocity 4c lands, and it is the change most likely
   to raise the map's 11% and make the whole design work as intended.
3. **Collapse commitment** — one floor, escalation on effect rather than
   schedule, with budget deepening as the final stage once the rewind is
   genuinely exhausted. Deletes more knobs than it adds.
4. **Ablate** — turn each survivor off across the 14-level board and find out
   what it is worth. The code already asks for this: *"If ToE holds up with this
   off, the whole reachability model ... is unnecessary and should not be
   built."*
5. **Only then, new capability** — the left-to-right reachability pass, and
   `(y, vy)` branch pruning (velocity 4b, the one use that genuinely needs an
   acceleration and reversal model).

The discipline this document is meant to enforce: **nothing ships that has not
been isolated, and nothing stays that has been measured inert.**
