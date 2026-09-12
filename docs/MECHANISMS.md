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

*What it should be:* the distance the player moves in one decision interval —
**tested 2026-09-11, and it fails.** See below.

> ### RESULT — the deadband is a governor, not a fitted accident
>
> Implemented as `steerDeadband` mode 1: measured climb rate × `airBranchInterval`
> = 7.2 units for a normal ship, against 30 in a 120-unit corridor.
>
> Mechanically it did exactly what it was supposed to. The map went from having
> an opinion on **37%** of air decisions to **94%** (`geoSteer 32997/35037`).
>
> And Clutterfunk went from solving at 99.00% to **stalling at 33.72%**.
>
> That number is already in the source. A comment on the forward-scan band
> records: *"scaling it to the intersection fired the steer on 92.6% of decisions
> against 36.7% without the scan, and Clutterfunk stalled at 33.72% instead of
> solving in 38s."*
>
> **Two unrelated mechanisms, both driving the steer rate past ~92%, produce the
> identical stall at the identical percentage.** That is a reproducible,
> mechanism-independent property: it is not about *how* the map gains authority,
> only *how much* it has.
>
> **Conclusion: the map's steering TARGET is wrong, and the deadband is what
> limits the damage.** The target is the centre of the live window — a position,
> with no notion of trajectory. Aiming at the centre of a corridor is not how a
> ship is flown through saws. Because steer-following is free, raising the map's
> authority hands the flying to a controller that does not know how to fly.
>
> This reframes much of §3 and §5. The pile of heuristics is not redundancy —
> several of them are **governors on a controller that aims at the wrong thing.**
> Grounding any of them in physics without first fixing the target will make
> things worse, predictably, and 0.25 is not a number to be replaced but a
> symptom to be explained.
>
> **The real steering item is what the map aims at, not when it speaks.** That is
> what `geomRouteSteer` and `geomUrgencySteer` were reaching for — a target
> derived from the upcoming constriction and a feasible trajectory to it, rather
> than the midpoint of wherever the player currently is.

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

---

## 10. Revised ordering, 2026-09-12 (morning) - SUPERSEDED by section 11

Kept for the record. Its top items are stale: `geomBranchMode` went from an
untested restore to the committed default, and two defects found after it was
written now sit above everything here.

The ordering in section 9 was written before a night of testing that inverted its
premise. Recording the revision here so it stops living in chat history.

### What happened

Two things moved the board, and **neither was on the ordering**:

- `tapStateSurvivesRestore` — `6f7562d` retired it on evidence from two levels,
  one of which that commit itself calls a NULL TEST. Restored and controlled:
  Clubstep **solves at 20,274 deaths with it on** and stalls at 71.75% after
  27,430 with it off. First F4-verified demon on this lineage.
- `geomBranchMode` — `c57f14d` (mine) deleted it as "unreachable", because mode 0
  short-circuits the rule. Mode 0 is the default; mode 1 shipped on a hotkey.
  Same build, same level: mode 0 stalls Clubstep at 58.43%, mode 1 **solves it in
  60 seconds at 6,206 deaths**, reproduced bit-for-bit eleven days apart.

Everything *built* that night was reverted: two escape-ladder triggers that failed
in opposite directions (one demoted on any 0.01% creep and so never escalated; its
replacement never demoted and pinned every level at maximum window, costing
Electroman its solve).

### The principle that follows

**Keypress experiments before builds.** Both wins came from exercising a flag that
already existed. Four of my judgements that night were wrong in the same way -
`geomUrgencySteer`, the `geomLookaheadSteps` ternary, `tapAllowance` and
`geomBranchMode` - all "this is inert" reasoned from the default without checking
whether the hotkey had ever been pressed. The last of those deleted the best result
in the project's history.

So: **a flag is not dead because its default makes it inert. Grep the logs for the
hotkey before deleting anything.**

### Revised list

**Tier 0 - keypress, zero build**

1. `geomBranchMode` - `M` x1 then x2, on Clubstep, the suite, and Base After Base.
2. `escapeArchiveRestart` - `A`. Promoted sharply. Its own build comment says the
   rewind "re-derives the same limit cycle from a slightly different starting
   depth every time - the detachment failure Go-Explore names", and that is
   verbatim what Electroman now measures: twelve identical escapes dropping 596
   decisions and 5,047 steps, and Clubstep dropping the same 671 forever. Built
   for this pathology, never once run.
3. Reach mode - `L`, WINDOWED. Only after item 6 gives it measured numbers; its
   input being a fitted constant is what retired it in `9803757`.
4. The predictive steer trio - `geomForwardScan`, `geomUrgencySteer`,
   `geomRouteSteer`. Note urgency is nested inside route.

**Tier 1 - measurement, no build**

5. UFO velocity, per `docs/STEER_TARGET.md` section 4.2. One tap for the impulse,
   no input for gravity and terminal fall, two consecutive taps to settle whether
   the flap sets or adds. Does not need frame-perfect spamming.

**Tier 2 - build**

6. **The steer target** - velocity 4c plus the reach envelope. See
   `docs/STEER_TARGET.md`. Probe 17 on the verified Clubstep macro reads
   `OFF-MAP 0 / DEAD 0 / OUT-OF-WINDOW 0`: the map contains every step of a
   winning route through a demon and the search could not find it. That is the
   whole case, and that zero is also the acceptance test.
7. Unmeasured bands - `bandHeightForPortal`. Ship/UFO 300 and ball 240 are
   measured; the rest are guesses, and the ceiling offset is `band/2 - 15`, so a
   wrong band gives a wrong roof. Feeds Base After Base, whose map roof measures a
   uniform 240 units above the enforced ceiling (`roofMargin 240@13761/240/240`).
8. Velocity 4b - `(y, vy)` pruning. The one item that genuinely needs an
   acceleration and reversal model. Strictly an improvement on item 6's outer
   bound, never a substitute.
9. The left-to-right reachability pass. Liveness sweeps right-to-left and answers
   "can I get OUT of this gap". It never asks whether the gap is reachable from
   the start at all.

**Dead or frozen**

- *Bounded rewind by mode transitions* - **disproven.** Electroman reproduced its
  frontier twelve times with 5,047 steps of rewind available. More reach does not
  help; the search re-derives the same wrong path because the target is the same.
- *Escape-ladder tuning* - frozen. Two attempts failed in opposite directions. It
  is compensation for the steer target, and tuning compensation before fixing the
  cause is how a whole session went.
- *Restore-path variants* - no evidence they cost anything.

### Debt

Two mechanisms now earn their defaults empirically without a mechanism we
understand. `tapStateSurvivesRestore`'s stated mechanism is measurably wrong - free
taps *fall* with it on, 1.6% against 6.2% - and what it really does is keep the
frontier moving so the escape ladder never reaches its fixed point. `geomBranchMode`
is likely similar. Both are flagged as such in the source so they get revisited
rather than enshrined. Fixing the steer target is what should retire them.

---

## 11. Ordering, revision 2 — 2026-09-12 (afternoon)

Section 10 was written before the `geomBranchMode` suite run. Two things have
changed since: the board moved for the first time in the project's history, and
two defects were found that sit upstream of everything section 9 and 10 listed.

### Where the board actually is

Committed default (`geomBranchMode = 1`), full 15-level suite:

**14 of 15 solve. Clutterfunk is the only failure.** Base After Base solved for
the first time ever, in 3 seconds. Every other level is faster or equal to the old
default, no exceptions — Clubstep 3.5x, xStep 2.5x, Can't Let Go 2.8x.

Uncommitted: the `lastGeoLo/Hi` carried-state fix. It is *correct* — mode 0
reproduces bit-for-bit (Clutterfunk 3,743 and Clubstep 20,274, both exact), and
Clutterfunk goes from a permanent 55.65% stall to **2,051 deaths / 20s, its fastest
ever**. But it trades Clubstep away (5,805 solve becomes a 58.41% stall), so it is
not committable as it stands. See item 3.

### The two new defects, both upstream

**A. `GeoMap::playerH` is one value for the whole level.**

```
3956:  double playerH = 15.0;   // smallest hitbox height: permissive
4008:  m.playerH = max(1.0, geomPlayerHeight);
```

Every liveness test, free interval, clearance check and the mode 2 tight-corridor
test (`geomTightHeights * playerH`) uses the *smallest* hitbox, for the whole
level, with no knowledge that a portal changed the player's size.

So **gaps between 15 and 30 units are false positives for a normal-size player.**
This is not inferred - it was observed directly. Watching the Clubstep stall under
F9: a mini ship falls, enters a cube portal, the hitbox expands, and it strikes a
spike above the portal that the map had scored as free space.

It is a correctness bug in the map, in the same class as the roof, and it is
upstream of the steer target: a reach envelope aiming at a window computed with the
wrong hitbox is still aiming at the wrong place.

It also means **the roof modes cannot be fairly re-evaluated until this is fixed.**
`geomPortalRoof` measures 11/14 against 13/14 and is off for that reason, but that
measurement was taken on a size-blind map. Whether it survives a corrected one is
unknown - stated as a question, not a mechanism.

**B. Exhaustion-driven deepening has no brake.**

Exposed, not caused, by the carried-state fix. With a stable tree, exhaustion
finally fires - and then runs away:

```
13:09:37 Deepening to 3    13:09:44 Deepening to 6
13:09:39 Deepening to 4    13:09:46 Deepening to 7
13:09:41 Deepening to 5    13:09:48 Deepening to 8 ... 9
```

Budget 2 to 9 in fifteen seconds, frozen at 58.41%, window never leaving 240. Mode
1 solves Clubstep at budget 2. The project already recorded why high budgets are
fatal rather than merely slow: *"a budget of 11 over a 7,400-step window is a
hopeless subtree that guarantees the next escape fails too."*

Before the fix this was invisible because exhaustion never fired at all.

### The list

**Tier 0 — keypress, no build**

1. ~~Mode 2 with the carried-state fix.~~ **CLOSED, 2026-09-12 13:30.** Clutterfunk
   solves (2,342 / 23s); Clubstep stalls at **the same 58.41%** as mode 1, budget 9,
   window 7680. Mode 2 is not the answer, and the fact that both modes wall at the
   identical percentage is the useful part - see the note below on why that
   promotes item 3.
2. `airBranchIntervalMax` sweep. Undocumented, never measured, and load-bearing
   since mode 1 became the default - it is the floor on how long the search may go
   without a decision. Needs a cycle key; that is the one constant worth tuning
   before the causes are fixed, because mode 1's whole behaviour routes through it.

**Tier 1 — map correctness. PROMOTED to the direct blocker**

3. Size-aware clearance. Defect A, and no longer merely upstream: with the
   carried-state fix the search exhausts budgets 2 through 9 at 58.41% on Clubstep
   in both branch modes, which is the search *correctly refuting* a region that has
   no path - because the map claims a gap that does not exist for a 30-unit cube.
   That is the spot observed under F9: mini ship, cube portal, hitbox expands,
   spike. Exhaustion at climbing budgets is the symptom of a map that lies.

   **Not a build-time-only fix.** A build-time size profile per x inherits the
   ambiguity that gave `geomPortalRoof` its two modes - which portal governs this x
   when the player may never take it - and the safe direction *inverts*: too low a
   roof deletes space so the roof takes the max, while too large a hitbox deletes
   gaps so a hitbox profile must take the min. The min is 15, which is what the map
   already uses. A single scalar per x cannot be permissive about routes and honest
   about clearance at once.

   So: **build time stays permissive** (liveness keeps its `DEAD = 0` exactness and
   never deletes a route), and a **live clearance test at the player's actual size**
   feeds the steer and the mode 1/2 branch rule. The live test is immune to fake
   portals by construction - it reads the size the player has, not the size a portal
   predicts - and it is the smaller change.

   Known limit, stated up front: the live test stops the steer aiming at a fatal gap
   and lets the branch rule see it, but liveness still calls that gap live, so the
   search can still enter, die and backtrack. If that is not enough for 58.41%, the
   fallback is a build-time per-section profile with the fake-portal risk accepted
   and measured.
4. Re-run the `geomPortalRoof` comparison on the corrected map. Only after 3, and
   at low priority: the roof programme existed to rescue Base After Base, and Base
   After Base now solves in 3 seconds without it. It is not needed for the board.

**Tier 2 — land the carried-state fix**

5. Brake the deepening. Defect B. This is the *only* escape/budget work now
   justified, and the reason is different from every previous attempt: it is
   blocking a fix that is demonstrably correct, rather than compensating for a
   steer target that is wrong. Three earlier attempts at this machinery failed -
   two triggers of mine in opposite directions, and `escapeArchiveRestart`, which
   collapses the commit floor to 0 and runs the budget to 15 - so the bar for
   touching it is a named, measured failure, and defect B is one.

**Tier 3 — the steering**

6. UFO velocity measurement (`docs/STEER_TARGET.md` section 4.2). One tap for the
   impulse, no input for gravity and terminal fall, two consecutive taps to settle
   whether the flap sets or adds. No frame-perfect spamming needed.
7. The steer target - the reach envelope. Now **downstream of item 3**, which is a
   change from section 9: fix what the map thinks the player is before fixing what
   it aims at. Probe 17 on the F4-verified Clubstep macro still reads
   `OFF-MAP 0 / DEAD 0 / OUT-OF-WINDOW 0`, and that zero remains the acceptance
   test.
8. Velocity 4b, `(y, vy)` pruning. Strictly an improvement on item 7's outer bound.

**Tier 4 — cleanup and capability**

9. Delete the dead code found while auditing: `tapFirst` / `tapNeedFallSpeed`
   (computed every Tap decision and never read - `c57f14d` removed the consumer and
   left the producer, which is why every log line in the project reads
   `tapFirst 0/0`), plus `geomUrgencyFrac` and `geomNarrowFrac` behind off flags.
   Do **not** delete `geomLookaheadSteps` or `geomVerticalReach`; both become live
   in item 7.
10. The fitted constants, in descending load-bearing order: `geomLeadHold 8.0 /
    geomLeadMini 4.0 / geomLeadTap 2.0` (three magic numbers from `af008c1`, live in
    every steer call), `geomClearanceBand 0.25` (condemned twice in its own
    comments), `stallLimit 400`, and the ladder constants. After items 3 and 7, not
    before - `geomClearanceBand` already has a recorded instance of the trap:
    *"Three levels wanted three different values, which is overfitting rather than
    tuning."*
11. The left-to-right reachability pass. Liveness sweeps right-to-left and answers
    "can I get OUT of this gap"; it never asks whether the gap is reachable from
    the start at all.

### Explicitly not on the list

- **Velocity 4c as a standalone flag flip.** Retracted. It was measured and it
  breaks a level: lookahead scaling alone takes Electrodynamix from a 1m51s clear
  to a 92.92% stall, and with reach to 29.47%. The fixed 300-unit lookahead is
  *compensating* for `geomVerticalReach` also being distance-based; the two are
  wrong in opposite directions and partially cancel. They have to land together
  with the speed-portal scan, inside item 7.
- **Bounded rewind by mode transitions.** Disproven. Electroman reproduced its
  frontier twelve times with 5,047 steps of rewind available.
- **General escape-ladder tuning.** Frozen, except item 5.
- **Restore-path variants.** No evidence they cost anything.

---

## 12. Settled register

**Rule for this file from 2026-09-12 onward: anything measured to work or not
work goes here, with the conditions it was measured under.** A result that lives
only in a log or a chat gets re-litigated or, worse, deleted - `geomBranchMode`
was removed as "unreachable" while holding the best Clubstep result in the
project's history, and `tapStateSurvivesRestore` was retired on a run its own
commit message calls a null test. Both cost days. Conditions matter as much as the
verdict: "works" without "under what" is how both of those happened.

### PROVEN

| result | conditions | evidence |
| --- | --- | --- |
| `geomBranchMode = 1` beats mode 0 | full 15-level suite, nothing else pressed | 14/15. Base After Base solved **for the first time** (318 deaths / 3s) after being the permanent failure. Every other level faster or equal, no exceptions. Clubstep 3.5x, xStep 2.5x, Can't Let Go 2.8x. **Committed as default.** |
| `tapStateSurvivesRestore = true` | Clubstep, default otherwise | on: solves at 20,274 deaths. off: stalls 71.75% after 27,430. **Committed as default.** |
| Physics determinism holds across sessions | same build, same level, same config | Clubstep 6,206 deaths / depth 3160 reproduced bit-for-bit **eleven days apart** (09-01 04:11 and 09-12 02:37). |
| `lastGeoLo/Hi` must be restored on reposition | modes 1 and 2 only; unread in mode 0 | Clutterfunk went from a permanent 55.65% stall to **2,051 deaths / 20s, its fastest ever**, and the exhaustion line fires where it never had. Mode 0 verified bit-identical: Clutterfunk 3,743 and Clubstep 20,274, both exact. **Correct but NOT committable - trades Clubstep away, see DISPROVEN-ADJACENT.** |
| The committed build's search tree was history-dependent | `a02573c`, full suite, counters only | **Measured, not inferred.** `lastGeoLo/Hi` is read by the branch rule and appears nowhere in `RestoreState`, so after a rewind it holds state from whichever path ran last. Divergence - the branch rule returning a different answer than it would with correctly-restored state - ran **0.47% to 4.75%** per level (1,727 of 36,388 on Clubstep; 1,049 of 38,608 on Clutterfunk). Control: the same counter reads **0/58,394** with the restore enabled. Consequence, measured: Clutterfunk triggered exhaustion **zero times** across the level, budget pinned at 1 through 2,867 deaths; with the restore it exhausts at 55.65% and deepens. "Exhausted every path with <= N toggles" is false when the path set is not fixed. Solutions found were still real (Clubstep F4-verified from frame 0) - the answers were sound, the evidence taken on top of them was not. |
| `geoStateRestore = true` costs a level and is worth it anyway | full suite, both positions, one keypress apart | 14/15 either way. Clutterfunk's stall becomes a 2,051 solve; Clubstep's 5,805 solve becomes a 58.41% stall. Base After Base survives (318 -> 303). Ten levels faster, three slower; the 13 both solve total 16,572 deaths OFF against 16,575 ON - same work, redistributed. Committed as default on the correctness argument above, not on the board. |
| The motion model is unusable because 53.65% of keys have nonzero spread | first census, threshold > 0.0001 | **RETRACTED - the threshold was wrong.** The key quantizes `vy` into 0.01 buckets, so two true velocities in one bucket necessarily give two next-velocities: ~0.01 of spread is forced by the measurement and can never be zero. **94.2% of the "bad" keys sat at exactly one bucket.** The gate was also wrong in principle - 4b computes a reachable RANGE and the model already stores `vyNextMin`/`vyNextMax`, so bounded spread widens the range conservatively rather than invalidating it. |
| `m_isAccelerating` and `m_jumpBuffered` are missing axes of the transition function | full-suite census, before and after, threshold > 1 bucket | **Confirmed.** True inconsistency fell **5.33% -> 2.97%** overall and **6.64% -> 3.40% on ship** - the mode `camila314/gdp` actually describes. Indicative rather than controlled (different runs, different level coverage). Residual 2.97% concentrates mildly in UFO (6.23%), onGround (4.15% vs 2.38%) and holding (3.93% vs 2.45%); no single axis dominates. |
| The motion model's inconsistency is a missing physics term | census with an `event` axis, 917k transitions | **No - it is events, and it is now closed.** Marking steps where an orb/pad was in contact, the ground state changed, the mode changed, or vy was forced to zero while airborne takes air free-flight residual from **3.11% to 0.36%**: ship 68/17,488 = 0.39%, **UFO 0/1,341 = 0.00%**. Predicted 0.4% before the run. |
| `m_touchedPad` / `m_touchedRing` / `m_touchedCustomRing` are per-step flags | direct measurement | **No - they are sticky for the attempt.** Testing the level rather than the rising edge marked nearly everything as an event and collapsed air free-flight from 18,790 keys to **40**. Fixed with edge detection - after which they proved **measured inert**: +32 event rows and a byte-identical residual. `m_hasEverHitRing` existing separately was the clue. |
| The last 0.39% is explainable | 68 surviving ship-air keys | **Unresolved, and accepted.** Only 9 of 68 sit on a pad/orb constant; the other 59 cluster at spread ~2.7 with no identified cause. 4.98% of ship-air samples. **Accepted rather than chased**, because the model is an interval: an imprecise key widens the reachable band, which makes 4b prune LESS. It cannot cause a wrong prune. The one case that could - true velocity leaving the recorded range - is exactly an event, and every event source (orb, pad, portal, floor, ceiling) is an object the geometry map already locates. |
| Velocity knowledge is useful at the current lookahead distance | interval propagation over the census | **No - it is short-horizon only.** Reachable band as a fraction of the naive terminal cone, from vy=0: **7% at 10 steps, 26% at 40, 65% at 120, 82% at 231**. At 20 steps it pins the ship to **9 units inside a 247-unit corridor**; at the current 300-unit lookahead (231 steps) the band is 612-729 units, wider than any corridor, so nothing is ever prunable. 4b belongs at the decision horizon (4-16 steps), not the steer lookahead. |
| 4b velocity pruning recovers Clubstep | full suite, prune on vs off | **No.** It fires **1,527 times on Clubstep and the wall stays at 58.41%.** Sound (14/15 held, nothing lost) and cheap (throughput UP almost everywhere: xStep +42%, ToE +32%, Clubstep +23%), with real wins - xStep 20s->11s, Electrodynamix 41s->28s, Clutterfunk 20s->17s - cancelled by ToE 38s->50s and Electroman 19s->26s. **Net -2% on total time.** Those losses are search-order divergence, not overhead: ToE ran 32% faster per step and simply explored more. Kept, default OFF. |
| Clubstep's 58.41% wall is a reachability problem | three independent tests | **No - it is decision granularity.** (1) mode 0, which offers a decision every 4 steps, SOLVES Clubstep while modes 1 and 2 do not; (2) `airBranchIntervalMax = 8` cleared 58.41% and hit a different wall at 66.74%; (3) 4b prunes 1,527 branches there and the wall does not move. The search is not exploring impossible branches - **the branch it needs is never offered as a decision.** |
| Correcting the steer lead's missing vy->dy conversion helps | full suite, `P` on | **No - 13/15, it LOSES Cycles.** The fix is dimensionally right (`y += lead*vy` omits dy = vy*0.2279, so a climbing ship is projected **63 units** where eight steps of travel is 14.4) and the split is the finding: **long air levels improved** (ToE 4215->3538, Electroman 1938->1711, xStep 2176->2012, Clutterfunk 2051->1999) while **short/cube-heavy levels collapsed** (Stereo Madness 216->655, Base After Base 303->903, Can't Let Go 500->1165). `geomClearanceBand` and the `geomLead` values were fitted WITH the overshoot present. **Second recorded instance of compensating dimensional errors** - the first is velocity 4c's lookahead/reach pair, whose comment already concluded "they have to be fixed together". Do not correct these one at a time. |
| `V` and `P` compose | full suite, both on | **No - 12/15**, worse than either alone, losing Clutterfunk AND Theory of Everything. Both change search order; stacking them compounds the divergence rather than adding their effects. |
| The steer is load-bearing | ABLATION, `J`, full suite, `geoSteer 0/13712` so the mute is complete | **Yes - 12/15 without it**, losing Theory of Everything and Electroman Adventures. But it is load-bearing on some levels and HARMFUL on others: Clutterfunk 2051->4929 and Cycles 490->1002 without it, while **xStep 2176->1542 and Electrodynamix 4261->2953 both improve**. Directionally right, badly calibrated - the profile of a mutually-compensating constant set. **Replace, do not repair**: two attempts to correct one term have now failed in opposite directions. |
| The steer's lead term can be corrected or removed | three runs, full suite | **No - it is strongly load-bearing, and the series is monotone.** Kept: **14/15**. Scaled by the measured 0.2279 (`P`): **13/15**. Removed entirely (`Q`): solved 2 of the first 3 attempted and **stalled on Polargeist**, a level that normally takes 482 deaths in 4s; the run was stopped there, so "2/15" is not a full-suite figure. Every attempt to change it regresses. Leave it alone until something replaces the whole construction. |
| A reach-CLAMPED steer target can work at the current lookahead | `Q`, full suite, plus arithmetic | **No, and it is inert by construction, not mis-tuned.** At the live 300-unit lookahead (230 steps) the measured band is **779 units - wider than the entire 786-unit map band** - so the clamp never binds and `Q`'s only effect was deleting the lead. **Structural: reach constrains only at <=100 units (161-unit band) while corridors change over 150-200 units. Those windows do not overlap**, so an endpoint clamp cannot work at any horizon. Doing it properly means propagating the band step-by-step and intersecting with the live corridor at each step. PROCESS NOTE: this arithmetic was available before the build and was written down as the likely outcome; it was built anyway. |
| Driving the steer's authority toward 100% collapses the board | `geomClearanceBand = 0.00`, full suite | **Third independent confirmation of the 92% law.** Band 0.00 (steer speaks on essentially every decision) solved 4 then stalled, and with the 4b prune solved 7 then stalled; both runs were stopped early, so these are not full-suite figures. Previously shown via `geomForwardScan` at 92.6% and the measured deadband at 94%. Also proves the deadband is doing real work rather than being inert. |
| The branch interval can be derived from physics | mode 3, full suite | **No, and the reason is conceptual, not a tuning miss.** `brIv 4/15.7/16` - the derived ceiling is pinned at the clamp ~98% of the time, so mode 3 collapses into mode 1: **10 of 14 levels bit-identical**, only Time Machine moving (691 -> 532). The arithmetic: a mean 247-unit corridor less a 15-unit hitbox is 232 units of slack, and 232/1.800 = **129 steps**, against `airBranchIntervalMax = 16`. Physics says coast 8x longer than the constant allows, while every Clubstep result says decide MORE often (mode 0 at 4 steps solves it; 8 clears the wall; 16 does not). **The branch interval is a search-expressiveness parameter that merely has units of steps - it belongs in the search-strategy category, not the physics one.** |
| Probe 16's ship figures are sound | independent cross-check | mini/normal measured at **1.1765** in both directions; the decompilation gives `m_vehicleSize = 0.85` for mini, and `1/0.85 = 1.17647`. Five-digit agreement with a documented game constant. |
| Liveness has no false negatives | `geomReachMode = 1`, 22,696 steps | `DEAD = 0`. Re-confirmed on the F4-verified Clubstep macro: `OFF-MAP 0 / DEAD 0 / OUT-OF-WINDOW 0` over 20,482 steps. |
| Clubstep's solution is real, not a savestate artifact | practice OFF, no savestates, from frame 0 | F4: zero diverging steps, clears the level. First verified demon on this lineage. |

### DISPROVEN

| claim | conditions | evidence |
| --- | --- | --- |
| Mode 2 rescues Clubstep | with the carried-state fix | stalls at **58.41%, budget 9, w7680 - the identical wall mode 1 hits.** Clutterfunk solves either way (2,342 / 23s). Mode 2's tight-corridor exemption is not what that stall needs. |
| `escapeArchiveRestart` helps the fixed point | `A` alone, Clutterfunk | 58.98% where **default SOLVES**. Archive works mechanically (2,000 cells, 36 restarts) but collapses the commit floor to 0 and runs the budget to 15. |
| Gating the ladder reset on clearing the escalated region | all levels | pinned every level at maximum window; **cost Electroman its solve.** Reverted. |
| Resetting the ladder on any progress | Clubstep | 0.01-0.03% creep demoted it six times in 12,000 deaths; the window never passed 480 and the anchor never released in 19 escapes. |
| Bounding the rewind by mode transitions | Electroman | twelve byte-identical escapes with **5,047 steps of rewind available**, re-deriving the same path. More reach does not help. |
| A measured steer deadband (`geomBandMode = 1`) | Clutterfunk | steer rate 37% -> 94%; collapses to 33.72%, the same figure `geomForwardScan` hits at 92.6%. The deadband is a governor, not a tuning constant. |
| Scaling the lookahead with speed | Electrodynamix | alone: 1m51s clear -> **92.92% stall**. With reach scaling too: 29.47%. The fixed 300-unit lookahead is *compensating* for `geomVerticalReach` also being distance-based. |
| Exempting taps from the toggle budget | Theory of Everything | stalled at 33.24% against a 19s clear. |
| `tapAllowance` / `isTapBudget` did anything | whole life of the file | gated on `tapRateBudget`, permanently false. **Never once called from the search.** |
| `deepenOnStall` | Electroman, Clubstep | cost both their solves outright. |
| `geomPortalRoof` modes 1/2 help | 14-level suite, size-blind map | 11/14 against 13/14 default. And **not needed**: the roof programme existed to rescue Base After Base, which now solves in 3s without it. |
| Probe 16's live table is usable during a solve | any solve run | off by 3-4x on three of four cells where purpose-built-level ground truth exists (`ship-mini` reads 8.680 against a measured 2.118). Use the purpose-built level only. |
| A build-time per-x hitbox profile can fix size-blindness | reasoning, not measured | inherits `geomPortalRoof`'s "which portal governs this x" ambiguity with the safe direction **inverted** - the permissive choice for a hitbox is the *minimum*, which is the 15 the map already uses. A single scalar per x cannot be permissive about routes and honest about clearance at once. |
| A live clearance check at the player's *current* size can fix it | reasoning, not measured | a mini ship reads 15, accepts a 15-unit gap, and dies at the same spot one frame later when the portal expands it to 30. **The check has to use the size at the x being evaluated, not the x being occupied.** |
| The map treats portals as obstacles | read the build | **No.** The filter is `Solid \|\| Slope \|\| Breakable` and `Hazard \|\| AnimatedHazard`, with an explicit comment "portals are neither". The portal gap is live; the map has no reason to avoid it. |
| Clubstep's 58.41% wall is the map lying about clearance | all three branch modes, with the carried-state fix | **No - size-blindness cannot explain it.** Same map, same `playerH`, same hitbox: mode 0 **SOLVES at 20,274**, modes 1 and 2 both stall at 58.41%. A map defect would wall all three. This retires the whole `segPlayerH` proposal as a fix for *this* stall. |
| Sweeping `airBranchIntervalMax` can fix Clubstep | 16/12/8/6/4, with the carried-state fix | **No, and the sweep is NON-MONOTONIC, which refutes the constant rather than tuning it.** 16 -> 58.41% (budget 9); 12 -> 58.40% (budget 23); **8 -> 66.74% (budget 5)**; 6 -> 58.43% (budget 24); 4 -> SOLVES, but 4 equals `airBranchInterval` so that is mode 0, not mode 1. 8 clears the 58.41% wall and hits a different one, confirming it *is* a granularity wall - but 6 being worse than 8 is the overfitting signature already recorded for `geomClearanceBand`. The interval has to be derived, not picked. |
| Mode 1's Clubstep solve was independent of the `lastGeoLo/Hi` bug | committed build vs the same build plus the fix | **No.** Without the fix mode 1 solves Clubstep at 5,805 deaths, budget 2. With the fix it stalls in every branch configuration. The instability was *manufacturing decision points*: replaying forward after a rewind created branches the original pass never had, which handed the search expressiveness it has no principled way to get. With a stable tree it correctly refutes a tree too coarse to express the solution - which is what budget climbing to 24 means. Mode 1's headline Clubstep number partly depended on a bug. |
| Clubstep's 58.41% wall is the commit floor or the budget | mode 1, at the stall | **No.** `budget 9  commit 820(w7680)` - ~960 of ~1780 decisions mutable, nine toggles allowed, and it exhausts budgets 2 through 9. The search has the freedom to release earlier and refutes every path it can express. |

### Open, with the measurement that would settle it

| question | measurement |
| --- | --- |
| Does size-aware lookahead clear 58.41%? | evaluate the lookahead window at the size implied by portals between here and there; Clubstep must pass 58.41% and the F4 macro must still read `OUT-OF-WINDOW 0`. |
| Is `airBranchIntervalMax = 16` right? | **Now the prime suspect for 58.41%.** It is the only thing forcing a decision when the corridor ahead is unchanged, and it *interpolates between the branch modes*: at 4 it equals `airBranchInterval` and mode 1 becomes mode 0, at 16 it is mode 1 as shipped. Mode 0 solves Clubstep and mode 1 does not, so the answer is between them. Sweep with `B` (16/12/8/6/4). |
| Does size-aware clearance matter at all? | **Demoted, not dead.** It cannot explain 58.41% (see DISPROVEN), but `playerH = 15` level-wide is still wrong and the F9 observation of a hitbox expanding into a spike is still real. Needs its own failing case before it is worth building. |
| Does the deepening runaway matter once the map stops lying? | budget 2 -> 9 in 15s is wasted time regardless, but it may stop being fatal. |

---

## 13. Ordering, revision 3 — physics foundation

Written after the `airBranchIntervalMax` sweep came back non-monotonic, which is
the fourth failure in a row from tuning a constant against a configuration that is
itself wrong. This revision is organised around one distinction the earlier ones
missed.

### The distinction

**Physics constants** have a true value and measurement finds it:

`geomLeadHold/Mini/Tap` · `geomVerticalReach` · `geomClimbPerStep` ·
`geomLookaheadSteps` / `geomLookaheadFixedX` · `airBranchInterval` /
`airBranchIntervalMax` · `geomPlayerHeight` · `geomBandHeight` ·
`geomCeilingOffset` · `geomTightHeights`

**Search-strategy constants** have no true value, because they describe how a tree
is explored, not how the game behaves:

`toggleBudget` / `maxToggleBudget` · `stallLimit` · `commitLookbackSteps` /
`wideningFactor` / `maxCommitLookbackSteps` / `minCommittedFraction` ·
`solverArchiveCap` / `archiveMinPctFrac` · `goCellX/Y/Vy`

**This predicts the record.** Everything that stuck touched the first group.
Everything attempted on the second failed: two escalation triggers in opposite
directions, `deepenOnStall`, `escapeArchiveRestart`, and the
`airBranchIntervalMax` sweep. There is nothing to tune toward.

**So physics does not help the search by tuning its constants. It helps by pruning
its tree** - which is item 2.4, and needs no constant at all.

### The finding that reorganised this

`geometrySteer` projects the altitude before asking the map anything:

```cpp
y += lead * vy;        // vy is m_yVelocity, raw; geomLeadHold = 8.0
```

Probe 16 established that `m_yVelocity` reads **4.4444x** the per-step dy. A
climbing normal ship is 1.800/step, so `vy` is 8.000, so the projection is
**64 world units** - a quarter of the 247-unit mean live window, shifted up,
before the corridor is consulted.

No interpretation of "8" yields that. Eight steps would be 14.4 units; projecting
to the lookahead x (300 units at 1.30 dx = 231 steps) would be 415 units. It is a
fitted number from `af008c1` sitting between two meanings and matching neither,
and it is live in every steer call on every level.

The deeper issue is shape, not value: a single projected point assumes the player
coasts, when it will be steering the whole way. The honest answer at t steps ahead
is the interval `[y - down*t, y + up*t]`. **`geomLeadHold`, `geomVerticalReach` and
the reach envelope are the same physics; the lead term is its degenerate fitted
form.** They should land as one change, not three.

### Phase 0 — decide the board (2 runs, no build)

The `lastGeoLo/Hi` carried-state fix is uncommitted and everything below depends on
which configuration survives it. It has only ever been tested on two levels.

- **0.1** Full suite, **mode 0 + fix**. Mode 0 + fix already solved *both* Clubstep
  (20,274) and Clutterfunk (3,743), which no other configuration has. The open
  question is Base After Base - that solve came from mode 1, and mode 0 without the
  fix failed it.
- **0.2** Full suite, **mode 1 + fix**, `B` x2 (ceiling 8). Same question, plus
  whether the 66.74% Clubstep wall is level-specific.

**Gate:** if neither keeps Base After Base, the fix is reverted and the committed
14/15 stands while phase 1 proceeds.

### Phase 1 — measurement (no build)

- **1.1 Run the motion model** (`W`, `motionModelEnabled`). A passive census of
  `vy' = f(mode, size, hold, prevHold, vy)`, already built, never once run.
  `prevHold` is in the key because UFO and swing respond to the press EDGE, so
  impulse and force modes are captured by one census.

  **Self-validating, and this is the gate:** f is a function, so one key must yield
  one `vy'`. The spread column reports any key that yields two, which means the key
  is missing a field. Nothing consumes this until spread is ~0.

  This is 4a generalised to every mode and it is the sole input to 2.4.

- **1.2 UFO envelope** (`docs/STEER_TARGET.md` 4.2) if 1.1 does not already cover
  it: one tap for the impulse, no input for gravity and terminal fall, two
  consecutive taps to settle whether the flap sets or adds.

### Phase 2 — physics replaces fitted scalars, one change per run

- **2.1 The lead term becomes a reach interval.** Retire `geomLeadHold/Mini/Tap`
  and `geomVerticalReach` together; the steer asks about
  `[y - down*t, y + up*t]` at the lookahead rather than a single projected point.
  Prediction to record: this is the largest single suspect for the observed "holds
  too long, releases too late, enters the portal too high".
- **2.2 The reach envelope / steer target** (`docs/STEER_TARGET.md`). Subsumes
  `geomClimbPerStep` and makes WINDOWED mean something. Acceptance test unchanged:
  Probe 17 on the F4-verified Clubstep macro must still read `OUT-OF-WINDOW 0`.
- **2.3 The branch interval derived from slack and climb rate.** `slack / climbRate`
  steps rather than a constant ceiling. Subsumes mode 2 and `geomTightHeights`,
  removing a mode and a constant. **Justified by the non-monotonic sweep, not by
  confidence** - 8 clears Clubstep's 58.41% wall and 6 does not, which refutes the
  constant rather than locating it.
- **2.4 `(y, vy)` reachability pruning - 4b.** From the measured f, the reachable
  y-range at `x+d`; a branch whose range intersects no live interval is dead and is
  killed soundly. **No constant.** This is the one item that makes the search
  smaller rather than better-steered, and therefore the one that makes the
  search-strategy constants matter less.

### Phase 3 — search strategy, treated as such

Not tunable toward a true value. These get structural fixes or deletion.

- **3.1 The deepening runaway.** Budget 2 -> 24 in seconds once exhaustion became
  reachable. Structural, not a constant: deepening has no relationship to whether
  the space it is deepening into is worth entering.
- **3.2 The escape ladder.** Frozen. Three independent failures. Revisit only after
  2.4, when a smaller tree may make it unnecessary rather than better.

### Phase 4 — cleanup

- Dead code: `tapFirst` / `tapNeedFallSpeed` (computed every Tap decision, never
  read), `geomUrgencyFrac`, `geomNarrowFrac`.
- `geomClearanceBand` last, not first: its measured replacement is already recorded
  as failed, and its own comments record the trap - *"three levels wanted three
  different values, which is overfitting rather than tuning."*

### Cut

- **The roof programme.** Existed to rescue Base After Base, which now solves in 3s
  without it. `geomPortalRoof` stays off.
- **`segPlayerH` / size-aware clearance.** Cannot explain 58.41% - mode 0 walks past
  the same geometry with the same hitbox. `playerH = 15` level-wide is still wrong,
  but it needs its own failing case before it is worth building.
- **`escapeArchiveRestart`.** Measured harmful: 58.98% where default solves.

---

## 14. Ordering, revision 4 — after the physics programme

Revision 3 set out to build a physics foundation. That is done, and it changed
what the ordering should be.

### What the measurement programme produced

- A validated free-flight model: **0.36% inconsistency in air, 0.00% for UFO**,
  with every exception identified as a velocity override (orb, pad, portal,
  landing, ceiling) that the geometry map already locates.
- **Where velocity knowledge lives:** 7% of the naive cone at 10 steps, 26% at 40,
  65% at 120, 82% at 231. It dies well before the steer's 300-unit lookahead.
- **The UFO impulse, +13.27 in one step, SETS rather than adds** - measured from
  the census, no custom level needed.
- **Probe 16 independently cross-validated** twice: mini/normal 1.1765 against a
  documented `m_vehicleSize = 0.85`, and 8.00 x 0.2279 = 1.823 units/step against
  a measured 1.800.
- **The velocity scale, dy = vy * 0.2279**, which is what finally makes
  `geomLeadHold` measurable rather than arguable.

Honest accounting: 4b itself is **net -2% and off by default**. The census was
necessary to build it and the payoff was the knowledge, not the feature. The next
item needs only Probe 16, which predates all of this.

### The one failure left, and what it is

Clubstep, 58.41%. **Decision granularity, not reachability or the map** - three
independent results in the register say so. And neither uniform setting works:

    mode 0 (decision every 4 steps)      solves Clubstep, LOSES Base After Base
    mode 1 (decision on corridor change) solves Base After Base, LOSES Clubstep
    airBranchIntervalMax 16/12/8/6       NON-MONOTONIC - 8 clears it, 6 does not

Two levels want opposite things from one constant, and the sweep between them is
not even monotonic. That is the signature of a quantity that must be derived.

### The list

**1. Derive the branch interval.** `slack / climbRate` steps rather than a fixed
   ceiling: fine where the corridor is tight, coarse where it is open. Uses the
   measured 1.800 (Probe 16). **Subsumes `geomBranchMode` mode 2 and
   `geomTightHeights`** - one fewer mode and one fewer constant. Success is
   recovering Clubstep while holding Base After Base; floor is 14/15.

**2. The lead term.** `geomLeadHold = 8.0` is live in every steer call on every
   level, and is now MEASURABLE rather than arguable: with dy = vy * 0.2279, a
   climbing ship has vy ~ 8.0, so `y += lead * vy` projects **64 world units** -
   a quarter of the 247-unit mean corridor - where a true 8-step projection is
   14.4. It matches no interpretation of "8". Retire it together with
   `geomVerticalReach` in favour of the reach interval; they are the same physics.

**3. The reach envelope / steer target** (`docs/STEER_TARGET.md`), now with
   measured inputs. Acceptance test unchanged: Probe 17 on the F4-verified
   Clubstep macro must still read `OUT-OF-WINDOW 0`.

**4. The search-strategy constants, by ablation, not tuning.** `toggleBudget`,
   `stallLimit`, `commitLookbackSteps`, `minCommittedFraction` have no physical
   value to converge on. Each has an off switch (`stallLimit = 0` restores pure
   DFS). Turn each off across three representative levels; anything that can be
   removed without losing a level gets deleted rather than tuned.

**5. Cleanup.** `tapFirst` / `tapNeedFallSpeed` (computed every Tap decision,
   never read), `geomUrgencyFrac`, `geomNarrowFrac`. Keep `geomLookaheadSteps` and
   `geomVerticalReach` until item 3 consumes them.

### Cut or parked

- **4b** - kept, default off. Revisit if item 1 changes the search shape enough to
  make its wins repeatable.
- **`escapeArchiveRestart`** - measured harmful.
- **The roof programme** - Base After Base solves without it.
- **`segPlayerH` / size-aware clearance** - cannot explain 58.41%; needs its own
  failing case.
- **Velocity 4c as a flag flip** - measured, breaks Electrodynamix.
- **Chasing the last 0.36%** of census inconsistency - it is events, and an
  interval model loses only tightness to it, never soundness.
