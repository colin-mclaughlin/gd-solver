# The steer target, the reach model, and what still has to be measured

Written 2026-09-12, against `mod/src/main.cpp`.

This is the design for ordering item 1 (lookahead / velocity 4c). It also
records a UFO bisect found while writing it, which is cheaper than item 1 and
should be run first.

---

## 0. Summary

The map's *coverage* is fine. Its *target* is degenerate. Under the default
reach mode the live window collapses to the gap the player is already standing
in, so the steer aims at the centre of the present and knows nothing about the
next 200 units. Every stage of the escape ladder exists to undo decisions that
target got wrong.

Two things follow, and they are independent:

1. **Item 1** — give the steer a reachability-bounded target. Needs one new
   measurement per mode class, listed in section 4.
2. **The UFO bisect** — a flag that was `true` when Clubstep cleared, flipped
   `false` on evidence from two levels that cannot test it, and then deleted.
   Section 6. One line to restore, no default change; it either settles Clubstep
   or clears the tap budget from suspicion.

---

## 1. What the evidence actually says

The map is not failing to speak, and it is not lying:

| level | air decisions steered | dead gaps reported |
| --- | --- | --- |
| Electroman Adventures | 13317 / 25016 (53%) | 0 |
| Clubstep | 17861 / 51561 (35%) | 2704 |
| Clutterfunk | 4486 / 11745 (38%) | 0 |

Probe 17 replayed a known-good macro and found `DEAD = 0` over 22,696 steps, so
liveness has no false negatives. Probe 19 found `window == gap` in **all twelve**
death cells on Clubstep.

The decisive observation is from the 2026-09-12 00.39 log, Electroman
Adventures, escapes #10 through #21:

```
dropped 596 of 1531 decisions to the floor
step 10087->5040, undoing 5047 steps behind best
floor idx 935 at step 5044, window 7680, anchor released
```

Twelve consecutive escapes, identical in every field. **Given 5,047 steps of
rewind — half the level — the search re-derives the same path and dies at the
same 49.69%.** More reach does not help. Whatever is wrong is not the
reachability of the search; it is that the search keeps being told to go to the
same wrong place.

---

## 2. What the steer computes now

`geomReachMode` defaults to `1` (OVERLAP), which sets reach to `1e18`. With
infinite reach the liveness sweep marks a gap live if it connects to anything at
all, and the live window `[wlo, whi]` widens to the whole gap. The steer target
is `0.5 * (wlo + whi)`; with `wlo`/`whi` equal to the gap bounds, that is just
the centre of the local gap.

Measured consequence: mean live window 247 units under ALLLIVE against 246 under
WINDOWED. The window carries essentially no information.

A ship moves roughly 1.6 vertical units per horizontal unit. Over the 150-200
units in which a corridor typically dissipates, that is 240-320 units of
vertical travel, comparable to the entire band height. So aiming at the centre
of the current gap is not a mild approximation; over the distance that matters
it is uncorrelated with where the player needs to be.

**What it should compute.** The target should be the centre of the set of
heights that are (a) reachable from the player's current state, and (b) still
live `N` steps ahead. That set is the intersection of a *reach envelope* with
the propagated live window. Both halves already exist in the code; only the
envelope is missing, because its input was a fitted constant that got retired in
`9803757` rather than replaced.

---

## 3. The reach envelope

### 3.1 Hold modes (ship, wave) — a cone

Holding converges to a constant climb rate; releasing converges to a constant
descent rate. Probe 16 measured both in a purpose-built level, identical at
0.5x / 1x / 2x / 3x / 4x, cross-checked to 0.1% against the game's own
`m_yVelocity`:

```
ship normal   1.800 up   1.440 down
ship mini     2.118 up   1.694 down
```

Over `t` steps the reachable interval from height `y` is

```
[ y - down * t ,  y + up * t ]
```

A **cone with straight edges**, and the numbers above are all it needs.

### 3.2 Tap modes (UFO, swing) — not a cone

This is the part the ship model does not cover, and the difference is real. A
tap is an instantaneous change to `m_yVelocity`, not a sustained force. Between
taps the player is in free fall. So:

- **The lower edge is a parabola**, not a line: free fall accelerates until it
  reaches terminal speed, then straightens out.
- **The upper edge is a sawtooth** — impulse, decay, impulse — whose average
  slope depends on how often a tap can be issued.

The second point is where "reach is different for UFO because it's impulses" is
exactly right, and it has a convenient consequence: **tap frequency is set by
`airBranchInterval`, which is ours to choose, not the game's.** The search cannot
tap more often than it makes decisions. So the upper edge is bounded by our own
decision rate and does not need to be measured — it needs to be *derived* from
two quantities that do.

---

## 4. What must be measured

### 4.1 Sufficient for a first cut: terminal rates only

For ship, the terminal rates in 3.1 are enough, and they are the **safe** choice,
for a reason worth stating precisely:

A player currently falling cannot instantly climb at 1.800 — it must decelerate
through zero first. So the true reachable set is always a **subset** of the cone
drawn with terminal rates. Terminal rates therefore give an **outer bound**.

That is the direction to err in. The existing comment at `climbPerStep` states
the asymmetry: *"Too SMALL narrows the window past the flyable set and deletes
routes; too large just fails to narrow."* An outer bound can never delete a route
the player could have flown. Velocity-aware tightening (ordering item "velocity
4b") is a strict improvement on top, and should not be attempted in the same
change.

Sanity check that the bound is not so loose as to be useless: at the measured
`dx` of 1.30 units/step, a 200-unit horizontal lookahead is 154 steps, giving 277
units of vertical reach against a band 786 units tall. It constrains about a
third of the band. That is a real corridor, not a formality.

### 4.2 Required new measurement: UFO

Two numbers, and **neither requires frame-perfect spamming**:

1. **Flap impulse.** From a resting/level UFO, one tap. Read `m_yVelocity` on the
   step immediately after. Call it `v_flap`.
2. **Free fall.** From the same state, no input at all. Log `m_yVelocity` every
   step until it stops changing. The per-step change is gravity `g`; the value it
   settles at is terminal fall speed `v_term_down`.

From those, max climb over `t` steps with a tap every `k` steps is computed, not
measured.

**One extra check that decides the shape of the model.** Tap twice in consecutive
steps and compare the post-tap velocities:

- if the second tap gives the **same** velocity as the first, the flap *sets*
  velocity. Max climb is then a constant `v_flap` per step, and the UFO upper
  edge is a straight line after all — same shape as ship, different number.
- if it gives roughly **double**, the flap *adds*, and the upper edge genuinely
  is a sawtooth whose slope depends on `k`.

I believe it sets, because an additive flap would let a spammed UFO accelerate
without bound and that is not how the game behaves. But that is an inference, not
a measurement, and the two-tap test costs nothing and settles it.

### 4.3 Explicitly deferred

Cube and ball reach. Neither is where anything is stalling, and a single value
per mode class is the first cut, not the end state. The map already knows where
the portals are, so per-section rates are a later refinement.

---

## 5. What must not change in this pass

- **Liveness.** `DEAD = 0` over 22,696 steps. It is exact, because slices are cut
  at object edges and the blocked profile is constant within a slice. Do not
  touch it.
- **The deadband.** `geomBandMode` mode 1 was built, tested and **failed**: the
  measured 7.2-unit band drove the steer rate from 37% to 94% and collapsed
  Clutterfunk to 33.72%, the same figure `geomForwardScan` hit at 92.6%. The
  deadband is a governor, not a tuning constant. Default stays 0.
- **The escape ladder.** Two attempts to retune its trigger have now failed in
  opposite directions: the original demoted on any 0.01% creep and so never
  escalated; the 2026-09-12 replacement never demoted and pinned every level at
  maximum window. Leave it alone until the steer target is fixed. It is
  compensation for this bug, and tuning compensation before fixing the cause is
  how the last three attempts were spent.
- **`decisionCost` for taps.** Making taps exempt from the budget was measured
  and **refuted**: Theory of Everything stalled at 33.24% against a 19s clear.

---

## 6. The UFO bisect — do this before item 1

Found while answering "what changed for UFO since Clubstep cleared". The chain:

| commit | date | what happened |
| --- | --- | --- |
| `5555e42` | | `tapStateSurvivesRestore = true` introduced. ToE 78.90%. |
| `9803757` | 2026-09-01 | **Clubstep clears, first demon.** Flag still `true`. |
| `6f7562d` | 2026-09-02 | Flag flipped to `false`. |
| `c57f14d` | 2026-09-11 | Flag **deleted**. |

The mechanism, from `6f7562d`'s own comment:

```
pushed choice=false (no tap)  ->  alternative is tap,    costs 1, GATED
pushed choice=true  (tap)     ->  alternative is no-tap, costs 0, FREE
```

Stale tap state left `sv.hold` alive past the tap, so the next Tap decision was
pushed *held*, and its alternative was free. Measured at the time as ~8% of UFO
branches escaping the toggle budget. It is a release valve on a budget that
charges every flap 1 while charging a ship climb of any length 1.

**Why the retirement is not safe evidence for Clubstep.** `6f7562d` justified the
flip on two levels:

- Theory of Everything — equal on time, slightly better on deaths.
- Clutterfunk — the commit states it reports `free 0T`, generating **no Tap gate
  evaluations at all**. Its own words: *"a NULL TEST"*.

Clubstep was never re-run. It is the level in the suite with the heavy UFO
section, and it is the level now stalling in a UFO section.

**Why the flag was still there to delete.** `6f7562d` ends: *"Kept as a flag
rather than deleted so a regression on a level with heavy UFO or swing can be
bisected against it. Delete once the 14-level suite is green."* The suite has
never been green — Base After Base has failed throughout. Deleting it in
`c57f14d` removed the bisect that note existed to preserve, for exactly the case
that has now occurred.

**RESULT, 2026-09-12.** Run, controlled, and settled.

| | Clubstep |
| --- | --- |
| `Y` on (stale) | **SOLVED at 20,274 deaths**, F4-verified from frame 0, zero diverging steps |
| `Y` off (control) | stuck at 71.75% after 27,430 deaths — 35% longer than the solve took |

Default is now `true`; `6f7562d`'s flip is reverted with the control run it never
had. This is the first F4-verified demon on this lineage.

**The mechanism in section 6 above is wrong, and the control is what proves it.**
If stale tap state worked by making Tap alternatives free, `free ...T` would rise
with the flag on. It falls: 865 of 54,083 gate evaluations (1.6%) with the flag
on, against 5979 of 96,814 (6.2%) with it off.

What the logs do show: the control spent **31 of its 57 escapes** in the escape
ladder's runaway state — window 7680, anchor released, dropping 671 decisions and
~7,450 steps on every escape, floor creeping by exactly 1 each time — while the
solving run never left window 240. The flag is not buying budget. It changes move
ordering enough to keep the frontier moving, which keeps the ladder out of its
fixed point.

So this flag is **masking an escape-ladder defect, not fixing one.** It earns its
default by the controlled run and nothing else, and it should be revisited once
the steer target is fixed.

**Superseded original plan.** Restore the flag defaulting to `false`, run Clubstep
with it on. Either it clears — in which case the tap budget is the Clubstep problem
and 3.2 / 4.2 become urgent — or it does not, and the flag is deleted for good
with a real measurement behind it this time.

---

## 7. Verification

Item 1 lands only if all of these hold. Any regression on a currently-solving
level fails it outright.

1. **Probe 17 first.** Replay a known-good macro against the new envelope and
   count how much of it falls outside the window. It must be **zero**. If a
   verified human-reachable path leaves the corridor, the envelope is too tight
   and the change is deleting routes. Stop there.

   The harness now exists: the F4-verified Clubstep solution of 2026-09-12 is a
   ground-truth path through a demon, and Probe 17 already scores it. Its reading
   against the *current* map is the baseline to beat:

   ```
   20482 steps checked
   OFF-MAP 0 (0.00%)   DEAD 0 (0.00%)   OUT-OF-WINDOW 0 (0.00%)
   ```

   That zero is what must survive. It is also the cleanest statement of why item 1
   is the right next move: the map already contains every step of a verified
   winning path, and the search still could not find it without a flag that masks
   an unrelated defect.
2. Mean live window must **fall** meaningfully below 247. If it does not, the
   envelope is not binding and nothing has changed.
3. `geoSteerDead` must stay 0 on Electroman and Clutterfunk.
4. Full suite. 13/14 is the floor, not the target.

The prediction to record before running, so that it can be wrong: Electroman's
identical-escape fixed point at 49.69% should break, because twelve identical
escapes are the signature of a target that does not vary with state, and this
change makes it vary with state.
