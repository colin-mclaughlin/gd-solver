# GD Solver

**A search that plays Geometry Dash by driving the real game.**

GD Solver turns the running game into a deterministic state machine — stepping physics headlessly
at around 30x real time, saving and restoring exact game states, and systematically searching for
an input sequence that clears the level.

Every input it produces was chosen by search. Nothing is recorded from human play, and no level
knowledge is supplied by hand.

It currently clears all fifteen of the first main levels end to end on one default configuration,
including **Clubstep** — a rated Demon — in about 52 seconds.

## Using it

1. Open a level.
2. Press **F2** to start the solver. Press it again to stop.
3. On success the macro is written to the mod's save directory as `<Level>_solution.txt`, one
   `0`/`1` per physics step.
4. Press **F4** to verify: the macro is replayed from frame 0 in normal mode, practice off,
   savestates disabled. That replay is the only result that counts — a solution found via
   savestates is a savestate artifact until it survives it.

Press **F9** at any time to watch the best path found so far at normal speed.

## Before you run it

- **Disable other mods that hook `update`.** Geode chains update hooks, so another mod's screen
  reads or per-tick logging end up running inside the physics stepping path. That will destroy
  throughput and can break determinism outright.
- Turn on Low Detail Mode, and turn off particles, shaders, glow, the ship trail and object glow.

## Scope

Cube, ship, ball, UFO and wave sections are supported. Robot, spider, swing and dual sections are
not yet — robot in particular needs a different action model, since its jump height varies
continuously with hold duration.

Built for **GD 2.2081** on Windows. Field offsets are version-specific.

Source, results and full technical write-up: <https://github.com/colin-mclaughlin/gd-solver>
