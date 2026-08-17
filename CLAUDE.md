# GD Solver

Build a Geode mod that drives the live Geometry Dash game as a
deterministic state machine, stepping physics headlessly at high
speed with savestates, and systematically searching for an input
sequence that clears the level. Output as a GDR macro.

## Before doing anything

Read `docs/PROJECT_PLAN.md` in full. It contains verified binding
info, corrections to widely-circulated but wrong techniques, and a
record of what was already tried. Do not start work without it.

## Non-negotiables
- I run Geometry Dash; you cannot. Write instrumentation, then ask
  me for the numbers.
- Never copy source from referenced repos (licensing). Reimplement.
- No logging or `glReadPixels` in the physics stepping path.
- Push back when something looks wrong. Search or ask when unsure.

## Build
`geode build` from inside `mod/`. Delete `mod/build/` and rebuild
clean after any CMake or mod.json change.