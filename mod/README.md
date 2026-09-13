# GD Solver — the Geode mod

This directory is the mod itself. **The project README is one level up:
[`../README.md`](../README.md)** — results, architecture, the full hotkey reference and the
technical write-up all live there.

`about.md` is the user-facing description shown on the Geode mod page.

## Build

```sh
geode build
```

Requires the Geode SDK (`GEODE_SDK` must point at your checkout), MSVC with C++23, and CMake 3.21
or newer. The build compiles, packages `colin.gd-solver.geode`, and installs it into your GD mods
directory.

> **Delete `build/` and rebuild clean after any change to `CMakeLists.txt` or `mod.json`.**

## Layout

| Path | What it is |
|---|---|
| `src/main.cpp` | The solver, the geometry map, every probe, and all the game hooks |
| `src/Probe.hpp` / `Probe.cpp` | Measurement substrate — no allocation, formatting or I/O in the stepping path |
| `src/PlayerFields.inc` | The 196 `PlayerObject` fields a direct checkpoint load leaves untouched |
| `src/Socket.hpp` / `Socket.cpp` | Optional localhost progress channel. Never called per tick |

Three rules apply to anything added to `src/main.cpp`:

- **No `glReadPixels`, ever.** It is a GPU→CPU sync point and would cap the whole project at
  real-time framerates.
- **No logging inside the stepping path.** String formatting at 10,000+ steps/sec dominates
  runtime. Report by sampling, or at attempt end.
- **Anything holding cross-frame state resets in BOTH `PlayLayer::init` and
  `PlayLayer::resetLevel`.** A global that survived a reset once held jump forever and produced a
  scrambled trajectory that looked like a flag-detection bug.
