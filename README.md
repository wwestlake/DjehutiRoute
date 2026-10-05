# DjehutiRoute

An MIT, clean-room PCB routing library for the Djehuti tools.

## What works

- **Board model**: copper stackup, net classes (width, clearance, via size),
  SMD and through-hole pads, keepouts, board outline, per-net widths
  (IPC-2221 width from a net's current).
- **Autorouter** (`routeBoard`): a grid at the default class's track pitch with
  exact clearance checks against pads, keepouts and the edge; A* with bend and
  via costs; Prim-style trees for multi-pin nets; PathFinder negotiated
  congestion (real rip-up and reroute); output as polylines and through vias
  in nanometres.
- **Design-rule check** (`checkDesignRules`): exact geometry independent of the
  grid (clearance, shorts, edge, keepouts, open nets).
- **Tests** (`tests/router_tests.cpp`): hand-calculated lengths and via counts,
  IPC-2221 widths, DRC self-checks, and random 2-layer boards that must route
  with zero DRC violations.
- **Demo** (`examples/router_demo`): routes a generated board and writes an SVG.

Design and roadmap: `docs/ROUTER_ARCHITECTURE.md`. Requirements:
`docs/ROUTER_REQUIREMENTS.md`.

## Build (Windows)

```text
cmake -S . -B build-vs -G "Visual Studio 17 2022" -A x64
cmake --build build-vs --config Debug --target router_tests -- /m:1
build-vs\Debug\router_tests.exe
```

A host project adds the library with `add_subdirectory(<DjehutiRoute>)` and
links `djehuti_route`; tests and the demo are built only standalone.

## Licence

MIT (see `LICENSE`). Clean-room: GPL implementation code is never copied or
consulted.
