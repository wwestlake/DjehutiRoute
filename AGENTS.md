# AGENTS.md

Rules for any AI agent working in DjehutiRoute.

## What this is

An MIT, clean-room PCB routing library (`libs/djehuti_route`) used by the
Djehuti Electronics Lab and other tools. Design: `docs/ROUTER_ARCHITECTURE.md`.

## Rules

- **Clean room.** Never copy or consult GPL router/EDA implementation code.
  Papers, standards and file-format specs are fine.
- **No demos or stubs presented as features.** Nothing that returns canned
  results or prints "placeholder" in place of the real work.
- **Windows only**, Visual Studio 2022, **Debug** builds, **single core**
  (`/m:1`), one build at a time on this machine. Build into `build-vs/`:
  `cmake -S . -B build-vs -G "Visual Studio 17 2022" -A x64`, then
  `cmake --build build-vs --config Debug --target router_tests -- /m:1`.
- **Every router change**: `router_tests` passes, with expected values worked
  out by hand first, and **zero DRC violations** on every routed board. The
  exact geometric DRC (`drc.h`) is the judge, never the router's own grid.
- Files are UTF-8.
- Commit with a message that says why, and push to `origin/master`.
