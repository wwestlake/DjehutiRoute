# Router Requirements

## Purpose

DjehutiRoute should become an MIT-licensed PCB routing engine and demo platform.
It should eventually produce board data that can be sent to common PCB
manufacturers using industry-standard files.

## Early Scope

- Board outline.
- Multi-layer stackup.
- Pads, vias, traces, copper zones, and keepouts.
- Netlist and ratsnest.
- Net classes.
- Clearance rules.
- Trace width and via rules.
- Interactive route-this-net command.
- Simple automatic route attempts.
- Manufacturing export planning.

## Manufacturing Outputs

Initial outputs to support:

- Gerber X2 or X3 copper/soldermask/silkscreen/mechanical layers.
- Excellon drill files.
- Drill map/report.
- Pick-and-place CSV.
- BOM CSV.
- Design-rule report.

Later outputs:

- IPC-2581.
- ODB++ only if licensing/distribution questions are acceptable.

## Routing Algorithm Roadmap

Start simple and build confidence:

- Grid or visibility-graph model.
- A* single-net router.
- Obstacle expansion for clearance.
- Layer-change/via costs.
- Rip-up and retry.
- Cost functions for length, vias, layer preference, congestion, and bends.
- Special routing for buses, differential pairs, and power traces.

## Clean-Room Rule

This repo is MIT. Do not copy GPL autorouter implementation code into this
project. Research papers, standards, behavior, and file-format documentation are
valid references; copied implementation code is not.
