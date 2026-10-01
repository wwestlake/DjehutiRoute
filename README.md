# DjehutiRoute

DjehutiRoute is an MIT-licensed research workspace for electronics design tools,
PCB routing, and manufacturable board output.

The long-term goal is a clean, open routing engine that can support hobby and
professional workflows without inheriting GPL implementation constraints from
existing autorouters.

## Goals

- MIT open-source PCB routing library.
- Standalone demo applications for routing experiments.
- Integration with the Djehuti Electronics Lab research tool.
- Multi-layer board routing.
- Industry-standard manufacturing outputs, including Gerber, Excellon, and
  eventually IPC-2581.
- Clean data models that can be driven by agents, scripts, graphical tools, and
  conventional UI workflows.

## Repository Layout

- `libs/djehuti_route` - planned router library.
- `examples` - small demos and experiments.
- `apps/DjehutiElectronicsLab` - electronics design tool research app.
- `docs` - architecture notes, requirements, and process notes.

## Routing Direction

The router should grow in stages:

1. Board, layer, pad, via, obstacle, and net data model.
2. Design-rule model for clearance, trace width, vias, layers, and net classes.
3. Single-net interactive routing.
4. A*/maze routing with via costs.
5. Rip-up and retry.
6. Multi-layer automatic routing.
7. Differential pairs, length matching, pours, and tuning.
8. Manufacturing export.

The implementation should be clean-room MIT code. Existing open-source routers
can be studied conceptually, but GPL implementation code must not be copied.

## Electronics Lab

The electronics design tool currently lives under:

```text
apps/DjehutiElectronicsLab
```

It is a research shell for schematic capture, component data, simulation
adapters, sourcing records, and agent-assisted electronics workflows.

## License

MIT. See `LICENSE`.
