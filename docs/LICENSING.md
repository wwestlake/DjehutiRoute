# Licensing

DjehutiRoute is a mixed-license repository.

## MIT Areas

The PCB router is intended to be clean-room MIT software.

MIT areas include:

- `libs/djehuti_route`
- `examples`
- router architecture documents
- manufacturing-output research documents
- general project documentation unless explicitly marked otherwise

The top-level `LICENSE` file is the MIT license for these areas.

## GPL-3.0 Area

The electronics design tool is GPL-3.0:

```text
apps/DjehutiElectronicsLab
```

Its license file is:

```text
apps/DjehutiElectronicsLab/LICENSE
```

The electronics tool is GPL-3.0 because it is the SPICE/Xyce-oriented tool. The
research direction intentionally treats Xyce/SPICE integration as part of that
application track.

## Practical Boundary

The intended split is:

```text
MIT router library
  -> clean board/routing/manufacturing models
  -> no copied GPL autorouter implementation code

GPL electronics tool
  -> schematic/editor/research app
  -> SPICE/Xyce simulation workflow
  -> agent-assisted electronics workbench
```

The MIT router may be used by the GPL electronics tool, but the router should
remain independently useful and independently licensed.

## Clean-Room Rule For The Router

Existing GPL routers and EDA tools may be studied for concepts, workflows, and
file-format interoperability. GPL implementation code must not be copied into
the MIT router.

If functionality needs to cross from the GPL electronics tool into the MIT
router, rewrite it cleanly in the router library rather than moving code across
the license boundary.
