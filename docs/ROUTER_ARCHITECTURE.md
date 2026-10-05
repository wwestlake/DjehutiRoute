# Router Architecture

Clean-room MIT design. Every technique here is the standard published one
(Lee/A* maze routing, PathFinder negotiated congestion, Prim-style Steiner
trees); no GPL router code is used or consulted.

## Units

Geometry is integer nanometres (`int64_t`), exact and free of rounding drift.
APIs take millimetres for convenience and convert once.

## Board model (`board.h`)

- **Stackup**: copper layers in order (`F.Cu`, `In1.Cu`, ..., `B.Cu`).
- **Design rules**: net classes with trace width, clearance, via drill and
  via diameter; a default class; nets reference a class.
- **Pads**: shape (circle, rectangle, oval), centre, size, layers (one for
  SMD, all for through-hole), net.
- **Keepouts**: polygons on given layers (no copper of any net).
- **Outline**: board edge polygon; copper keeps `edge clearance` inside it.
- **Nets**: name, class, pads. Constraints from simulation (RMS current ->
  IPC-2221 width) set a net's width.

## Routing grid (`routing_grid.h`)

A dense 3D lattice (flat arrays, index `(layer * H + y) * W + x`) at pitch
`p = default width + default clearance`, so centrelines one cell apart are
exactly legal for the default class.

Clearance model: every track claims a halo of `width/2 + clearance/2` around
its centreline; two nets are legal when their halos do not overlap, which is
the clearance rule exactly. Pads, keepouts and the board edge claim their
geometry plus `clearance/2` as fixed obstacles; a pad's own net may enter it.

A diagonal step also claims the two corner cells it passes between: at this
pitch a 45-degree segment is only `p/sqrt(2)` from those cell centres, inside
the halo of anything there. This forbids corner-cutting and crossing diagonals
by construction.

Wider nets (power) claim a disc of more cells around each centreline cell.

## Path search (`maze_search.h`)

A* over the lattice with direction in the state, so the cost can charge
bends:

- step cost 1 (straight) or sqrt(2) (45 degrees),
- bend cost (45-degree turn small, 90-degree larger, reversal forbidden),
- via cost, only where a via's pad and drill fit on every layer,
- optional preferred direction per layer,
- congestion cost from the negotiation below.

Multi-source / multi-target: a search starts from every cell of the net's
current tree and ends at any cell of the next pad.

## Multi-pin nets

Prim-style Steiner approximation: start the tree at one pad, then repeatedly
connect the nearest unconnected pad to the whole tree with a multi-source
search, adding each path to the tree.

## Negotiated congestion (`pathfinder.h`)

PathFinder (McMurchie and Ebeling, 1995). Each iteration routes every net
with cells allowed to be shared, at a cost

    cost(cell) = (base + history(cell)) * present(cell)

where `present` grows with how many other nets already use the cell this
iteration and `history` accumulates on cells that stayed overused. Nets
touching overused cells are ripped up and rerouted next iteration, and the
present-congestion factor rises each iteration, until no cell is shared.
This is real rip-up and reroute: nets negotiate for contested space instead
of the first-routed net winning.

## Output (`route_result.h`)

Per net: tracks as polylines (collinear points merged) with width and layer,
and vias with drill and diameter, in nanometres; unrouted connections listed
with the reason.

## Verification (`drc.h`)

An exact geometric design-rule check, independent of the grid: every track is
a capsule (segment plus half-width), pads are their shapes, vias are discs.
It reports clearance violations between different nets, shorts, copper
outside the edge clearance, copper in keepouts, and open connections. The
test suite requires zero violations on every case; the router is never
trusted on its own grid's say-so.

## Roadmap after the core

- Interactive route-this-connection and push-and-shove.
- Differential pairs and length matching.
- Copper pours with thermal reliefs.
- Gerber X2, Excellon, pick-and-place and BOM output; IPC-2581 later.
