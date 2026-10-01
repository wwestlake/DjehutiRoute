# VR Breadboard And Mechanical Board Plan

## The Muse

The muse for this work is bigger than PCB CAD. It is a virtual electronics lab where designing, learning, prototyping, and playing all happen in the same world.

Imagine standing at a workbench in VR with drawers full of parts. You open a resistor drawer, pick a value, pull out a component, bend the leads, and push it into a breadboard. You grab jumper wires, route power rails, place diodes into a clipping bridge, and watch the circuit come alive. The system knows the real schematic underneath the build, highlights nets, warns about shorts, and lets you probe the design with virtual instruments.

The experience should feel like building real devices, but with the freedom of software:

- no burned fingers
- no lost parts
- no waiting on shipping before trying an idea
- instant measurement tools
- instant undo
- instant comparison between schematic, breadboard, PCB, and final 3D assembly

For guitar pedals, this becomes especially powerful. A user can design a fuzz, boost, overdrive, filter, or modulation circuit, audition it as audio, breadboard it virtually, turn it into a PCB, inspect the mechanical fit, and eventually send the board out for prototype manufacture.

This is part engineering tool, part educational lab, part design game, and part invention engine.

## Purpose

Djehuti Electronics Lab should not stop at schematic capture or PCB routing. The product direction is a full design-to-inspection loop:

- draw the circuit as a schematic
- simulate and audition it
- lay out the PCB
- define the mechanical board and enclosure constraints
- inspect the assembled result as a 3D OpenGL model
- support a VR mode for headset-scale close inspection and teaching

This turns the tool into a virtual electronics bench, not just a CAD editor.

## Board Representations

The same project should carry several linked views:

- Schematic: logical circuit intent, components, nets, buses, instruments, probes.
- PCB layout: footprints, pads, tracks, vias, copper zones, layers, silkscreen.
- Mechanical board: board outline, holes, keepouts, connector placement, enclosure constraints, height limits.
- Assembly model: placed components with approximate 3D bodies, labels, polarity marks, lead geometry.
- Breadboard model: optional prototyping view with jumper wires and through-hole parts.

The authoritative data should remain structured JSON/IR. OpenGL renders a view of that model; it should not become the source of truth.

## VR/OpenGL Viewer

The viewer should support:

- orbit, pan, zoom, section cut, and exploded views
- selectable components and nets
- highlighted current paths and selected signals
- probe placement on pads, pins, or breadboard rows
- layer visibility and board transparency
- clearance/DRC visualization
- simulated instrument overlays
- headset mode for close inspection

Initial implementation can be OpenGL desktop first. VR should use the same scene graph and interaction model where possible.

## Breadboard Mode

Breadboard mode should help users prototype and teach circuits before PCB layout.

Core objects:

- solderless breadboard rails and terminal strips
- jumper wires
- through-hole components
- DIP IC packages
- off-board instruments and supplies

The schematic net model can validate whether breadboard rows match the intended circuit. A future assistant should be able to route a simple schematic onto a breadboard and explain the placement.

## VR Parts Drawer And Build Mode

The VR environment should also work as a hands-on electronics construction space. This is partly a serious prototyping tool and partly a game-like learning/building experience.

Core concepts:

- Parts drawers organized by resistors, capacitors, inductors, diodes, transistors, ICs, wires, knobs, switches, jacks, power supplies, and sensors.
- Pick up parts from drawers and place them onto breadboards, perfboards, or PCBs.
- Snap leads to breadboard holes, pads, sockets, and terminal blocks.
- Bend and dress wires/components in 3D.
- Color-coded resistor bands and capacitor markings.
- Inventory-aware parts so the same design can map to real drawers or purchasable parts.
- Live continuity/net highlighting as the user builds.
- Mistake detection for reversed diodes, wrong rails, floating grounds, overcurrent, and short circuits.
- Tutorial/challenge mode where the system asks the user to build a circuit and validates the result.

This lets the app become a virtual electronics bench where users can prototype, learn, and play with circuits before ordering hardware.

The experience should still be data-driven. A VR placement is another view over structured assembly/breadboard data, not a one-off 3D scene.

## Mechanical Board Design

Mechanical board data should include:

- board outline
- mounting holes
- connector positions
- pedal enclosure template
- knob, switch, jack, LED, and power connector placement
- component height constraints
- keepout and courtyard regions

For guitar pedals, enclosure templates matter. The board is not complete until it fits the box and the controls line up with real hardware.

## Agent Access

The agent should be able to:

- inspect schematic, PCB, mechanical, and 3D assembly data
- answer questions about clearances, fit, polarity, and serviceability
- propose breadboard layouts
- generate mechanical constraints from a selected enclosure
- place labels and callouts for documentation
- prepare manufacturing and assembly review packets

## First Milestone

Create a desktop OpenGL 3D assembly viewer that renders:

- board rectangle
- mounting holes
- simple component bodies
- labels/refdes
- highlighted selected component from schematic/PCB data

Once this works, add breadboard primitives and VR camera/input support.
