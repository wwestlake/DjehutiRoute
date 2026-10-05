#pragma once

// Board outlines: parametric shapes, standard board sizes, and validation.
// Outlines are straight-edged polygons. Arcs (rounded corners, circular
// boards) become short chords with their corners on the true curve, so the
// polygon lies just inside it: copper kept clear of the polygon is clear of
// the real edge too. Round holes are circumscribed instead (polygon outside
// the hole), for the same reason.

#include "djehuti_route/geometry.h"

#include <string>
#include <vector>

namespace djehuti::route
{
// A round hole through the board (mounting hole), centre and drill in nm.
struct BoardHole
{
    Point centre;
    Coord diameter = 0;
};

namespace outline
{
// All take millimetres; the board's lower-left corner is at (0, 0) and y grows upward.
Polygon rectangle(double widthMm, double heightMm);
Polygon roundedRectangle(double widthMm, double heightMm, double radiusMm, int segmentsPerCorner = 8);
Polygon chamferedRectangle(double widthMm, double heightMm, double chamferMm);
// An L: the full rectangle with the top-right corner notch (notchWidth x notchHeight) removed.
Polygon lShape(double widthMm, double heightMm, double notchWidthMm, double notchHeightMm);
// A U: the full rectangle with a slot (slotWidth wide, slotDepth deep) cut down from the middle of the top edge.
Polygon uShape(double widthMm, double heightMm, double slotWidthMm, double slotDepthMm);
// A T: a bar across the top (barWidth x barHeight) on a stem centred under it (stemWidth x stemHeight).
Polygon tShape(double barWidthMm, double barHeightMm, double stemWidthMm, double stemHeightMm);
// A circle of `segments` sides (corners on the circle).
Polygon circle(double diameterMm, int segments = 64);
// A regular polygon with `sides` corners on a circle of the given diameter; the
// first corner points along +x. A hexagon with flat top and bottom: sides = 6.
Polygon regularPolygon(int sides, double cornerDiameterMm);
// The same, sized by the distance across flats (wrench size) instead.
Polygon regularPolygonAcrossFlats(int sides, double acrossFlatsMm);

double areaMm2(const Polygon& polygon);
double perimeterMm(const Polygon& polygon);

// Why an outline cannot be a board: fewer than 3 corners, repeated corners,
// zero area, or edges that cross. Empty means it is fine.
std::vector<std::string> validate(const Polygon& polygon);
// A cutout must lie inside the outline without touching it, and not cross another cutout.
std::vector<std::string> validateCutouts(const Polygon& outline, const std::vector<Polygon>& cutouts);
}

// A standard board size: outline plus its mounting holes.
struct StandardBoard
{
    std::string id;          // "rpi-hat"
    std::string name;        // "Raspberry Pi HAT"
    std::string description; // size, corners, holes and where it comes from
    Polygon outline;
    std::vector<BoardHole> holes;
};

const std::vector<StandardBoard>& standardBoards();
const StandardBoard* findStandardBoard(const std::string& id);
}
