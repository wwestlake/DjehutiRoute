#pragma once

// Exact board geometry in integer nanometres, and the distance functions the
// router and the design-rule check share.

#include <cstdint>
#include <vector>

namespace djehuti::route
{
using Coord = std::int64_t; // nanometres

constexpr Coord nmPerMm = 1'000'000;
constexpr Coord mm(double millimetres) { return (Coord)(millimetres * (double)nmPerMm + (millimetres >= 0 ? 0.5 : -0.5)); }
constexpr double toMm(Coord nm) { return (double)nm / (double)nmPerMm; }

struct Point
{
    Coord x = 0, y = 0;
    bool operator==(const Point& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Point& o) const { return !(*this == o); }
};

struct Rect
{
    Coord minX = 0, minY = 0, maxX = 0, maxY = 0;
    bool contains(Point p) const { return p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY; }
    Rect expanded(Coord by) const { return { minX - by, minY - by, maxX + by, maxY + by }; }
    bool intersects(const Rect& o) const { return minX <= o.maxX && o.minX <= maxX && minY <= o.maxY && o.minY <= maxY; }
};

using Polygon = std::vector<Point>; // closed, either winding

double distance(Point a, Point b);
// Distance from p to segment ab.
double pointSegmentDistance(Point p, Point a, Point b);
// Distance between segments ab and cd (0 if they cross).
double segmentSegmentDistance(Point a, Point b, Point c, Point d);
bool polygonContains(const Polygon& polygon, Point p);
// Distance from a segment to a polygon's boundary, 0 if the segment enters it.
double segmentPolygonDistance(Point a, Point b, const Polygon& polygon);
Rect boundsOf(const Polygon& polygon);
Rect segmentBounds(Point a, Point b);

// A pad or via shape. Circles are ovals with equal size; an oval is a capsule
// along its longer axis; rectangles are axis-aligned (rotated pads are given
// as their rotated size for 0/90/180/270).
struct Shape
{
    enum class Kind { Circle, Rectangle, Oval };
    Kind kind = Kind::Circle;
    Point centre;
    Coord sizeX = 0, sizeY = 0;

    Rect bounds() const;
    bool contains(Point p) const;
    // Distance from the shape's copper to a point / a segment (0 when touching or inside).
    double distanceTo(Point p) const;
    double distanceTo(Point a, Point b) const;
    double distanceTo(const Shape& other) const;
};
}
