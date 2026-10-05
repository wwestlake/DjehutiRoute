#include "djehuti_route/geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace djehuti::route
{
namespace
{
double cross(double ax, double ay, double bx, double by) { return ax * by - ay * bx; }

int orientation(Point a, Point b, Point c)
{
    const auto v = cross((double)(b.x - a.x), (double)(b.y - a.y), (double)(c.x - a.x), (double)(c.y - a.y));
    return v > 0 ? 1 : v < 0 ? -1 : 0;
}

bool onSegment(Point a, Point b, Point p)
{
    return std::min(a.x, b.x) <= p.x && p.x <= std::max(a.x, b.x) && std::min(a.y, b.y) <= p.y && p.y <= std::max(a.y, b.y);
}

bool segmentsIntersect(Point a, Point b, Point c, Point d)
{
    const auto o1 = orientation(a, b, c), o2 = orientation(a, b, d), o3 = orientation(c, d, a), o4 = orientation(c, d, b);
    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && onSegment(a, b, c)) return true;
    if (o2 == 0 && onSegment(a, b, d)) return true;
    if (o3 == 0 && onSegment(c, d, a)) return true;
    if (o4 == 0 && onSegment(c, d, b)) return true;
    return false;
}

// The capsule axis of an oval (a circle has a zero-length axis).
void ovalAxis(const Shape& s, Point& a, Point& b, double& radius)
{
    if (s.sizeX >= s.sizeY)
    {
        radius = s.sizeY / 2.0;
        const auto half = (s.sizeX - s.sizeY) / 2;
        a = { s.centre.x - half, s.centre.y };
        b = { s.centre.x + half, s.centre.y };
    }
    else
    {
        radius = s.sizeX / 2.0;
        const auto half = (s.sizeY - s.sizeX) / 2;
        a = { s.centre.x, s.centre.y - half };
        b = { s.centre.x, s.centre.y + half };
    }
}

Polygon rectPolygon(const Shape& s)
{
    const auto hx = s.sizeX / 2, hy = s.sizeY / 2;
    return { { s.centre.x - hx, s.centre.y - hy }, { s.centre.x + hx, s.centre.y - hy },
             { s.centre.x + hx, s.centre.y + hy }, { s.centre.x - hx, s.centre.y + hy } };
}
}

double distance(Point a, Point b)
{
    return std::hypot((double)(a.x - b.x), (double)(a.y - b.y));
}

double pointSegmentDistance(Point p, Point a, Point b)
{
    const double dx = (double)(b.x - a.x), dy = (double)(b.y - a.y);
    const double len2 = dx * dx + dy * dy;
    if (len2 == 0.0) return distance(p, a);
    auto t = ((double)(p.x - a.x) * dx + (double)(p.y - a.y) * dy) / len2;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot((double)p.x - ((double)a.x + t * dx), (double)p.y - ((double)a.y + t * dy));
}

double segmentSegmentDistance(Point a, Point b, Point c, Point d)
{
    if (segmentsIntersect(a, b, c, d)) return 0.0;
    return std::min({ pointSegmentDistance(a, c, d), pointSegmentDistance(b, c, d),
                      pointSegmentDistance(c, a, b), pointSegmentDistance(d, a, b) });
}

bool polygonContains(const Polygon& polygon, Point p)
{
    bool inside = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
    {
        const auto& a = polygon[i];
        const auto& b = polygon[j];
        if ((a.y > p.y) != (b.y > p.y))
        {
            const double x = (double)a.x + (double)(p.y - a.y) * (double)(b.x - a.x) / (double)(b.y - a.y);
            if ((double)p.x < x) inside = !inside;
        }
    }
    return inside;
}

double segmentPolygonDistance(Point a, Point b, const Polygon& polygon)
{
    if (polygon.size() < 3) return std::numeric_limits<double>::infinity();
    if (polygonContains(polygon, a) || polygonContains(polygon, b)) return 0.0;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
        best = std::min(best, segmentSegmentDistance(a, b, polygon[j], polygon[i]));
    return best;
}

Rect boundsOf(const Polygon& polygon)
{
    Rect r { std::numeric_limits<Coord>::max(), std::numeric_limits<Coord>::max(), std::numeric_limits<Coord>::min(), std::numeric_limits<Coord>::min() };
    for (const auto& p : polygon)
    {
        r.minX = std::min(r.minX, p.x); r.minY = std::min(r.minY, p.y);
        r.maxX = std::max(r.maxX, p.x); r.maxY = std::max(r.maxY, p.y);
    }
    return r;
}

Rect segmentBounds(Point a, Point b)
{
    return { std::min(a.x, b.x), std::min(a.y, b.y), std::max(a.x, b.x), std::max(a.y, b.y) };
}

Rect Shape::bounds() const
{
    return { centre.x - sizeX / 2, centre.y - sizeY / 2, centre.x + sizeX / 2, centre.y + sizeY / 2 };
}

bool Shape::contains(Point p) const
{
    return distanceTo(p) == 0.0;
}

double Shape::distanceTo(Point p) const
{
    switch (kind)
    {
        case Kind::Rectangle:
        {
            const double dx = std::max(0.0, std::abs((double)(p.x - centre.x)) - sizeX / 2.0);
            const double dy = std::max(0.0, std::abs((double)(p.y - centre.y)) - sizeY / 2.0);
            return std::hypot(dx, dy);
        }
        default:
        {
            Point a, b;
            double r = 0.0;
            ovalAxis(*this, a, b, r);
            return std::max(0.0, pointSegmentDistance(p, a, b) - r);
        }
    }
}

double Shape::distanceTo(Point a, Point b) const
{
    switch (kind)
    {
        case Kind::Rectangle:
        {
            const auto poly = rectPolygon(*this);
            return segmentPolygonDistance(a, b, poly);
        }
        default:
        {
            Point p, q;
            double r = 0.0;
            ovalAxis(*this, p, q, r);
            return std::max(0.0, segmentSegmentDistance(a, b, p, q) - r);
        }
    }
}

double Shape::distanceTo(const Shape& other) const
{
    if (other.kind != Kind::Rectangle)
    {
        Point p, q;
        double r = 0.0;
        ovalAxis(other, p, q, r);
        return std::max(0.0, distanceTo(p, q) - r);
    }
    if (kind != Kind::Rectangle)
        return other.distanceTo(*this);
    const double dx = std::max(0.0, std::abs((double)(other.centre.x - centre.x)) - (sizeX + other.sizeX) / 2.0);
    const double dy = std::max(0.0, std::abs((double)(other.centre.y - centre.y)) - (sizeY + other.sizeY) / 2.0);
    return std::hypot(dx, dy);
}
}
