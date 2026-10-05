#include "djehuti_route/board.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace djehuti::route
{
Coord Board::traceWidth(int net) const
{
    if (net < 0)
        return netClasses[0].traceWidth;
    const auto& n = nets[(size_t)net];
    return n.widthOverride > 0 ? n.widthOverride : classOf(net).traceWidth;
}

Coord Board::clearance(int netA, int netB) const
{
    return std::max(clearance(netA), clearance(netB));
}

bool Board::contains(Point p) const
{
    if (outline.size() < 3 || !polygonContains(outline, p))
        return false;
    for (const auto& c : cutouts)
        if (c.size() >= 3 && polygonContains(c, p))
            return false;
    return true;
}

double Board::edgeDistance(Point p) const
{
    double best = std::numeric_limits<double>::infinity();
    auto edges = [&](const Polygon& poly) {
        for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
            best = std::min(best, pointSegmentDistance(p, poly[j], poly[i]));
    };
    if (outline.size() >= 2) edges(outline);
    for (const auto& c : cutouts)
        if (c.size() >= 2) edges(c);
    return best;
}

double Board::edgeDistance(Point a, Point b) const
{
    double best = std::numeric_limits<double>::infinity();
    auto edges = [&](const Polygon& poly) {
        for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
            best = std::min(best, segmentSegmentDistance(a, b, poly[j], poly[i]));
    };
    if (outline.size() >= 2) edges(outline);
    for (const auto& c : cutouts)
        if (c.size() >= 2) edges(c);
    return best;
}

void Board::addHole(double xMm, double yMm, double diameterMm, int segments)
{
    segments = std::max(6, segments);
    // Circumscribed polygon: its edges are tangent to the hole, so the hole is inside it.
    const double r = diameterMm / 2.0 / std::cos(3.14159265358979323846 / segments);
    Polygon hole;
    for (int i = 0; i < segments; ++i)
    {
        const double a = 2.0 * 3.14159265358979323846 * i / segments;
        hole.push_back({ mm(xMm + r * std::cos(a)), mm(yMm + r * std::sin(a)) });
    }
    cutouts.push_back(hole);
}

int Board::addNet(const std::string& name, int netClass)
{
    nets.push_back({ name, netClass, 0 });
    return (int)nets.size() - 1;
}

int Board::addRectBoard(double widthMm, double heightMm)
{
    outline = { { 0, 0 }, { mm(widthMm), 0 }, { mm(widthMm), mm(heightMm) }, { 0, mm(heightMm) } };
    return 0;
}

void Board::addSmdPad(const std::string& reference, double xMm, double yMm, double wMm, double hMm, int net, int layer)
{
    Pad pad;
    pad.reference = reference;
    pad.shape = { Shape::Kind::Rectangle, { mm(xMm), mm(yMm) }, mm(wMm), mm(hMm) };
    pad.net = net;
    pad.firstLayer = pad.lastLayer = layer;
    pads.push_back(pad);
}

void Board::addRoundPad(const std::string& reference, double xMm, double yMm, double diameterMm, double drillMm, int net)
{
    Pad pad;
    pad.reference = reference;
    pad.shape = { Shape::Kind::Circle, { mm(xMm), mm(yMm) }, mm(diameterMm), mm(diameterMm) };
    pad.net = net;
    pad.firstLayer = 0;
    pad.lastLayer = layerCount() - 1;
    pad.drill = mm(drillMm);
    pads.push_back(pad);
}

void Board::addKeepoutRect(double x0Mm, double y0Mm, double x1Mm, double y1Mm, int firstLayer, int lastLayer)
{
    Keepout k;
    k.area = { { mm(x0Mm), mm(y0Mm) }, { mm(x1Mm), mm(y0Mm) }, { mm(x1Mm), mm(y1Mm) }, { mm(x0Mm), mm(y1Mm) } };
    k.firstLayer = firstLayer;
    k.lastLayer = lastLayer < 0 ? layerCount() - 1 : lastLayer;
    keepouts.push_back(k);
}

Coord ipc2221TraceWidth(double currentAmps, double temperatureRiseC, double copperOz, bool outerLayer)
{
    const double k = outerLayer ? 0.048 : 0.024;
    const double areaSqMil = std::pow(currentAmps / (k * std::pow(temperatureRiseC, 0.44)), 1.0 / 0.725);
    const double thicknessMil = 1.378 * copperOz;
    const double widthMil = areaSqMil / thicknessMil;
    return mm(widthMil * 0.0254);
}
}
