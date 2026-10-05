#include "djehuti_route/outline.h"

#include <algorithm>
#include <cmath>

namespace djehuti::route
{
namespace
{
constexpr double pi = 3.14159265358979323846;

Point at(double xMm, double yMm) { return { mm(xMm), mm(yMm) }; }

bool properlyCross(Point a, Point b, Point c, Point d)
{
    auto orient = [](Point p, Point q, Point r) {
        const double v = (double)(q.x - p.x) * (double)(r.y - p.y) - (double)(q.y - p.y) * (double)(r.x - p.x);
        return v > 0 ? 1 : v < 0 ? -1 : 0;
    };
    const int o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
    if (o1 != o2 && o3 != o4 && o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0)
        return true;
    // Collinear overlap also counts as crossing.
    auto within = [](Point p, Point q, Point r) {
        return std::min(p.x, q.x) <= r.x && r.x <= std::max(p.x, q.x) && std::min(p.y, q.y) <= r.y && r.y <= std::max(p.y, q.y);
    };
    if (o1 == 0 && within(a, b, c) && c != a && c != b) return true;
    if (o2 == 0 && within(a, b, d) && d != a && d != b) return true;
    if (o3 == 0 && within(c, d, a) && a != c && a != d) return true;
    if (o4 == 0 && within(c, d, b) && b != c && b != d) return true;
    return false;
}

}

namespace outline
{
Polygon rectangle(double w, double h)
{
    return { at(0, 0), at(w, 0), at(w, h), at(0, h) };
}

Polygon roundedRectangle(double w, double h, double r, int segmentsPerCorner)
{
    r = std::clamp(r, 0.0, std::min(w, h) / 2.0);
    if (r <= 0.0) return rectangle(w, h);
    segmentsPerCorner = std::max(1, segmentsPerCorner);
    Polygon p;
    const double cx[4] = { w - r, w - r, r, r };
    const double cy[4] = { r, h - r, h - r, r };
    const double start[4] = { -pi / 2, 0.0, pi / 2, pi };
    for (int c = 0; c < 4; ++c)
        for (int i = 0; i <= segmentsPerCorner; ++i)
        {
            const double a = start[c] + (pi / 2) * i / segmentsPerCorner;
            p.push_back(at(cx[c] + r * std::cos(a), cy[c] + r * std::sin(a)));
        }
    return p;
}

Polygon chamferedRectangle(double w, double h, double c)
{
    c = std::clamp(c, 0.0, std::min(w, h) / 2.0);
    if (c <= 0.0) return rectangle(w, h);
    return { at(c, 0), at(w - c, 0), at(w, c), at(w, h - c), at(w - c, h), at(c, h), at(0, h - c), at(0, c) };
}

Polygon lShape(double w, double h, double nw, double nh)
{
    nw = std::clamp(nw, 0.0, w);
    nh = std::clamp(nh, 0.0, h);
    return { at(0, 0), at(w, 0), at(w, h - nh), at(w - nw, h - nh), at(w - nw, h), at(0, h) };
}

Polygon uShape(double w, double h, double sw, double sd)
{
    sw = std::clamp(sw, 0.0, w);
    sd = std::clamp(sd, 0.0, h);
    const double l = (w - sw) / 2.0, r = (w + sw) / 2.0;
    return { at(0, 0), at(w, 0), at(w, h), at(r, h), at(r, h - sd), at(l, h - sd), at(l, h), at(0, h) };
}

Polygon tShape(double bw, double bh, double sw, double sh)
{
    sw = std::clamp(sw, 0.0, bw);
    const double l = (bw - sw) / 2.0, r = (bw + sw) / 2.0;
    return { at(l, 0), at(r, 0), at(r, sh), at(bw, sh), at(bw, sh + bh), at(0, sh + bh), at(0, sh), at(l, sh) };
}

Polygon circle(double d, int segments)
{
    segments = std::max(8, segments);
    Polygon p;
    for (int i = 0; i < segments; ++i)
    {
        const double a = 2.0 * pi * i / segments;
        p.push_back(at(d / 2.0 + d / 2.0 * std::cos(a), d / 2.0 + d / 2.0 * std::sin(a)));
    }
    return p;
}

Polygon regularPolygon(int sides, double cornerDiameter)
{
    sides = std::max(3, sides);
    const double r = cornerDiameter / 2.0;
    // Bounding box starts at (0, 0).
    double minX = 1e300, minY = 1e300;
    std::vector<std::pair<double, double>> pts;
    for (int i = 0; i < sides; ++i)
    {
        const double a = 2.0 * pi * i / sides;
        pts.push_back({ r * std::cos(a), r * std::sin(a) });
        minX = std::min(minX, pts.back().first);
        minY = std::min(minY, pts.back().second);
    }
    Polygon p;
    for (const auto& [x, y] : pts) p.push_back(at(x - minX, y - minY));
    return p;
}

Polygon regularPolygonAcrossFlats(int sides, double acrossFlats)
{
    sides = std::max(3, sides);
    // Corner circle radius from the inscribed radius: R = r / cos(pi / n). For odd n the
    // "across flats" distance is corner-to-flat; we size by the inscribed diameter throughout.
    return regularPolygon(sides, acrossFlats / std::cos(pi / sides));
}

double areaMm2(const Polygon& poly)
{
    double twice = 0.0;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
        twice += toMm(poly[j].x) * toMm(poly[i].y) - toMm(poly[i].x) * toMm(poly[j].y);
    return std::abs(twice) / 2.0;
}

double perimeterMm(const Polygon& poly)
{
    double total = 0.0;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
        total += distance(poly[j], poly[i]);
    return total / (double)nmPerMm;
}

std::vector<std::string> validate(const Polygon& poly)
{
    std::vector<std::string> problems;
    if (poly.size() < 3)
    {
        problems.push_back("An outline needs at least 3 corners.");
        return problems;
    }
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
        if (poly[i] == poly[j])
        {
            problems.push_back("Corner " + std::to_string(i + 1) + " repeats the corner before it.");
            return problems;
        }
    // Crossing edges first: a self-crossing outline's signed area can cancel to zero.
    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i)
        for (size_t j = i + 1; j < n; ++j)
        {
            if (j == i + 1 || (i == 0 && j == n - 1)) continue; // neighbours share a corner
            if (properlyCross(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n]))
            {
                problems.push_back("Edges " + std::to_string(i + 1) + " and " + std::to_string(j + 1) + " cross; an outline must not cross itself.");
                return problems;
            }
        }
    if (areaMm2(poly) < 1e-6)
        problems.push_back("The outline has no area (its corners are in a line).");
    return problems;
}

std::vector<std::string> validateCutouts(const Polygon& board, const std::vector<Polygon>& cutouts)
{
    std::vector<std::string> problems;
    for (size_t c = 0; c < cutouts.size(); ++c)
    {
        const auto label = "Cutout " + std::to_string(c + 1);
        for (const auto& p : validate(cutouts[c])) problems.push_back(label + ": " + p);
        bool inside = true;
        for (const auto& p : cutouts[c]) if (!polygonContains(board, p)) inside = false;
        if (!inside) { problems.push_back(label + " is not entirely inside the board."); continue; }
        bool touches = false;
        for (size_t i = 0, j = cutouts[c].size() - 1; i < cutouts[c].size() && !touches; j = i++)
            for (size_t k = 0, l = board.size() - 1; k < board.size() && !touches; l = k++)
                if (segmentSegmentDistance(cutouts[c][j], cutouts[c][i], board[l], board[k]) == 0.0) touches = true;
        if (touches)
            problems.push_back(label + " touches the board edge.");
        for (size_t d = c + 1; d < cutouts.size(); ++d)
        {
            bool overlap = false;
            for (size_t i = 0, j = cutouts[c].size() - 1; i < cutouts[c].size() && !overlap; j = i++)
                for (size_t k = 0, l = cutouts[d].size() - 1; k < cutouts[d].size() && !overlap; l = k++)
                    if (segmentSegmentDistance(cutouts[c][j], cutouts[c][i], cutouts[d][l], cutouts[d][k]) == 0.0) overlap = true;
            if (overlap || polygonContains(cutouts[d], cutouts[c][0]) || polygonContains(cutouts[c], cutouts[d][0]))
                problems.push_back(label + " overlaps cutout " + std::to_string(d + 1) + ".");
        }
    }
    return problems;
}
}

namespace
{
BoardHole hole(double xMm, double yMm, double diameterMm) { return { at(xMm, yMm), mm(diameterMm) }; }

std::vector<StandardBoard> buildStandardBoards()
{
    std::vector<StandardBoard> boards;
    boards.push_back({ "eurocard", "Eurocard 100 x 160 mm",
                       "IEC 60297-3 single Eurocard, 160 mm deep by 100 mm high. Card guides, no mounting holes.",
                       outline::rectangle(160.0, 100.0), {} });
    boards.push_back({ "half-eurocard", "Half Eurocard 100 x 80 mm",
                       "Half of a single Eurocard, 80 x 100 mm.",
                       outline::rectangle(80.0, 100.0), {} });
    boards.push_back({ "double-eurocard", "Double Eurocard 233.35 x 160 mm",
                       "IEC 60297-3 double-height Eurocard, 160 mm deep by 233.35 mm high.",
                       outline::rectangle(160.0, 233.35), {} });
    // Raspberry Pi HAT mechanical specification: 65 x 56.5 mm, 3 mm corner radii,
    // four 2.75 mm holes for M2.5 on a 58 x 49 mm pattern, 3.5 mm in from the edges.
    boards.push_back({ "rpi-hat", "Raspberry Pi HAT 65 x 56.5 mm",
                       "Raspberry Pi HAT: 65 x 56.5 mm, 3 mm corner radii, four 2.75 mm M2.5 holes on a 58 x 49 mm pattern 3.5 mm from the edges.",
                       outline::roundedRectangle(65.0, 56.5, 3.0),
                       { hole(3.5, 3.5, 2.75), hole(61.5, 3.5, 2.75), hole(3.5, 52.5, 2.75), hole(61.5, 52.5, 2.75) } });
    // Arduino Uno: 2.7 x 2.1 in board; holes at (0.55, 0.1), (0.6, 2.0), (2.6, 0.3), (2.6, 1.4) in.
    // The Uno's angled edge is left out: this is the rectangular envelope.
    boards.push_back({ "arduino-uno-shield", "Arduino Uno shield 68.58 x 53.34 mm",
                       "Arduino Uno footprint (2.7 x 2.1 in) as a rectangle, with the Uno's four 3.2 mm holes at (0.55, 0.1), (0.6, 2.0), (2.6, 0.3), (2.6, 1.4) in. The Uno's angled edge is not drawn.",
                       outline::rectangle(68.58, 53.34),
                       { hole(13.97, 2.54, 3.2), hole(15.24, 50.8, 3.2), hole(66.04, 7.62, 3.2), hole(66.04, 35.56, 3.2) } });
    // ISO/IEC 7810 ID-1: 85.60 x 53.98 mm, corner radius 3.18 mm.
    boards.push_back({ "credit-card", "Credit card 85.6 x 53.98 mm",
                       "ISO/IEC 7810 ID-1 card size: 85.60 x 53.98 mm with 3.18 mm corner radii.",
                       outline::roundedRectangle(85.60, 53.98, 3.18), {} });
    boards.push_back({ "fab-100", "100 x 100 mm",
                       "100 x 100 mm, the common price break at low-cost board houses.",
                       outline::rectangle(100.0, 100.0), {} });
    boards.push_back({ "fab-50", "50 x 50 mm",
                       "50 x 50 mm square.",
                       outline::rectangle(50.0, 50.0), {} });
    return boards;
}
}

const std::vector<StandardBoard>& standardBoards()
{
    static const auto boards = buildStandardBoards();
    return boards;
}

const StandardBoard* findStandardBoard(const std::string& id)
{
    for (const auto& b : standardBoards())
        if (b.id == id) return &b;
    return nullptr;
}
}
