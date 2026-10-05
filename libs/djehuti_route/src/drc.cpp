#include "djehuti_route/drc.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

namespace djehuti::route
{
namespace
{
// One piece of copper: a track segment (capsule), a via (disc on its layers) or a pad.
struct Item
{
    enum class Type { Segment, Via, Pad };
    Type type;
    int net;
    int firstLayer, lastLayer;
    Point a, b;          // segment ends; via centre in a
    double radius = 0;   // segment half width / via radius
    int pad = -1;
    Rect box;

    bool onLayer(int l) const { return l >= firstLayer && l <= lastLayer; }
};

// Distance between the copper of two items (0 if they touch or overlap).
double copperDistance(const Board& board, const Item& x, const Item& y)
{
    auto asShape = [&](const Item& i) { return board.pads[(size_t)i.pad].shape; };
    if (x.type == Item::Type::Pad && y.type == Item::Type::Pad)
        return asShape(x).distanceTo(asShape(y));
    if (x.type == Item::Type::Pad)
        return std::max(0.0, asShape(x).distanceTo(y.a, y.b) - y.radius);
    if (y.type == Item::Type::Pad)
        return std::max(0.0, asShape(y).distanceTo(x.a, x.b) - x.radius);
    return std::max(0.0, segmentSegmentDistance(x.a, x.b, y.a, y.b) - x.radius - y.radius);
}

std::string netName(const Board& board, int net)
{
    return net >= 0 ? board.nets[(size_t)net].name : std::string("(no net)");
}
}

const char* kindName(Violation::Kind kind)
{
    switch (kind)
    {
        case Violation::Kind::Clearance: return "clearance";
        case Violation::Kind::Short: return "short";
        case Violation::Kind::Edge: return "edge";
        case Violation::Kind::Keepout: return "keepout";
        case Violation::Kind::Open: return "open";
    }
    return "";
}

std::vector<Violation> checkDesignRules(const Board& board, const RouteResult& result)
{
    std::vector<Violation> violations;
    std::vector<Item> items;
    for (const auto& t : result.tracks)
        for (size_t i = 1; i < t.points.size(); ++i)
        {
            Item it { Item::Type::Segment, t.net, t.layer, t.layer, t.points[i - 1], t.points[i], t.width / 2.0 };
            it.box = segmentBounds(it.a, it.b).expanded((Coord)std::ceil(it.radius));
            items.push_back(it);
        }
    for (const auto& v : result.vias)
    {
        Item it { Item::Type::Via, v.net, v.firstLayer, v.lastLayer, v.at, v.at, v.diameter / 2.0 };
        it.box = segmentBounds(v.at, v.at).expanded((Coord)std::ceil(it.radius));
        items.push_back(it);
    }
    const auto firstPadItem = items.size();
    for (int p = 0; p < (int)board.pads.size(); ++p)
    {
        const auto& pad = board.pads[(size_t)p];
        Item it { Item::Type::Pad, pad.net, pad.firstLayer, pad.lastLayer, pad.shape.centre, pad.shape.centre, 0.0 };
        it.pad = p;
        it.box = pad.shape.bounds();
        items.push_back(it);
    }

    Coord maxClearance = 0;
    for (const auto& c : board.netClasses) maxClearance = std::max(maxClearance, c.clearance);

    // Clearance and shorts between copper of different nets on a shared layer.
    for (size_t i = 0; i < items.size(); ++i)
        for (size_t j = i + 1; j < items.size(); ++j)
        {
            const auto& x = items[i];
            const auto& y = items[j];
            if (x.net == y.net && x.net >= 0) continue;
            if (i >= firstPadItem && j >= firstPadItem) continue; // pad-to-pad spacing is the footprint's, not the router's
            const int shared = std::max(x.firstLayer, y.firstLayer);
            if (shared > std::min(x.lastLayer, y.lastLayer)) continue;
            if (!x.box.expanded(maxClearance).intersects(y.box)) continue;
            const double required = (double)board.clearance(x.net, y.net);
            const double d = copperDistance(board, x, y);
            if (d + 1.0 < required) // 1 nm of rounding slack
            {
                Violation v;
                v.kind = d <= 0.0 ? Violation::Kind::Short : Violation::Kind::Clearance;
                v.netA = x.net; v.netB = y.net; v.layer = shared; v.at = x.a;
                v.actualMm = d / (double)nmPerMm; v.requiredMm = required / (double)nmPerMm;
                v.message = std::string(kindName(v.kind)) + " between " + netName(board, x.net) + " and " + netName(board, y.net)
                          + " on " + board.layers[(size_t)shared] + ": " + std::to_string(v.actualMm) + " mm, need " + std::to_string(v.requiredMm) + " mm";
                violations.push_back(v);
            }
        }

    // Board edge and keepouts (routed copper only).
    for (size_t i = 0; i < firstPadItem; ++i)
    {
        const auto& it = items[i];
        double edge = std::numeric_limits<double>::infinity();
        for (size_t a = 0, b = board.outline.size() - 1; a < board.outline.size(); b = a++)
            edge = std::min(edge, segmentSegmentDistance(it.a, it.b, board.outline[b], board.outline[a]));
        const bool inside = polygonContains(board.outline, it.a) && polygonContains(board.outline, it.b);
        if (!inside || edge - it.radius + 1.0 < (double)board.edgeClearance)
        {
            Violation v;
            v.kind = Violation::Kind::Edge;
            v.netA = it.net; v.layer = it.firstLayer; v.at = it.a;
            v.actualMm = (edge - it.radius) / (double)nmPerMm; v.requiredMm = toMm(board.edgeClearance);
            v.message = "copper of " + netName(board, it.net) + " too close to the board edge";
            violations.push_back(v);
        }
        for (const auto& k : board.keepouts)
        {
            bool shares = false;
            for (int l = it.firstLayer; l <= it.lastLayer; ++l) if (k.onLayer(l)) shares = true;
            if (!shares) continue;
            if (segmentPolygonDistance(it.a, it.b, k.area) < it.radius)
            {
                Violation v;
                v.kind = Violation::Kind::Keepout;
                v.netA = it.net; v.layer = it.firstLayer; v.at = it.a;
                v.message = "copper of " + netName(board, it.net) + " inside a keepout";
                violations.push_back(v);
            }
        }
    }

    // Connectivity: every pad of a routed net joined through touching copper of that net.
    std::vector<int> parent(items.size());
    std::iota(parent.begin(), parent.end(), 0);
    std::function<int(int)> find = [&](int a) { return parent[(size_t)a] == a ? a : parent[(size_t)a] = find(parent[(size_t)a]); };
    for (size_t i = 0; i < items.size(); ++i)
        for (size_t j = i + 1; j < items.size(); ++j)
        {
            const auto& x = items[i];
            const auto& y = items[j];
            if (x.net != y.net || x.net < 0) continue;
            bool shared = false;
            for (int l = std::max(x.firstLayer, y.firstLayer); l <= std::min(x.lastLayer, y.lastLayer); ++l) shared = true;
            if (!shared || !x.box.expanded(2).intersects(y.box)) continue;
            if (copperDistance(board, x, y) <= 1.0)
                parent[(size_t)find((int)i)] = find((int)j);
        }
    std::vector<char> netHasUnrouted(board.nets.size(), 0);
    for (const auto& u : result.unrouted) if (u.net >= 0) netHasUnrouted[(size_t)u.net] = 1;
    for (int net = 0; net < (int)board.nets.size(); ++net)
    {
        if (netHasUnrouted[(size_t)net]) continue; // reported as unrouted already
        int root = -1;
        for (size_t i = firstPadItem; i < items.size(); ++i)
        {
            if (items[i].net != net) continue;
            const int r = find((int)i);
            if (root < 0) root = r;
            else if (r != root)
            {
                Violation v;
                v.kind = Violation::Kind::Open;
                v.netA = net;
                v.at = items[i].a;
                v.message = "net " + netName(board, net) + " is not connected at pad " + board.pads[(size_t)items[i].pad].reference;
                violations.push_back(v);
                break;
            }
        }
    }
    return violations;
}
}
