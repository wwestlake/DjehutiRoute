#include "djehuti_route/router.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <tuple>
#include <unordered_map>

namespace djehuti::route
{
namespace
{
// Directions in the plane: E, NE, N, NW, W, SW, S, SE. Index 8 = none (start, after a via).
constexpr int dirDX[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
constexpr int dirDY[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };
constexpr int noDirection = 8;
constexpr int states = 9;
constexpr double sqrt2 = 1.4142135623730951;

constexpr int freeCell = -1;   // static map values: >= 0 means "only this net may use it"
constexpr int hardCell = -2;
constexpr int blockedAll = -3;

struct ObstacleRef
{
    enum class Type : std::uint8_t { Pad, Keepout, Edge };
    Type type;
    int index;
};

struct Grid
{
    int w = 0, h = 0, layers = 0;
    Coord pitch = 0;
    Coord x0 = 0, y0 = 0;

    int cells() const { return w * h * layers; }
    int xy(int x, int y) const { return y * w + x; }
    int index(int x, int y, int layer) const { return (layer * h + y) * w + x; }
    int layerOf(int cell) const { return cell / (w * h); }
    int xOf(int cell) const { return cell % w; }
    int yOf(int cell) const { return (cell / w) % h; }
    Point centre(int x, int y) const { return { x0 + (Coord)x * pitch, y0 + (Coord)y * pitch }; }
    Point centreOf(int cell) const { return centre(xOf(cell), yOf(cell)); }
    // Cell range whose centres lie in [lo, hi].
    void span(Coord lo, Coord hi, Coord origin, int count, int& from, int& to) const
    {
        from = std::max(0, (int)std::ceil((double)(lo - origin) / (double)pitch));
        to = std::min(count - 1, (int)std::floor((double)(hi - origin) / (double)pitch));
    }
};

// Static obstacles for one combination of trace width / clearance / via size.
struct StaticMap
{
    Coord width = 0, clearance = 0, viaDiameter = 0;
    std::vector<int> block;                                   // per layer cell
    std::vector<int> viaBlock;                                // per xy (through via, every layer)
    std::unordered_map<int, std::vector<ObstacleRef>> near;   // layer cell -> obstacles to check exactly on moves
};

void mark(int& slot, int net)
{
    if (slot == hardCell || slot == blockedAll) return;
    if (net < 0) { slot = blockedAll; return; }
    if (slot == freeCell) slot = net;
    else if (slot != net) slot = blockedAll;
}

bool allowed(int slot, int net) { return slot == freeCell || slot == net; }

struct Terminal
{
    int cell = -1;
    bool stub = false; // the cell centre is outside the pad: join it to the pad centre
};

struct NetRoute
{
    std::vector<std::vector<int>> paths;   // layer cells, source to target
    std::vector<int> footprint;            // cells claimed (unique)
    std::vector<int> failedPads;
    std::map<int, std::pair<int, Terminal>> terminalAt; // cell -> (pad, terminal)
};

class Router
{
public:
    Router(const Board& b, const RouterOptions& o) : board(b), options(o) {}

    RouteResult run()
    {
        RouteResult result;
        const auto started = std::chrono::steady_clock::now();
        if (!setUpGrid(result.error))
            return result;
        result.pitch = grid.pitch;
        result.gridWidth = grid.w;
        result.gridHeight = grid.h;

        usage.assign((size_t)grid.cells(), 0);
        history.assign((size_t)grid.cells(), 0.0f);
        claimStamp.assign((size_t)grid.cells(), 0);
        routes.assign(board.nets.size(), {});

        std::vector<int> order;
        for (int n = 0; n < (int)board.nets.size(); ++n)
        {
            const auto pads = padsOf(n);
            if (pads.size() >= 2)
            {
                order.push_back(n);
                result.connections += (int)pads.size() - 1;
            }
        }
        // Short nets first in the first pass.
        std::sort(order.begin(), order.end(), [&](int a, int b) { return spanOf(a) < spanOf(b); });

        presentFactor = options.presentFactor;
        int pass = 0;
        for (pass = 1; pass <= options.maxIterations; ++pass)
        {
            for (int net : order)
                if (pass == 1 || sharesCells(net))
                {
                    ripUp(net);
                    routeNet(net);
                    commit(net);
                }
            int overused = 0;
            for (size_t c = 0; c < usage.size(); ++c)
                if (usage[c] > 1)
                {
                    ++overused;
                    history[c] += (float)options.historyIncrement;
                }
            if (overused == 0)
                break;
            presentFactor *= options.presentGrowth;
        }
        result.iterations = std::min(pass, options.maxIterations);

        // Still shared after the last pass: give up the most-conflicted nets until none is.
        std::vector<char> givenUp(board.nets.size(), 0);
        for (;;)
        {
            int worst = -1, worstCount = 0;
            for (int net : order)
            {
                int count = 0;
                for (int c : routes[(size_t)net].footprint)
                    if (usage[(size_t)c] > 1) ++count;
                if (count > worstCount) { worstCount = count; worst = net; }
            }
            if (worst < 0) break;
            ripUp(worst);
            givenUp[(size_t)worst] = 1;
        }

        for (int net : order)
        {
            const auto& r = routes[(size_t)net];
            if (givenUp[(size_t)net])
            {
                const auto pads = padsOf(net);
                for (size_t i = 1; i < pads.size(); ++i)
                    result.unrouted.push_back({ net, board.pads[(size_t)pads[i]].reference, "no room: the net could not get space of its own (congestion)" });
                continue;
            }
            for (int pad : r.failedPads)
                result.unrouted.push_back({ net, board.pads[(size_t)pad].reference, failureReason[pad] });
            emit(net, result);
        }
        result.routedConnections = result.connections - (int)result.unrouted.size();
        result.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        return result;
    }

private:
    const Board& board;
    const RouterOptions& options;
    Grid grid;
    std::vector<double> edgeDist;   // per xy, distance from the cell centre to the nearest board edge (outline or cutout)
    std::vector<char> insideBoard;  // per xy
    std::map<std::tuple<Coord, Coord, Coord>, StaticMap> staticMaps;
    std::vector<int> usage;
    std::vector<float> history;
    std::vector<std::uint32_t> claimStamp;
    std::uint32_t claimGeneration = 0;
    double presentFactor = 1.0;
    std::vector<NetRoute> routes;
    std::map<int, std::string> failureReason;

    // A* scratch, reused across searches.
    std::vector<float> gScore;
    std::vector<int> parent;
    std::vector<std::uint32_t> seen;
    std::vector<std::uint32_t> targetStamp;
    std::vector<int> targetPad;
    std::uint32_t generation = 0;

    bool setUpGrid(std::string& error)
    {
        if (board.outline.size() < 3) { error = "The board has no outline."; return false; }
        if (board.nets.empty()) { error = "The board has no nets."; return false; }
        const auto& def = board.netClasses[0];
        grid.pitch = options.pitch > 0 ? options.pitch : def.traceWidth + def.clearance;
        grid.layers = board.layerCount();
        const auto b = boundsOf(board.outline);
        grid.w = (int)((b.maxX - b.minX) / grid.pitch);
        grid.h = (int)((b.maxY - b.minY) / grid.pitch);
        if (grid.w < 2 || grid.h < 2) { error = "The board is smaller than two routing pitches."; return false; }
        if ((std::size_t)grid.w * (std::size_t)grid.h * (std::size_t)grid.layers > options.maxCells)
        {
            error = "The routing grid would have more than " + std::to_string(options.maxCells) + " cells; use a larger pitch.";
            return false;
        }
        // Centre the lattice on the board.
        grid.x0 = b.minX + ((b.maxX - b.minX) - (Coord)(grid.w - 1) * grid.pitch) / 2;
        grid.y0 = b.minY + ((b.maxY - b.minY) - (Coord)(grid.h - 1) * grid.pitch) / 2;

        edgeDist.assign((size_t)(grid.w * grid.h), 0.0);
        insideBoard.assign((size_t)(grid.w * grid.h), 0);
        for (int y = 0; y < grid.h; ++y)
            for (int x = 0; x < grid.w; ++x)
            {
                const auto c = grid.centre(x, y);
                insideBoard[(size_t)grid.xy(x, y)] = board.contains(c) ? 1 : 0;
                edgeDist[(size_t)grid.xy(x, y)] = board.edgeDistance(c);
            }

        const auto states9 = (size_t)grid.cells() * states;
        gScore.assign(states9, 0.0f);
        parent.assign(states9, -1);
        seen.assign(states9, 0);
        targetStamp.assign((size_t)grid.cells(), 0);
        targetPad.assign((size_t)grid.cells(), -1);
        return true;
    }

    std::vector<int> padsOf(int net) const
    {
        std::vector<int> pads;
        for (int i = 0; i < (int)board.pads.size(); ++i)
            if (board.pads[(size_t)i].net == net) pads.push_back(i);
        return pads;
    }

    Coord spanOf(int net) const
    {
        Rect r { std::numeric_limits<Coord>::max(), std::numeric_limits<Coord>::max(), std::numeric_limits<Coord>::min(), std::numeric_limits<Coord>::min() };
        for (int p : padsOf(net))
        {
            const auto c = board.pads[(size_t)p].shape.centre;
            r.minX = std::min(r.minX, c.x); r.minY = std::min(r.minY, c.y);
            r.maxX = std::max(r.maxX, c.x); r.maxY = std::max(r.maxY, c.y);
        }
        return (r.maxX - r.minX) + (r.maxY - r.minY);
    }

    // Radius, in cells, of the disc a net claims around its centreline: every cell
    // whose centre is closer than (halo + p/2), so that a default track (halo p/2)
    // in any unclaimed cell keeps the clearance. A default track claims only its
    // own cell (radius exactly 1, strict).
    double claimRadius(Coord width, Coord clearance) const
    {
        const double halo = width / 2.0 + clearance / 2.0;
        return (halo + grid.pitch / 2.0) / (double)grid.pitch;
    }

    StaticMap& staticMapFor(int net)
    {
        const auto width = board.traceWidth(net);
        const auto clearance = board.clearance(net);
        const auto viaDiameter = board.classOf(net).viaDiameter;
        const auto key = std::make_tuple(width, clearance, viaDiameter);
        auto found = staticMaps.find(key);
        if (found != staticMaps.end())
            return found->second;

        StaticMap m;
        m.width = width; m.clearance = clearance; m.viaDiameter = viaDiameter;
        m.block.assign((size_t)grid.cells(), freeCell);
        m.viaBlock.assign((size_t)(grid.w * grid.h), freeCell);
        const auto p = grid.pitch;
        const double halfWidth = width / 2.0;

        // Pads: no copper of another net within clearance.
        for (int i = 0; i < (int)board.pads.size(); ++i)
        {
            const auto& pad = board.pads[(size_t)i];
            const auto padClearance = std::max(clearance, pad.net >= 0 ? board.clearance(pad.net) : clearance);
            const double r = halfWidth + padClearance;
            const double rv = viaDiameter / 2.0 + padClearance;
            const auto bb = pad.shape.bounds().expanded((Coord)std::ceil(std::max(r, rv)) + p);
            int x0, x1, y0, y1;
            grid.span(bb.minX, bb.maxX, grid.x0, grid.w, x0, x1);
            grid.span(bb.minY, bb.maxY, grid.y0, grid.h, y0, y1);
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x)
                {
                    const double d = pad.shape.distanceTo(grid.centre(x, y));
                    if (d < rv) mark(m.viaBlock[(size_t)grid.xy(x, y)], pad.net);
                    for (int l = 0; l < grid.layers; ++l)
                    {
                        if (!pad.onLayer(l)) continue;
                        const auto cell = grid.index(x, y, l);
                        if (d < r) mark(m.block[(size_t)cell], pad.net);
                        else if (d < r + (double)p) m.near[cell].push_back({ ObstacleRef::Type::Pad, i });
                    }
                }
        }
        // Keepouts: no copper inside.
        for (int i = 0; i < (int)board.keepouts.size(); ++i)
        {
            const auto& k = board.keepouts[(size_t)i];
            const auto bb = boundsOf(k.area).expanded((Coord)std::ceil(std::max(halfWidth, viaDiameter / 2.0)) + p);
            int x0, x1, y0, y1;
            grid.span(bb.minX, bb.maxX, grid.x0, grid.w, x0, x1);
            grid.span(bb.minY, bb.maxY, grid.y0, grid.h, y0, y1);
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x)
                {
                    const auto c = grid.centre(x, y);
                    const double d = segmentPolygonDistance(c, c, k.area);
                    if (d < viaDiameter / 2.0) m.viaBlock[(size_t)grid.xy(x, y)] = hardCell;
                    for (int l = k.firstLayer; l <= k.lastLayer && l < grid.layers; ++l)
                    {
                        const auto cell = grid.index(x, y, l);
                        if (d < halfWidth) m.block[(size_t)cell] = hardCell;
                        else if (d < halfWidth + (double)p) m.near[cell].push_back({ ObstacleRef::Type::Keepout, i });
                    }
                }
        }
        // Board edge.
        const double re = halfWidth + board.edgeClearance;
        const double rve = viaDiameter / 2.0 + board.edgeClearance;
        for (int y = 0; y < grid.h; ++y)
            for (int x = 0; x < grid.w; ++x)
            {
                const auto xy = (size_t)grid.xy(x, y);
                const bool inside = insideBoard[xy] != 0;
                if (!inside || edgeDist[xy] < rve) m.viaBlock[xy] = hardCell;
                for (int l = 0; l < grid.layers; ++l)
                {
                    const auto cell = grid.index(x, y, l);
                    if (!inside || edgeDist[xy] < re) m.block[(size_t)cell] = hardCell;
                    else if (edgeDist[xy] < re + (double)p) m.near[cell].push_back({ ObstacleRef::Type::Edge, 0 });
                }
            }
        return staticMaps.emplace(key, std::move(m)).first->second;
    }

    // The segment between two cell centres keeps clearance from every nearby static obstacle.
    bool segmentClear(const StaticMap& m, int net, Point a, Point b, const std::vector<ObstacleRef>* near1, const std::vector<ObstacleRef>* near2) const
    {
        const double halfWidth = m.width / 2.0;
        for (const auto* list : { near1, near2 })
        {
            if (list == nullptr) continue;
            for (const auto& ref : *list)
            {
                switch (ref.type)
                {
                    case ObstacleRef::Type::Pad:
                    {
                        const auto& pad = board.pads[(size_t)ref.index];
                        if (pad.net == net && net >= 0) break;
                        const auto padClearance = std::max(m.clearance, pad.net >= 0 ? board.clearance(pad.net) : m.clearance);
                        if (pad.shape.distanceTo(a, b) < halfWidth + padClearance) return false;
                        break;
                    }
                    case ObstacleRef::Type::Keepout:
                        if (segmentPolygonDistance(a, b, board.keepouts[(size_t)ref.index].area) < halfWidth) return false;
                        break;
                    case ObstacleRef::Type::Edge:
                        if (board.edgeDistance(a, b) < halfWidth + board.edgeClearance) return false;
                        break;
                }
            }
        }
        return true;
    }

    const std::vector<ObstacleRef>* nearList(const StaticMap& m, int cell) const
    {
        const auto found = m.near.find(cell);
        return found != m.near.end() ? &found->second : nullptr;
    }

    std::vector<Terminal> terminalsFor(StaticMap& m, int net, int padIndex)
    {
        std::vector<Terminal> terminals;
        const auto& pad = board.pads[(size_t)padIndex];
        const auto bb = pad.shape.bounds();
        int x0, x1, y0, y1;
        grid.span(bb.minX, bb.maxX, grid.x0, grid.w, x0, x1);
        grid.span(bb.minY, bb.maxY, grid.y0, grid.h, y0, y1);
        for (int l = 0; l < grid.layers; ++l)
        {
            if (!pad.onLayer(l)) continue;
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x)
                {
                    const auto cell = grid.index(x, y, l);
                    if (pad.shape.contains(grid.centre(x, y)) && allowed(m.block[(size_t)cell], net))
                        terminals.push_back({ cell, false });
                }
        }
        if (!terminals.empty())
            return terminals;
        // No cell centre inside the pad: the nearest legal cells, joined by a stub to the pad centre.
        const auto c = pad.shape.centre;
        const auto cx = (int)std::lround((double)(c.x - grid.x0) / (double)grid.pitch);
        const auto cy = (int)std::lround((double)(c.y - grid.y0) / (double)grid.pitch);
        std::vector<std::pair<double, int>> candidates;
        for (int l = 0; l < grid.layers; ++l)
        {
            if (!pad.onLayer(l)) continue;
            for (int y = cy - 2; y <= cy + 2; ++y)
                for (int x = cx - 2; x <= cx + 2; ++x)
                {
                    if (x < 0 || y < 0 || x >= grid.w || y >= grid.h) continue;
                    const auto cell = grid.index(x, y, l);
                    if (!allowed(m.block[(size_t)cell], net)) continue;
                    const auto centre = grid.centre(x, y);
                    if (!segmentClear(m, net, centre, c, nearList(m, cell), nullptr)) continue;
                    candidates.push_back({ distance(centre, c), cell });
                }
        }
        std::sort(candidates.begin(), candidates.end());
        for (size_t i = 0; i < candidates.size() && i < 4; ++i)
            terminals.push_back({ candidates[i].second, true });
        return terminals;
    }

    double turnCost(int from, int to) const
    {
        if (from == noDirection) return 0.0;
        auto d = std::abs(from - to);
        d = std::min(d, 8 - d);
        switch (d)
        {
            case 0: return 0.0;
            case 1: return options.bend45Cost;
            case 2: return options.bend90Cost;
            case 3: return options.bend135Cost;
            default: return std::numeric_limits<double>::infinity();
        }
    }

    // Congestion price of claiming the cells of a disc (wide tracks, vias) and diagonal corners.
    double claimPrice(int x, int y, int layer, const std::vector<std::pair<int, int>>& disc) const
    {
        double price = 0.0;
        for (const auto& [dx, dy] : disc)
        {
            const int cx = x + dx, cy = y + dy;
            if (cx < 0 || cy < 0 || cx >= grid.w || cy >= grid.h) continue;
            const auto c = (size_t)grid.index(cx, cy, layer);
            if (usage[c] > 0) price += presentFactor * usage[c] * (1.0 + history[c]);
        }
        return price;
    }

    static std::vector<std::pair<int, int>> discOffsets(double radius)
    {
        std::vector<std::pair<int, int>> offsets;
        const int r = (int)std::ceil(radius);
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx)
                if (dx * dx + dy * dy < radius * radius - 1e-9 || (dx == 0 && dy == 0))
                    offsets.push_back({ dx, dy });
        return offsets;
    }

    // A* from `sources` (layer cells) to any marked target cell. Returns the path, source first.
    bool search(int net, StaticMap& m, const std::vector<int>& sources, const std::vector<Point>& targetCentres,
                const std::vector<std::pair<int, int>>& trackDisc, const std::vector<std::pair<int, int>>& viaDisc,
                std::vector<int>& path, int& reachedPad)
    {
        ++generation;
        using Entry = std::pair<float, int>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
        auto heuristic = [&](int cell) {
            const double x = (double)grid.xOf(cell), y = (double)grid.yOf(cell);
            double best = std::numeric_limits<double>::infinity();
            for (const auto& t : targetCentres)
            {
                const double dx = std::abs(x - (double)(t.x - grid.x0) / (double)grid.pitch);
                const double dy = std::abs(y - (double)(t.y - grid.y0) / (double)grid.pitch);
                best = std::min(best, std::max(dx, dy) + (sqrt2 - 1.0) * std::min(dx, dy));
            }
            return best;
        };
        for (int cell : sources)
        {
            const auto s = (size_t)cell * states + noDirection;
            seen[s] = generation;
            gScore[s] = 0.0f;
            parent[s] = -1;
            open.push({ (float)heuristic(cell), (int)s });
        }
        const auto preferred = [&](int layer) { return layer < (int)options.preferredDirection.size() ? options.preferredDirection[(size_t)layer] : 0; };

        while (!open.empty())
        {
            const auto [f, state] = open.top();
            open.pop();
            const int cell = state / states, dir = state % states;
            const float g = gScore[(size_t)state];
            if (f - (float)heuristic(cell) > g + 1e-3f) continue; // stale entry
            if (targetStamp[(size_t)cell] == generation)
            {
                reachedPad = targetPad[(size_t)cell];
                path.clear();
                for (int s = state; s >= 0; s = parent[(size_t)s])
                    path.push_back(s / states);
                std::reverse(path.begin(), path.end());
                return true;
            }
            const int x = grid.xOf(cell), y = grid.yOf(cell), layer = grid.layerOf(cell);
            auto relax = [&](int nextCell, int nextDir, double stepCost) {
                const auto ns = (size_t)nextCell * states + nextDir;
                const double ng = g + stepCost;
                if (seen[ns] == generation && gScore[ns] <= ng) return;
                seen[ns] = generation;
                gScore[ns] = (float)ng;
                parent[ns] = state;
                open.push({ (float)(ng + heuristic(nextCell)), (int)ns });
            };
            const auto* nearHere = nearList(m, cell);
            const auto here = grid.centre(x, y);
            for (int d = 0; d < 8; ++d)
            {
                const double turn = turnCost(dir, d);
                if (!std::isfinite(turn)) continue;
                const int nx = x + dirDX[d], ny = y + dirDY[d];
                if (nx < 0 || ny < 0 || nx >= grid.w || ny >= grid.h) continue;
                const int next = grid.index(nx, ny, layer);
                if (!allowed(m.block[(size_t)next], net)) continue;
                const auto* nearNext = nearList(m, next);
                if ((nearHere != nullptr || nearNext != nullptr) && !segmentClear(m, net, here, grid.centre(nx, ny), nearHere, nearNext))
                    continue;
                const bool diagonal = (d % 2) == 1;
                const double length = diagonal ? sqrt2 : 1.0;
                double cost = length * (1.0 + history[(size_t)next]) * (1.0 + presentFactor * usage[(size_t)next]) + turn;
                cost += claimPrice(nx, ny, layer, trackDisc) - (usage[(size_t)next] > 0 ? presentFactor * usage[(size_t)next] * (1.0 + history[(size_t)next]) : 0.0);
                if (diagonal)
                {
                    // The two corner cells a 45-degree step passes between.
                    cost += claimPrice(x + dirDX[d], y, layer, trackDisc) + claimPrice(x, y + dirDY[d], layer, trackDisc);
                }
                const int pref = preferred(layer);
                if ((pref == 1 && dirDY[d] != 0) || (pref == 2 && dirDX[d] != 0))
                    cost += options.wrongDirectionCost * length;
                relax(next, d, cost);
            }
            // A through via to every other layer.
            if (grid.layers > 1 && allowed(m.viaBlock[(size_t)grid.xy(x, y)], net))
            {
                double viaPrice = 0.0;
                for (int l = 0; l < grid.layers; ++l)
                    viaPrice += claimPrice(x, y, l, viaDisc);
                for (int l = 0; l < grid.layers; ++l)
                {
                    if (l == layer) continue;
                    const int next = grid.index(x, y, l);
                    if (!allowed(m.block[(size_t)next], net)) continue;
                    relax(next, noDirection, options.viaCost + viaPrice);
                }
            }
        }
        return false;
    }

    void routeNet(int net)
    {
        auto& route = routes[(size_t)net];
        route = {};
        auto& m = staticMapFor(net);
        const auto trackDisc = discOffsets(claimRadius(m.width, m.clearance));
        const auto viaDisc = discOffsets(claimRadius(m.viaDiameter, m.clearance));
        const auto pads = padsOf(net);

        std::vector<std::vector<Terminal>> terminals;
        for (int pad : pads)
        {
            terminals.push_back(terminalsFor(m, net, pad));
            for (const auto& t : terminals.back())
                route.terminalAt[t.cell] = { pad, t };
        }
        std::vector<int> tree;
        std::vector<size_t> remaining;
        for (size_t i = 0; i < pads.size(); ++i)
        {
            if (terminals[i].empty())
            {
                route.failedPads.push_back(pads[i]);
                failureReason[pads[i]] = "the pad has no legal routing cell (too close to other copper or the edge)";
            }
            else if (tree.empty())
                for (const auto& t : terminals[i]) tree.push_back(t.cell);
            else
                remaining.push_back(i);
        }
        while (!remaining.empty())
        {
            ++generation; // fresh target marks; search() advances it again for its own state
            const auto markGeneration = generation + 1;
            std::vector<Point> centres;
            for (auto i : remaining)
            {
                centres.push_back(board.pads[(size_t)pads[i]].shape.centre);
                for (const auto& t : terminals[i])
                {
                    targetStamp[(size_t)t.cell] = markGeneration;
                    targetPad[(size_t)t.cell] = (int)i;
                }
            }
            std::vector<int> path;
            int reached = -1;
            if (!search(net, m, tree, centres, trackDisc, viaDisc, path, reached))
            {
                for (auto i : remaining)
                {
                    route.failedPads.push_back(pads[i]);
                    failureReason[pads[i]] = "no path to the rest of the net";
                }
                break;
            }
            route.paths.push_back(path);
            tree.insert(tree.end(), path.begin(), path.end());
            // The pad's copper joins all of its cells: each is part of the tree now.
            for (const auto& t : terminals[(size_t)reached]) tree.push_back(t.cell);
            remaining.erase(std::remove(remaining.begin(), remaining.end(), (size_t)reached), remaining.end());
        }
        claimFootprint(net, trackDisc, viaDisc);
    }

    void claimFootprint(int net, const std::vector<std::pair<int, int>>& trackDisc, const std::vector<std::pair<int, int>>& viaDisc)
    {
        auto& route = routes[(size_t)net];
        ++claimGeneration;
        auto claim = [&](int x, int y, int layer, const std::vector<std::pair<int, int>>& disc) {
            for (const auto& [dx, dy] : disc)
            {
                const int cx = x + dx, cy = y + dy;
                if (cx < 0 || cy < 0 || cx >= grid.w || cy >= grid.h) continue;
                const auto c = grid.index(cx, cy, layer);
                if (claimStamp[(size_t)c] == claimGeneration) continue;
                claimStamp[(size_t)c] = claimGeneration;
                route.footprint.push_back(c);
            }
        };
        for (const auto& path : route.paths)
            for (size_t i = 0; i < path.size(); ++i)
            {
                const int x = grid.xOf(path[i]), y = grid.yOf(path[i]), l = grid.layerOf(path[i]);
                claim(x, y, l, trackDisc);
                if (i == 0) continue;
                const int px = grid.xOf(path[i - 1]), py = grid.yOf(path[i - 1]), pl = grid.layerOf(path[i - 1]);
                if (pl != l)
                {
                    for (int layer = 0; layer < grid.layers; ++layer)
                        claim(x, y, layer, viaDisc);
                }
                else if (px != x && py != y)
                {
                    claim(x, py, l, trackDisc);
                    claim(px, y, l, trackDisc);
                }
            }
    }

    void commit(int net)
    {
        for (int c : routes[(size_t)net].footprint)
            ++usage[(size_t)c];
    }

    void ripUp(int net)
    {
        for (int c : routes[(size_t)net].footprint)
            --usage[(size_t)c];
        routes[(size_t)net].footprint.clear();
    }

    bool sharesCells(int net) const
    {
        for (int c : routes[(size_t)net].footprint)
            if (usage[(size_t)c] > 1) return true;
        return false;
    }

    void emit(int net, RouteResult& result) const
    {
        const auto& route = routes[(size_t)net];
        const auto width = board.traceWidth(net);
        const auto& cls = board.classOf(net);
        std::vector<Point> viaPoints;
        auto addVia = [&](Point at) {
            if (std::find(viaPoints.begin(), viaPoints.end(), at) != viaPoints.end()) return;
            viaPoints.push_back(at);
            result.vias.push_back({ net, at, cls.viaDiameter, cls.viaDrill, 0, grid.layers - 1 });
        };
        auto stubPoint = [&](int cell, int layer, Point& out) {
            const auto found = route.terminalAt.find(cell);
            if (found == route.terminalAt.end() || !found->second.second.stub) return false;
            const auto& pad = board.pads[(size_t)found->second.first];
            if (!pad.onLayer(layer)) return false;
            out = pad.shape.centre;
            return true;
        };
        for (const auto& path : route.paths)
        {
            size_t start = 0;
            while (start < path.size())
            {
                const int layer = grid.layerOf(path[start]);
                size_t end = start;
                while (end + 1 < path.size() && grid.layerOf(path[end + 1]) == layer) ++end;
                Track track;
                track.net = net;
                track.layer = layer;
                track.width = width;
                Point stub;
                if (start == 0 && stubPoint(path[start], layer, stub)) track.points.push_back(stub);
                for (size_t i = start; i <= end; ++i)
                {
                    const auto p = grid.centreOf(path[i]);
                    // Merge collinear points.
                    if (track.points.size() >= 2)
                    {
                        const auto& a = track.points[track.points.size() - 2];
                        const auto& b = track.points.back();
                        if ((b.x - a.x) * (p.y - b.y) == (b.y - a.y) * (p.x - b.x) && ((b.x - a.x) * (p.x - b.x) + (b.y - a.y) * (p.y - b.y)) > 0)
                        {
                            track.points.back() = p;
                            continue;
                        }
                    }
                    if (track.points.empty() || track.points.back() != p) track.points.push_back(p);
                }
                if (end + 1 == path.size() && stubPoint(path[end], layer, stub) && track.points.back() != stub) track.points.push_back(stub);
                if (track.points.size() >= 2) result.tracks.push_back(std::move(track));
                if (end + 1 < path.size()) addVia(grid.centreOf(path[end]));
                start = end + 1;
            }
        }
    }
};
}

double RouteResult::trackLengthMm(int net) const
{
    double length = 0.0;
    for (const auto& t : tracks)
        if (net < 0 || t.net == net)
            for (size_t i = 1; i < t.points.size(); ++i)
                length += distance(t.points[i - 1], t.points[i]);
    return length / (double)nmPerMm;
}

RouteResult routeBoard(const Board& board, const RouterOptions& options)
{
    Router router(board, options);
    return router.run();
}
}
