// Router checks. Expected values are worked out by hand from the grid
// geometry in the comments; every routed board must pass the exact DRC with
// zero violations.
//
// Grid for a 30 x 20 mm board, default class (0.25 mm track, 0.2 mm
// clearance): pitch 0.45 mm, 66 x 44 cells, centred, so cell centres are at
// x = 0.375 + 0.45 i and y = 0.325 + 0.45 j (mm).

#include "djehuti_route/board.h"
#include "djehuti_route/drc.h"
#include "djehuti_route/outline.h"
#include "djehuti_route/router.h"

#include <cmath>
#include <crtdbg.h>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <set>
#include <string>

using namespace djehuti::route;

namespace
{
int failures = 0;

void check(bool ok, const std::string& what, const std::string& detail = {})
{
    std::printf("%s  %s %s\n", ok ? "PASS" : "FAIL", what.c_str(), detail.c_str());
    if (!ok) ++failures;
}

void checkNear(double actual, double expected, double tolerance, const std::string& what)
{
    const bool ok = std::abs(actual - expected) <= tolerance;
    std::printf("%s  %-52s actual %-12.7g expected %-12.7g (+/- %g)\n", ok ? "PASS" : "FAIL", what.c_str(), actual, expected, tolerance);
    if (!ok) ++failures;
}

std::string drcSummary(const std::vector<Violation>& v)
{
    std::string s = "(" + std::to_string(v.size()) + " violation(s)";
    for (size_t i = 0; i < v.size() && i < 3; ++i) s += "; " + v[i].message;
    return s + ")";
}

void requireClean(const Board& board, const RouteResult& r, const std::string& name)
{
    const auto v = checkDesignRules(board, r);
    check(v.empty(), name + ": DRC clean", drcSummary(v));
}

int segments(const RouteResult& r)
{
    int n = 0;
    for (const auto& t : r.tracks) n += (int)t.points.size() - 1;
    return n;
}
}

int main()
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    _set_error_mode(_OUT_TO_STDERR);
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDERR);

    // 1. Straight. 1 x 1 mm pads at (5, 10) and (15, 10). Cells inside the
    //    first pad: x = 4.875, 5.325; inside the second: x = 14.775, 15.225.
    //    The track joins the facing cells: 14.775 - 5.325 = 9.45 mm, one segment.
    {
        Board b;
        b.addRectBoard(30, 20);
        const auto n = b.addNet("A");
        b.addSmdPad("P1", 5, 10, 1, 1, n);
        b.addSmdPad("P2", 15, 10, 1, 1, n);
        const auto r = routeBoard(b);
        check(r.error.empty() && r.routedConnections == 1, "straight: routed", r.error);
        checkNear(r.pitch / 1e6, 0.45, 1e-9, "straight: pitch (mm)");
        checkNear(r.trackLengthMm(), 9.45, 1e-6, "straight: track length (mm)");
        check(segments(r) == 1 && r.vias.empty(), "straight: one segment, no vias", "(" + std::to_string(segments(r)) + " segments)");
        requireClean(b, r, "straight");
    }

    // 2. 45 degrees. Pads (5, 5) and (12, 12): nearest cells (5.325, 5.275)
    //    and (11.625, 11.575), dx = dy = 6.3 mm -> one diagonal of 6.3 sqrt 2 = 8.909545 mm.
    {
        Board b;
        b.addRectBoard(30, 20);
        const auto n = b.addNet("A");
        b.addSmdPad("P1", 5, 5, 1, 1, n);
        b.addSmdPad("P2", 12, 12, 1, 1, n);
        const auto r = routeBoard(b);
        checkNear(r.trackLengthMm(), 6.3 * std::sqrt(2.0), 1e-6, "diagonal: track length (mm)");
        check(segments(r) == 1, "diagonal: one segment", "(" + std::to_string(segments(r)) + ")");
        requireClean(b, r, "diagonal");
    }

    // 3. A keepout wall across the whole board on F.Cu only, between two F.Cu
    //    pads: the route must go down a via, under the wall on B.Cu, and up: 2 vias.
    {
        Board b;
        b.addRectBoard(30, 20);
        const auto n = b.addNet("A");
        b.addSmdPad("P1", 5, 10, 1, 1, n, 0);
        b.addSmdPad("P2", 25, 10, 1, 1, n, 0);
        b.addKeepoutRect(14, -1, 16, 21, 0, 0);
        const auto r = routeBoard(b);
        check(r.routedConnections == 1, "via: routed");
        check(r.vias.size() == 2, "via: exactly two vias", "(" + std::to_string(r.vias.size()) + ")");
        bool under = false;
        for (const auto& t : r.tracks) if (t.layer == 1) under = true;
        check(under, "via: crosses the wall on B.Cu");
        requireClean(b, r, "via");
    }

    // 4. Three pads in an L: (5, 5), (20, 5), (20, 15). First branch 5.325 ->
    //    19.725 = 14.4 mm along a row; the second leaves the second pad's top
    //    cell row (y = 5.275) for the third pad's bottom row (y = 14.725): 9.45 mm.
    //    Total 23.85 mm.
    {
        Board b;
        b.addRectBoard(30, 20);
        const auto n = b.addNet("A");
        b.addSmdPad("P1", 5, 5, 1, 1, n);
        b.addSmdPad("P2", 20, 5, 1, 1, n);
        b.addSmdPad("P3", 20, 15, 1, 1, n);
        const auto r = routeBoard(b);
        check(r.connections == 2 && r.routedConnections == 2, "tree: both connections routed");
        checkNear(r.trackLengthMm(), 23.85, 1e-6, "tree: total length (mm)");
        requireClean(b, r, "tree");
    }

    // 5. IPC-2221 widths, by hand: 2 A, 10 C rise, 1 oz:
    //    outer: (2 / (0.048 * 10^0.44))^(1/0.725) = 42.393 mil^2 / 1.378 mil = 30.764 mil = 0.781411 mm
    //    inner (k = 0.024): 110.283 mil^2 -> 80.031 mil = 2.032793 mm
    {
        checkNear(toMm(ipc2221TraceWidth(2.0, 10.0, 1.0, true)), 0.781411, 1e-6, "IPC-2221 outer width, 2 A / 10 C / 1 oz (mm)");
        checkNear(toMm(ipc2221TraceWidth(2.0, 10.0, 1.0, false)), 2.032793, 1e-6, "IPC-2221 inner width, 2 A / 10 C / 1 oz (mm)");
    }

    // 6. A 2 A power net at its IPC width beside a signal net running parallel:
    //    the power track has the computed width and the DRC holds the clearance.
    {
        Board b;
        b.addRectBoard(30, 20);
        const auto power = b.addNet("VCC");
        b.nets[(size_t)power].widthOverride = ipc2221TraceWidth(2.0, 10.0, 1.0, true);
        const auto signal = b.addNet("SIG");
        b.addSmdPad("V1", 4, 9, 1.6, 1.6, power);
        b.addSmdPad("V2", 26, 9, 1.6, 1.6, power);
        b.addSmdPad("S1", 4, 11, 1, 1, signal);
        b.addSmdPad("S2", 26, 11, 1, 1, signal);
        const auto r = routeBoard(b);
        check(r.routedConnections == 2, "wide: both nets routed");
        bool width = false;
        for (const auto& t : r.tracks) if (t.net == power && std::abs(toMm(t.width) - 0.781411) < 1e-6) width = true;
        check(width, "wide: power track at its IPC width");
        requireClean(b, r, "wide");
    }

    // 7. The DRC itself: tracks 0.1 mm apart edge to edge (centres 0.35 mm, 0.25 mm wide)
    //    are a clearance violation (need 0.2 mm); crossing tracks are a short; a net
    //    with no copper between its pads is open.
    {
        Board b;
        b.addRectBoard(30, 20);
        const auto a = b.addNet("A");
        const auto c = b.addNet("B");
        const auto d = b.addNet("C");
        b.addSmdPad("C1", 10, 3, 1, 1, d);
        b.addSmdPad("C2", 20, 3, 1, 1, d);
        RouteResult r;
        r.tracks.push_back({ a, 0, mm(0.25), { { mm(5), mm(10) }, { mm(15), mm(10) } } });
        r.tracks.push_back({ c, 0, mm(0.25), { { mm(5), mm(10.35) }, { mm(15), mm(10.35) } } });
        auto v = checkDesignRules(b, r);
        int clearance = 0, shorts = 0, opens = 0;
        for (const auto& x : v)
        {
            if (x.kind == Violation::Kind::Clearance) ++clearance;
            if (x.kind == Violation::Kind::Short) ++shorts;
            if (x.kind == Violation::Kind::Open) ++opens;
        }
        check(clearance == 1, "drc: parallel tracks 0.1 mm apart flagged", drcSummary(v));
        if (!v.empty()) checkNear(v[0].actualMm, 0.1, 1e-6, "drc: measured gap (mm)");
        check(opens == 1, "drc: unconnected net flagged as open");
        r.tracks.push_back({ c, 0, mm(0.25), { { mm(10), mm(5) }, { mm(10), mm(15) } } });
        v = checkDesignRules(b, r);
        shorts = 0;
        for (const auto& x : v) if (x.kind == Violation::Kind::Short) ++shorts;
        check(shorts >= 1, "drc: crossing tracks flagged as a short");
    }

    // 8. Random two-layer boards: SMD pads on F.Cu at 2.54 mm sites, nets of 2-4 pads.
    //    Every routed board must be DRC clean; negotiation must never route fewer
    //    connections than a single pass that gives up shared nets.
    {
        int negotiatedTotal = 0, singleTotal = 0, connectionsTotal = 0;
        for (unsigned seed = 1; seed <= 6; ++seed)
        {
            std::mt19937 rng(seed);
            Board b;
            b.addRectBoard(60, 45);
            std::vector<std::pair<int, int>> sites;
            for (int y = 2; y < 17; ++y)
                for (int x = 2; x < 23; ++x)
                    sites.push_back({ x, y });
            std::shuffle(sites.begin(), sites.end(), rng);
            size_t next = 0;
            for (int n = 0; n < 24; ++n)
            {
                const auto net = b.addNet("N" + std::to_string(n));
                const int pins = 2 + (int)(rng() % 3);
                for (int p = 0; p < pins; ++p, ++next)
                    b.addSmdPad("N" + std::to_string(n) + "." + std::to_string(p), sites[next].first * 2.54, sites[next].second * 2.54, 1.2, 1.2, net);
            }
            const auto r = routeBoard(b);
            RouterOptions single;
            single.maxIterations = 1;
            const auto s = routeBoard(b, single);
            const auto v = checkDesignRules(b, r);
            negotiatedTotal += r.routedConnections;
            singleTotal += s.routedConnections;
            connectionsTotal += r.connections;
            std::printf("      seed %u: %d/%d connections in %d pass(es), %.2f s; single pass %d/%d; %zu vias; %zu DRC violation(s)\n",
                        seed, r.routedConnections, r.connections, r.iterations, r.seconds, s.routedConnections, s.connections, r.vias.size(), v.size());
            check(v.empty(), "random board " + std::to_string(seed) + ": DRC clean", drcSummary(v));
            check(r.routedConnections >= s.routedConnections, "random board " + std::to_string(seed) + ": negotiation routes at least as much as one pass");
        }
        std::printf("      totals: negotiated %d/%d, single pass %d/%d\n", negotiatedTotal, connectionsTotal, singleTotal, connectionsTotal);
    }

    // 9. Outline shapes, areas by hand:
    //    L 40 x 30 less a 15 x 10 notch = 1200 - 150 = 1050; U with a 10 x 12 slot = 1080;
    //    T bar 40 x 10 on a 10 x 25 stem = 650; chamfered 40 x 30, 5 mm = 1200 - 4 * 12.5 = 1150;
    //    rounded 40 x 30, r 5, 8 chords per corner = 1200 - 4 r^2 + 4 * (8 * r^2 sin(pi/16) / 2) = 1178.0361;
    //    circle d 20 as 64 chords = 32 * 100 * sin(2 pi / 64) = 313.6548;
    //    hexagon, corners on d 40: (3 sqrt3 / 2) * 20^2 = 1039.2305, 34.6410 mm tall;
    //    hexagon 20 mm across flats: 200 sqrt3 = 346.4102.
    {
        namespace o = djehuti::route::outline;
        checkNear(o::areaMm2(o::lShape(40, 30, 15, 10)), 1050.0, 1e-6, "outline: L area (mm^2)");
        checkNear(o::areaMm2(o::uShape(40, 30, 10, 12)), 1080.0, 1e-6, "outline: U area (mm^2)");
        checkNear(o::areaMm2(o::tShape(40, 10, 10, 25)), 650.0, 1e-6, "outline: T area (mm^2)");
        checkNear(o::areaMm2(o::chamferedRectangle(40, 30, 5)), 1150.0, 1e-6, "outline: chamfered area (mm^2)");
        checkNear(o::areaMm2(o::roundedRectangle(40, 30, 5, 8)), 1178.0361, 1e-3, "outline: rounded-corner area (mm^2)");
        checkNear(o::areaMm2(o::circle(20, 64)), 313.6548, 1e-3, "outline: circle area (mm^2)");
        const auto hex = o::regularPolygon(6, 40);
        checkNear(o::areaMm2(hex), 1039.2305, 1e-3, "outline: hexagon area (mm^2)");
        checkNear(toMm(boundsOf(hex).maxY - boundsOf(hex).minY), 34.6410, 1e-3, "outline: hexagon height (mm)");
        checkNear(o::areaMm2(o::regularPolygonAcrossFlats(6, 20)), 346.4102, 1e-3, "outline: hexagon by flats area (mm^2)");
        check(o::validate(hex).empty(), "outline: hexagon is valid");
        const Polygon bowtie { { 0, 0 }, { mm(10), mm(10) }, { mm(10), 0 }, { 0, mm(10) } };
        check(!o::validate(bowtie).empty() && o::validate(bowtie)[0].find("cross") != std::string::npos, "outline: a self-crossing outline is rejected as crossing", "(" + (o::validate(bowtie).empty() ? std::string() : o::validate(bowtie)[0]) + ")");
        const auto box = o::rectangle(40, 30);
        Polygon outside { { mm(35), mm(25) }, { mm(45), mm(25) }, { mm(45), mm(28) }, { mm(35), mm(28) } };
        check(!o::validateCutouts(box, { outside }).empty(), "outline: a cutout off the board is rejected");
        Polygon a { { mm(5), mm(5) }, { mm(15), mm(5) }, { mm(15), mm(15) }, { mm(5), mm(15) } };
        Polygon b2 { { mm(10), mm(10) }, { mm(20), mm(10) }, { mm(20), mm(20) }, { mm(10), mm(20) } };
        check(!o::validateCutouts(box, { a, b2 }).empty(), "outline: overlapping cutouts are rejected");
        check(o::validateCutouts(box, { a }).empty(), "outline: a cutout inside the board is accepted");
        const auto* hat = findStandardBoard("rpi-hat");
        check(hat != nullptr && hat->holes.size() == 4, "standard: Raspberry Pi HAT has 4 holes");
        if (hat != nullptr)
        {
            checkNear(toMm(boundsOf(hat->outline).maxX), 65.0, 1e-6, "standard: HAT width (mm)");
            checkNear(toMm(boundsOf(hat->outline).maxY), 56.5, 1e-6, "standard: HAT height (mm)");
            checkNear(toMm(hat->holes[1].centre.x - hat->holes[0].centre.x), 58.0, 1e-6, "standard: HAT hole spacing x (mm)");
            checkNear(toMm(hat->holes[2].centre.y - hat->holes[0].centre.y), 49.0, 1e-6, "standard: HAT hole spacing y (mm)");
        }
    }

    // 10. Odd boards. An L (40 x 30 less the 20 x 15 top-right notch) with pads in its two arms:
    //     the straight line between them passes exactly through the notch corner (20, 15), so the
    //     route must bend around it and stay inside. A hexagon with a route along a slanted edge.
    //     A window cut out of a 40 x 30 board between two pads forces a detour longer than the
    //     straight 30 mm. All DRC clean against the true (slanted / cut-out) edges.
    {
        Board l;
        l.outline = outline::lShape(40, 30, 20, 15);
        const auto n = l.addNet("A");
        l.addSmdPad("P1", 5, 25, 1, 1, n);
        l.addSmdPad("P2", 35, 5, 1, 1, n);
        const auto r = routeBoard(l);
        check(r.routedConnections == 1, "L board: routed", r.error);
        requireClean(l, r, "L board");

        Board h;
        h.outline = outline::regularPolygon(6, 40); // corners (40,17.32) (30,34.64) (10,34.64) (0,17.32) (10,0) (30,0)
        const auto m = h.addNet("A");
        const auto k = h.addNet("B");
        h.addSmdPad("A1", 4, 19, 1, 1, m);   // 2.6 mm in from the upper-left slanted edge
        h.addSmdPad("A2", 10, 29, 1, 1, m);  // 2.8 mm in from the same edge
        h.addSmdPad("B1", 8, 10, 1, 1, k);
        h.addSmdPad("B2", 32, 24, 1, 1, k);
        const auto rh = routeBoard(h);
        check(rh.routedConnections == 2, "hexagon board: routed", rh.error);
        requireClean(h, rh, "hexagon board");

        Board w;
        w.addRectBoard(40, 30);
        w.cutouts.push_back({ { mm(15), mm(5) }, { mm(25), mm(5) }, { mm(25), mm(25) }, { mm(15), mm(25) } });
        const auto c = w.addNet("A");
        w.addSmdPad("P1", 5, 15, 1, 1, c);
        w.addSmdPad("P2", 35, 15, 1, 1, c);
        const auto rw = routeBoard(w);
        check(rw.routedConnections == 1, "cutout board: routed around the window");
        check(rw.trackLengthMm() > 30.0, "cutout board: longer than the blocked straight line", "(" + std::to_string(rw.trackLengthMm()) + " mm)");
        requireClean(w, rw, "cutout board");

        // A standard board with its mounting holes as cutouts; a net passing between two holes.
        Board hat;
        const auto* spec = findStandardBoard("rpi-hat");
        hat.outline = spec->outline;
        for (const auto& hl : spec->holes) hat.addHole(toMm(hl.centre.x), toMm(hl.centre.y), toMm(hl.diameter));
        const auto s = hat.addNet("S");
        hat.addSmdPad("S1", 3.5, 10, 1, 1, s);
        hat.addSmdPad("S2", 61.5, 10, 1, 1, s);
        const auto rhat = routeBoard(hat);
        check(rhat.routedConnections == 1, "HAT board with mounting holes: routed");
        requireClean(hat, rhat, "HAT board with mounting holes");
    }

    std::printf("\n%s: %d failure(s)\n", failures == 0 ? "ALL PASSED" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
