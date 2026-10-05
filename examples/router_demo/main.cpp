// Routes a generated two-layer board and writes it as an SVG to look at:
// F.Cu red, B.Cu blue, vias white rings, pads grey, unrouted connections as
// dashed lines. Usage: router_demo [seed] [output.svg]

#include "djehuti_route/board.h"
#include "djehuti_route/drc.h"
#include "djehuti_route/router.h"

#include <cstdio>
#include <fstream>
#include <random>
#include <string>

using namespace djehuti::route;

int main(int argc, char** argv)
{
    const unsigned seed = argc > 1 ? (unsigned)std::stoul(argv[1]) : 3u;
    const std::string out = argc > 2 ? argv[2] : "route_demo.svg";

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
    const auto v = checkDesignRules(b, r);
    std::printf("%d/%d connections, %zu vias, %d pass(es), %.2f s, %zu DRC violation(s)\n",
                r.routedConnections, r.connections, r.vias.size(), r.iterations, r.seconds, v.size());

    const double scale = 15.0; // px per mm
    auto px = [&](Coord c) { return toMm(c) * scale; };
    std::ofstream svg(out);
    svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << 60 * scale << "' height='" << 45 * scale << "' style='background:#0e141a'>\n";
    svg << "<polygon fill='#16301f' stroke='#c9b45a' stroke-width='1' points='";
    for (const auto& p : b.outline) svg << px(p.x) << "," << px(p.y) << " ";
    svg << "'/>\n";
    for (int layer = 1; layer >= 0; --layer)
        for (const auto& t : r.tracks)
        {
            if (t.layer != layer) continue;
            svg << "<polyline fill='none' stroke-linecap='round' stroke-linejoin='round' stroke='" << (layer == 0 ? "#e0533d" : "#3d82e0")
                << "' stroke-opacity='0.85' stroke-width='" << px(t.width) << "' points='";
            for (const auto& p : t.points) svg << px(p.x) << "," << px(p.y) << " ";
            svg << "'/>\n";
        }
    for (const auto& pad : b.pads)
        svg << "<rect fill='#b8b8b8' x='" << px(pad.shape.centre.x - pad.shape.sizeX / 2) << "' y='" << px(pad.shape.centre.y - pad.shape.sizeY / 2)
            << "' width='" << px(pad.shape.sizeX) << "' height='" << px(pad.shape.sizeY) << "'/>\n";
    for (const auto& via : r.vias)
        svg << "<circle fill='#ffffff' stroke='#222' stroke-width='" << px(via.diameter - via.drill) / 2 << "' cx='" << px(via.at.x) << "' cy='" << px(via.at.y)
            << "' r='" << px(via.diameter) / 2 - px(via.diameter - via.drill) / 4 << "'/>\n";
    for (const auto& u : r.unrouted)
        for (const auto& pad : b.pads)
            if (pad.reference == u.pad)
                for (const auto& other : b.pads)
                    if (other.net == u.net && &other != &pad)
                    {
                        svg << "<line stroke='#ffd24a' stroke-dasharray='4 3' x1='" << px(pad.shape.centre.x) << "' y1='" << px(pad.shape.centre.y)
                            << "' x2='" << px(other.shape.centre.x) << "' y2='" << px(other.shape.centre.y) << "'/>\n";
                        break;
                    }
    svg << "</svg>\n";
    std::printf("wrote %s\n", out.c_str());
    return v.empty() ? 0 : 1;
}
