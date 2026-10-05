#pragma once

// What a board is, as far as routing cares: stackup, design rules, pads, nets,
// keepouts and the outline. Everything in nanometres (see geometry.h).

#include "djehuti_route/geometry.h"

#include <string>
#include <vector>

namespace djehuti::route
{
struct NetClass
{
    std::string name = "Default";
    Coord traceWidth = mm(0.25);
    Coord clearance = mm(0.2);
    Coord viaDiameter = mm(0.6);
    Coord viaDrill = mm(0.3);
};

struct Net
{
    std::string name;
    int netClass = 0;          // index into Board::netClasses
    Coord widthOverride = 0;   // > 0: this net's trace width (e.g. from its current)
};

struct Pad
{
    std::string reference;     // "U1.3", "R2.1"
    Shape shape;
    int net = -1;              // index into Board::nets, -1 = no net
    int firstLayer = 0;        // SMD: firstLayer == lastLayer; through-hole: 0 .. layers-1
    int lastLayer = 0;
    Coord drill = 0;           // through-hole drill, 0 for SMD

    bool onLayer(int layer) const { return layer >= firstLayer && layer <= lastLayer; }
};

struct Keepout
{
    Polygon area;
    int firstLayer = 0;
    int lastLayer = 0;
    bool onLayer(int layer) const { return layer >= firstLayer && layer <= lastLayer; }
};

struct Board
{
    std::vector<std::string> layers { "F.Cu", "B.Cu" };
    Polygon outline;
    Coord edgeClearance = mm(0.3);
    std::vector<NetClass> netClasses { NetClass {} };
    std::vector<Net> nets;
    std::vector<Pad> pads;
    std::vector<Keepout> keepouts;

    int layerCount() const { return (int)layers.size(); }
    const NetClass& classOf(int net) const { return netClasses[(size_t)nets[(size_t)net].netClass]; }
    Coord traceWidth(int net) const;
    Coord clearance(int net) const { return net >= 0 ? classOf(net).clearance : netClasses[0].clearance; }
    // Clearance two nets must keep: the larger of their classes'.
    Coord clearance(int netA, int netB) const;

    // Convenience builders (millimetres).
    int addNet(const std::string& name, int netClass = 0);
    int addRectBoard(double widthMm, double heightMm);
    void addSmdPad(const std::string& reference, double xMm, double yMm, double wMm, double hMm, int net, int layer = 0);
    void addRoundPad(const std::string& reference, double xMm, double yMm, double diameterMm, double drillMm, int net);
    void addKeepoutRect(double x0Mm, double y0Mm, double x1Mm, double y1Mm, int firstLayer = 0, int lastLayer = -1);
};

// IPC-2221 trace width for a current: area = (I / (k * dT^0.44))^(1/0.725)
// square mils, k = 0.048 for outer layers and 0.024 for inner layers; width =
// area / thickness (1 oz/ft^2 = 1.378 mil).
Coord ipc2221TraceWidth(double currentAmps, double temperatureRiseC, double copperOz, bool outerLayer);
}
