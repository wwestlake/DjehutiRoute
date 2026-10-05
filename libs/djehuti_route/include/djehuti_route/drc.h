#pragma once

// Exact geometric design-rule check of a routed board, independent of the
// router's grid: tracks are capsules (segment + half width), vias discs, pads
// their shapes. The tests require zero violations on every routed board.

#include "djehuti_route/board.h"
#include "djehuti_route/router.h"

#include <string>
#include <vector>

namespace djehuti::route
{
struct Violation
{
    enum class Kind { Clearance, Short, Edge, Keepout, Open };
    Kind kind = Kind::Clearance;
    int netA = -1, netB = -1;
    int layer = -1;
    Point at;
    double actualMm = 0.0;   // distance found
    double requiredMm = 0.0; // distance required
    std::string message;
};

std::vector<Violation> checkDesignRules(const Board& board, const RouteResult& result);
const char* kindName(Violation::Kind kind);
}
