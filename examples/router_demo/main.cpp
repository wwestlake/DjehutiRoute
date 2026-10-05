#include "djehuti_route/grid.h"
#include "djehuti_route/constraints.h"
#include "djehuti_route/cost_function.h"
#include "djehuti_route/router.h"
#include "djehuti_route/autorouter.h"
#include <iostream>
#include <memory>

using namespace djehuti::route;

int main() {
    std::cout << "--- DjehutiRoute Engine Demo ---\n";

    // 1. Create a 3D board grid (100x100mm, 2 layers, 0.5mm resolution)
    auto grid = std::make_shared<BoardGrid>(200, 200, 2, 0.5);

    // 2. Setup SPICE constraints
    auto constraints = std::make_shared<ConstraintManager>();
    
    NetConstraint pwr_constraint;
    pwr_constraint.net_name = "VCC_5V";
    pwr_constraint.rms_current_amps = 2.5; // High current net
    constraints->add_constraint(pwr_constraint);

    NetConstraint sig_constraint;
    sig_constraint.net_name = "SPI_CLK";
    sig_constraint.max_parallel_run_mm = 10.0; // Crosstalk sensitive
    constraints->add_constraint(sig_constraint);

    // 3. Initialize routing algorithms
    auto cost_function = std::make_shared<CostFunction>(constraints);
    auto router = std::make_shared<Router>(grid, cost_function);
    Autorouter autorouter(grid, router, constraints);

    // 4. Define nets to route
    Net vcc_net{1, "VCC_5V", {10, 10, 0}, {180, 180, 0}};
    Net clk_net{2, "SPI_CLK", {15, 10, 0}, {175, 180, 0}};

    autorouter.add_net(vcc_net);
    autorouter.add_net(clk_net);

    // 5. Run Autorouter
    std::cout << "Running A* Autorouter with IPC-2152 trace expansion...\n";
    bool success = autorouter.route_all(1);
    
    if (success) {
        std::cout << "Successfully routed all nets!\n";
    } else {
        std::cout << "Routing failed to complete all nets.\n";
    }

    return 0;
}
