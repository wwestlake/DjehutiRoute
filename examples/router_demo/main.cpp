#include "djehuti_route/grid.h"
#include "djehuti_route/constraints.h"
#include "djehuti_route/cost_function.h"
#include "djehuti_route/router.h"
#include "djehuti_route/autorouter.h"
#include <iostream>
#include <memory>
#include <vector>

using namespace djehuti::route;

int main() {
    auto grid = std::make_shared<BoardGrid>(200, 200, 2, 0.5);
    auto constraints = std::make_shared<ConstraintManager>();
    
    NetConstraint pwr_constraint;
    pwr_constraint.net_name = "VCC_5V";
    pwr_constraint.rms_current_amps = 2.5; 
    constraints->add_constraint(pwr_constraint);

    NetConstraint sig_constraint;
    sig_constraint.net_name = "SPI_CLK";
    sig_constraint.max_parallel_run_mm = 10.0; 
    constraints->add_constraint(sig_constraint);

    auto cost_function = std::make_shared<CostFunction>(constraints);
    auto router = std::make_shared<Router>(grid, cost_function);
    Autorouter autorouter(grid, router, constraints);

    Net vcc_net{1, "VCC_5V", {10, 10, 0}, {180, 180, 0}};
    Net clk_net{2, "SPI_CLK", {15, 10, 0}, {175, 180, 0}};
    Net gnd_net{3, "GND", {10, 190, 0}, {180, 20, 1}}; 
    Net data_net{4, "SPI_MOSI", {20, 10, 0}, {170, 180, 0}};
    Net data_miso{5, "SPI_MISO", {25, 10, 0}, {165, 180, 0}};

    autorouter.add_net(vcc_net);
    autorouter.add_net(clk_net);
    autorouter.add_net(gnd_net);
    autorouter.add_net(data_net);
    autorouter.add_net(data_miso);

    autorouter.route_all(1);

    // Output JSON for the UI
    std::cout << "{\n";
    std::cout << "  \"grid\": {\"width\": 200, \"height\": 200, \"layers\": 2},\n";
    std::cout << "  \"nets\": [\n";
    
    auto nets = autorouter.get_nets();
    for (size_t i = 0; i < nets.size(); ++i) {
        const auto& net = nets[i];
        std::cout << "    {\n";
        std::cout << "      \"id\": " << net.id << ",\n";
        std::cout << "      \"name\": \"" << net.name << "\",\n";
        std::cout << "      \"is_routed\": " << (net.is_routed ? "true" : "false") << ",\n";
        std::cout << "      \"path\": [\n";
        for (size_t j = 0; j < net.path.size(); ++j) {
            const auto& p = net.path[j];
            std::cout << "        {\"x\": " << p.x << ", \"y\": " << p.y << ", \"layer\": " << p.layer << "}";
            if (j < net.path.size() - 1) std::cout << ",";
            std::cout << "\n";
        }
        std::cout << "      ]\n";
        std::cout << "    }";
        if (i < nets.size() - 1) std::cout << ",";
        std::cout << "\n";
    }
    std::cout << "  ]\n";
    std::cout << "}\n";

    return 0;
}
