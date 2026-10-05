add_library(djehuti_route
    src/constraints.cpp
    src/industrial.cpp
    src/design_advisor.cpp
    src/grid.cpp
    src/cost_function.cpp
    src/router.cpp
)

target_include_directories(djehuti_route PUBLIC include)

# Optional: target_link_libraries if we had dependencies
