cmake_minimum_required(VERSION 3.20)
project(DjehutiRoute VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(libs/djehuti_route)

# Standalone builds get the tests and the demo; a host project that pulls the
# library in with add_subdirectory only gets the library.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    add_subdirectory(examples/router_demo)

    add_executable(router_tests tests/router_tests.cpp)
    target_link_libraries(router_tests PRIVATE djehuti_route)

    enable_testing()
    add_test(NAME router_tests COMMAND router_tests)
endif()
