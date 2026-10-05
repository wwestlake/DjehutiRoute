add_library(djehuti_route STATIC
    include/djehuti_route/geometry.h
    include/djehuti_route/board.h
    include/djehuti_route/router.h
    include/djehuti_route/drc.h
    include/djehuti_route/outline.h
    src/geometry.cpp
    src/board.cpp
    src/router.cpp
    src/drc.cpp
    src/outline.cpp
)

target_include_directories(djehuti_route PUBLIC include)
target_compile_features(djehuti_route PUBLIC cxx_std_20)
if(MSVC)
    target_compile_options(djehuti_route PRIVATE /W4 /permissive-)
endif()
