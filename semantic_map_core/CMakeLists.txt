cmake_minimum_required(VERSION 3.16)
project(semantic_map_core CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

add_library(semantic_map_core STATIC
  src/obsmap.cpp
  src/field.cpp
  src/skeletonize.cpp
  src/graph.cpp
  src/objects.cpp
  src/scene_graph.cpp
  src/surface.cpp
)
target_include_directories(semantic_map_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(bench_obsmap bench/bench_obsmap.cpp)
target_link_libraries(bench_obsmap semantic_map_core)

add_executable(bench_field bench/bench_field.cpp)
target_link_libraries(bench_field semantic_map_core)

add_executable(prof_field bench/prof_field.cpp)
target_link_libraries(prof_field semantic_map_core)

add_executable(bench_graph bench/bench_graph.cpp)
target_link_libraries(bench_graph semantic_map_core)

add_executable(bench_skel bench/bench_skel.cpp)
target_link_libraries(bench_skel semantic_map_core)

add_executable(bench_rooms bench/bench_rooms.cpp)
target_link_libraries(bench_rooms semantic_map_core)

add_executable(bench_objects bench/bench_objects.cpp)
target_link_libraries(bench_objects semantic_map_core)

add_executable(bench_scene bench/bench_scene.cpp)
target_link_libraries(bench_scene semantic_map_core)

add_executable(bench_surface bench/bench_surface.cpp)
target_link_libraries(bench_surface semantic_map_core)
