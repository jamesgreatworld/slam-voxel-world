cmake_minimum_required(VERSION 3.16)
project(semantic_map_core CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  add_compile_options(${OpenMP_CXX_FLAGS})
  link_libraries(OpenMP::OpenMP_CXX)
endif()

add_library(semantic_map_core STATIC
  src/obsmap.cpp
  src/field.cpp
  src/skeletonize.cpp
  src/graph.cpp
  src/objects.cpp
  src/scene_graph.cpp
  src/surface.cpp
  src/features.cpp
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

add_executable(bench_feat bench/bench_feat.cpp)
target_link_libraries(bench_feat semantic_map_core)

add_executable(bench_online bench/bench_online.cpp)
target_link_libraries(bench_online semantic_map_core)

add_executable(bench_timing bench/bench_timing.cpp)
target_link_libraries(bench_timing semantic_map_core)

add_executable(bench_fullpipe bench/bench_fullpipe.cpp)
target_link_libraries(bench_fullpipe semantic_map_core)

add_executable(bench_transforms bench/bench_transforms.cpp)
target_link_libraries(bench_transforms semantic_map_core)

# ---- rclcpp 在线驱动节点(需 source ROS 后 -DBUILD_ROS_NODE=ON)----
option(BUILD_ROS_NODE "build rclcpp live node" OFF)
if(BUILD_ROS_NODE)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  add_executable(smc_live_node src/smc_live_node.cpp)
  target_link_libraries(smc_live_node semantic_map_core rclcpp::rclcpp
    tf2_ros::tf2_ros message_filters::message_filters
    ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${visualization_msgs_TARGETS}
    ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})
endif()
