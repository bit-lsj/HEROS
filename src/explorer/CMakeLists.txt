cmake_minimum_required(VERSION 3.0.2)
project(explorer)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

add_compile_options(-std=c++17)

find_package(ufomap REQUIRED)
find_package(PCL REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ortools CONFIG REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ufomap_msgs
  ufomap_ros
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(explorer_node
    src/explorer/explorer.cpp
    src/explorer/explorer_node.cpp
    src/graph/graph.cpp
    src/subregion_global/subregion_global.cpp
    # src/subregion/subregion_manager.cpp
    src/viewpoint/viewpoint_manager.cpp
    src/tsp_solver/tsp_solver.cpp
  )
  
add_executable(terrain_map_node
    src/terrain_map/terrain_map.cpp
    src/terrain_map/terrain_map_node.cpp
)
  
target_link_libraries(explorer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  UFO::Map
  ortools::ortools
)

target_link_libraries(terrain_map_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
