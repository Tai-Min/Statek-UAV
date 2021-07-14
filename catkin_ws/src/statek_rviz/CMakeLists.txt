cmake_minimum_required(VERSION 3.0.2)
project(statek_rviz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  statek_map
)

catkin_package()

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${visualization_msgs_INCLUDE_DIRS})
include_directories(${nav_msgs_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_to_markers_node src/map_to_markers_node.cpp)
target_link_libraries(map_to_markers_node ${catkin_LIBRARIES})

add_executable(graph_to_markers_node src/graph_to_markers_node.cpp)
target_link_libraries(graph_to_markers_node ${catkin_LIBRARIES})