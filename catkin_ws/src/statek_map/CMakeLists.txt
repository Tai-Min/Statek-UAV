cmake_minimum_required(VERSION 3.0.2)
project(statek_map)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${visualization_msgs_INCLUDE_DIRS})
include_directories(${nav_msgs_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    rospy
    roscpp
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(static_map_generator_node src/static_map_generator_node.cpp)
target_link_libraries(static_map_generator_node ${catkin_LIBRARIES})