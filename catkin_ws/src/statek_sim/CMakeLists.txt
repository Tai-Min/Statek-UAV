cmake_minimum_required(VERSION 3.0.2)
project(statek_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gazebo_ros
  statek_hw
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

add_library(statek_motor_control src/statek_motor_control.cpp)
add_dependencies(statek_motor_control statek_hw_gencpp)
target_link_libraries(statek_motor_control ${CATKIN_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS
    roscpp
    std_msgs
    gazebo_ros
    statek_hw
)