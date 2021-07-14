cmake_minimum_required(VERSION 3.0.2)
project(statek_sim)

add_compile_options(-std=c++11)
add_compile_options(-std=gnu++11)

find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_ros
  roscpp
  rospy
  std_msgs
  gazebo_ros
  statek_hw
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    tf2
    tf2_ros
    roscpp
    rospy
    std_msgs
    gazebo_ros
    statek_hw
)

add_library(statek_motor_control src/statek_motor_control.cpp)
add_dependencies(statek_motor_control statek_hw_gencpp)
target_link_libraries(statek_motor_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})