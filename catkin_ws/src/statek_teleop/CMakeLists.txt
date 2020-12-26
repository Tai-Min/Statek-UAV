cmake_minimum_required(VERSION 3.0.2)
project(statek_teleop)

find_package(catkin REQUIRED COMPONENTS
  ds4_driver
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)