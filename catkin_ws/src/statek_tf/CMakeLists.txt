cmake_minimum_required(VERSION 3.0.2)
project(statek_tf)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
  statek_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)