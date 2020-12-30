cmake_minimum_required(VERSION 3.0.2)
project(statek_hw)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rospkg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
