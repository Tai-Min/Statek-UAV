cmake_minimum_required(VERSION 3.0.2)
project(statek_ml)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
   FILES
   DynamicDetection.msg
   DynamicDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)