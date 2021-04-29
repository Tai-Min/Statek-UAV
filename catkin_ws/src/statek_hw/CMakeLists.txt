cmake_minimum_required(VERSION 3.0.2)
project(statek_hw)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  image_transport
  cv_bridge
  message_generation
)
find_package(OpenCV)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_message_files(
   FILES
   Encoder.msg
   Velocity.msg
)

add_service_files(
  FILES
  RunImuCalibration.srv
  RunModelIdentification.srv
  RunVelocityTest.srv
  SetImuParams.srv
  SetMotorParams.srv
  SetOdomParams.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    DEPENDS
    rospy
    roscpp
    std_msgs
    image_transport
    cv_bridge
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_publisher_node src/vision_publisher_node.cpp src/camera.cpp src/ros_camera.cpp)
target_link_libraries(vision_publisher_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(laser_to_img_node src/laser_to_img_node.cpp)
target_link_libraries(laser_to_img_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})