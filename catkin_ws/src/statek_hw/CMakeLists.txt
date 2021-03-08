cmake_minimum_required(VERSION 3.0.2)
project(statek_hw)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    rospy
    roscpp
    std_msgs
    image_transport
    cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_publisher_node src/vision_publisher_node.cpp src/camera.cpp src/ros_camera.cpp)
target_link_libraries(vision_publisher_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})