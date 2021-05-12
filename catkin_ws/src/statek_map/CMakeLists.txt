cmake_minimum_required(VERSION 3.0.2)
project(statek_map)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2
  tf2_ros
  std_msgs
)
find_package(OpenCV)

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

add_executable(local_mapper_node 
  src/abstract_map.cpp
  src/local_mapper/local_mapper_node.cpp 
  src/local_mapper/map_fuser.cpp
  src/local_mapper/laser_scan_map.cpp
)
target_link_libraries(local_mapper_node ${catkin_LIBRARIES})

add_executable(voronoi_mapper_node 
  src/abstract_map.cpp 
  src/voronoi_mapper/voronoi_mapper_node.cpp
  src/voronoi_mapper/voronoi_map.cpp
)
target_link_libraries(voronoi_mapper_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(gps_to_tf_node 
  src/gps_to_tf_node.cpp
)
target_link_libraries(gps_to_tf_node ${catkin_LIBRARIES})
