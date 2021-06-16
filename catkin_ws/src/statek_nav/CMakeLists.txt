cmake_minimum_required(VERSION 3.0.2)
project(statek_nav)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package()

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_executable(short_term_navigator src/short_term_navigator.cpp)
target_link_libraries(short_term_navigator ${catkin_LIBRARIES})