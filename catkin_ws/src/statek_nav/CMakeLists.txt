cmake_minimum_required(VERSION 3.0.2)
project(statek_nav)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2
  tf2_ros
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package()

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(short_term_navigator src/short_term_navigator.cpp src/mpc.cpp src/fg_eval.cpp)
target_compile_definitions(short_term_navigator PRIVATE NDEBUG)
target_link_libraries(short_term_navigator
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ipopt
)
