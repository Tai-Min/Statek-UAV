cmake_minimum_required(VERSION 3.0.2)
project(statek_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)
 
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
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)