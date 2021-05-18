# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "statek_hw: 2 messages, 6 services")

set(MSG_I_FLAGS "-Istatek_hw:/home/mateusz/ros/catkin_ws/src/statek_hw/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(statek_hw_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunVelocityTest.srv" NAME_WE)
add_custom_target(_statek_hw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "statek_hw" "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunVelocityTest.srv" ""
)

get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Encoder.msg" NAME_WE)
add_custom_target(_statek_hw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "statek_hw" "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Encoder.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunModelIdentification.srv" NAME_WE)
add_custom_target(_statek_hw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "statek_hw" "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunModelIdentification.srv" ""
)

get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetImuParams.srv" NAME_WE)
add_custom_target(_statek_hw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "statek_hw" "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetImuParams.srv" ""
)

get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetOdomParams.srv" NAME_WE)
add_custom_target(_statek_hw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "statek_hw" "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetOdomParams.srv" ""
)

get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunImuCalibration.srv" NAME_WE)
add_custom_target(_statek_hw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "statek_hw" "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunImuCalibration.srv" ""
)

get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetMotorParams.srv" NAME_WE)
add_custom_target(_statek_hw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "statek_hw" "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetMotorParams.srv" ""
)

get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Velocity.msg" NAME_WE)
add_custom_target(_statek_hw_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "statek_hw" "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Velocity.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/statek_hw
)
_generate_msg_cpp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/statek_hw
)

### Generating Services
_generate_srv_cpp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunVelocityTest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/statek_hw
)
_generate_srv_cpp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetOdomParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/statek_hw
)
_generate_srv_cpp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetImuParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/statek_hw
)
_generate_srv_cpp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunModelIdentification.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/statek_hw
)
_generate_srv_cpp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunImuCalibration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/statek_hw
)
_generate_srv_cpp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetMotorParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/statek_hw
)

### Generating Module File
_generate_module_cpp(statek_hw
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/statek_hw
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(statek_hw_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(statek_hw_generate_messages statek_hw_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunVelocityTest.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_cpp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Encoder.msg" NAME_WE)
add_dependencies(statek_hw_generate_messages_cpp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunModelIdentification.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_cpp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetImuParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_cpp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetOdomParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_cpp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunImuCalibration.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_cpp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetMotorParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_cpp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Velocity.msg" NAME_WE)
add_dependencies(statek_hw_generate_messages_cpp _statek_hw_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(statek_hw_gencpp)
add_dependencies(statek_hw_gencpp statek_hw_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS statek_hw_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/statek_hw
)
_generate_msg_eus(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/statek_hw
)

### Generating Services
_generate_srv_eus(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunVelocityTest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/statek_hw
)
_generate_srv_eus(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetOdomParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/statek_hw
)
_generate_srv_eus(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetImuParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/statek_hw
)
_generate_srv_eus(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunModelIdentification.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/statek_hw
)
_generate_srv_eus(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunImuCalibration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/statek_hw
)
_generate_srv_eus(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetMotorParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/statek_hw
)

### Generating Module File
_generate_module_eus(statek_hw
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/statek_hw
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(statek_hw_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(statek_hw_generate_messages statek_hw_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunVelocityTest.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_eus _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Encoder.msg" NAME_WE)
add_dependencies(statek_hw_generate_messages_eus _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunModelIdentification.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_eus _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetImuParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_eus _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetOdomParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_eus _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunImuCalibration.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_eus _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetMotorParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_eus _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Velocity.msg" NAME_WE)
add_dependencies(statek_hw_generate_messages_eus _statek_hw_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(statek_hw_geneus)
add_dependencies(statek_hw_geneus statek_hw_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS statek_hw_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/statek_hw
)
_generate_msg_lisp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/statek_hw
)

### Generating Services
_generate_srv_lisp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunVelocityTest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/statek_hw
)
_generate_srv_lisp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetOdomParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/statek_hw
)
_generate_srv_lisp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetImuParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/statek_hw
)
_generate_srv_lisp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunModelIdentification.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/statek_hw
)
_generate_srv_lisp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunImuCalibration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/statek_hw
)
_generate_srv_lisp(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetMotorParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/statek_hw
)

### Generating Module File
_generate_module_lisp(statek_hw
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/statek_hw
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(statek_hw_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(statek_hw_generate_messages statek_hw_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunVelocityTest.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_lisp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Encoder.msg" NAME_WE)
add_dependencies(statek_hw_generate_messages_lisp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunModelIdentification.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_lisp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetImuParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_lisp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetOdomParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_lisp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunImuCalibration.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_lisp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetMotorParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_lisp _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Velocity.msg" NAME_WE)
add_dependencies(statek_hw_generate_messages_lisp _statek_hw_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(statek_hw_genlisp)
add_dependencies(statek_hw_genlisp statek_hw_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS statek_hw_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/statek_hw
)
_generate_msg_nodejs(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/statek_hw
)

### Generating Services
_generate_srv_nodejs(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunVelocityTest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/statek_hw
)
_generate_srv_nodejs(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetOdomParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/statek_hw
)
_generate_srv_nodejs(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetImuParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/statek_hw
)
_generate_srv_nodejs(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunModelIdentification.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/statek_hw
)
_generate_srv_nodejs(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunImuCalibration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/statek_hw
)
_generate_srv_nodejs(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetMotorParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/statek_hw
)

### Generating Module File
_generate_module_nodejs(statek_hw
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/statek_hw
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(statek_hw_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(statek_hw_generate_messages statek_hw_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunVelocityTest.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_nodejs _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Encoder.msg" NAME_WE)
add_dependencies(statek_hw_generate_messages_nodejs _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunModelIdentification.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_nodejs _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetImuParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_nodejs _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetOdomParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_nodejs _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunImuCalibration.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_nodejs _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetMotorParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_nodejs _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Velocity.msg" NAME_WE)
add_dependencies(statek_hw_generate_messages_nodejs _statek_hw_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(statek_hw_gennodejs)
add_dependencies(statek_hw_gennodejs statek_hw_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS statek_hw_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/statek_hw
)
_generate_msg_py(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/statek_hw
)

### Generating Services
_generate_srv_py(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunVelocityTest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/statek_hw
)
_generate_srv_py(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetOdomParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/statek_hw
)
_generate_srv_py(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetImuParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/statek_hw
)
_generate_srv_py(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunModelIdentification.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/statek_hw
)
_generate_srv_py(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunImuCalibration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/statek_hw
)
_generate_srv_py(statek_hw
  "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetMotorParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/statek_hw
)

### Generating Module File
_generate_module_py(statek_hw
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/statek_hw
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(statek_hw_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(statek_hw_generate_messages statek_hw_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunVelocityTest.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_py _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Encoder.msg" NAME_WE)
add_dependencies(statek_hw_generate_messages_py _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunModelIdentification.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_py _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetImuParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_py _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetOdomParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_py _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/RunImuCalibration.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_py _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/srv/SetMotorParams.srv" NAME_WE)
add_dependencies(statek_hw_generate_messages_py _statek_hw_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mateusz/ros/catkin_ws/src/statek_hw/msg/Velocity.msg" NAME_WE)
add_dependencies(statek_hw_generate_messages_py _statek_hw_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(statek_hw_genpy)
add_dependencies(statek_hw_genpy statek_hw_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS statek_hw_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/statek_hw)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/statek_hw
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(statek_hw_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/statek_hw)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/statek_hw
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(statek_hw_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/statek_hw)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/statek_hw
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(statek_hw_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/statek_hw)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/statek_hw
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(statek_hw_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/statek_hw)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/statek_hw\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/statek_hw
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(statek_hw_generate_messages_py std_msgs_generate_messages_py)
endif()
