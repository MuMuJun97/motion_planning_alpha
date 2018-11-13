# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "autopilot_msgs: 8 messages, 2 services")

set(MSG_I_FLAGS "-Iautopilot_msgs:/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(autopilot_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg" NAME_WE)
add_custom_target(_autopilot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autopilot_msgs" "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg" "std_msgs/Header:autopilot_msgs/RouteNode"
)

get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg" NAME_WE)
add_custom_target(_autopilot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autopilot_msgs" "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg" "std_msgs/Header:autopilot_msgs/RouteNode"
)

get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg" NAME_WE)
add_custom_target(_autopilot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autopilot_msgs" "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg" ""
)

get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv" NAME_WE)
add_custom_target(_autopilot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autopilot_msgs" "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv" ""
)

get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg" NAME_WE)
add_custom_target(_autopilot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autopilot_msgs" "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg" "autopilot_msgs/RouteNode"
)

get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg" NAME_WE)
add_custom_target(_autopilot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autopilot_msgs" "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg" "autopilot_msgs/RouteArea:autopilot_msgs/RouteEdge:std_msgs/Header:autopilot_msgs/RouteNode"
)

get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg" NAME_WE)
add_custom_target(_autopilot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autopilot_msgs" "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg" "sensor_msgs/NavSatStatus:geometry_msgs/Twist:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3:geometry_msgs/Point:geometry_msgs/TwistWithCovariance:sensor_msgs/Imu:geometry_msgs/PoseWithCovariance:geometry_msgs/Pose:nav_msgs/Odometry:sensor_msgs/NavSatFix"
)

get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg" NAME_WE)
add_custom_target(_autopilot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autopilot_msgs" "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg" "autopilot_msgs/RouteNode"
)

get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv" NAME_WE)
add_custom_target(_autopilot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autopilot_msgs" "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv" ""
)

get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg" NAME_WE)
add_custom_target(_autopilot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autopilot_msgs" "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_cpp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_cpp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_cpp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_cpp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_cpp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_cpp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_cpp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs
)

### Generating Services
_generate_srv_cpp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs
)
_generate_srv_cpp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs
)

### Generating Module File
_generate_module_cpp(autopilot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(autopilot_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(autopilot_msgs_generate_messages autopilot_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_cpp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_cpp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_cpp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_cpp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_cpp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_cpp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_cpp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_cpp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_cpp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_cpp _autopilot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autopilot_msgs_gencpp)
add_dependencies(autopilot_msgs_gencpp autopilot_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autopilot_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_eus(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_eus(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_eus(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_eus(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_eus(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_eus(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_eus(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs
)

### Generating Services
_generate_srv_eus(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs
)
_generate_srv_eus(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs
)

### Generating Module File
_generate_module_eus(autopilot_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(autopilot_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(autopilot_msgs_generate_messages autopilot_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_eus _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_eus _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_eus _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_eus _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_eus _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_eus _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_eus _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_eus _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_eus _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_eus _autopilot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autopilot_msgs_geneus)
add_dependencies(autopilot_msgs_geneus autopilot_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autopilot_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_lisp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_lisp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_lisp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_lisp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_lisp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_lisp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_lisp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs
)

### Generating Services
_generate_srv_lisp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs
)
_generate_srv_lisp(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs
)

### Generating Module File
_generate_module_lisp(autopilot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(autopilot_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(autopilot_msgs_generate_messages autopilot_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_lisp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_lisp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_lisp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_lisp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_lisp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_lisp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_lisp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_lisp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_lisp _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_lisp _autopilot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autopilot_msgs_genlisp)
add_dependencies(autopilot_msgs_genlisp autopilot_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autopilot_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_nodejs(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_nodejs(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_nodejs(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_nodejs(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_nodejs(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_nodejs(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_nodejs(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs
)

### Generating Services
_generate_srv_nodejs(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs
)
_generate_srv_nodejs(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs
)

### Generating Module File
_generate_module_nodejs(autopilot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(autopilot_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(autopilot_msgs_generate_messages autopilot_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_nodejs _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_nodejs _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_nodejs _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_nodejs _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_nodejs _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_nodejs _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_nodejs _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_nodejs _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_nodejs _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_nodejs _autopilot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autopilot_msgs_gennodejs)
add_dependencies(autopilot_msgs_gennodejs autopilot_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autopilot_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_py(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_py(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_py(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_py(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_py(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_py(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg"
  "${MSG_I_FLAGS}"
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs
)
_generate_msg_py(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs
)

### Generating Services
_generate_srv_py(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs
)
_generate_srv_py(autopilot_msgs
  "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs
)

### Generating Module File
_generate_module_py(autopilot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(autopilot_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(autopilot_msgs_generate_messages autopilot_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_py _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_py _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_py _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_py _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_py _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_py _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_py _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_py _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_py _autopilot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg" NAME_WE)
add_dependencies(autopilot_msgs_generate_messages_py _autopilot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autopilot_msgs_genpy)
add_dependencies(autopilot_msgs_genpy autopilot_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autopilot_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autopilot_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(autopilot_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(autopilot_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(autopilot_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(autopilot_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autopilot_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(autopilot_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(autopilot_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(autopilot_msgs_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(autopilot_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autopilot_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(autopilot_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(autopilot_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(autopilot_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(autopilot_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autopilot_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(autopilot_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(autopilot_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(autopilot_msgs_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(autopilot_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autopilot_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(autopilot_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(autopilot_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(autopilot_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(autopilot_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
