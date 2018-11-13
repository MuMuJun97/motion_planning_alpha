# Install script for directory: /home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autopilot_msgs/msg" TYPE FILE FILES
    "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteNode.msg"
    "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteEdge.msg"
    "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/Location.msg"
    "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteArea.msg"
    "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RouteMap.msg"
    "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/RoutePath.msg"
    "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/MotionState.msg"
    "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/msg/WayPoints.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autopilot_msgs/srv" TYPE FILE FILES
    "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/WGS2Map.srv"
    "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/srv/Map2WGS.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autopilot_msgs/cmake" TYPE FILE FILES "/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/autopilot_msgs/catkin_generated/installspace/autopilot_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/devel/include/autopilot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/devel/share/roseus/ros/autopilot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/devel/share/common-lisp/ros/autopilot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/devel/share/gennodejs/ros/autopilot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/devel/lib/python2.7/dist-packages/autopilot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/devel/lib/python2.7/dist-packages/autopilot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/autopilot_msgs/catkin_generated/installspace/autopilot_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autopilot_msgs/cmake" TYPE FILE FILES "/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/autopilot_msgs/catkin_generated/installspace/autopilot_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autopilot_msgs/cmake" TYPE FILE FILES
    "/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/autopilot_msgs/catkin_generated/installspace/autopilot_msgsConfig.cmake"
    "/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/autopilot_msgs/catkin_generated/installspace/autopilot_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autopilot_msgs" TYPE FILE FILES "/home/trouble/ros_ws/motion_planning_alpha/src/autopilot_msgs/package.xml")
endif()

