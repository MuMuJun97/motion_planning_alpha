execute_process(COMMAND "/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/obstacle_ws/src/vision_opencv/cv_bridge/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/trouble/ros_ws/motion_planning_alpha/src/cmake-build-debug/obstacle_ws/src/vision_opencv/cv_bridge/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
