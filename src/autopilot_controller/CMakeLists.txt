cmake_minimum_required(VERSION 2.8)
project(autopilot_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive -O3 -g")

find_package(catkin REQUIRED COMPONENTS
        autopilot_msgs
        roscpp
        roslib
        rospy
        std_msgs
        tf
        )

find_package(Boost COMPONENTS system REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

catkin_package(
        LIBRARIES autopilot_controller
        CATKIN_DEPENDS autopilot_msgs boost roscpp rospy std_msgs tf
        DEPENDS Boost OpenGL GLUT
)

include_directories(
        common/include
        Controller/include
        Geography
        msg/obu_lcm
        ${catkin_INCLUDE_DIRS}
)

add_subdirectory(common)
add_subdirectory(Controller)
add_subdirectory(AX7controller)
add_subdirectory(tools/controllerWatchdog)
