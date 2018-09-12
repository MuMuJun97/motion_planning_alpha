cmake_minimum_required(VERSION 2.8)
project(autopilot_controller_ax7controller)

find_package(Boost COMPONENTS system REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

find_library(LCM lcm)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
)

# set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to release")

aux_source_directory(src DIR_SRCS)

add_executable(AX7controller ${DIR_SRCS})
target_link_libraries(
        AX7controller -pthread
        autopilot_controller_controller_Controller
        autopilot_controller_common_common
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${LCM}
        ${catkin_LIBRARIES}
)
add_dependencies(AX7controller
        autopilot_controller_controller_Controller
        autopilot_controller_common_common
        ${autopilot_msgs_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
