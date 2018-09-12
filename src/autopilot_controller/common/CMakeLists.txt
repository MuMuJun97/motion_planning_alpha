cmake_minimum_required(VERSION 2.8)
project(autopilot_controller_common)

find_package(Boost COMPONENTS system REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(
        include
        ${Boost_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
)

# message(${PROJECT_SOURCE_DIR})

aux_source_directory(src DIR_SRCS)
add_library(${PROJECT_NAME}_common ${DIR_SRCS})
set_target_properties(${PROJECT_NAME}_common PROPERTIES OUTPUT_NAME common PREFIX "")

