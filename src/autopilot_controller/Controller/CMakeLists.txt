cmake_minimum_required(VERSION 2.8)
project(autopilot_controller_controller)

include_directories(include)

# # set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to release")

aux_source_directory(src DIR_SRCS)
add_library(${PROJECT_NAME}_Controller ${DIR_SRCS})
set_target_properties(${PROJECT_NAME}_Controller PROPERTIES OUTPUT_NAME Controller PREFIX "")
add_dependencies(${PROJECT_NAME}_Controller autopilot_controller_common_common)
