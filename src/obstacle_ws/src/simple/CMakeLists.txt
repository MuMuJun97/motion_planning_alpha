cmake_minimum_required(VERSION 2.8)

project(simple)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(OpenCV 3.3.1 EXACT REQUIRED)
#find_package(catkin_simple 0.1.0 REQUIRED )
#catkin_simple(ALL_DEPS_REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_ros
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  geometry_msgs
  sensor_msgs
  octomap_msgs
  filters
)

find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
#  DEPENDS system_lib
)


include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIR}
)

## Declare a cpp executable
add_executable(
  grid_test
  simple.cpp
)

target_link_libraries(
  grid_test
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)



