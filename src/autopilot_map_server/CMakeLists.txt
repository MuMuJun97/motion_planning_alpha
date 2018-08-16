cmake_minimum_required(VERSION 2.8.3)
project(autopilot_map_server)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        autopilot_msgs
        roscpp
        rospy
        std_msgs
        tf
        )

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES autopilot_map_server
        CATKIN_DEPENDS autopilot_msgs boost roscpp rospy std_msgs tf
        DEPENDS Boost PCL
)

include_directories(
        include
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_map_server src/map_server_node.cpp)

set_target_properties(${PROJECT_NAME}_map_server PROPERTIES OUTPUT_NAME map_server PREFIX "")

add_dependencies(${PROJECT_NAME}_map_server ${autopilot_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_map_server
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )


add_executable(${PROJECT_NAME}_rtk_mapping src/rtk_mapping_node.cpp)

set_target_properties(${PROJECT_NAME}_rtk_mapping PROPERTIES OUTPUT_NAME rtk_mapping PREFIX "")

add_dependencies(${PROJECT_NAME}_rtk_mapping ${autopilot_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_rtk_mapping
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})
