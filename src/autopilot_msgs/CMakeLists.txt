cmake_minimum_required(VERSION 2.8.3)
project(autopilot_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        )

add_message_files(
        DIRECTORY msg
        FILES
        RouteNode.msg
        RouteEdge.msg
        Location.msg
        RouteArea.msg
        RouteMap.msg
        RoutePath.msg
        MotionState.msg
        WayPoints.msg
)

add_service_files(
        DIRECTORY srv
        FILES
        WGS2Map.srv
        Map2WGS.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs sensor_msgs nav_msgs geometry_msgs
)
