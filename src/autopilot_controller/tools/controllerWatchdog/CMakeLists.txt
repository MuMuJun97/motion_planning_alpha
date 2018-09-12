cmake_minimum_required(VERSION 2.8)
project(autopilot_controller_controller_watchdog)

find_package(Boost COMPONENTS system REQUIRED)
find_library(LCM lcm)

include_directories(
        src/common
        ${Boost_INCLUDE_DIRS}
)

aux_source_directory(src DIR_controllerWatchdog)
aux_source_directory(src/common COMMON_SRC)

add_executable(${PROJECT_NAME}_controller_watchdog ${DIR_controllerWatchdog} ${COMMON_SRC})
set_target_properties(${PROJECT_NAME}_controller_watchdog PROPERTIES OUTPUT_NAME controller_watchdog PREFIX "")
target_link_libraries(${PROJECT_NAME}_controller_watchdog -pthread ${Boost_LIBRARIES} ${LCM} ${catkin_LIBRARIES})
