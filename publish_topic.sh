#!/bin/bash

source ~/ros_ws/motion_planning_alpha/devel/setup.bash

rostopic pub /route/goal std_msgs/Int64 54
