#!/bin/bash

source devel/setup.bash

rostopic pub /route/goal std_msgs/Int64 54
