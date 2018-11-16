#!/bin/bash

source devel/setup.bash

#rostopic pub /route/goal autopilot_msgs/RoutePath '{stamp: now, frame_id: map}' '[{latitude: -0.0014948, longitude: -0.000192, x: 175.448605157, y: -327.772607166}]' '[0.0]'

rostopic pub /route/goal autopilot_msgs/RoutePath '{stamp: now, frame_id: map}' '[{latitude: -0.0005519, longitude: -0.0009406, x: 92.253048185, y: -223.685044387}]' '[0.0]'
