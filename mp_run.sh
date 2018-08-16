#!/bin/bash

catkin_make

source ~/trouble/motion_planning_alpha/devel/setup.bash

{
gnome-terminal -t "roscore" -x bash -c "roscore;exec bash"
}&
 
sleep 1s
{
gnome-terminal -t "motion" --tab -- bash -c "rosrun motion_planner motion_planner;exec bash"
}&
 
sleep 1s
{
gnome-terminal -t "global" --tab -- bash -c "rosrun global_planner planner.py;exec bash"
}&

sleep 1s
{
gnome-terminal -t "map_server" --tab -- bash -c "roslaunch autopilot_map_server load_route_map_with_rviz.launch"
}&
 
sleep 1s
{
gnome-terminal -t "routmap_bag" --tab -- bash -c "cd src/global_planner/; rosbag play routemap.bag;exec bash"
}&
 
sleep 1s
{
gnome-terminal -t "grid_map" --tab -- bash -c "cd src/motion_planner/; rosbag play grip_map.bag;exec bash"
}&
 
sleep 1s
{
gnome-terminal -t "source" --tab -- bash -c "rostopic pub \
/localization/location autopilot_msgs/Location '{stamp: now, frame_id: map}' \
'11290.4667969' '8706.98730469' '[0.0, 0.0, 0.0, 0.0]' \
'23.067371' '113.3795204' '[0.0, 0.0, 0.0, 0.0]';exec bash"
}&

sleep 1s
{
gnome-terminal -t "target" --tab -- bash -c "rostopic pub \
/route/goal autopilot_msgs/RoutePath '{stamp: now, frame_id: map}' \
'[{latitude: 23.0677242, longitude: 113.3795769, \
x: 11296.2177734, y: 8786.04199219}]' '[0.0]';exec bash"
}