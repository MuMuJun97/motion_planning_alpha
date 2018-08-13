#!/bin/bash

catkin_make

echo "source ~/trouble/motion_planning_alpha/devel/setup.bash" >> ~/.bashrc

sleep 1s
{
gnome-terminal -t "LAUNCH ALL" --tab -- bash -c "roslaunch launch/motion_planning.launch"
}&

sleep 5s
{
gnome-terminal -t "PLAY 3D-points-map" --tab -- bash -c "rosbag play ../dataset/2017-07-24-20-37-00.bag"
}&

sleep 1s
{
gnome-terminal -t "RVIZ" --tab -- bash -c "rviz"
}&