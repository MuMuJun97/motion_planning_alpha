#!/usr/bin/env python
import rospy
import collections
import math
import tf

import numpy as np
import matplotlib.pyplot as plt

from carla.planner import city_track
from autopilot_msgs.msg import *
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray

from carla.client import make_carla_client
from carla.planner.map import CarlaMap
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError

source = ()
source_ori = ()
target_number = 11
route = []
_player_start_spots = []

_city_track = city_track.CityTrack(city_name = 'Town01')

def find_path(source, source_ori, target, target_ori):
    global route
    track_source = _city_track.project_node(source)
    track_target = _city_track.project_node(target)
    route = _city_track.compute_route(track_source, source_ori, 
                                      track_target, target_ori)
    print(route)
    talker()

def loc_callback(odometry):
    rospy.loginfo("Location callback")
    global source_ori, source
    source = (
        odometry.pose.pose.position.x,
        odometry.pose.pose.position.y,
        odometry.pose.pose.position.z
    )
    quat = (
        odometry.pose.pose.orientation.x,
        odometry.pose.pose.orientation.y,
        odometry.pose.pose.orientation.z,
        odometry.pose.pose.orientation.w
    )
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
    source_ori = (roll, pitch, yaw)


def path_callback(requested_goal):
    rospy.loginfo("Path callback")
    rospy.loginfo(_player_start_spots)
    global target_number
    target_number = 2
    target = (
        _player_start_spots[target_number][0],
        _player_start_spots[target_number][1],
        _player_start_spots[target_number][2],
    )
    target_ori = (0,0,0)
    find_path(source, source_ori, target, target_ori)

def get_start_spots(Float64MultiArray):
    rospy.loginfo("Starting get player start spots")
    global _player_start_spots
    _player_start_spots = []
    height = Float64MultiArray.layout.dim[0].size
    width = Float64MultiArray.layout.dim[1].size
    for h in range(height):
        spot = [ Float64MultiArray.data[h * width + _] for _ in range(width) ]
        _player_start_spots.append(spot)
    rospy.loginfo(_player_start_spots)
    rospy.loginfo("Finished get player start spots")


def listener():
    rospy.Subscriber('player_odometry', Odometry, loc_callback)
    rospy.Subscriber('/route/goal', RoutePath, path_callback)
    rospy.Subscriber('player_start_spots',Float64MultiArray,
                     get_start_spots)


def talker():
    pub = rospy.Publisher('/route/path', RoutePath, queue_size=500, latch=True)
    rp = RoutePath()
    for (x, y) in route:
        node = RouteNode()
        node.x = x
        node.y = y
        rp.goals.append(node)
    rp.headings = [0.0] * len(rp.goals)
    pub.publish(rp)

if __name__ == '__main__':
    rospy.init_node('global_planner')
    listener()
    rospy.spin()