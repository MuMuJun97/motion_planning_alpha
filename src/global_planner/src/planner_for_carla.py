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
from carla.planner.map import CarlaMap

source = ()
source_ori = ()
target_number = 11
route = []
_player_start_spots = []
city_name = 'Town01'

_city_track = city_track.CityTrack(city_name)

def find_path(source, source_ori, target, target_ori):
    rospy.loginfo("Starting find global route")
    global route
    track_source = _city_track.project_node(source)
    track_target = _city_track.project_node(target)
    route = _city_track.compute_route(track_source, source_ori, 
                                      track_target, target_ori)
    rospy.loginfo("Finished find global route")
    talker()

def loc_callback(odometry):
    rospy.loginfo("Starting get current location")
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
    rospy.loginfo("Finished get current location")


def path_callback(requested_goal):
    rospy.loginfo("Starting get target from player start spots")
    global target_number
    target_number = 11
    target = (
        _player_start_spots[target_number][0],
        _player_start_spots[target_number][1],
        _player_start_spots[target_number][2],
    )
    rospy.loginfo(target)
    rospy.loginfo(source)
    rospy.loginfo(source_ori)
    target_ori = (0,0,0)
    rospy.loginfo("Finished get target from player start spots")
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
    rospy.loginfo("Finished get player start spots")


def listener():
    rospy.Subscriber('player_odometry', Odometry, loc_callback)
    rospy.Subscriber('/route/goal', RoutePath, path_callback)
    rospy.Subscriber('player_start_spots',Float64MultiArray,
                     get_start_spots)


def talker():
    rospy.loginfo("Starting publish global route to motion planner")
    pub = rospy.Publisher('/route/path', RoutePath, queue_size=500, latch=True)
    rp = RoutePath()
    for node in route:
        world_point = CarlaMap(city_name).convert_to_world(node)
        route_node = RouteNode()
        route_node.x = world_point[0]
        route_node.y = world_point[1]
        rp.goals.append(route_node)
    rp.headings = [0.0] * len(rp.goals)
    rospy.loginfo(rp.goals)
    pub.publish(rp)
    rospy.loginfo("Finished publish global route to motion planner")

if __name__ == '__main__':
    rospy.init_node('global_planner')
    listener()
    rospy.spin()