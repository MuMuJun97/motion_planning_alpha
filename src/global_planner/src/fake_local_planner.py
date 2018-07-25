#!/usr/bin/env python
import rospy

from autopilot_msgs.msg import *

pub = rospy.Publisher("/planner/way_points", WayPoints, queue_size=1)

desired_speed = 1.38


def global_planning_result_callback(route_path):
    rospy.loginfo("Global Path Received")

    way_points = WayPoints()
    way_points.header.stamp = rospy.Time.now()
    way_points.points = route_path.goals
    way_points.speeds = [desired_speed] * len(way_points.points)

    pub.publish(way_points)


if __name__ == '__main__':
    rospy.init_node('fake_local_planner')
    subscriber = rospy.Subscriber("/route/path", RoutePath, global_planning_result_callback)
    rospy.spin()
