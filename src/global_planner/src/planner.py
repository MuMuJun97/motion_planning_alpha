#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt

from autopilot_msgs.msg import *

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import libGaussLocalGeographicCS as MapServer

list_edges = []
list_points = []
list_goals = []
ini_location = -1
Path = []  # Globle Path
alpha = 0.0085
beta = 0.0017


def draw_map(p, e):
    for i in e:
        plt.plot([p[i[0]][0], p[i[1]][0]], [p[i[0]][1], p[i[1]][1]])
    plt.plot(11296.2177734, 8746.04199219, 'bo')
    plt.plot(11290.4667969, 8706.98730469, 'bo')
    plt.savefig('map')
    plt.show()


def draw_path(pth):
    for i in list_edges:
        plt.plot([list_points[i[0]][0], list_points[i[1]][0]], [list_points[i[0]][1], list_points[i[1]][1]])
    for p in pth:
        plt.plot(list_points[p][0], list_points[p][1], 'bo')
    plt.savefig('path')
    plt.show()


def Map_callback(RouteMap):
    print("RouteMap callback")
    global list_goals
    list_goals = []
    for e in RouteMap.edges:
        # print(e.begin.x)
        if [e.begin.x, e.begin.y, e.begin.latitude, e.begin.longitude] not in list_points:
            list_points.append([e.begin.x, e.begin.y, e.begin.latitude, e.begin.longitude])
        if [e.end.x, e.end.y, e.end.latitude, e.end.longitude] not in list_points:
            list_points.append([e.end.x, e.end.y, e.end.latitude, e.end.longitude])
    # print(list_points)

    for e in RouteMap.edges:
        list_edges.append([list_points.index([e.begin.x, e.begin.y, e.begin.latitude, e.begin.longitude]),
                           list_points.index([e.end.x, e.end.y, e.end.latitude, e.end.longitude])])
    # print(list_edges)
    # print(list_points)
    print("points:", len(list_points))
    print("edges:", len(list_edges))
    # draw_map(list_points, list_edges)
    # find_Path(list_edges, 2, [28])


def Loc_callback(Location):
    print("Location callback")
    flag = True
    Location.x = Location.x / (1 + beta)
    Location.y = - Location.y / (1 + alpha)
    for i in range(len(list_points)):
        if Location.x == list_points[i][0] and Location.y == list_points[i][1]:
            flag = False
            global ini_location
            ini_location = list_points.index([Location.x, Location.y, Location.latitude, Location.longitude])
    if flag == True:
        # print("else\n")
        min_dis = 999999
        for i in range(len(list_points)):
            n = list_points[i]
            if np.sqrt((n[0] - Location.x) ** 2 + (n[1] - Location.y) ** 2) < min_dis:
                # print(np.sqrt((n[0]-Location.x)**2 + (n[1]-Location.y)**2))
                min_dis = np.sqrt((n[0] - Location.x) ** 2 + (n[1] - Location.y) ** 2)
                ini_location = i
    print("Location finish")


def Path_callback(RoutePath):
    print("RoutePath callback")
    for n in RoutePath.goals:
        if [n.x, n.y, n.latitude, n.longitude] in list_points:
            list_goals.append(list_points.index([n.x, n.y, n.latitude, n.longitude]))
        else:
            min_dis = 9999999
            temp_index = -1
            for i in range(len(list_points)):
                if (list_points[i][0] - n.x) ** 2 + (list_points[i][1] - n.y) ** 2 < min_dis:
                    temp_index = i
                    min_dis = (list_points[i][0] - n.x) ** 2 + (list_points[i][1] - n.y) ** 2
            list_goals.append(temp_index)

    print("ini_location", ini_location)

    if len(list_edges) != 0 and len(list_points) != 0 and len(list_goals) != 0 and ini_location != -1:
        find_Path(list_edges, ini_location, list_goals)
        talker()
    print("RoutePath finish")


def listener():
    rospy.Subscriber('/map/route_map', RouteMap, Map_callback)
    rospy.Subscriber('/localization/location', Location, Loc_callback)
    rospy.Subscriber('/route/goal', RoutePath, Path_callback)


def talker():
    pub = rospy.Publisher('/route/path', RoutePath, queue_size=5000, latch=True)
    motion_goal_pub = rospy.Publisher( '/motion/goal', RouteNode, queue_size=5000, latch=True )
    visualization_pub = rospy.Publisher('/route/path_visualization', MarkerArray, queue_size=5000, latch=True)
    path_arrows = MarkerArray()
    rp = RoutePath()
    final_path = rp.goals
    last_i = -1
    for i in Path:
        node = RouteNode()

        node.x = list_points[i][0] * (1 + beta)
        node.y = - list_points[i][1] * (1 + alpha)
        node.latitude = list_points[i][2]
        node.longitude = list_points[i][3]
        final_path.append(node)
        if last_i >= 0:
            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.header.stamp = rospy.Time.now()
            arrow.ns = "global_planner"
            arrow.id = last_i
            arrow.type = Marker.ARROW
            start_point = Point()
            end_point = Point()

            start_point.x = list_points[last_i][0]
            start_point.y = list_points[last_i][1]
            start_point.z = 1.0
            end_point.x = list_points[i][0]
            end_point.y = list_points[i][1]
            end_point.z = 1.0

            arrow.points.append(start_point)
            arrow.points.append(end_point)

            length = np.sqrt(np.square(start_point.x - end_point.x) + np.square(start_point.y - end_point.y))
            arrow.scale.x = length / 10.0
            arrow.scale.y = length / 5.0
            arrow.color.a = 1.0
            arrow.color.r = 1.0
            path_arrows.markers.append(arrow)
        last_i = i

    rp.headings = [0.0] * len(rp.goals)
    for pn in final_path:
        rospy.loginfo('Global path is:{}, {}'.format(pn.x, pn.y))

    motion_goal = RouteNode()
    motion_goal = final_path[-1]
    pub.publish(rp)
    visualization_pub.publish(path_arrows)
    motion_goal_pub.publish(motion_goal)


class Edge:
    def __init__(self, source, destine):
        # print(list_points)
        self.weight = np.sqrt((list_points[source][0] - list_points[destine][0]) ** 2 + (
            list_points[source][1] - list_points[destine][1]) ** 2)
        self.source = source
        self.destine = destine


'''
construct a Graph
'''


class Graph:
    def __init__(self):
        self.vertices = set([])
        self.edges = set([])
        self.adjacents = {}

    def add_edge(self, edge):
        self.vertices.add(edge.source)
        self.vertices.add(edge.destine)
        if edge.source not in self.adjacents.keys():
            self.adjacents[edge.source] = set([])
        self.adjacents[edge.source].add(edge)
        self.edges.add(edge)
        # print("add edge from {} to {}, weight {}".format(edge.source, edge.destine, edge.weight))

    def get_adjacents(self, vertex):
        # print("get the adjacent vertices of vertex {}".format(vertex))
        if vertex not in self.adjacents.keys():
            return set([])
        return self.adjacents[vertex]

    def vertex_number(self):
        return len(self.vertices)

    def edge_number(self):
        return len(self.edges)


'''
min Priority Queue
'''


class MinPQ:
    def __init__(self):
        self.queue = [(0, 0)]
        # print("create a min Priority Queue to record the distance")

    def is_empty(self):
        return len(self.queue) == 1

    def size(self):
        return len(self.queue) - 1

    def min(self):
        return self.queue[1]

    def insert(self, vertex, new_value):
        self.queue.append((vertex, new_value))
        self.swim(self.size())

    def del_min(self):
        self.queue[1], self.queue[-1] = self.queue[-1], self.queue[1]
        temp = self.queue.pop(-1)
        self.sink(1)
        return temp

    def swim(self, index):
        while index > 1 and self.queue[index // 2][1] > self.queue[index][1]:
            self.queue[index // 2], self.queue[index] = self.queue[index], self.queue[index // 2]
            index = index // 2

    def sink(self, index):
        while 2 * index <= self.size():
            next_level = 2 * index
            if next_level < self.size() and self.queue[next_level][
                1] > self.queue[next_level + 1][1]:
                next_level += 1
            if self.queue[index][1] <= self.queue[next_level][1]:
                return
            self.queue[index], self.queue[next_level] = self.queue[next_level], self.queue[index]
            index = next_level

    def contains(self, vertex):
        for index in range(1, len(self.queue)):
            if self.queue[index][0] == vertex:
                return True
        return False

    def change_dist(self, vertex, new_dist):
        for index in range(1, len(self.queue)):
            if self.queue[index][0] == vertex:
                self.queue[index] = (vertex, new_dist)


class ShortestPath:
    def __init__(self, graph, start_point):
        self.dist_to = {}
        self.edge_to = {}
        for vertex in graph.vertices:
            self.dist_to[vertex] = float("inf")
        self.dist_to[start_point] = start_point
        self.start_point = start_point
        self.dist_queue = MinPQ()
        self.dist_queue.insert(start_point, self.dist_to[start_point])
        # print("insert the start point into the priority queue and initialize the distance")
        while not self.dist_queue.is_empty():
            vertex, _ = self.dist_queue.del_min()
            # print("grow the mini-distance tree by poping vertex {} from the queue".format(vertex))
            for edge in graph.get_adjacents(vertex):
                self.relax(edge)

    def relax(self, edge):
        # print("relax edge from {} to {}".format(edge.source, edge.destine))
        source = edge.source
        destine = edge.destine
        if self.dist_to[destine] > self.dist_to[source] + edge.weight:
            self.dist_to[destine] = self.dist_to[source] + edge.weight
            self.edge_to[destine] = edge
            if self.dist_queue.contains(destine):
                self.dist_queue.change_dist(destine, self.dist_to[destine])
            else:
                self.dist_queue.insert(destine, self.dist_to[destine])

    def dist_to(self, vertex):
        return self.dist_to[vertex]

    def path_to(self, vertex):
        lPath = [vertex]
        temp_vertex = vertex
        while temp_vertex != self.start_point:
            print("temp_vertex", temp_vertex)
            print("start_point", self.start_point)
            print("edge_to[temp_vertex].source", self.edge_to[temp_vertex].source)
            temp_vertex = self.edge_to[temp_vertex].source
            lPath.append(temp_vertex)
        lPath.reverse()
        return lPath


def find_Path(list_Graph, location, list_Goals):
    global Path
    Path = []
    test = Graph()  # Create an  graph
    for item in list_Graph:
        test.add_edge(Edge(item[0], item[1]))
    now_point = location
    for item in list_Goals:
        path = ShortestPath(test, now_point)
        distPath = path.path_to(item)
        for i in range(len(distPath) - 1):
            Path.append(distPath[i])
        now_point = item
    Path.append(list_Goals[-1])
    print("the Path is :")
    print(Path)
    print("in other words:")
    for i in range(len(Path)):
        print(list_points[Path[i]])
        # draw_path(Path)


def tet():
    # start_point end_point weight
    edge_1 = [1, 2]
    edge_2 = [2, 5]
    edge_3 = [2, 4]
    edge_4 = [2, 3]
    edge_5 = [3, 4]
    edge_6 = [3, 6]
    edge_7 = [4, 6]
    edge_8 = [6, 1]
    list_edge = [edge_1, edge_2, edge_3, edge_4, edge_5, edge_6, edge_7, edge_8]
    list_goals = [3, 6]
    find_Path(list_edge, 1, list_goals)


if __name__ == '__main__':
    rospy.init_node('global_planner')
    listener()
    rospy.spin()
    # test()
