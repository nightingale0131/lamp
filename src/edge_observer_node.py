#!/usr/bin/env python

"""
Node for doing checks on the costmap that move_base won't do
"""

import rospy, rospkg
import os
import shapely.geometry as sh
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import policy.utility as util
from policy import tgraph

from policy.msg import EdgeUpdate
from geometry_msgs.msg import Point, Polygon, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

PADDING = 0.3 # must be greater than xy_goal_tolerance

class EdgeObserver():
    def __init__(self, base_graph):
        rospy.init_node('check_costmap')

        self.base_graph = base_graph
        self.costmap = None
        self.robot_pose = self.base_graph.pos('s')
        self.robot_range = self.get_robot_range()

        self.map_sub = rospy.Subscriber("move_base/global_costmap/costmap", OccupancyGrid, self.map_callback, queue_size=1, buff_size=2**24)
        self.pose_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped,
                                                self.pose_callback, queue_size=1)

        self.edge_state_pub = rospy.Publisher("policy/edge_update", EdgeUpdate,
                                              queue_size=10)

        self.edge_updater()

    def map_callback(self, data):
        # keep costmap updated
        self.costmap = data

    def pose_callback(self, data):
        # keep robot location updated
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.robot_pose = (x,y)

    def get_robot_range(self):
        # select range of robot's sensors, set by move_base parameters
        obst_range = rospy.get_param('/move_base/global_costmap/obstacles_layer/scan/obstacle_range')
        clear_range = rospy.get_param('/move_base/global_costmap/obstacles_layer/scan/raytrace_range')

        return min(obst_range, clear_range)

    def edge_updater(self):
        # publishes to edge_state
        rate = rospy.Rate(10)   # 10 Hz
        while not rospy.is_shutdown():
            # check that costmap subscriber has started receiving messages
            if self.costmap == None:
                continue

            for (u,v) in self.base_graph.edges():
                dist_to_u = util.euclidean_distance(self.base_graph.pos(u),
                                                    self.robot_pose)
                dist_to_v = util.euclidean_distance(self.base_graph.pos(v),
                                                    self.robot_pose)

                if (dist_to_u <= self.robot_range) and (dist_to_v <= self.robot_range):
                    edge_state = self.check_edge(u,v)

                    # prepare message
                    msg = EdgeUpdate(str(u), str(v), edge_state)
                    self.edge_state_pub.publish(msg)
                    rate.sleep() # not sure if this is the right place to put this

    def check_edge(self, u, v):
        polygon = self.base_graph.get_polygon(u,v)
        bufpoly = polygon.buffer(PADDING)
        submap = SubMap(self.costmap, bufpoly)
        bounds = [submap.minx, submap.maxx, submap.miny, submap.maxy]
        rospy.loginfo("Submap height: {}, width: {}".format(submap.height, submap.width))

        (startx, starty) = self.base_graph.pos(u)
        (goalx, goaly) = self.base_graph.pos(v)

        startpx = submap.cell(startx, starty)
        goalpx = submap.cell(goalx, goaly)

        rospy.loginfo("Searching for path from {} to {}...".format(u,v))

        # TODO: check if area around start or goal is not completely occupied
        if not (submap.passable(startpx) and submap.passable(goalpx)):
            rospy.loginfo("No path found because start or goal not valid!\n")
            return tgraph.BLOCKED


        # see if A* returns a valid path
        came_from, cost_so_far = util.a_star_search(submap, startpx, goalpx)
        rospy.loginfo("Finished search")

        try:
            self.path = util.reconstruct_path(came_from, startpx, goalpx)
        except KeyError:
            rospy.loginfo("No path found!\n")
            return tgraph.BLOCKED

        rospy.loginfo("Path found!\n")
        return tgraph.UNKNOWN # should return original state, need to update map

    def point_to_tuple(self, point):
        return "({:.2f},{:.2f})".format(point.x, point.y)

class SubMap():
    # ASSUMES POLYGON IS BOX!!! Cannot deal with other shaped polygons
    def __init__(self, costmap, polygon):
        # costmap -> nav_msgs/OccupancyGrid msg 
        # polygon -> shapely polygon

        boundary = polygon
        (self.minx, self.miny, self.maxx, self.maxy) = boundary.bounds # m
        self.res = costmap.info.resolution       # m/cell

        origin = costmap.info.origin.position   # m 
        map_width = costmap.info.width          # cell
        map_height = costmap.info.height        # cell

        leftpx = max(0, int((self.minx - origin.x)/self.res))
        rightpx = min(map_width - 1, int((self.maxx - origin.x)/self.res))
        toppx = max(0, int((self.miny - origin.y)/self.res))
        botpx = min(map_height - 1, int((self.maxy - origin.y)/self.res))

        rospy.loginfo("Extraction site: left: {}, right: {}, top: {}, bottom: {}"
                .format(leftpx, rightpx, toppx, botpx))

        # do some validation
        assert (leftpx < map_width and rightpx >= 0
                and toppx < map_height and botpx >=0), (
                "Submap is completely outside of costmap")

        self.width = rightpx - leftpx + 1
        self.height = botpx - toppx + 1
        self.grid = np.empty((self.height, self.width))

        # fill in rows of grid
        for row in range(toppx, botpx + 1):
            self.grid[(row - toppx), :] = costmap.data[(row*map_width + leftpx):(row*map_width +
                rightpx + 1)]

    def in_bounds(self, cell):
        (row, col) = cell
        if (row >= 0 and row < self.height) and (col >= 0 and col < self.width):
            return True
        else:
            return False

    def passable(self, cell):
        (row, col) = cell
        
        if not self.in_bounds(cell): return False
        elif self.grid[row,col] >= 99: return False
        else: return True

    def cell(self, x, y):
        # (x,y) - cartesian coordinates
        assert (x >= self.minx and x <= self.maxx), (
                "x: {:.3f}, minx: {:.3f}, maxx: {:.3f}".format(x, self.minx, self.maxx))
        assert (y >= self.miny and y <= self.maxy), (
                "y: {:.3f}, miny: {:.3}, maxy: {:.3f}".format(y, self.miny, self.maxy))

        col = int((x - self.minx)/self.res)
        row = int((y - self.miny)/self.res)

        return (row, col)

    def neighbours(self, cell):
        # 4-way grid, return adjacent cells that are not occupied
        (row, col) = cell
        temp = [(row-1, col), (row+1, col), (row, col-1), (row, col+1)] 
        neighbours = []

        for possible_neighbour in temp:
            if self.passable(possible_neighbour): neighbours.append(possible_neighbour)

        return neighbours

    def weight(self, cell1, cell2):
        (r1, c1) = cell1
        (r2, c2) = cell2

        if not (self.passable(cell1) and self.passable(cell2)): return float('inf')

        if (abs(c1 - c2) == 1 or abs(r1 - r2) == 1):
            return 1 + self.grid[r1, c1] + self.grid[r2,c2] 
        else: return float('inf')

    def dist(self, cell1, cell2):
        # heuristic for A*, cell1 and cell2 don't have to be neighbours

        if not (self.passable(cell1) and self.passable(cell2)): return float('inf')

        return util.euclidean_distance(cell1, cell2)


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    pkgdir = rospack.get_path('policy')
    graph = nx.read_yaml(pkgdir + '/tests/tristan_maze_tgraph.yaml')
    poly_dict = tgraph.polygon_dict_from_csv(pkgdir + '/tests/tristan_maze_polygons.csv')

    base_graph = tgraph.TGraph(graph, poly_dict)

    try:
        EdgeObserver(base_graph)
    except rospy.ROSInterruptException:
        pass
