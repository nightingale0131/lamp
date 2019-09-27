#!/usr/bin/env python

"""
Node for doing checks on the costmap that move_base won't do
"""

import rospy, rospkg
import os
import math
import shapely.geometry as sh
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import policy.utility as util
from policy.tgraph import TGraph, polygon_dict_from_csv
from policy import visibility as vis
from copy import copy

from policy.msg import EdgeUpdate, CurrEdge
from geometry_msgs.msg import Point, Polygon, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

from std_srvs.srv import *

PADDING = 0.3 # must be greater than xy_goal_tolerance
PKGDIR = rospkg.RosPack().get_path('policy')
# MAP = 'tristan_maze'
MAP = 'test_large'

class EdgeObserver():
    def __init__(self, base_graph):
        # assumes base_graph starts with 's'
        rospy.init_node('edge_observer')

        self.base_graph = base_graph
        # self.curr_graph = copy(self.base_graph) 
        self.costmap = None
        self.vprev = 's'
        self.vnext = None
        self.vis_v_in_submap = set() 
        self.curr_submap = None
        self.robot_pose = None
        self.robot_range = self.get_robot_range()

        self.map_sub = rospy.Subscriber("move_base/global_costmap/costmap", OccupancyGrid, self.map_callback, queue_size=1, buff_size=2**24)
        self.pose_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped,
                                                self.pose_callback, queue_size=1)
        self.v_sub = rospy.Subscriber("policy/current_edge", CurrEdge, self.v_callback, queue_size=1)

        self.edge_state_pub = rospy.Publisher("policy/edge_update", EdgeUpdate,
                                              queue_size=10)

        # s = rospy.Service('reset_edge_observer', Empty, self.reset_graph_service)

        self.edge_updater()
        # test visibility here

    def map_callback(self, data):
        # keep costmap updated
        self.costmap = data

    def pose_callback(self, data):
        # keep robot location updated
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        self.robot_pose = (x,y)

    def v_callback(self, data):
        # vertex robot is leaving
        u = data.vprev
        if u.isdigit(): u = int(u)

        v = data.vnext
        if v.isdigit(): v = int(v)

        # maintain current submap and visible portals
        if self.vprev != u or self.vnext != v:
            self.vprev = u
            self.vnext = v

            if self.vprev != self.vnext:
                # reset the 'visible' portals/vertices when curr_submap changes
                rospy.loginfo("Changed submaps, resetting list of visible vertices")
                self.curr_submap = self.base_graph.get_polygon(self.vprev, self.vnext)
                self.vis_v_in_submap = set()

    def get_robot_range(self):
        # select range of robot's sensors, set by move_base parameters
        obst_range = rospy.get_param('/move_base/global_costmap/obstacles_layer/scan/obstacle_range')
        clear_range = rospy.get_param('/move_base/global_costmap/obstacles_layer/scan/raytrace_range')

        return min(obst_range, clear_range)

    def edge_updater(self):
        # publishes to edge_state
        rate = rospy.Rate(10)   # Hz
        while not rospy.is_shutdown():
            # check that costmap subscriber has started receiving messages
            if self.costmap == None or self.robot_pose == None:
                continue

            rospy.loginfo("-----")
            rospy.loginfo("Calculating visibility polygon...")
            self.set_visibility_polygon()

            rospy.loginfo("Checking edges...")
            for (u,v) in self.base_graph.edges():
                dist_to_u = util.euclidean_distance(self.base_graph.pos(u),
                                                    self.robot_pose)
                dist_to_v = util.euclidean_distance(self.base_graph.pos(v),
                                                    self.robot_pose)

                edge_in_range = ((dist_to_u <= self.robot_range) and 
                                 (dist_to_v <= self.robot_range))

                edge_in_submap = (self.curr_submap == self.base_graph.get_polygon(u,v))

                if edge_in_range or edge_in_submap:
                    edge_state = self.check_edge(u,v)

                    # prepare message
                    msg = EdgeUpdate(str(u), str(v), edge_state)
                    self.edge_state_pub.publish(msg)
                    rate.sleep() # not sure if this is the right place to put this

    def check_edge(self, u, v):
        rospy.loginfo("-----")

        polygon = self.base_graph.get_polygon(u,v)
        bufpoly = polygon.buffer(PADDING)
        submap = SubMap(self.costmap, bufpoly)
        bounds = [submap.minx, submap.maxx, submap.miny, submap.maxy]
        rospy.loginfo("Submap height: {}, width: {}".format(submap.height, submap.width))

        u_is_visible = self.visible(u)
        v_is_visible = self.visible(v)

        # if both vertices are visible, return UNBLOCKED, 
        # if one vertex is visible and the other one is where we came from, return UNBLOCKED
        # otherwise search for path

        if u_is_visible and v_is_visible:
            return TGraph.UNBLOCKED
        elif self.vprev == u and v_is_visible:
            return TGraph.UNBLOCKED
        elif self.vprev == v and u_is_visible:
            return TGraph.UNBLOCKED

        (startx, starty) = self.base_graph.pos(u)
        (goalx, goaly) = self.base_graph.pos(v)

        startpx = submap.cell(startx, starty)
        goalpx = submap.cell(goalx, goaly)

        rospy.loginfo("Robot leaving from {}".format(self.vprev))
        rospy.loginfo("Searching for path from {} to {}...".format(u,v))

        # TODO: check if area around start or goal is not completely occupied
        if not (submap.passable(startpx) and submap.passable(goalpx)):
            rospy.loginfo("No path found because start or goal not valid!")
            return TGraph.BLOCKED

        # see if A* returns a valid path
        came_from, cost_so_far = util.a_star_search(submap, startpx, goalpx)
        rospy.logdebug("Finished search")

        try:
            self.path = util.reconstruct_path(came_from, startpx, goalpx)
        except KeyError:
            rospy.loginfo("Path NOT FOUND!")
            return TGraph.BLOCKED

        rospy.loginfo("Path FOUND!")

        return TGraph.UNKNOWN

    def set_visibility_polygon(self):
        # 1) get submap of robot's visible range 
        (x,y) = self.robot_pose
        rospy.logdebug("robot loc: {:.2f}, {:.2f}".format(x,y))

        vis_box = sh.box(x - self.robot_range, y - self.robot_range,
                        x + self.robot_range, y + self.robot_range)
        vis_submap = SubMap(self.costmap, vis_box)
        # print("minx={:.5f}, maxx={:.5f}, miny={:.5f}, maxy={:.5f}"
              # .format(vis_submap.minx, vis_submap.maxx, 
                      # vis_submap.miny, vis_submap.maxy))

        # 2) extract obstacles in submap
        obstacles = vis.find_obstacles(vis_submap.grid, thresh=50, unknown=-1)

        # 3) get obsv pt in vis_submap
        (oy, ox) = vis_submap.cell(x,y)

        # 4) get visibility polygon
        vis_poly = vis.visibility_polygon(ox, oy, obstacles)
        # vis.save_print(vis_poly)

        # 5) convert visibility polygon to shapely polygon, in the right frame
        self.vis_poly = self.to_shapely(vis_poly, vis_submap)

        # for debugging
        # vis.draw(vis_submap.grid, vis_poly, (ox,oy), unknown=-1)

    def visible(self, vertex):
        # checks if vertex is visible to the robot
        location = self.base_graph.pos(vertex) # convert vertex to coordinates
        location = sh.Point(location)
        v_is_visible =  self.vis_poly.contains(location)
        result = False

        # debugging
        rospy.logdebug("{} ({}) is visible: {}"
                      .format(vertex, list(location.coords), v_is_visible))

        if v_is_visible:
            if vertex in self.base_graph.get_vertices_in_polygon(self.curr_submap):
                rospy.loginfo("{} added to visible portals in submap".format(vertex))
                self.vis_v_in_submap.add(vertex)
            result = True
        elif vertex in self.vis_v_in_submap:
            rospy.logdebug("{} was visible in submap".format(vertex))
            # if vertex was visible at some point while traversing current submap
            result = True
        else:
            result = False

        rospy.loginfo("{} ({}) visibility: {}"
                      .format(vertex, list(location.coords), result))

        return result

    def to_shapely(self, vis_poly, vis_submap):
        boundary = []
        rospy.logdebug("Converted visibility polygon:")
        for i in range(vis_poly.n()):
            px = vis_poly[i].x()
            py = vis_poly[i].y()
            (x,y) = vis_submap.coord(py, px)
            boundary.append((x,y))

            rospy.logdebug(" {:.2f}, {:.2f}".format(x, y))

        return sh.Polygon(boundary)

    def point_to_tuple(self, point):
        return "({:.2f},{:.2f})".format(point.x, point.y)
    """
    def reset_graph_service(self, req):
        # req is empty, just triggers this service
        self.curr_graph = copy(self.base_graph) 
        rospy.loginfo('Reset graph')
        return {}
    """

class SubMap():
    # ASSUMES POLYGON IS BOX!!! Cannot deal with other shaped polygons
    def __init__(self, costmap, polygon):
        # costmap -> nav_msgs/OccupancyGrid msg 
        # polygon -> shapely polygon

        boundary = polygon
        # (self.minx, self.miny, self.maxx, self.maxy) = boundary.bounds # m
        (minx, miny, maxx, maxy) = boundary.bounds # m
        self.res = costmap.info.resolution       # m/cell

        origin = costmap.info.origin.position   # m 
        map_width = costmap.info.width          # cell
        map_height = costmap.info.height        # cell

        # min/max adjustments
        self.minx = max(minx, origin.x)
        self.maxx = min(maxx, origin.x + map_width*self.res)
        self.miny = max(miny, origin.y)
        self.maxy = min(maxy, origin.y + map_height*self.res)

        leftpx = int((self.minx - origin.x)/self.res)
        rightpx = int((self.maxx - origin.x)/self.res)
        toppx = int((self.miny - origin.y)/self.res)
        botpx = int((self.maxy - origin.y)/self.res)

        # to avoid 
        if rightpx >= map_width: rightpx = map_width - 1
        if botpx >= map_height: botpx = map_height - 1

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
            try:
                self.grid[(row - toppx), :] = costmap.data[(row*map_width + 
                                                            leftpx):(row*map_width +
                                                                     rightpx + 1)]
            except ValueError, e:
                # fill in with unblocked 
                self.grid[(row - toppx), :] = np.zeros(self.width)
                rospy.logwarn("{}".format(e))
                rospy.logwarn("row: {}, map_width: {}, map_height: {}".format(row,
                                                                              map_width,
                                                                              map_height))

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

    def coord(self, row, col):
        # given pixel, return coord of center of pixel
        c = int(col)
        r = int(row)
        assert (c >= 0 and c < self.width), (
            "col: {}, width: {}".format(c, self.width))
        assert (r >= 0 and r < self.height), (
            "row: {}, height: {}".format(r, self.height))

        x = c*self.res + self.minx + self.res/2
        y = r*self.res + self.miny + self.res/2

        return (x,y)

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
            w1 = self.grid[r1, c1]
            w2 = self.grid[r2, c2]

            assert(w1 >= 0 and w2 >= 0),("({},{}): {}, ({},{}): {}"
                                         .format(r1,c1,w1,r2,c2,w2))
            return 1 + w1 + w2
        else: return float('inf')

    def dist(self, cell1, cell2):
        # heuristic for A*, cell1 and cell2 don't have to be neighbours

        if not (self.passable(cell1) and self.passable(cell2)): return float('inf')

        return util.euclidean_distance(cell1, cell2)


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    pkgdir = rospack.get_path('policy')
    graph = nx.read_yaml(PKGDIR + '/maps/' + MAP + '/' + MAP + '_tgraph.yaml')
    poly_dict = polygon_dict_from_csv(PKGDIR + '/maps/' + MAP + '/' + MAP + '_polygons.csv')

    base_graph = TGraph(graph, poly_dict)

    try:
        EdgeObserver(base_graph)
    except rospy.ROSInterruptException:
        pass
