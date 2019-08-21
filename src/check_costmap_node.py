#!/usr/bin/env python

"""
Node for doing checks on the costmap that move_base won't do
"""

import rospy

from policy.srv import CheckEdge
import policy.utility as util
from geometry_msgs.msg import Point, Polygon
from nav_msgs.msg import OccupancyGrid
import shapely.geometry as sh
import numpy as np
import matplotlib.pyplot as plt

class CheckCostmapNode():
    def __init__(self):
        rospy.init_node('check_costmap')

        self.map_subscriber = rospy.Subscriber("move_base/global_costmap/costmap", OccupancyGrid, self.map_callback, queue_size=1, buff_size=2**24)

        s = rospy.Service('check_edge', CheckEdge, self.check_edge_handle)
        rospy.loginfo('Ready to check costmap')

        rospy.spin()

    def map_callback(self, data):
        # keep costmap updated
        self.costmap = data

    def check_edge_handle(self, req):
        submap = SubMap(self.costmap, req.polygon)
        bounds = [submap.minx, submap.maxx, submap.miny, submap.maxy]
        rospy.loginfo("Submap height: {}, width: {}".format(submap.height, submap.width))

        startpx = submap.cell(req.start.x, req.start.y)
        goalpx = submap.cell(req.goal.x, req.goal.y)

        # see if A* returns a valid path
        came_from, cost_so_far = util.a_star_search(submap, startpx, goalpx)

        try:
            self.path = util.reconstruct_path(came_from, startpx, goalpx)
        except KeyError:
            rospy.loginfo("No path in submap from {} to {}"
                          .format(self.point_to_tuple(req.start), self.point_to_tuple(req.goal)))
            return False

        # debugging code that should be removed later
        # plt.imshow(submap.grid, cmap='gray', interpolation='bicubic', extent=bounds)
        # plt.show()
        # col, row = submap.cell(0,0)
        # rospy.loginfo("grid[{},{}] = {}".format(row, col, submap.grid[row,col]))
        rospy.loginfo("Path exists in submap from {} to {}"
                          .format(self.point_to_tuple(req.start), self.point_to_tuple(req.goal)))
        return True

    def point_to_tuple(self, point):
        return "({:.2f},{:.2f},{:.2f})".format(point.x, point.y, point.z)

class SubMap():
    # ASSUMES POLYGON IS BOX!!! Cannot deal with other shaped polygons
    def __init__(self, costmap, polygon):
        # costmap -> nav_msgs/OccupancyGrid msg 
        # polygon -> geometry_msgs/Polygon msg

        boundary = sh.Polygon([(p.x, p.y) for p in polygon.points])
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
    CheckCostmapNode()
