#!/usr/bin/env python

"""
Node for doing checks on the costmap that move_base won't do
"""

import rospy

from policy.srv import CheckEdge
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
        plt.imshow(submap.grid, cmap='gray', interpolation='bicubic', extent=bounds)
        plt.show()
        return True

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

        self.grid = np.empty((botpx - toppx + 1, rightpx - leftpx + 1))

        # fill in rows of grid
        for row in range(botpx - toppx + 1):
            self.grid[row, :] = costmap.data[(row*map_width + leftpx):(row*map_width +
                rightpx + 1)]

    def cell(self, x, y):
        # (x,y) - cartesian coordinates
        assert (x >= self.minx and x <= self.maxx), (
                "x: {:.2f}, minx: {:.2f}, maxx: {:.2f}".format(x, self.minx, self.maxx))
        assert (y >= self.miny and y <= self.maxy), (
                "y: {:.2f}, miny: {:.2f}, maxy: {:.2f}".format(y, self.miny, self.maxy))

        px = int((x - self.minx)/self.res)
        py = int((y - self.miny)/self.res)

        return (px, py)


if __name__ == '__main__':
    CheckCostmapNode()
