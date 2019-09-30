#!/usr/bin/env python
"""
OBSOLETE - needs to be updated
"""

import rospy
from geometry_msgs.msg import Point, Polygon, box
import shapely.geometry as sh
import cv2

from policy import visibility as vis

def check_edge_client(start, goal, polygon):
    rospy.wait_for_service('check_edge')
    try:
        check_edge = rospy.ServiceProxy('check_edge', CheckEdge)
        result = check_edge(start, goal, polygon)
        return result.result
    except rospy.ServiceException, e:
        print("Service call failed: {}".format(e))

if __name__ == "__main__":
    start = Point(8.20,2,0)
    goal = Point(2.5,2,0)

    poly_list = [Point(9.8,-2.3,0),
            Point(-2.3,-2.3,0),
            Point(-2.3,2.3,0),
            Point(9.8,2.3,0)]

    bad_poly_list = [Point(-5,7,0),
            Point(-4,7,0),
            Point(-4,4,0),
            Point(-5,4,0)]

    poly = Polygon(poly_list)

    # check_edge_client(start, goal, poly)
