#!/usr/bin/env python

import rospy
from policy.srv import CheckEdge
from geometry_msgs.msg import Point, Polygon

def check_edge_client(start, goal, polygon):
    rospy.wait_for_service('check_edge')
    try:
        check_edge = rospy.ServiceProxy('check_edge', CheckEdge)
        result = check_edge(start, goal, polygon)
        return result.result
    except rospy.ServiceException, e:
        print("Service call failed: {}".format(e))

if __name__ == "__main__":
    start = Point(0,0,0)
    goal = Point(8,-1,0)

    poly_list = [Point(-2,9,0),
            Point(1,9,0),
            Point(1,-2,0),
            Point(-2,-2,0)]
    poly = Polygon(poly_list)

    check_edge_client(start, goal, poly)
