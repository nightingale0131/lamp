#!/usr/bin/env python
import rospy, rospkg
from nav_msgs.msg import OccupancyGrid
import os
import math
import networkx as nx

from nav_seq import MoveBaseSeq
from policy.gridgraph import GridGraph
from policy import utility as util

if __name__ == '__main__':
    # specify start and goal
    start = (0.0, 0.0)
    goal = (-8.0, 4.5)
    # load map and determine graph
    rospack = rospkg.RosPack()
    pkgdir = rospack.get_path('policy')
    mapdir = pkgdir + '/maps/'
    pgm0 = mapdir + 'simple2.pgm'
    yaml0 = mapdir + 'simple2.yaml'

    map0 = GridGraph(pgm0, yaml0, goal, graph_res=1.5, robot_width=0.5)
    print(nx.info(map0.graph))
    # nx.write_adjlist(map0.graph, pkgdir + '/src/map0.adjlist', delimiter=',')

    # run a* on custom map object
    came_from, cost_so_far = util.a_star_search(map0, start, goal)
    path = util.reconstruct_path(came_from, start, goal)
    # add z value to path tuples
    for i in range(len(path)):
        (x,y) = path[i]
        path[i] = (x,y,0.0)
        print(path[i])

    # give path to MoveBaseSeq
    try:
        MoveBaseSeq(path)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
