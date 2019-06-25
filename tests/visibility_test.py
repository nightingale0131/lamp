#!/usr/bin/env python
from __future__ import print_function
import cv2
import rospy, rospkg
import imutils
import numpy as np

import visilibity as vis
from policy.gridgraph import GridGraph, LiveGridGraph
import policy.visibility as vis

def test_find_obstacles(gridgraph):
    obstacles = vis.find_obstacles(gridgraph)
    img = gridgraph.occ_grid.copy()
    img[np.where(img == 0)] = 255
    for o in obstacles:
        vis.save_print(o)
        c = vis.save_print_contour(o)
        cv2.drawContours(img, [c], -1, 0, 2)

        cv2.imshow("Obstacles", img)
        cv2.waitKey(0)

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    pkgdir = rospack.get_path('policy')
    mapdir = pkgdir + '/tests/maps/'
    pgm0 = mapdir + 'simple1.pgm'
    yaml0 = mapdir + 'simple1.yaml'
    pgm1 = mapdir + 'simple3.pgm'
    yaml1 = mapdir + 'simple3.yaml'
    start = (0.0, 0.0)
    #goal = (4.0, -4.0) #for robohub
    goal = (-8.0, 4.5)
    #goal = (0.0,8.5)
    # goal = (-6.0, 3.7)

    map0 = GridGraph(pgm0, yaml0, goal, graph_res=1.5, robot_width=0.5)
    map1 = GridGraph(pgm1, yaml1, goal, refmap=map0, graph_res=1.5, robot_width=0.5)

    # test_find_obstacles(map0)

    des_map = map0
    obsvpt = des_map.pos(201)
    obstacles = vis.find_obstacles(des_map)
    vis_set = vis.visible_set(des_map, obsvpt, obstacles)
