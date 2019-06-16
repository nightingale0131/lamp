#!/usr/bin/env python
"""
use MoveBaseSeq class from 
- reminder: move base seq takes in a path and executes it
      but if path is blocked it will use openloop to try to get to the goal (map.goal)

for now just do one task execution given maps
"""
import logging
logger = logging.getLogger(__name__) 
from openloop import MoveBaseSeq
import rospy, rospkg 
import os, glob

from policy.gridgraph import GridGraph
from policy.classes import Map
from policy import rpp

def update_p_est(M,t):
    """ Return updated estimated prob distribution
        M = list of maps
        t = total number of tasks so far (+1 b/c of base_graph)
    """
    p = []
    for m in M:
        n = m.n # number of times this map has been encountered
        prob = float(n)/t
        p.append(prob)

    return p

def import_maps(folder, supermaps, goal):
    # imports all pgm/yaml files in folder
    # assumes they are all for the same environment and start/goal is the same
    count = 0 
    for pgm_path in glob.glob(folder + "/*.pgm"):
        count += 1

        # get yaml file as well
        (root, ext) = os.path.splitext(pgm_path)
        yaml_path = root + ".yaml"
        print(pgm_path)
        print(yaml_path)

        if count == 1:
            # assume 00 is the zero map
            gridgraph = GridGraph(pgm_path, yaml_path, goal, graph_res=1.5) 
            supermaps.append(Map(gridgraph))
        else:
            gridgraph = GridGraph(pgm_path, yaml_path, goal,
                    refmap=supermaps[0].G)
            supermaps.append(Map(gridgraph))

    print("Imported {} maps.".format(count))

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    pkgdir = rospack.get_path('policy')
    mapdir = pkgdir + '/maps/'
    start = (0.0, 0.0)
    goal = (-8.0, 4.5)

    # enable logging other than roslog
    logging.basicConfig(filename = pkgdir + '/debug.log', filemode='w',
            level=logging.DEBUG)

    # import maps
    M = []
    import_maps(mapdir, M, goal)
    p = update_p_est(M, 2)
    features = M[0].features()
    # run rpp
    policy = rpp.solve_RPP(M, p, features, M[0].G.start, M[0].G.goal)
    policy[0].print_policy()
    # execute policy
        # follow path
        # once path is done, make observation
        # set next leg of outcome as next path
    
