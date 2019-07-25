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
import networkx as nx

from policy.gridgraph import GridGraph
from policy.classes import Map
from policy import rpp
from policy import timing

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
    for pgm_path in sorted(glob.glob(folder + "/*.pgm")):
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
            timing.log("Finished visibility check")
        else:
            gridgraph = GridGraph(pgm_path, yaml_path, goal,
                    refmap=supermaps[0].G)
            supermaps.append(Map(gridgraph))
            timing.log("Finished visibility check")

    print("Imported {} maps.".format(count))

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    pkgdir = rospack.get_path('policy')
    mapdir = pkgdir + '/maps/'
    start = (0.0, 0.0)
    # goal = (4.0, -4.0) # for robohub
    # goal = (-8.0, 4.5)
    # goal = (7.5, -5)
    # goal = (-0.7, 8)
    # goal = (8, 0) # for maze
    goal = (8.5, 8.0) # for maze

    # enable logging other than roslog
    logging.basicConfig(filename = pkgdir + '/lrpp_debug.log', filemode='w',
            level=logging.INFO)

    # import maps
    M = []
    import_maps(mapdir, M, goal)
    p = update_p_est(M, 2) # pmf of maps is hard coded right now
    features = M[0].features()
    base_map = M[0].G
    base_map.show_img()
    # nx.write_adjlist(base_map.graph, pkgdir + '/src/map0.adjlist')

    # run rpp
    policy = rpp.solve_RPP(M, p, features, M[0].G.start, M[0].G.goal)
    logging.info(policy[0].print_policy())
    """
    raw_input('Press any key to begin execution')
    # execute policy
    try:
        MoveBaseSeq(base_map, policy=policy)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

    # save pgm/yaml file of map
    os.system("rosrun map_server map_saver -f " + mapdir + "test")
    # shut down cartographer
    os.system("rosnode kill /cartographer_node /cartographer_occupancy_grid_node /move_base")
    """
