#!/usr/bin/env python

import logging
logger = logging.getLogger(__name__)
import rospy, rospkg
import networkx as nx
from copy import copy

from policy import utility as util
from policy import rpp, tgraph
from policy.classes import Map
from policy import mapfilters as mf

PKGDIR = rospkg.RosPack().get_path('policy')
MAP = 'test_large'

if __name__ == '__main__':
    logging.basicConfig(filename=PKGDIR + "/tests/rpp_debug.log", filemode='w', 
            level=logging.DEBUG)
    filepath = PKGDIR + "/tests/rpp_sample.txt"
    env_info = util.import_node_info(filepath)

    base_graph = nx.read_yaml(PKGDIR + '/maps/' + MAP + '/' + MAP + '_tgraph.yaml')
    poly_dict = tgraph.polygon_dict_from_csv(PKGDIR + '/maps/' + MAP + '/' + MAP + '_polygons.csv')

    # transfer node info
    maps = []
    for i,g in enumerate(env_info):
        map_tgraph = tgraph.TGraph(base_graph, poly_dict)
        map_tgraph.graph.add_edges_from(g.edges(data=True))
        maps.append(Map(map_tgraph))

    # print states
    logger.info("Verifying map states...")
    for edge in base_graph.edges():
        line = "{:<10}".format(edge) 
        u,v = edge
        line += "{:8.3f}".format(maps[0].G.weight(u,v))
        for m in maps:
            line += "{:3}".format(m.G.edge_state(u,v))

        logger.info(line)

    '''
    # check costs
    logger.debug("Printing cost to goal...")
    logger.debug(maps[1]._cost['g'].cost)
    logger.debug(maps[1]._cost['g'].paths)

    '''
    # some modifications to probabilities
    maps[0].n = 1
    p = mf.update_p_est(maps, 4) # update prob estimation

    features = maps[0].features()
    policy = rpp.solve_RPP(maps, p, features, 's', 'g')
    logger.info(policy[0].print_policy())
    print(policy[0].print_policy())
