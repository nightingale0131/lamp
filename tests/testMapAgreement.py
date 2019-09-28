#!/usr/bin/env python

import logging
logger = logging.getLogger(__name__)
import os, glob
from copy import copy

from policy import tgraph
from policy import utility as util
from policy import mapfilters as mf
from policy.classes import Map
import networkx as nx
import cv2, math
from matplotlib import pyplot as plt

def setup_graph1(G):
    G.set_edge_state('s', 7, G.UNBLOCKED)
    G.set_edge_state(6, 7, G.UNBLOCKED)
    G.set_edge_state(6, 5, G.UNBLOCKED)
    G.set_edge_state(5, 12, G.UNBLOCKED)
    G.set_edge_state(11, 12, G.UNBLOCKED)
    G.set_edge_state(11, 10, G.UNBLOCKED)
    G.set_edge_state(9, 10, G.UNBLOCKED)
    G.set_edge_state(9, 'g', G.UNBLOCKED)

    set_blocked(G, 7, 8)
    set_blocked(G, 6, 8)
    set_blocked(G, 4, 5)
    set_blocked(G, 16, 10)

    logger.info("Successfully setup graph1")

def setup_graph2(G):
    G.set_edge_state('s', 7, G.UNBLOCKED)
    G.set_edge_state(6, 7, G.UNBLOCKED)
    G.set_edge_state(6, 5, G.UNBLOCKED)
    G.set_edge_state(5, 12, G.UNBLOCKED)
    G.set_edge_state(13, 12, G.UNBLOCKED)
    G.set_edge_state(13, 16, G.UNBLOCKED)
    G.set_edge_state(9, 16, G.UNBLOCKED)
    G.set_edge_state(9, 'g', G.UNBLOCKED)

    set_blocked(G, 6, 8)
    set_blocked(G, 4, 5)
    set_blocked(G, 4, 12)
    set_blocked(G, 11, 13)
    set_blocked(G, 8, 16)

    logger.info("Successfully setup graph2")

def set_blocked(G, u, v):
    G.set_edge_state(u, v, G.BLOCKED)
    G.set_edge_weight(u, v, float('inf'))

if __name__ == '__main__':
    testpath = os.path.dirname(os.path.abspath(__file__))
    # logging.basicConfig(filename=testpath + '/testMapAgreement_debug.log', filemode='w',level=logging.INFO)
    logging.basicConfig(level=logging.DEBUG)

    graph = nx.read_yaml('../maps/tristan_maze/tristan_maze_tgraph.yaml')
    poly_dict = tgraph.polygon_dict_from_csv('../maps/tristan_maze/tristan_maze_polygons.csv')

    test_tgraph1 = tgraph.TGraph(graph, poly_dict)
    test_tgraph2 = tgraph.TGraph(graph, poly_dict)

    setup_graph1(test_tgraph1)
    setup_graph2(test_tgraph2)

    map1 = Map(copy(test_tgraph1))
    map2 = Map(copy(test_tgraph2))

    mf.filter1([map1], map2)

    logger.info(map1.G)

    logger.info("Test passed")
