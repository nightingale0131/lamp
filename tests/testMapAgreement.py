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
    # map to be merged with graph1
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

def setup_graph3(G):
    # disagree with graph2
    G.set_edge_state('s', 7, G.UNBLOCKED)
    G.set_edge_state(6, 7, G.UNBLOCKED)
    G.set_edge_state(6, 8, G.UNBLOCKED)
    G.set_edge_state(6, 5, G.UNBLOCKED)
    G.set_edge_state(5, 12, G.UNBLOCKED)
    G.set_edge_state(9, 16, G.UNBLOCKED)
    G.set_edge_state(9, 'g', G.UNBLOCKED)

    set_blocked(G, 13, 16)
    set_blocked(G, 13, 12)
    set_blocked(G, 4, 5)
    set_blocked(G, 4, 12)
    set_blocked(G, 11, 13)
    set_blocked(G, 8, 16)

    logger.info("Successfully setup graph3")

def setup_graph4(G):
    # agree with graph2 w/o merging
    G.set_edge_state('s', 7, G.UNBLOCKED)
    G.set_edge_state(6, 7, G.UNBLOCKED)
    G.set_edge_state(6, 5, G.UNBLOCKED)
    G.set_edge_state(5, 12, G.UNBLOCKED)
    G.set_edge_state(13, 12, G.UNBLOCKED)
    G.set_edge_state(13, 16, G.UNBLOCKED)
    G.set_edge_state(9, 16, G.UNBLOCKED)
    G.set_edge_state(9, 10, G.UNBLOCKED)
    G.set_edge_state(9, 'g', G.UNBLOCKED)

    set_blocked(G, 6, 8)
    set_blocked(G, 7, 8)
    set_blocked(G, 4, 5)
    set_blocked(G, 4, 12)
    set_blocked(G, 11, 13)
    set_blocked(G, 8, 16)

    logger.info("Successfully setup graph4")

def set_blocked(G, u, v):
    G.set_edge_state(u, v, G.BLOCKED)
    G.set_edge_weight(u, v, float('inf'))

if __name__ == '__main__':
    testpath = os.path.dirname(os.path.abspath(__file__))
    # logging.basicConfig(filename=testpath + '/testMapAgreement_debug.log', filemode='w',level=logging.INFO)
    logging.basicConfig(level=logging.INFO)

    graph = nx.read_yaml('../maps/tristan_maze/tristan_maze_tgraph.yaml')
    poly_dict = tgraph.polygon_dict_from_csv('../maps/tristan_maze/tristan_maze_polygons.csv')

    test_tgraph1 = tgraph.TGraph(graph, poly_dict)
    test_tgraph2 = tgraph.TGraph(graph, poly_dict)
    test_tgraph3 = tgraph.TGraph(graph, poly_dict)
    test_tgraph4 = tgraph.TGraph(graph, poly_dict)

    setup_graph1(test_tgraph1)
    setup_graph2(test_tgraph2)
    setup_graph3(test_tgraph3)
    setup_graph4(test_tgraph4)

    map1 = Map(copy(test_tgraph1))
    map1_copy = Map(copy(test_tgraph1))
    map2 = Map(copy(test_tgraph2))
    map3 = Map(copy(test_tgraph3))
    map4 = Map(copy(test_tgraph4))

    logger.info("Following should be a successful merge with map 0")
    mf.filter1([map1, map3, map1_copy], map2)

    logger.info("No merge should happen.")
    mf.filter1([map1_copy, map3, map4], map2)

    logger.info("Map should be added.")
    mf.filter1([map3], map2)

    logger.info("Test passed")
