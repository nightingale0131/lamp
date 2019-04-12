#!/usr/bin/env python

import logging
logger = logging.getLogger(__name__)
import os

from policy.gridgraph import GridGraph
import networkx as nx
import cv2
from matplotlib import pyplot as plt

def show_init_graph(gridgraph):
    # pos = {n: n for n in list(gridgraph.graph) }
    pos = nx.get_node_attributes(gridgraph.graph, 'pos') # for random graph
    # for k,v in pos.items(): print("{}: {}".format(k,v))

    """
    fig, [ax1, ax2] = plt.subplots(1,2)
    ax1.imshow(gridgraph.occ_grid, cmap='gray', interpolation='bicubic', extent=gridgraph.bounds)
    nx.draw(gridgraph.graph, pos, ax=ax1, node_size=20)
    ax1.set_axis_on()
    """

    # extract edges that are not blocked
    not_blocked, unknown = _get_not_blocked_edges(gridgraph)

    plt.imshow(gridgraph.occ_grid, cmap='gray', interpolation='bicubic', extent=gridgraph.bounds)
    nx.draw(gridgraph.graph, pos, node_size=20, edgelist=not_blocked)
    nx.draw(gridgraph.graph, pos, node_size=20, edgelist=unknown, edge_color='g')
    plt.show(block=False)

def show_comparison(refmap, newmap):
    # pos = {n: n for n in list(gridgraph.graph) }
    pos = nx.get_node_attributes(refmap.graph, 'pos') # for random graph

    # extract edges that are not blocked
    not_blocked, unknown = _get_not_blocked_edges(refmap)

    # draw refmap
    fig, [ax1, ax2] = plt.subplots(1,2)
    ax1.imshow(refmap.occ_grid, cmap='gray', interpolation='bicubic',
            extent=refmap.bounds)
    nx.draw(refmap.graph, pos, ax=ax1, node_size=20, edgelist=not_blocked)
    nx.draw(refmap.graph, pos, ax=ax1, node_size=20, edgelist=unknown, edge_color='g')
    ax1.set_axis_on()

    # draw newmap
    ax2.imshow(newmap.occ_grid, cmap='gray', interpolation='bicubic', extent=newmap.bounds)
    other_not_blocked, other_unknown = _get_not_blocked_edges(newmap)

    nx.draw(newmap.graph, pos, ax=ax2, node_size=20, edgelist=other_not_blocked)
    nx.draw(newmap.graph, pos, ax=ax2, node_size=20, edgelist=other_unknown, edge_color='g')
    ax2.set_axis_on()
    plt.show(block=False)

def _get_not_blocked_edges(gridgraph):
    # return list of unblocked and unknown edges
    not_blocked = []
    unknown=[]

    for edge in list(gridgraph.graph.edges()):
        (u,v) = edge
        if gridgraph.graph[u][v]['state'] == gridgraph.UNBLOCKED:
            not_blocked.append(edge)
        elif gridgraph.graph[u][v]['state'] == gridgraph.UNKNOWN:
            unknown.append(edge)

    return not_blocked, unknown


def check_weights(GG):
    u = 5
    v = 89
    print('Checking weight of ({}, {})'
            .format(GG.graph.node[u]['pos'], GG.graph.node[v]['pos']))
    weight = GG.weight(u,v)
    print(weight)

def check_neighbours(GG):
    v = 5
    print('Checking neighbours of {}'.format(GG.graph.node[v]['pos']))
    neighbours = GG.neighbours(v)
    print(neighbours)

if __name__ == '__main__':
    testpath = os.path.dirname(os.path.abspath(__file__))
    logging.basicConfig(filename=testpath + '/debug.log', filemode='w',level=logging.DEBUG)

    # build base graph on initial map 
    pgm = testpath + '/maps/simple1.pgm'
    yaml = testpath + '/maps/simple1.yaml'
    nav_graph = GridGraph(pgm, yaml, (-8, 4.5), graph_res=1.5)
    print(nx.info(nav_graph.graph))
    # nx.write_adjlist(nav_graph.graph, "test.adjlist", delimiter=',')

    # overlay grid on image
    show_init_graph(nav_graph)

    # save new occ grid as GridGraph with refmap graph overlaid on top 
    otherpgm = testpath + '/maps/simple2.pgm'
    otheryaml = testpath + '/maps/simple2.yaml'
    other = GridGraph(otherpgm, otheryaml, (-8, 4.5), refmap=nav_graph)
    show_comparison(nav_graph, other)

    # test weights
    check_weights(nav_graph)

    # test neighbours
    check_neighbours(nav_graph)

    raw_input('Press any key to continue')
