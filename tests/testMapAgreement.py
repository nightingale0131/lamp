#!/usr/bin/env python

import logging
logger = logging.getLogger(__name__)
import os, glob

from policy.gridgraph import GridGraph
from policy import utility as util
from policy import mapFilters as mf, timing
from policy.classes import Map
import networkx as nx
import cv2, math
from matplotlib import pyplot as plt

def import_maps(folder, goal, graph=None):
    # imports all pgm/yaml files in folder
    # assumes they are all for the same environment and start/goal is the same
    supermaps = []
    count = 0 
    for pgm_path in sorted(glob.glob(folder + "/*.pgm")):
        count += 1

        # get yaml file as well
        (root, ext) = os.path.splitext(pgm_path)
        filename = os.path.basename(root)
        yaml_path = root + ".yaml"
        print("Analyzing " + filename)

        if count == 1:
            # assume 00 is the zero map
            if graph != None:
                gridgraph = GridGraph(pgm_path, yaml_path, goal, custom_graph=graph)
            else:
                gridgraph = GridGraph(pgm_path, yaml_path, goal, graph_res=1.5) 
            supermaps.append(Map(gridgraph))
            print(nx.info(supermaps[0].G.graph))
            timing.log("Finished visibility check")
        else:
            gridgraph = GridGraph(pgm_path, yaml_path, goal,
                    refmap=supermaps[0].G)
            new_map = Map(gridgraph)
            timing.log("Finished visibility check")

            supermaps = mf.filter1(supermaps, new_map)
            timing.log("Finished filtering")

    print("Imported {} maps.".format(len(supermaps)))
    return supermaps

def draw_graphs(maps):
    # maps - list of type Map
    fig = plt.figure()
    nplots = len(maps)
    pos = nx.get_node_attributes(maps[0].G.graph, 'pos')
    nrows = int(math.ceil(nplots/3.0))
    ncols = 3

    for i, m in enumerate(maps):
        ax = fig.add_subplot(nrows, ncols, i+1)
        gridgraph = m.G
        graph = gridgraph.graph
        not_blocked, unknown = _get_not_blocked_edges(gridgraph)
        ax.imshow(gridgraph.occ_grid, cmap='gray', interpolation='bicubic',
                extent=gridgraph.bounds)
        nx.draw(graph, pos, ax=ax, node_size=20, edgelist=unknown, edge_color='g')
        nx.draw(graph, pos, ax=ax, node_size=20, node_color='k', edgelist=not_blocked, alpha=0.5,
                with_labels=True, font_color='m', font_size=14 )
        ax.set_axis_on()

    plt.show()

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

if __name__ == '__main__':
    testpath = os.path.dirname(os.path.abspath(__file__))
    logging.basicConfig(filename=testpath + '/testMapAgreement_debug.log', filemode='w',level=logging.INFO)
    mapdir = testpath + '/results/tristan maze/SameTrial'
    # mapdir = testpath + '/results/simple_maps/test'
    goal = (8.5, 8.0) # tristan_maze 
    # goal = (-8, 4.5) # simple

    # create custom graph for test case
    G = nx.read_yaml(testpath + '/tristan_maze_custom.yaml')

    M = import_maps(mapdir, goal)
 
    # show plots
    draw_graphs(M)

