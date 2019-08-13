#!/usr/bin/env python
import logging
logger = logging.getLogger(__name__)
import os
import networkx as nx
from matplotlib import pyplot as plt
import cv2

import policy.tgraph as tgraph
from policy import utility as util

TESTPATH = os.path.dirname(os.path.abspath(__file__))

def get_bounds_from_yaml(grid, yamlfile):
    origin, res = util.get_origin_and_res_from_yaml(yamlfile)
    iheight, iwidth = grid.shape
    return util.calc_bounding_coord_of_grid(origin, iwidth, iheight, res)

def test_drawing(G):
    # G - tgraph type
    filename = '/results/tristan maze/00'
    gridfile = TESTPATH + '{}.pgm'.format(filename)
    yamlfile = TESTPATH + '{}.yaml'.format(filename) 

    img = cv2.imread(gridfile, cv2.IMREAD_GRAYSCALE)
    bounds = get_bounds_from_yaml(img, yamlfile)

    fig, ax1 = plt.subplots(1,1)
    ax1.imshow(img, cmap='gray', interpolation='bicubic', extent=bounds)
    G.draw_vertices(ax1)
    G.draw_polygons(ax1)
    plt.show()

def test_get(G):
    print("\nTesting get_polygon and get_vertices_in_polygon...")

    for u,v in G.graph.edges():
        polygon = G.get_polygon(u,v)
        logger.debug(list(polygon.exterior.coords))
        vset = G.get_vertices_in_polygon(polygon)
        logger.debug(list(vset))

        # validation
        while vset != set():
            a = vset.pop()
            for b in vset:
                logger.debug("a: {}, b: {}".format(a,b))
                if G.get_polygon(a,b) != polygon:
                    raise Exception("Returned polygon for ({},{}) doesn't match ({},{})"
                                    .format(a,b,u,v))

    print("Test passed\n")


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    # init TGraph class
    graph = nx.read_yaml(TESTPATH + '/tristan_maze_tgraph.yaml')
    poly_dict = tgraph.polygon_dict_from_csv(TESTPATH + '/tristan_maze_polygons.csv')
    tgraph1 = tgraph.TGraph(graph, poly_dict)

    test_get(tgraph1)
    test_drawing(tgraph1)