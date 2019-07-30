#!/usr/bin/env python
import logging
logger = logging.getLogger(__name__)
import os
import networkx as nx
from matplotlib import pyplot as plt
import cv2

import policy.tgraph as tgraph
from policy import utility as util

def get_bounds_from_yaml(grid, yamlfile):
    origin, res = util.get_origin_and_res_from_yaml(yamlfile)
    iheight, iwidth = grid.shape
    return util.calc_bounding_coord_of_grid(origin, iwidth, iheight, res)


if __name__ == '__main__':
    testpath = os.path.dirname(os.path.abspath(__file__))
    filename = '00'
    gridfile = testpath + '/results/tristan maze/{}.pgm'.format(filename)
    yamlfile = testpath + '/results/tristan maze/{}.yaml'.format(filename) 
    
    logging.basicConfig(level=logging.DEBUG)

    img = cv2.imread(gridfile, cv2.IMREAD_GRAYSCALE)
    bounds = get_bounds_from_yaml(img, yamlfile)

    graph = nx.read_yaml(testpath + '/tristan_maze_tgraph.yaml')
    tgraph1 = tgraph.TGraph(graph, {})

    fig, ax1 = plt.subplots(1,1)
    ax1.imshow(img, cmap='gray', interpolation='bicubic', extent=bounds)
    tgraph1.draw_vertices(ax1)
    plt.show()

