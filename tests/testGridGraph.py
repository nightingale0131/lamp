#!/usr/bin/env python

import logging
logger = logging.getLogger(__name__)
import os

from policy.gridgraph import GridGraph, BoundingBox, boxblur, boxblurV, gaussblur
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
    nx.draw(gridgraph.graph, pos, node_size=20, edgelist=not_blocked, with_labels=True)
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
    nx.draw(refmap.graph, pos, ax=ax1, node_size=20, edgelist=not_blocked, )
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
    v = (0,0)
    print('Checking neighbours of {}'.format(GG.graph.node[v]['pos']))
    neighbours = GG.neighbours(v)
    for neighbour in neighbours:
        (x,y) = neighbour
        print(' ({:.3f},{:.3f})'.format(x,y))

def check_blur(GG):
    """
    # test data taken from debug file:
    DEBUG:policy.gridgraph:Bounding box of edge ((0.571, 0.031),(0.000, 0.000))
    DEBUG:policy.gridgraph: left=-0.263, right=0.834, top=0.295, down=-0.263
    DEBUG:policy.gridgraph: In pixels: left=181, right=203, top=141, down=152
    DEBUG:policy.gridgraph: P(free) = 1, P(unknown)  = 0.0

    DEBUG:policy.gridgraph:Bounding box of edge ((0.571, 0.031),(0.753, -1.124))
    DEBUG:policy.gridgraph: left=0.285, right=1.039, top=0.317, down=-1.410
    DEBUG:policy.gridgraph: In pixels: left=192, right=207, top=140, down=175
    DEBUG:policy.gridgraph: P(free) = 1, P(unknown)  = 0.0
    """

    (a,b) = ((-9.122, 3.354),(-9.304, 4.509)) 
    box = BoundingBox(0.25, a, b)
    print("box bounds: left={:.3f}, right={:.3f}, top={:.3f}, bottom={:.3f}"
          .format(box.left[0], box.right[0], box.top[1], box.bottom[1]))
    bounds = GG.bounds
    pxbounds = (0,10,51,85)
    kernel = 3
    # W = boxblur(box, bounds, pxbounds, GG.img_res, kernel)
    # W = boxblur(box, bounds, pxbounds, GG.img_res, kernel, W)
    # W = boxblur(box, bounds, pxbounds, GG.img_res, kernel, W)
    W = gaussblur(box, bounds, pxbounds, GG.img_res, kernel)

if __name__ == '__main__':
    testpath = os.path.dirname(os.path.abspath(__file__))
    logging.basicConfig(filename=testpath + '/debug.log', filemode='w',level=logging.INFO)

    # build base graph on initial map 
    pgm = testpath + '/maps/simple1.pgm'
    yaml = testpath + '/maps/simple1.yaml'
    nav_graph = GridGraph(pgm, yaml, (-8, 4.5), graph_res=1.5, robot_width=0.5)
    cv2.imwrite(testpath + 'cv2_img.jpg', nav_graph.occ_grid)
    print(nx.info(nav_graph.graph))
    nx.write_adjlist(nav_graph.graph, "test.adjlist", delimiter=',')

    # overlay grid on image
    show_init_graph(nav_graph)

    # save new occ grid as GridGraph with refmap graph overlaid on top 
    otherpgm = testpath + '/maps/problem.pgm'
    otheryaml = testpath + '/maps/problem.yaml'
    other = GridGraph(otherpgm, otheryaml, (-8, 4.5), refmap=nav_graph, robot_width=0.5)
    show_comparison(nav_graph, other)

    """
    # calculate all pairs shortest path
    first = nx.floyd_warshall(nav_graph.graph)
    second = nx.floyd_warshall(other.graph)

    diff = [] 
    for v,targets in first.items():
        for u,length in targets.items():
            if length > 0:
                difference = (second[v][u] - length)/length
            else: difference = second[v][u] - length
            print("{:.3f}".format(difference))
            diff.append(difference)

    print(min(diff))
    count = 0
    for i in diff:
        if i < -0.5: count +=1

    print("num of diff < -0.5: {}".format(count))
    """

    # test weights
    # check_weights(nav_graph)

    # test neighbours
    # check_neighbours(nav_graph)

    # check_blur(nav_graph)
    raw_input('Press any key to continue')
