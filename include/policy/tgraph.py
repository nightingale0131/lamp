#!/usr/bin/env python

"""
Creation date: July 30, 2019
By: Florence Tsang

Purpose: Class for defining the topological graph for LRPP
"""

import logging
logger = logging.getLogger(__name__)

import networkx as nx
from matplotlib import pyplot as plt
from shapely.geometry import Point, LineString, Polygon

# import timing

UNKNOWN = -1
UNBLOCKED = 0
BLOCKED = 1

class TGraph(object):
    """
    Topological graph where vertices can be a point, line, or set of points (future work).
    Each edge in the graph is associated with a convex polygon.
    Each edge has 3 states: unblocked, blocked, unknown
        - the robot has to have attempted to traverse the edge, otherwise it is unknown
        - an edge is unblocked if the robot can travel to the other end
        - edge is blocked otherwise

    Polygons, points, lines are defined using Shapely library
    """

    def __init__(self, nxgraph, polygon_dict):
        """
        nxgraph - networkx type undirected graph
                - vertex attr: defn: <point, line, or multipoints>
        polygon_dict - dict: {edge: polygon, ...}
        """
        logger.info("Validating input...")
        # check to make sure each edge has an associated polygon
        for edge in nxgraph.edges():
            # set edge attributes
            nxgraph.edge[edge]['state'] = UNKNOWN
            nxgraph.edge[edge]['weight'] = self._calc_init_weight(edge)
            # nxgraph.edge[edge]['polygon'] = polygon_dict[edge]

        self.graph = nxgraph
        self.polygon_dict = polygon_dict
        # print(self.graph.edges())

    def _calc_init_weight(self, edge):
        # estimate initial weight of edges by calc min distance from one vertex to another
        (v1, v2) = edge
        a = self.graph.node[v1]['defn']
        b = self.graph.node[v2]['defn']
        return a.distance(b)

    def draw_vertices(self, ax):
        # vertices are points or portals/lines
        # ax - pyplot axes
        for v in self.graph.nodes(data=True):
            obj = v[1]['defn']
            if obj.geom_type == 'Point':
                ax.plot(obj.x, obj.y, 'bo') 
            elif obj.geom_type == 'LineString':
                X,Y = obj.xy
                ax.plot(X,Y, 'b')
            # placeholder for multipoints

    def draw_polygons(self, ax, edges=[]):
        # draw polygons
        # either draw all of them by default, or just the ones specified by edges
        if is_empty(edges): edges = self.graph.edges()

        for edge in edges:
            poly = self.graph.edge[edge]['polygon']
            X,Y = poly.exterior.xy
            ax.plot(X,Y,'r')
