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
from shapely.geometry import Point, LineString, box
import csv

import utility as util

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

    boxes, points, lines are defined using Shapely library
    """

    def __init__(self, nxgraph, polygon_dict):
        """
        nxgraph - networkx type undirected graph
                - vertex attr: defn: <point, line, or multipoints>
        polygon_dict - dict: {edge: polygon, ...}
        """
        self.graph = nxgraph

        logger.info("Validating input...")
        # check to make sure each edge has an associated polygon
        for u,v in self.graph.edges():
            # set edge attributes
            self.set_edge_state(u, v, UNKNOWN)
            self.set_edge_weight(u, v, self.min_mid_dist(u,v))
            try:
                self.graph.edge[u][v]['polygon'] = polygon_dict[(u,v)]
            except KeyError:
                self.graph.edge[u][v]['polygon'] = polygon_dict[(v,u)]

            # TODO: check that straight line between portals ('edge') is contained in polygon

        logger.debug(self.graph.edges(data=True))

    def min_dist(self, u, v):
        # calculate shortest distance between two vertices
        # If u and v are lines, then the shortest dist b/w the two lines are returned
        a = self.graph.node[u]['defn']
        b = self.graph.node[v]['defn']
        return a.distance(b)

    def min_mid_dist(self, u, v):
        a = self.pos(u)
        b = self.pos(v)
        return util.euclidean_distance(a,b)

    def pos(self, v):
        # returns (x,y) coord of v
        obj = self.graph.node[v]['defn']
        midpt = obj.centroid

        return (midpt.x, midpt.y)

    def get_polygon(self, u, v):
        # returns polygon associated w/ edge
        return self.graph.edge[u][v]['polygon']

    def get_vertices_in_polygon(self, des_poly):
        # polygon - shapely polygon
        # Returns all vertices that are a portal or are in des_poly
        # Possible speed up here if needed
        vset= set()

        for u,v,poly in self.graph.edges(data='polygon'):
            if poly == des_poly: # should not cause problems, otherwise use poly.equals(des_poly)
                vset.update([u,v])

        return vset
    
    def set_edge_state(self, u, v, state):
        assert (state == UNBLOCKED or state == BLOCKED or state == UNKNOWN), (
            "Attempted to assign invalid state ({}) to edge ({},{}).".format(state, u, v))

        self.graph.edge[u][v]['state'] = state

    def set_edge_weight(self, u, v, weight):
        assert isinstance(weight, (float, int, long)), (
            "Attempted to assign a non-number ({}) to edge ({},{}).".format(weight, u, v))

        self.graph.edge[u][v]['weight'] = weight

    def check_edge_state(self, u, v, rospath, padding=0):
        # (u,v) - edge we want to check
        # rospath - list of (x,y) coords repr a path
        # Sets edge state: if rospath is contained in edge polygon -> edge is unblocked
        # Also returns updated state of edge

        # if rospath is empty, planner failed to find a valid path so edge must be blocked
        if rospath == []:
            self.set_edge_state(u,v,BLOCKED)
            self.set_edge_weight(u,v,float('inf'))
        else:

            path = LineString(rospath)

            polygon = self.get_polygon(u,v)
            infl_poly = polygon.buffer(padding)
            print("Checking if path crosses the following: {}".format(infl_poly.bounds))
            print("Path: {}".format(util.print_coord_list(path.coords)))

            if infl_poly.contains(path): self.set_edge_state(u,v,UNBLOCKED)
            else: 
                self.set_edge_state(u,v,BLOCKED)
                self.set_edge_weight(u,v,float('inf'))

        return self.graph.edge[u][v]['state']

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
        if edges == []: edges = self.graph.edges()

        for u,v in edges:
            poly = self.graph.edge[u][v]['polygon']
            X,Y = poly.exterior.xy
            ax.plot(X,Y,'r')

def polygon_dict_from_csv(path):
    # each line in csv should be in following format:
    #   u,v,minx,maxx,miny,maxy
    # Assumes all polygons are boxes

    poly_dict = {}

    with open(path, 'rb') as csvfile:
        data = csv.reader(csvfile, delimiter=',')
        lines = []
        for row in data:
            logger.debug(row)
            u = row[0]
            v = row[1]
            if u.isdigit(): u = int(u)
            if v.isdigit(): v = int(v)
            minx, maxx, miny, maxy = row[2:]

            # convert to box
            poly = box(float(minx), float(miny), float(maxx), float(maxy))
            poly_dict[(u,v)] = poly

    return poly_dict