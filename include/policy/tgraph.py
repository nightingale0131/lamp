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
    UNKNOWN = -1
    UNBLOCKED = 0
    BLOCKED = 1

    def __init__(self, nxgraph, polygon_dict=None):
        """
        nxgraph - networkx type undirected graph
                - vertex attr: defn: <point, line, or multipoints>
        polygon_dict - dict: {edge: polygon, ...}
        """
        self.graph = nxgraph.copy()
        self.goal = 'g'
        self.submaps = set()

        logger.info("Validating input...")
        # check to make sure each edge has an associated polygon
        if polygon_dict == None:
            for u,v in self.graph.edges():
                self.graph.edge[u][v]['polygon']
        else:
            for u,v in self.graph.edges():
                # set edge attributes
                self.set_edge_state(u, v, self.UNKNOWN)
                self.set_edge_weight(u, v, self.min_mid_dist(u,v))
                try:
                    self.graph.edge[u][v]['polygon'] = polygon_dict[(u,v)]
                except KeyError:
                    self.graph.edge[u][v]['polygon'] = polygon_dict[(v,u)]

                # TODO: check that straight line between portals ('edge') is contained in polygon

        logger.debug(str(self))

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
        # currently just the midpoint
        obj = self.graph.node[v]['defn']
        midpt = obj.centroid

        return (midpt.x, midpt.y)

    def get_polygon(self, u, v):
        # returns polygon associated w/ edge
        return self.graph.edge[u][v]['polygon']

    def in_polygon(self, pos, poly):
        # pos - (x,y)
        # poly - shapely polygon
        # takes cartesian coordinates and returns whether location is in given polygon

        p = Point(pos)
        return poly.contains(p)

    def get_vertices_in_polygon(self, des_poly):
        # polygon - shapely polygon
        # Returns all vertices that are a portal or are in des_poly
        # Possible speed up here if needed
        vset= set()

        for u,v,poly in self.graph.edges(data='polygon'):
            if poly == des_poly: # should not cause problems, otherwise use poly.equals(des_poly)
                vset.update([u,v])

        return vset

    def add_connected_vertex(self, label, location, last_vertex):
        # automatically adds edges to portals in the same polygon
        # label - string
        # location - (x,y), in reference to map
        # last_vertex - last vertex that robot passed

        # add vertex
        self.graph.add_node(label, defn=Point(location))

        # find polygon (search all edges that are adjacent to originating portal)
        searched_polygons = [] # there's only two associated with any portal
        for neighbor in self.graph.neighbors_iter(last_vertex):
            poly = self.get_polygon(last_vertex, neighbor)
            if poly not in searched_polygons:
                searched_polygons.append(poly)
                if self.in_polygon(location, poly):
                    break

        # add edges to all portals that are unblocked/unknown to originating portal
        for portal in self.get_vertices_in_polygon(poly):
            print("Is ({},{}) blocked?".format(last_vertex, portal))
            if last_vertex == portal or self.edge_state(last_vertex, portal) != self.BLOCKED:
                weight = util.euclidean_distance(self.pos(label), self.pos(portal))
                self.graph.add_edge(label, portal, weight=weight)

    def remove_vertex(self, v):
        # v - vertex label
        self.graph.remove_node(v)

    def vertices(self, data=False):
        return self.graph.nodes(data=data)

    def edge_state(self, u, v):
        return self.graph[u][v]['state']

    def edge_info(self, u, v):
        return self.graph[u][v]

    def edges(self, data=False):
        return self.graph.edges(data=data)

    def weight(self, u, v, allow_inf=True):
        # assumes weights are set correctly (blocked edges are inf)
        weight = self.graph.edge[u][v]['weight']

        # don't return infinity if user wants edge weight if edge were unblocked
        if allow_inf == False and weight == float('inf'):
            return self.graph.edge[u][v]['unblocked_weight']
        else:
            return weight
    
    def set_edge_state(self, u, v, state):
        assert (state == self.UNBLOCKED or state == self.BLOCKED or state == self.UNKNOWN), (
            "Attempted to assign invalid state ({}) to edge ({},{}).".format(state, u, v))

        self.graph.edge[u][v]['state'] = state

    def set_edge_weight(self, u, v, weight):
        assert isinstance(weight, (float, int, long)), (
            "Attempted to assign a non-number ({}) to edge ({},{}).".format(weight, u, v))

        # initial weight is never going to be inf
        if 'weight' in self.graph.edge[u][v].keys():
            curr_edge_weight = self.graph.edge[u][v]['weight']

            if weight == float('inf') and curr_edge_weight != float('inf'):
                self.graph.edge[u][v]['unblocked_weight'] = curr_edge_weight

        self.graph.edge[u][v]['weight'] = weight

    def update_edge(self, u, v, new_state, new_weight=-1):
        # combining set_edge_state and set_edge_weight
        # assuming attr state and weight already exist for edge
        assert (new_state == self.UNBLOCKED or new_state == self.BLOCKED or new_state == self.UNKNOWN), (
            "Attempted to assign invalid state ({}) to edge ({},{}).".format(state, u, v))

        old_state = self.edge_state(u, v)
        old_weight = self.weight(u, v, allow_inf=False)

        if new_weight >= 0:
            new_weight = util.moving_average(old_weight, new_weight)

        # update state
        if new_state != self.UNKNOWN:
            logger.info("set ({},{}): state = {}".format(u,v,new_state))
            self.set_edge_state(u, v, new_state)

        # update weight
        if (old_state == self.UNKNOWN or old_state == self.UNBLOCKED):
            if (new_state != self.BLOCKED and new_weight >= 0):
                self.set_edge_weight(u, v, new_weight)
            elif new_state == self.BLOCKED:
                self.set_edge_weight(u, v, float('inf'))
        elif old_state == self.BLOCKED:
            if (new_state == self.UNBLOCKED):
                if new_weight >= 0:
                    self.set_edge_weight(u, v, new_weight)
                else:
                    self.set_edge_weight(u, v, old_weight)
            else:
                self.set_edge_weight(u, v, float('inf'))

    def check_edge_state(self, u, v, path, padding=0, set_unblocked=True):
        # (u,v) - edge we want to check
        # path - list of (x,y) coords repr a path
        # Sets edge state: if path is contained in edge polygon -> edge is unblocked
        # Also returns updated state of edge

        '''
        # don't want to change edge state if it is already set to UNBLOCKED
        #   ^ this is causing some serious issues, need to rethink
        if self.graph.edge[u][v]['state'] != self.UNBLOCKED:
            # if path is empty, planner failed to find a valid path so edge must be blocked
            # ^ not true, sometimes it fails to find a path b/c robot's location is 'in'
            # an obstacle. Wait for controller to abort to consider it blocked
            if path == []:
                self.update_edge(u, v, self.BLOCKED)
            else:
        '''
        if path != []:
            path = LineString(path)

            polygon = self.get_polygon(u,v)
            infl_poly = polygon.buffer(padding)
            logger.debug("Checking if path crosses the following: {}".format(infl_poly.bounds))
            logger.debug("Path: {}".format(util.print_coord_list(path.coords)))

            path_in_submap = infl_poly.contains(path)

            if path_in_submap and set_unblocked: 
                self.update_edge(u,v,self.UNBLOCKED)
            elif not path_in_submap: 
                self.update_edge(u,v,self.BLOCKED)

        return self.graph.edge[u][v]['state']

    def observe(self, v):
        # estimate what robot will observe at vertex v
        # Basically assume states of adjacent edges will be known
        edges = []
        for u in self.graph.neighbors_iter(v):
            edges.append((u,v))

        return edges

    def update(self, new_info):
        # updates this object with the new information
        # new_info - {feature: {state: 0, weight: 1.4, ....}, feature:{...}, ... }
        
        for edge, data in new_info.items():
            logger.info("{}: {}".format(edge, data))
            (u,v) = edge
            assert (self.graph[u][v]['state'] == self.UNKNOWN),(
                    "Overwriting non-unknown edge: {}, state: {}!".format(edge,
                        self.graph[u][v]['state']))

            self.set_edge_state(u, v, data['state']) 
            self.set_edge_weight(u, v, data['weight'])

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

    def __copy__(self):
        new = type(self)(self.graph)
        return new

    def __str__(self):
        msg = "#Nodes: {}   Edges:\n".format(self.graph.number_of_nodes())
        for (u,v) in self.edges():
            msg += "({},{})\t{}\t{:.3f}\n".format(u,v,self.graph.edge[u][v]['state'],
                    self.graph.edge[u][v]['weight'])

        return msg


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
