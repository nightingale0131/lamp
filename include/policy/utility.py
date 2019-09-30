#!/usr/bin/env python

"""
Sample code from http://www.redblobgames.com/pathfinding/
Copyright 2014 Red Blob Games <redblobgames@gmail.com>

Feel free to use this code in your own projects, including commercial projects
License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>


utility functions that don't belong anywhere else
Modified by: Florence Tsang
"""
from __future__ import print_function # for end="" to work
import logging
logger = logging.getLogger(__name__)
import sys, os, glob
import matplotlib.pyplot as plt
import numpy as np
import math

from priority_queue import PriorityQueue

def secondsToStr(t):
    return "%d:%02d:%02d.%06d" % \
        reduce(lambda ll,b : divmod(ll[0],b) + ll[1:], [(t*1000,),1000,60,60])

def print_coord_list(coords):
    # print a list of coordinates in a nice format
    # assume floats
    msg = "["
    for tuple in coords:
        msg += "("
        for i, c in enumerate(tuple):
            msg += "{:.2f}".format(c)
            if i < len(tuple) - 1:
                msg += ", "
        msg += ") "

    msg += "]"

    return msg

def calc_bounding_coord_of_grid(origin, img_width, img_height, img_res):
    # returns min and max of x and y respectively
    (ox, oy, ot) = origin

    width = img_width*img_res
    height = img_height*img_res

    minx = round(ox, 2)
    maxx = round(width + ox, 2)
    miny = round(oy, 2)
    maxy = round(height + oy, 2)

    return [minx, maxx, miny, maxy]

def get_origin_and_res_from_yaml(path):
    # path - path to yaml file we are interested in
    # returns origin and resolution in yamlfile
    origin = []

    with open(path, 'rb') as yamlfile:
        for line in yamlfile:
            if "origin" in line:
                lb = line.find("[")
                rb = line.find("]")
                raw_origin = line[lb + 1:rb].split(', ') # extract origin coordinates

            if "resolution" in line:
                colon = line.find(":")
                resolution = float(line[colon + 2:])

    if raw_origin != None and resolution != None:
        for item in raw_origin:
            origin.append(float(item))
        logger.info("origin: {}, resolution: {}".format(origin, resolution))
        return origin, resolution
    else:
        logger.error("Failed to get data")

def isclose(a,b,rel_tol=1e-09, abs_tol=0.0):
    # Compares equality of two floats
    # implementation provided in python documentation
    return abs(a-b) <= max(rel_tol*max(abs(a), abs(b)), abs_tol)

def reconstruct_path(came_from, start, goal):
    """ redblobgames@gmail.com
        Reconstruct a shortest path from a dictionary of back-pointers
    """
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()  # optional
    return path

def calc_path_distance(path, base_graph):
    """ Inputs: list path - list of vertices, defined in base_graph
                WeightedGraph base_graph

        Output: float distance (or cost of path)
    """
    distance = 0
    for i in xrange(len(path) - 1):
        distance += base_graph.weight((path[i],path[i+1]))

    return distance

def get_coordinates(string):
    """ Assumes string is in "x y" format
    """
    coordinates = [int(c) for c in string.split()]
    return tuple(coordinates[:2])

def euclidean_distance(a, b):
    """ redblobgames@gmail.com """
    (x1, y1) = a
    (x2, y2) = b
    return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))
    #return abs(x1 - x2) + abs(y1 - y2)

def is_same_edge(a,b):
    # a,b - (u,v) 
    # when comparing edges of undirected graphs, check if they are actually the same edge
    (a1, a2) = a
    (b1, b2) = b
    if a1 == b1 and a2 == b2: return True
    if a1 == b2 and a2 == b1: return True

    return False

def a_star_search(graph, start, goal, check_edges=False):
    """A star algorithm courtesy of http://www.redblobgames.com/pathfinding/

    """
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.pop()

        if current == goal:
            break

        for next in graph.neighbours(current):
            if check_edges:
                graph._edge_check((current,next))
            new_cost = cost_so_far[current] + graph.weight(current, next)
            assert (new_cost >= 0), ("Cost is negative: {:.2f}".format(new_cost))
            if (next not in cost_so_far or new_cost < cost_so_far[next]):# and new_cost < 99999:
                cost_so_far[next] = new_cost
                priority = new_cost + graph.dist(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far
