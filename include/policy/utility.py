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
import sys
import matplotlib.pyplot as plt
import numpy as np
import math

from priority_queue import PriorityQueue

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

def heuristic(a, b):
    """ redblobgames@gmail.com """
    (x1, y1) = a
    (x2, y2) = b
    return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))
    #return abs(x1 - x2) + abs(y1 - y2)

def a_star_search(graph, start, goal):
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
            new_cost = cost_so_far[current] + graph.weight(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far
