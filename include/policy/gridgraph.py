#!/usr/bin/env python

import logging
logger = logging.getLogger(__name__)

import networkx as nx
import cv2
from matplotlib import pyplot as plt
from align import alignImages 
import math
import random
import utility

class GridGraph(object):
    """
    Combines occupancy grid and graph.
        Note: I need to keep the occupancy grid if I intend to align images to ensure the
        same reference point is kept.
    """
    def __init__(self, gridfile, yamlfile, goal, refmap=None, graph_res=0.5, robot_width=0.5):
        logger.info('Initializing {}'.format(gridfile))
        """
          refmap - reference GridGraph obj

        Attributes:
        robot_width
        occ_grid (cv2 img object)
        origin - [x, y, z] Coordinates of lower left corner of occ_grid in reference
                to origin of map (location where mapping module was started)
        img_res - resolution of occ_grid
        bounds - [minx maxx miny maxy] of occ_grid
        graph - navigation graph
        """

        # set up constants
        self.BLOCKED = 1
        self.UNBLOCKED = 0
        self.UNKNOWN = -1
        self.robot_width = 1.0 # assuming robot is a square, what is the max_length?

        # set up graph as soon as you read occ_grid in
        # if graph=None then build a graph
        # otherwise align the graph with the new occ grid, assuming the 
        #   origins are all in the same location in ref to the world
        self.occ_grid = cv2.imread(gridfile, cv2.IMREAD_GRAYSCALE)
        self.origin, self.img_res = get_origin(yamlfile)

        if refmap==None:
            self.bounds = self.calc_bounding_coord()
            self.graph = self.build_graph(graph_res, (0,0), goal, n=300)
            self._collision_check(True)
            logger.info('Built graph')
        else:
            self.graph = refmap.graph.copy()
            logger.info('Imported graph')
            self._align(refmap.occ_grid)
            self.origin = refmap.origin
            # self.origin = self._calc_transformed_origin(refmap)
            logger.debug('Modified origin: {}'.format(self.origin))
            self.bounds = self.calc_bounding_coord()
            self._collision_check()

        # check which edges are blocked/unblocked
        logger.info('Finished initialization')

    def _align(self, refimg):
        # refimg - cv2 image object
        imAligned, h = alignImages(self.occ_grid, refimg)
        print("Estimated homeography: \n {}".format(h))
        # cv2.imwrite("aligned.jpg", imAligned)

        # change occ_grid to aligned image
        self.occ_grid = imAligned

        # return [x2, y2, z2]

    def _transform_pixel(self, pixel, h):
        """
            Assuming h = a b c
                         d e f
                         g h i
        """
        (x1, y1) = pixel
        skew = (h[2][0]*x1 + h[2][1]*y1 + h[2][2])

        x2 = (h[0][0]*x1 + h[0][1]*y1 + h[0][2])/skew
        y2 = (h[1][0]*x1 + h[1][1]*y1 + h[1][2])/skew

        return (x2, y2)

    def neighbours(self, v):
        # return iterator of all neighbors of v
        return self.graph.neighbors(v)

    def weight(self, u, v, attr=None):
        # assuming this will be used with networkx dijkstra
        # u,v - node label, should be in (x,y) format
        # attr - dictionary of edge attributes, this is not used, but these 3 arguments
        #        are needed to satisfy networkx dijkstra function requirements
        # considers unknowns as unblocked edges
        (x1, y1) = self.graph.node[u]['pos']
        (x2, y2) = self.graph.node[v]['pos']
        # need to check edge state 
        return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

    def known_weight(self, u, v, attr=None):
        # assuming this will be used with networkx dijkstra
        # u,v - node label, should be in (x,y) format
        # attr - dictionary of edge attributes, this is not used, but these 3 arguments
        #        are needed to satisfy networkx dijkstra function requirements
        # considers unknowns as blocked edges
        return None # placeholder

    def _edge_check(self, edge):
        # get bounding box on graph edge
        (minx, maxx, miny, maxy) = self.bounds
        (imgheight, imgwidth) = self.occ_grid.shape
        # (a,b) = edge # comment this for random graph
        (u,v) = edge
        # Below is for random graph
        a = self.graph.node[u]['pos']
        b = self.graph.node[v]['pos']
        """
        if (b[0] - a[0] < 0) or (b[1] - a[1] > 0):
            (x1, y1) = b
            (x2, y2) = a
        else:
            (x1, y1) = a
            (x2, y2) = b
        """

        padding = self.robot_width/2
        box = BoundingBox( padding, a, b)

        left = max(box.left[0], minx)
        right = min(box.right[0], maxx)
        up = min(box.top[1], maxy)
        down = max(box.bottom[1], miny)

        # Calculate which pixels need to be checked, these pixels have to be included
        # (0,0) is in top left of image
        leftpx = int(abs(left-minx)/self.img_res)
        rightpx = min(int(abs(right-minx)/self.img_res), imgwidth - 1)
        toppx = int(abs(maxy - up)/self.img_res)
        downpx = max(int(abs(maxy - down)/self.img_res), imgheight - 1)

        logger.debug('Bounding box of edge ({},{}): \nleft={}, right={}, top={}, down={}'
                .format(a, b, left, right, up, down))
        logger.debug('In pixels: left={}, right={}, top={}, down={}'
                .format(leftpx, rightpx, toppx, downpx))

        # Do probability check to see state of edge, assign to edge attribute
        # pixel mapping: 0 - unknown, otherwise x/255 to get probability of pixel being
        #               FREE 

        k = 0.0 # number of unknown pixels
        n = 0.0 # number of pixels in bounding box

        prob_free = 1
        for col in xrange(leftpx, rightpx + 1):
            # calc Y range of slice, consider both sides of pixel
            # clamp Y to border of image, and x to bounding box left & right
            leftx = max(col*self.img_res + minx, left)
            rightx = min((col + 1)*self.img_res + minx, right)
            slice_topy = min(up, max(box.maxY(leftx), box.maxY(rightx)))
            slice_boty = max(down, min(box.minY(leftx), box.minY(rightx)))

            toppx = int(abs(maxy - slice_topy)/self.img_res)
            downpx = min(int(abs(maxy - slice_boty)/self.img_res), imgheight - 1)

            for row in xrange(toppx, downpx + 1):
                p = self.occ_grid[row,col]/255.0
                n += 1
                if p == 0:
                    p = 1
                    k += 1
                elif p > 0.50: p = 1
                prob_free = prob_free*p

        # Assign the following states to the edge:
        # 0:unblocked, 1:blocked, -1:unknown
        prob_unknown = float(k)/n
        logger.debug("P(free) = {}, P(unknown)  = {}"
                .format(prob_free, prob_unknown))

        # Thresholding of edges to states
        # The following is problematic/too simple
        if prob_free < 0.2: self.graph[u][v]['state'] = self.BLOCKED
        elif prob_unknown > 0.5: self.graph[u][v]['state'] = self.UNKNOWN
        elif prob_free > 0.7: self.graph[u][v]['state'] = self.UNBLOCKED
        else: self.graph[u][v]['state'] = self.UNKNOWN

    def _collision_check(self, modify=False):
        # do collision checking on all edges
        for edge in list(self.graph.edges()):
            self._edge_check(edge)

        # The following removes singletons and blocked edges
        # Should only be run in init run
        if modify == True:
            for u, v in self.graph.edges():
                if self.graph[u][v]['state'] == self.BLOCKED:
                    self.graph.remove_edge(u,v)

            for node in self.graph.nodes():
                if self.graph.degree(node) == 0:
                    self.graph.remove_node(node)

    def _cart_to_pixel(self, pt):
        # Assumes pt is within bounds of occ_grid
        (minx, maxx, miny, maxy) = self.bounds
        (x,y) = pt
        px = int((x - minx)/self.img_res)
        py = int((maxy - y)/self.img_res)

        return (px, py)

    def build_graph(self, res, start, goal, opt='halton', n=500):
        """
        res - resolution of graph vertices 
        start, goal - (x,y) cartesian coordinates
        opt - 'grid' will break a lot of stuff because it doesn't use 'pos' attr
        n - # samples to take, only needed if using (pseudo)random sampling

        returns graph where each vertex is (x,y) coordinates in reference to the origin of the
        image.
        """
        (minx, maxx, miny, maxy) = self.bounds

        if opt == 'grid':
            # determine (0,0) node
            left_x = int(abs(minx)/res)
            right_x = int(abs(maxx)/res)
            down_y = int(abs(miny)/res)
            up_y = int(abs(maxy)/res)

            def real_coord(point):
                (x,y) = point
                x = round((x - left_x)*res, 2)
                y = round((y - down_y)*res, 2)

                return (x,y)

            num_x = left_x + right_x + 1
            num_y = up_y + down_y + 1

            graph = nx.grid_2d_graph(num_x, num_y)
            graph = nx.relabel_nodes(graph, real_coord)
        else:
            # different sampling options
            if opt == 'random':
                pos = {i: (random.uniform(minx, maxx), random.uniform(miny, maxy)) for i in
                        range(n)}
            elif opt == 'smartrand':
                pos = self.sample_points(n)
            elif opt == 'halton':
                pos = self.halton_sample(n, 2, 3)
            else:
                logger.error('Not a valid option!')

            pos.update({n: start, n+1: goal})
            graph = nx.random_geometric_graph(n+2, res, pos=pos)

        # TODO: modify graph to select better edges

        return graph

    def sample_points(self, n):
        (minx, maxx, miny, maxy) = self.bounds
        (imgheight, imgwidth) = self.occ_grid.shape

        pos = dict()
        i = 0
        while i < n:
            sample = (random.uniform(minx, maxx), random.uniform(miny, maxy))

            # check if sample is feasible
            box = BoundingBox(self.robot_width/2, sample)
 
            left = max(box.left[0], minx)
            right = min(box.right[0], maxx)
            up = min(box.top[1], maxy)
            down = max(box.bottom[1], miny)

            # Calculate which pixels need to be checked, these pixels have to be included
            # (0,0) is in top left of image
            leftpx = int(abs(left - minx)/self.img_res)
            rightpx = min(int(abs(right - minx)/self.img_res), imgwidth - 1)
            toppx = int(abs(maxy - up)/self.img_res)
            downpx = min(int(abs(maxy - down)/self.img_res), imgheight - 1)

            # do a quick, dirty check for a good sample
            # aka as soon as a pixel is too dark don't put down the sample
            for col in xrange(leftpx, rightpx + 1):
                for row in xrange(toppx, downpx + 1):
                    value = self.occ_grid[row, col]
                    if value < 30 and value != 0.0:
                        i = i - 1 # repeat until you get a 'good' sample
                        break
                else:
                    continue
                break
            else:
                pos.update({i: sample}) # only executed if it does not break
                i = i + 1

        return pos

    def halton_sample(self, n, xb=2, yb=3):
        # returns n halton samples in 2-d 
        # xb, yb are the bases for each dimension respectively, must be PRIME
        (minx, maxx, miny, maxy) = self.bounds

        pos = dict()
        i = 0
        # for each halton point, multiply by width or height and translate as needed
        for i in xrange(n):
            x = halton_seq(i,xb)*(maxx - minx) + minx
            y = halton_seq(i,yb)*(maxy - miny) + miny
            pos.update({i: (x,y)})

        return pos

    def calc_bounding_coord(self):
        # returns min and max of x and y respectively
        (imgheight, imgwidth) = self.occ_grid.shape
        (ox, oy, ot) = self.origin

        width = imgwidth*self.img_res
        height = imgheight*self.img_res

        minx = round(ox, 2)
        maxx = round(width + ox, 2)
        miny = round(oy, 2)
        maxy = round(height + oy, 2)

        return [minx, maxx, miny, maxy]


class BoundingBox(object):
    # bounding box of an edge

    def __init__(self, padding,  a, b=None):
        """
        Determine corner coordinates
        edge - (a,b) where a,b are cartesian coordinates
        padding - how wide box should extend from edge

        Corners can be outside of image boundaries 

        """
        # if no other point is specified, create bounding box around a point
        if b == None: b = a 

        (x1, y1) = a
        (x2, y2) = b

        # edge line parameters
        if utility.isclose(x1 - x2, 0) or utility.isclose(y1 - y2, 0):
            # if edge is vertical or horizontal
            minx = min(x1 - padding, x2 - padding)
            maxx = max(x1 + padding, x2 + padding)
            maxy = max(y1 + padding, y2 + padding)
            miny = min(y1 - padding, y2 - padding)
            self.left = (minx, maxy)    # top left
            self.right = (maxx, miny)   # bottom right
            self.top = (maxx, maxy)     # top right
            self.bottom = (minx, miny)  # bottom left

            self.m = None # indicates vertical edge
        else:
            m = (y1 - y2)/(x1 - x2)
            b = y1 - m*x1

            # calculate boundary lines parallel to edge line
            # dy_ll = padding*math.sin(math.atan(-m))
            dy_ll = padding/(math.cos(math.pi/2 - abs(math.atan(-1/m))))
            b_ll1 = b - dy_ll
            b_ll2 = b + dy_ll

            # calculate boundary lines perpendicular to edge line
            dy_T = padding/(math.cos(math.pi/2 - abs(math.atan(m))))
            b1 = y1 + x1/m
            b2 = y2 + x2/m
            if y1 > y2:
                b_T1 = b1 + dy_T
                b_T2 = b2 - dy_T
            else:
                b_T1 = b1 - dy_T
                b_T2 = b2 + dy_T

            # now we have the parameters for all the boundary lines,
            # we need to calc the four corners
            cx1 = (b_T1 - b_ll1)/(m + 1/m)
            cy1 = m*cx1 + b_ll1

            cx2 = (b_T1 - b_ll2)/(m + 1/m)
            cy2 = m*cx2 + b_ll2

            cx3 = (b_T2 - b_ll1)/(m + 1/m)
            cy3 = m*cx3 + b_ll1

            cx4 = (b_T2 - b_ll2)/(m + 1/m)
            cy4 = m*cx4 + b_ll2

            corners = [(cx1, cy1), (cx2, cy2), (cx3, cy3), (cx4, cy4)]

            # order corners
            self.left = min(corners, key=lambda e: e[0])
            self.right = max(corners, key=lambda e: e[0])
            self.bottom = min(corners, key=lambda e: e[1])
            self.top = max(corners, key=lambda e: e[1])

            self.m = m # useful to have

    def maxY(self, x):
        # x is the coordinate of the 'slice' I want to check
        # return the top of the slice that's in the box

        if x < self.left[0] or x > self.right[0]: 
            logger.error("x is outside of the bounding box!")
            return None

        if self.m == None or utility.isclose(self.m, 0):
            # vertical/horizontal edge, return maxy
            return self.top[1]

        # otherwise if x < top corner, calc intersection w/ left to top
        if x < self.top[0]:
            m = (self.top[1] - self.left[1])/(self.top[0] - self.left[0])
            b = self.top[1] - m*self.top[0]
            return m*x + b
        # if x > top corner, calc intersection w/ top to right
        else:
            m = (self.top[1] - self.right[1])/(self.top[0] - self.right[0])
            b = self.top[1] - m*self.top[0]
            return m*x + b

    def minY(self, x):
         # x is the coordinate of the 'slice' I want to check
        # return the bottom of the slice that's in the box

        if x < self.left[0] or x > self.right[0]: 
            logger.error("x is outside of the bounding box!")
            return None

        if self.m == None or utility.isclose(self.m, 0):
            # vertical/horizontal edge, return maxy
            return self.bottom[1]

        # otherwise if x < bottom corner, calc intersection w/ left to bottom 
        if x < self.bottom[0]:
            m = (self.bottom[1] - self.left[1])/(self.bottom[0] - self.left[0])
            b = self.bottom[1] - m*self.bottom[0]
            return m*x + b
        # if x > bottom corner, calc intersection w/ bottom to right
        else:
            m = (self.bottom[1] - self.right[1])/(self.bottom[0] - self.right[0])
            b = self.bottom[1] - m*self.bottom[0]
            return m*x + b

def halton_seq(i,b):
    # i - index
    # b - base
    # calculates ith number in halton sequence of base b

    f = 1.0
    r = 0.0

    while i > 0:
        f = f/b
        r = r + f*(i % b)
        i = math.floor(i/b)

    return r

def halton_gen(size, b):
    # generator for halton sequence
    n = 0
    while n < size:
        yield halton_seq((n + 1),b)
        n += 1

def get_origin(path):
    # path - path to yaml file we are interested in
    # returns diff corresponding .pgm file that is the origin
    # (if for some reason the returned diff is negative, graph creator will have to deal
    # with that)
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
