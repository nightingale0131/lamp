#!/usr/bin/env python

import logging
logger = logging.getLogger(__name__)

import networkx as nx
import numpy as np
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
        self.robot_width = robot_width # assuming robot is a square, what is the max_length?

        # set up graph as soon as you read occ_grid in
        # if graph=None then build a graph
        # otherwise align the graph with the new occ grid, assuming the 
        #   origins are all in the same location in ref to the world
        self.occ_grid = cv2.imread(gridfile, cv2.IMREAD_GRAYSCALE)
        self.origin, self.img_res = get_origin(yamlfile)
        self.cost_to_goal = {}
        self.path_to_goal = {}

        if refmap==None:
            (self.imgheight, self.imgwidth) = self.occ_grid.shape
            self.bounds = self.calc_bounding_coord()
            self.graph = self.build_graph(graph_res, (0,0), goal, n=200)
            self._collision_check(True)
            logger.info('Built graph')
        else:
            self.graph = refmap.graph.copy() 
            self.start = refmap.start
            self.goal = refmap.goal
            logger.info('Imported graph')
            # self._align(refmap.occ_grid)
            # self.origin = refmap.origin
            # logger.debug('Modified origin: {}'.format(self.origin))
            (self.imgheight, self.imgwidth) = self.occ_grid.shape
            self.bounds = self.calc_bounding_coord()
            self._collision_check()

        # check which edges are blocked/unblocked
        logger.info('Finished initialization')

    def update_cost_to_goal(self):
        cost, paths = nx.single_source_dijkstra(self.graph, self.goal, weight = 'dist')
        self.cost_to_goal = cost
        self.path_to_goal = paths

    def get_cost_to_goal(self, v):
        return self.cost_to_goal[v]

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
            Currently not used.
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

    def pos(self, v):
        return self.graph.node[v]['pos']

    def dist(self, u, v):
        (x1, y1) = self.graph.node[u]['pos']
        (x2, y2) = self.graph.node[v]['pos']

        return math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

    def weight(self, u, v, attr=None):
        # assuming this will be used with networkx dijkstra
        # u,v - node label, should be in (x,y) format
        # attr - dictionary of edge attributes, this is not used, but these 3 arguments
        #        are needed to satisfy networkx dijkstra function requirements
        # considers unknowns as unblocked edges
        # need to check edge state 
        if self.graph[u][v]['state'] == self.BLOCKED:
            return float('inf')
        else:
            return self.dist(u,v)

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
        # (a,b) = edge # comment this for random graph
        (u,v) = edge
        # Below is for random graph
        # a = u
        # b = v
        a = self.graph.node[u]['pos']
        b = self.graph.node[v]['pos']

        # TODO: if edge is outside of map bounds, return as unknown

        padding = self.robot_width/2
        box = BoundingBox( padding, a, b)
        # calculate boundaries of bounding box to fit in occ_grid boundaries
        (left, right, up, down) = box.mod_bounds(self.bounds)

        # Calculate which pixels need to be checked, these pixels have to be included
        # (0,0) is in top left of image
        leftpx = int(abs(left-minx)/self.img_res)
        rightpx = min(int(abs(right-minx)/self.img_res), self.imgwidth - 1)
        toppx = int(abs(maxy - up)/self.img_res)
        downpx = min(int(abs(maxy - down)/self.img_res), self.imgheight - 1)

        logger.info('Checking edge (({:.3f}, {:.3f}),({:.3f}, {:.3f}))'
                    .format(a[0], a[1], b[0], b[1])) 
        logger.debug('\tleft={:.3f}, right={:.3f}, top={:.3f}, down={:.3f}'
                    .format(left, right, up, down))
        logger.debug('\tIn pixels: left={}, right={}, top={}, down={}'
                    .format(leftpx, rightpx, toppx, downpx))

        pxbounds = [leftpx, rightpx, toppx, downpx]

        # If needed, calculate the importance of each pixel in bounding box
        '''
        try:
            importance = self.graph.edge[a][b]['importance']
            # ^ dictionary: {(row,col): value}
        except KeyError:
            logger.info('\tCalculating blur')
            logger.debug('Box corners: {}, {}, {}, {}'
                    .format(box.left, box.right, box.top, box.bottom))
            importance = gaussblur(box, self.bounds, pxbounds, self.img_res, 3)
            self.graph.edge[a][b]['importance'] = importance
        '''      
        logger.info('\tCalculating blur')
        logger.debug('Box corners: {}, {}, {}, {}'
                .format(box.left, box.right, box.top, box.bottom))
        #importance = gaussblur(box, self.bounds, pxbounds, self.img_res, 3)
        
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

            # dealing with case where corner is in pixel
            if box.top[0] > leftx and box.top[0] < rightx:
                slice_topy = box.top[1]
            else:
                slice_topy = max(box.maxY(leftx), box.maxY(rightx))

            if box.bottom[0] > leftx and box.bottom[0] < rightx:
                slice_boty = box.bottom[1]
            else:
                slice_boty = min(box.minY(leftx), box.minY(rightx))

            toppx = int(abs(maxy - slice_topy)/self.img_res)
            downpx = min(int(abs(maxy - slice_boty)/self.img_res), self.imgheight - 1)

            for row in xrange(toppx, downpx + 1):
                p = self.occ_grid[row,col]/254.0
                n += 1
                if p == 0:
                    p = 1
                    k += 1
                elif p > 0.50: p = 1

                #w = importance[(row,col)]
                #prob_free = prob_free*math.pow(p, w)
                prob_free = prob_free*p

        # Assign the following states to the edge:
        # 0:unblocked, 1:blocked, -1:unknown
        prob_unknown = float(k)/n
        logger.info("\tP(free) = {:.3f}, P(unknown)  = {:.3f}"
                .format(prob_free, prob_unknown))

        self.graph[u][v]['prob'] = prob_free
        self.graph[u][v]['dist'] = self.dist(u,v)
        self.graph[u][v]['weight'] = self.graph[u][v]['dist']*prob_free

        # Thresholding of edges to states
        # The following is problematic/too simple
        if prob_free < 0.2: self.graph[u][v]['state'] = self.BLOCKED
        elif prob_unknown > 0.5: 
            self.graph[u][v]['state'] = self.UNKNOWN
            self.graph[u][v]['prob'] = 0.5 
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
            self.start = n
            self.goal = n+1
            # graph = nx.relabel_nodes(graph, pos)

        # TODO: modify graph to select better edges

        return graph

    def sample_points(self, n):
        (minx, maxx, miny, maxy) = self.bounds

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
            rightpx = min(int(abs(right - minx)/self.img_res), self.imgwidth - 1)
            toppx = int(abs(maxy - up)/self.img_res)
            downpx = min(int(abs(maxy - down)/self.img_res), self.imgheight - 1)

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
        (ox, oy, ot) = self.origin

        width = self.imgwidth*self.img_res
        height = self.imgheight*self.img_res

        minx = round(ox, 2)
        maxx = round(width + ox, 2)
        miny = round(oy, 2)
        maxy = round(height + oy, 2)

        return [minx, maxx, miny, maxy]

class LiveGridGraph(GridGraph):
    # child class of GridGraph
    # for capturing info from a live \map feed instead of image
    def __init__(self, occ_grid, refmap, robot_width):
        logger.info('Capturing map info from live feed')
        """
        - take in map topic from cartographer, get all of the needed parameters
          occ_grid - OccupancyGrid type msg from ROS
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
        self.BLOCKED = 1
        self.UNBLOCKED = 0
        self.UNKNOWN = -1

        self.robot_width = robot_width
        self.occ_grid = occ_grid.data
        self.origin = [occ_grid.info.origin.position.x,
                       occ_grid.info.origin.position.y,
                       occ_grid.info.origin.position.z]
        self.img_res = occ_grid.info.resolution
        self.imgwidth = occ_grid.info.width
        self.imgheight = occ_grid.info.height
        self.bounds = self.calc_bounding_coord()
        self.graph = refmap.graph.copy()
        self.start = refmap.start
        self.goal = refmap.goal

        # convert occ_grid.data into numpy matrix so then occ_grid[row, col] will work
        self.occ_grid = list_to_matrix(occ_grid.data, self.imgwidth, self.imgheight)
        self.resize_map(refmap)

    def resize_map(self, refmap):
        #assuming no rotational offset between maps
        # assuming same resolution
        new_y = refmap.imgheight #cells
        new_x = refmap.imgwidth #cells
        new_occ = np.empty((new_y,new_x))
        delta_x = refmap.origin[0] - self.origin[0] #meters
        delta_y = refmap.origin[1] - self.origin[1] #meters
        delta_x = round(delta_x/self.img_res) #cells
        delta_y = round(delta_y/self.img_res) #cells
        delta_y = self.imgheight-delta_y-refmap.imgheight
        for row in xrange(new_y):
            for col in xrange(new_x):
                if -delta_x <= col < self.imgwidth-delta_x and -delta_y <= row < self.imgheight-delta_y:
                    new_occ[row,col] = self.occ_grid[row+delta_y,col+delta_x]
                else:
                    new_occ[row,col] = 0 #fill extra cells with unknown
        self.imgheight = new_y
        self.imgwidth = new_x
        self.occ_grid = new_occ
        self.origin = refmap.origin
        self.bounds = self.calc_bounding_coord()
        return

def list_to_matrix(raw_data, width, height):
    matrix = np.empty((height, width))
    for row in xrange(height):
        for col in xrange(width):
            # change to match value in pgm files
            data = raw_data[(height - row - 1)*width + col]
            if data <= 55 : data = 0 
            matrix[row,col] = (100 - data)*2.54

    return matrix

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

    def mod_bounds(self, bounds):
        # assuming bounds = [minx, maxx, miny, maxy]
        # Given these bounds (must be rectangle), determine the left, right, top, bottom
        # of the bounding box such that it is completely contained in the bounds

        [minx, maxx, miny, maxy] = bounds

        # left
        if self.left[0] < minx: left = minx
        elif self.left[1] > maxy: left = self.minX(maxy)
        elif self.left[1] < miny: left = self.minX(miny)
        else: left = self.left[0]

        # right
        if self.right[0] > maxx: right = maxx 
        elif self.right[1] > maxy: right = self.maxX(maxy)
        elif self.right[1] < miny: right = self.maxX(miny)
        else: right = self.right[0]

        # top
        if self.top[1] > maxy: top = maxy
        elif self.top[0] < minx: top = self.maxY(minx)
        elif self.top[0] > maxx: top = self.maxY(maxx)
        else: top = self.top[1]

        # bottom
        if self.bottom[1] < miny: bottom = miny
        elif self.bottom[0] < minx: bottom = self.minY(minx)
        elif self.bottom[0] > maxx: bottom = self.minY(maxx)
        else: bottom = self.bottom[1]
 
        return (left, right, top, bottom)

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
            # vertical/horizontal edge, return miny 
            return self.bottom[1]

        # otherwise if x < bottom corner, calc intersection w/ left to bottom 
        if x < self.bottom[0]:
            m = (self.bottom[1] - self.left[1])/(self.bottom[0] - self.left[0])
            b = self.bottom[1] - m*self.bottom[0]
        # if x > bottom corner, calc intersection w/ bottom to right
        else:
            m = (self.bottom[1] - self.right[1])/(self.bottom[0] - self.right[0])
            b = self.bottom[1] - m*self.bottom[0]

        return m*x + b

    def minX(self, y):
        # y is the coordinate of the horizontal slice I want to check
        # return left-most corner of slice

        if y < self.bottom[1] or y > self.top[1]: 
            logger.error("y is outside of the bounding box!")
            return None

        if self.m == None or utility.isclose(self.m, 0):
            # if box is aligned with axis, return minx 
            return self.left[0]

        # otherwise if y > left corner, calc intersection w/ left to top
        if y > self.left[1]:
            m = (self.top[1] - self.left[1])/(self.top[0] - self.left[0])
            b = self.left[1] - m*self.left[0]
        # if y < left corner, calc intersection w/ bottom to left 
        else:
            m = (self.bottom[1] - self.left[1])/(self.bottom[0] - self.left[0])
            b = self.left[1] - m*self.left[0]

        return (y - b)/m

    def maxX(self, y):
        # y is the coordinate of the horizontal slice I want to check
        # return right-most corner of slice

        if y < self.bottom[1] or y > self.top[1]: 
            logger.error("y is outside of the bounding box!")
            return None

        if self.m == None or utility.isclose(self.m, 0):
            # if box is aligned with axis, return maxx 
            return self.right[0]

        # otherwise if y > right corner, calc intersection w/ right to top
        if y > self.right[1]:
            m = (self.top[1] - self.right[1])/(self.top[0] - self.right[0])
            b = self.right[1] - m*self.right[0]
        # if y > top corner, calc intersection w/ right to bottom 
        else:
            m = (self.bottom[1] - self.right[1])/(self.bottom[0] - self.right[0])
            b = self.right[1] - m*self.right[0]

        return (y - b)/m

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

def gaussblur(box, bounds, pxbounds, img_res, k):
    # approx gauss blur by repeating box blur
    # return dict of weights w/ pixel coordinates as keys
    # more optimization: http://blog.ivank.net/fastest-gaussian-blur.html
    W = {}
    logger.debug("\tFirst blur")
    W = boxblur(box, bounds, pxbounds, img_res, k, W)
    logger.debug("\tSecond blur")
    W = boxblur(box, bounds, pxbounds, img_res, k, W)
    logger.debug("\tThird blur")
    W = boxblur(box, bounds, pxbounds, img_res, k, W)

    return W

def boxblur(box, bounds, pxbounds, img_res, k, weight):
    """
    Return weights for bounding box. Assume 1 for in box and 0 for out of box
    box - BoundingBox object 
    bounds - [minx, maxx, miny, maxy] cartesian boundaries of occ_grid
    pxbounds - [leftpx, rightpx, toppx, botpx] pixel boundaries

    k -  kernel size, must be odd number
    """
    logger.debug("size of weight = {}".format(len(weight)))
    W = boxblurV(box, bounds, pxbounds, img_res, k, weight)
    W = boxblurH(box, bounds, pxbounds, img_res, k, W)

    return W

def boxblurH(box, bounds, pxbounds, img_res, k, weight):
    # assume this will be run first
    (leftpx, rightpx, toppx, botpx) = pxbounds # px bounds of box
    (minx, maxx, miny, maxy) = bounds # bounds of environment
    r = k/2 # kernel radius
    if not weight: first_time = True
    else: first_time = False

    logger.debug("Horizontal blur...")

    for row in xrange(toppx, botpx + 1):
        topy = min(maxy - row*img_res, box.top[1])
        boty = max(maxy - (row + 1)*img_res, box.bottom[1])

        # Dealing with case where pixel contains corner
        if box.left[1] < topy and box.left[1] > boty:
            slice_leftx = box.left[0]
        else:
            slice_leftx = min(box.minX(topy), box.minX(boty))

        if box.right[1] < topy and box.right[1] > boty:
            slice_rightx = box.right[0]
        else:
            slice_rightx = max(box.maxX(topy), box.maxX(boty))

        # clamp pixel limits to within image
        slice_leftpx = max(int((slice_leftx - minx)/img_res), leftpx)
        slice_rightpx = min(int((slice_rightx - minx)/img_res), rightpx) 

        logger.debug("{}: {} - {}".format(row, slice_leftpx, slice_rightpx) +
            " [topy:{:.3f} boty:{:.3f}, {:.3f} - {:.3f}]"
            .format(topy, boty, slice_leftx, slice_rightx))

        # nspaces = slice_leftpx - leftpx # for debugging
        # print("".rjust(nspaces*5)), # for debugging

        for col in xrange(slice_leftpx, slice_rightpx + 1):
            # finally have the pixel I want to calc weight for
            val = 0
            for i in xrange(col - r, col + r + 1):
                if i < slice_leftpx or i > slice_rightpx: val += 0.0
                elif first_time == False: val += weight[(row, i)]
                else: val += 1.0

            weight.update({(row,col): val/(r + r + 1)})
            # print("{:.2f}".format(weight[(row,col)])), # for debugging

        # print(" ") # for debugging
    return weight

def boxblurV(box, bounds, pxbounds, img_res, k, weight):
    # assume this will be run first
    (leftpx, rightpx, toppx, botpx) = pxbounds # px bounds of box
    (minx, maxx, miny, maxy) = bounds # bounds of environment
    r = k/2 # kernel radius
    if not weight: first_time = True
    else: first_time = False


    logger.debug("Vertical blur...") # for debugging

    for col in xrange(leftpx, rightpx + 1):
        leftx = max(col*img_res + minx, box.left[0])
        rightx = min((col + 1)*img_res + minx, box.right[0])

        # dealing with case where corner is in pixel
        if box.top[0] > leftx and box.top[0] < rightx:
            slice_topy = box.top[1]
        else:
            slice_topy = max(box.maxY(leftx), box.maxY(rightx))

        if box.bottom[0] > leftx and box.bottom[0] < rightx:
            slice_boty = box.bottom[1]
        else:
            slice_boty = min(box.minY(leftx), box.minY(rightx))

        slice_toppx = max(int((maxy - slice_topy)/img_res), toppx)
        slice_botpx = min(int(abs(maxy - slice_boty)/img_res), botpx)

        logger.debug("{}: {} - {}".format(col, slice_toppx, slice_botpx) +
            " [leftx:{:.3f} rightx:{:.3f}, {:.3f} - {:.3f}]"
            .format(leftx, rightx, slice_topy, slice_boty))

        for row in xrange(slice_toppx, slice_botpx + 1):
            # finally have the pixel I want to calc weight for
            val = 0
            for i in xrange(row - r, row + r + 1):
                if i < slice_toppx or i > slice_botpx: val += 0.0
                elif first_time == False: val += weight[(i, col)]
                else: val += 1.0

            weight.update({(row,col): val/(r + r + 1)})

    return weight
