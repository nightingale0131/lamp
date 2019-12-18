#!/usr/bin/env python

'''
Given an np array (grid), return a graph of entrances and connections
Based off of pre-processing algorithm in hpa* paper
'''

import networkx as nx
import itertools as it
from matplotlib import pyplot as plt

class HpaStar():
    def __init__(self, grid, threshold, size=10):
        # threshold - if cell value exceeds this, then consider it occupied
        self.threshold = threshold
        self.graph = nx.Graph()
        self.clusters = []
        self.entrances = [] 

        self.grid = grid

        self.abstract_grid(size)
        self.build_graph()

    def abstract_grid(self, size):
        self.build_clusters(size)
        for (c1, c2) in it.combinations(self.clusters, 2):
            if c1.adjacent(c2):
                self.entrances += self.build_entrances(c1, c2)

    def build_graph(self):
        # for each entrance in E:
        return

    def build_clusters(self, size):
        # break up grid into clusters
        (rows, cols) = self.grid.shape

        for r in xrange(0,rows,size):
            for c in xrange(0,cols,size):
                self.clusters.append(Cluster(self.grid, r, c, size))

    def build_entrances(self, c1, c2):
        # assume c1, c2 are adjacent
        entrances = []
        dr = c2.key[0] - c1.key[0]
        dc = c2.key[1] - c1.key[1]

        # Extract border from cluster (2 x size)
        # make sure borders are set correctly!
        if (dr, dc) == (0,1):
            c1_border = c1.min_row.copy()
            c2_border = c2.max_row.copy()
        elif (dr, dc) == (0,-1):
        elif (dr, dc) == (1,0):
        elif (dr, dc) == (-1,0):

        # set all free cells to 0
        c1_border[np.where(c1_border < self.threshold)] = 0
        c2_border[np.where(c2_border < self.threshold)] = 0
        border = np.array([c1_border, c2_border])

        # find entrance segments and get entrance pixel
        logger.info("Checking border:\n{}".format(border))
        new_seg = True
        seg_start = None
        for i,col in enumerate(border.T):
            if not col.any():
                if new_seg == True: 
                    seg_start = i
                    new_seg = False

                if i == len(border.T) - 1:
                    logger.debug("Entrance segment: ({},{})".format(seg_start, i))
                    entrances.append((i - seg_start)/2 + seg_start)
            else:
                new_seg = True
                if seg_start != None:
                    logger.debug("Entrance segment: ({},{})".format(seg_start, i-1))
                    entrances.append((i - 1 - seg_start)/2 + seg_start)

            logger.debug("i: {}, col: {}, new_seg: {}, seg_start: {}"
                        .format(i, col, new_seg, seg_start))

        logger.info("Entrances: {}".format(entrances))

        # need to set cluster entrances and modify entrances to full grid

        return entrances

    def get_cluster(self, cell):
        (r,c) = cell
        return

    def add_node(self, e):
        return

    def draw_clusters(self):
        # debugging purposes
        (rows, cols) = self.grid.shape
        (max_clr_r, gb) = max([cluster.key for cluster in self.clusters], key=lambda k: k[0])
        (gb, max_clr_c) = max([cluster.key for cluster in self.clusters], key=lambda k: k[1])
        print("subplots({}, {})".format(max_clr_r + 1, max_clr_c + 1))
        fig, axes = plt.subplots(max_clr_r + 1, max_clr_c + 1)

        for cluster in self.clusters:
            cluster.draw(axes)

        plt.show()

class Cluster():
    def __init__(self, grid, minr, minc, size):
        # assume minx, miny > 0
        # but we'll have to crop max if necessary
        self.entrances = {}
        self.minr = minr
        self.minc = minc

        (max_rows, max_cols) = grid.shape
        self.maxr = min(self.minr + size, max_rows)
        self.maxc = min(self.minc + size, max_cols) 
        self.key = (self.minr/size, self.minc/size)

        self.grid = grid[self.minr:self.maxr, self.minc:self.maxc]

    def adjacent(self, cluster):
        (r,c) = self.key
        if cluster.key in [(r+1, c), (r-1, c), (r, c+1), (r, c-1)]:
            return True

        return False

    def draw(self, axes):
        # debugging purposes
        # axes - array of ax objects from pyplot
        (r,c) = self.key
        ax = axes[r,c]
        ax.imshow(self.grid, cmap='gray')

    def min_row(self):
        return self.grid[0, :]

    def max_row(self):
        return self.grid[-1, :]

    def min_col(self):
        return self.grid[:, 0]

    def max_col(self):
        return self.grid[:, -1]
