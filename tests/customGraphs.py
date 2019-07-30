#!/usr/bin/env python

"""
Create custom graphs for the test cases
"""
import networkx as nx
from shapely.geometry import Point, LineString, Polygon

def create_graph(filename, node_list, edge_list):
    G = nx.Graph()
    G.add_nodes_from(node_list)
    G.add_edges_from(edge_list)
    nx.write_yaml(G,filename)

def tristan_maze():
    node_list=[
            ('s', {'pos':(0.0, 0.0)}),
            ('g', {'pos':(8.5, 8.5)}),
            (1, {'pos':(3.0, 0.0)}),
            (2, {'pos':(8.5, 0.0)}),
            (3, {'pos':(3.0, 3.5)}),
            (4, {'pos':(8.5, 3.5)}),
            (5, {'pos':(8.5, 6.0)}),
            (6, {'pos':(3.0, 8.5)}),
            (7, {'pos':(3.0, 6.0)}),
            (8, {'pos':(3.0, 5.5)}),
            (9, {'pos':(-1.0, 3.0)}),
            (10, {'pos':(-1.0, 5.5)}),
            (11, {'pos':(-1.0, 8.0)})
            ]
    edge_list=[('s', 1), (1,2), (2,4), (4,5), (5,'g'),
            (1,3), (3,8), (3,4), (8,7), (8,10), (7,5),
            (7,6), (6,'g'), (9,10), (10,11)]

    create_graph('tristan_maze_custom.yaml', node_list, edge_list)

def tristan_maze_tgraph():
    node_list=[
            ('s', {'defn': Point((0.0, 0.0))}),
            ('g', {'defn': Point((8.5, 8.5))}),
            (1, {'defn': LineString([(1.25, 1.75),(1.25, -1.75)])}),
            (2, {'defn': LineString([(3.8, 1.75),(3.8, -1.75)])}),
            (3, {'defn': LineString([(6.9, 1.75),(6.9, -1.75)])}),
            (4, {'defn': LineString([(6.9, 2.0),(9.5, 2.0)])}),
            (5, {'defn': LineString([(6.8, 4.3),(6.8, 2.5)])}),
            (6, {'defn': LineString([(3.75, 4.25),(3.75, 2.5)])}),
            (7, {'defn': LineString([(1.25, 1.75),(3.75, 1.75)])}),
            (8, {'defn': LineString([(1.25, 4.4),(3.75, 4.4)])}),
            (9, {'defn': LineString([(1, 6.5),(1, 4.4)])}),
            (10, {'defn': LineString([(3.75, 6.75),(3.75, 5)])}),
            (11, {'defn': LineString([(6.8, 6.8),(6.8, 5)])}),
            (12, {'defn': LineString([(6.8, 5),(9.5, 5)])}),
            (13, {'defn': LineString([(6.8, 7),(9.5, 7)])}),
            (14, {'defn': LineString([(6.75, 9.5),(6.75, 7.5)])}),
            (15, {'defn': LineString([(3.6, 9.5),(3.6, 7.5)])}),
            (16, {'defn': LineString([(1, 7),(3.6, 7)])}),
            ]

    # edge_list=[('s',1),(1,2),(2,3),(3,4)]
    edge_list=[]

    create_graph('tristan_maze_tgraph.yaml', node_list, edge_list)

if __name__ == '__main__':
    tristan_maze_tgraph()
