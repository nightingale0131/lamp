#!/usr/bin/env python

"""
Create custom graphs for the test cases
"""
import networkx as nx

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

if __name__ == '__main__':
    tristan_maze()
