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
            ('g', {'defn': Point((-1, 8.5))}),
            # (1, {'defn': LineString([(1.25, 1.75),(1.25, -1.75)])}),
            # (2, {'defn': LineString([(3.8, 1.75),(3.8, -1.75)])}),
            # (3, {'defn': LineString([(6.9, 1.75),(6.9, -1.75)])}),
            (4, {'defn': LineString([(6.9, 2.0),(9.5, 2.0)])}),
            (5, {'defn': LineString([(6.8, 4.3),(6.8, 2.5)])}),
            (6, {'defn': LineString([(3.75, 4.25),(3.75, 2.5)])}),
            (7, {'defn': LineString([(1.25, 2.0),(3.75, 2.0)])}),
            (8, {'defn': LineString([(1.25, 4.4),(3.75, 4.4)])}),
            (9, {'defn': LineString([(1, 6.5),(1, 4.4)])}),
            (10, {'defn': LineString([(3.75, 6.75),(3.75, 5)])}),
            (11, {'defn': LineString([(6.8, 6.8),(6.8, 5)])}),
            (12, {'defn': LineString([(6.8, 4.4),(9.5, 4.4)])}),
            (13, {'defn': LineString([(6.8, 7),(9.5, 7)])}),
            # (14, {'defn': LineString([(6.75, 9.5),(6.75, 7.5)])}),
            # (15, {'defn': LineString([(3.6, 9.5),(3.6, 7.5)])}),
            (16, {'defn': LineString([(1, 7),(3.6, 7)])}),
            ]
    """
    edge_list=[('s',1),(1,2),(2,3),(3,4),(4,5),(4,12),
               (5,6),(6,7),(7,1),(7,2),(7,8),(8,9),(8,16),(8,10),
               (10,11),(11,12),(12,13),(11,13),(13,14),(14,15),(15,16),
               (13,'g'),(14,'g')
              ]
    """
    edge_list=[('s',7),('s',4),(7,4),
               (7,8),(7,6),(6,8),
               (5,6),
               (4,5),(4,12),(5,12),
               (8,9),(8,16),(8,10),(9,10),(9,16),(10,16),
               (10,11),
               (12,11),(12,13),(11,13),
               (16,13),
               (9,'g') 
              ]
    # edge_list=[]

    create_graph('tristan_maze_tgraph.yaml', node_list, edge_list)

def test_large():
    node_list=[
            ('s', {'defn': Point((18.5, 18.5))}),
            ('g', {'defn': Point((18.3, 3.7))}),
            (1, {'defn': LineString([(17.0, 17.5),(19.6, 17.5)])}),
            (2, {'defn': LineString([(13.3, 17.5),(15.3, 17.5)])}),
            (3, {'defn': LineString([(9.2, 17.5),(11.2, 17.5)])}),
            (4, {'defn': LineString([(0.5, 17.5),(2.8, 17.5)])}),
            (5, {'defn': LineString([(3.0, 12.6),(6.0, 12.6)])}),
            (6, {'defn': LineString([(16.7, 5.6),(16.7, 7.6)])}),
            (7, {'defn': LineString([(13.2, 8.2),(15.2, 8.2)])}),
            (8, {'defn': LineString([(9.2, 8.2),(11.2, 8.2)])}),
            (9, {'defn': LineString([(5.8, 2.5),(5.8, 4.5)])}),
            (10, {'defn': LineString([(2.8, 5.8),(2.8, 7.8)])}),
            (11, {'defn': LineString([(16.8, 2.2),(19.6, 2.2)])}),
            (12, {'defn': LineString([(16.7, 2.6),(16.7, 4.6)])}),
            (13, {'defn': LineString([(0.5, 2.2),(2.8, 2.2)])}),
            (14, {'defn': LineString([(10.0, 0.5),(10.0, 2.2)])})
            ]

    edge_list=[('s',1),('s',2),('s',3),('s',4),
               (1,6),(1,2),(1,3),(1,4),(2,3),(2,4),(3,4),
               (2,7),(8,5),(3,5),(4,10),(10,13),(4,13),
               (12,6),(12,7),(12,8),(12,9),
               (6,7),(6,8),(6,9),(7,8),(7,9),(8,9),
               (9,10),(11,14),(13,14),(11,12),
               ('g',11), ('g',12)
            ]

    create_graph('test_large_tgraph.yaml', node_list, edge_list)

def robohub_tgraph():
    # point locations need to be redefined later
    node_list=[
            ('s', {'defn': Point((0.0, 0.0))}),
            ('g', {'defn': Point((5.0, 0.0))}),
            (1, {'defn': LineString([(0.8, 4.5),(0.8, 2.1)])}),
            (2, {'defn': LineString([(0.8, 1.5),(0.8, 0.4)])}),
            (3, {'defn': LineString([(0.8, -0.2),(0.8, -1.4)])}),
            (4, {'defn': LineString([(0.8, -2.0),(0.8, -4.5)])}),
            (5, {'defn': LineString([(1.8, 2.1),(1.8, 1.0)])}),
            (6, {'defn': LineString([(1.8, -0.7),(1.8, -2.0)])}),
            (7, {'defn': LineString([(3.0, 4.5),(3.0, 2.1)])}),
            (8, {'defn': LineString([(3.0, 0.8),(3.0, -0.6)])}),
            (9, {'defn': LineString([(3.0, -2.0),(3.0, -4.5)])}),
            (10, {'defn': LineString([(0.8, 0.0),(1.8, 0.0)])})
            ]

    edge_list=[('s',1),('s',2),('s',3),('s',4),
               (1,2), (1,3), (1,4), (2,3), (2,4), (3,4),
               (1,7), (4,9),
               (2,5), (2,10), (5,10), (3,10), (3,6), (6,10),
               (5,6), (5,8), (6,8),
               (7,8), (7,9), (8,9),
               ('g',7), ('g',8), ('g', 9)
            ]

    create_graph('robohub_test_tgraph.yaml', node_list, edge_list)

def e7_tgraph():
    # point locations need to be redefined later
    node_list=[
            ('s', {'defn': Point((7.1, 4.8))}),
            ('g', {'defn': Point((15.0, 10.0))}),
            (1, {'defn': LineString([(4.2, 9.1),(7.7, 9.1)])}),
            (2, {'defn': LineString([(7.7, 9.1),(9.7, 9.1)])}),
            (3, {'defn': LineString([(9.7, 5.1),(9.7, 8.1)])}),
            (4, {'defn': LineString([(9.7, 3.1),(9.7, 5.1)])}),
            (5, {'defn': LineString([(7.7, 17.1),(7.7, 20.1)])}),
            (6, {'defn': LineString([(7.7, 20.1),(7.7, 24)])}),
            (7, {'defn': LineString([(9.7, 13.1),(9.7, 15.1)])}),
            (8, {'defn': LineString([(13.7, 15.1),(13.7, 18.1)])}),
            (9, {'defn': LineString([(11.7, 15.1),(13.7, 15.1)])}),
            (10, {'defn': LineString([(13.7, 8.5),(13.7, 11.5)])}),
            (11, {'defn': LineString([(13.7, 3.1),(13.7, 5.1)])}),
            (12, {'defn': LineString([(13.7, 21.1),(13.7, 25.1)])})
            ]

    edge_list=[('s',1), ('s',2), ('s',3), ('s',4),
               (1,2), (1,3), (1,4), (2,3), (2,4), (3,4),
               (1,5), (1,6), (5,6),
               (2,7),
               (5,8), (5,9), (8,9),
               (3,7), (3,9), (3,10), (7,9), (7,10), (9,10),
               (4,11),
               ('g',8), ('g',10), ('g',11), ('g',12),
               (8,10), (8,11), (8,12), (10,11), (10,12), (11,12),
               (6,12)
            ]

    create_graph('e7_tgraph.yaml', node_list, edge_list)

if __name__ == '__main__':
    # tristan_maze_tgraph()
    test_large()
    # robohub_tgraph()
    # e7_tgraph()
