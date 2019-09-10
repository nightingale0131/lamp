"""
Written by: Florence Tsang
Creation date: 2019-07-25

Possible different map filters to try out
"""

#!/usr/bin/python
import logging
logger = logging.getLogger(__name__)

import networkx as nx

def filter1(supermaps, new_map):
    # supermaps - list of type Maps
    # new_map - type Map
    agree_once = False # flag to indicate if new_map agrees w/ any stored maps
    
    for i,m in enumerate(supermaps):
        in_agreement, new_info = m.agrees_with(new_map)
        if in_agreement:
            agree_once = True
            m.n += 1
            print("New map agrees with supermap {}".format(i))
            print(new_info)

    if not agree_once:
        supermaps.append(new_map)
        print("Added new map")
    else:
        print("No map added")

    return supermaps

