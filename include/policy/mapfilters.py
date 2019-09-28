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
    """
    This filter only merges maps with the first encountered agreeing map, provided it is
    not a subset of any other map.
    It will update n of all agreeing maps, regardless of whether it is merging or not.

    supermaps - list of type Maps
    new_map - type Map
    """
    agree_once = False # flag to indicate if new_map agrees w/ any stored maps
    map_merge = None
    update = {}

    for i,m in enumerate(supermaps):
        in_agreement, new_info = m.agrees_with(new_map)
        if in_agreement:
            m.n += 1
            logger.info("New map agrees with supermap {}".format(i))

            if (new_info != {} and agree_once == False):
                map_merge = m
                n_map_merge = i
                update = new_info
            elif new_info == {}:
                map_merge = None

            agree_once = True

    if not agree_once:
        # no maps were in agreement, add new map to supermaps
        supermaps.append(new_map)
        print("Added new map")
    elif map_merge != None:
        # merge
        print("Merging to map {}".format(n_map_merge))
        logger.info(new_info)
        map_merge.updateG(update)
    else:
        print("No map added")

    return supermaps

