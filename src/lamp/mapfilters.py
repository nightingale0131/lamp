"""
Written by: Florence Tsang
Creation date: 2019-07-25

Possible different map filters to try out
"""

#!/usr/bin/python
import logging
logger = logging.getLogger(__name__)

import networkx as nx
import utility as util
from copy import copy

def update_p_est(M,t):
    """ Return updated estimated prob distribution
        M = list of maps
        t = total number of tasks so far (+1 b/c of base_graph)
    """
    p = []
    for m in M:
        n = m.n # number of times this map has been encountered
        prob = float(n)/t
        p.append(prob)

    return p

def update_weights(supermaps, new_map):
    new_G = copy(new_map.G) # in case new_map was added to supermaps
    base_map = supermaps[0]
    for (u,v) in base_map.G.edges():
        if new_G.edge_state(u,v) == new_G.UNBLOCKED:
            old_weight = base_map.G.weight(u,v)
            new_weight = util.moving_average(old_weight, new_G.weight(u,v))
            base_map.G.set_edge_weight(u,v,new_weight)
            logger.debug("Modifying edge ({},{}): {:.2f} -> {:.2f}"
                  .format(u,v,old_weight, new_weight))

        for m in supermaps[1:]:
            if m.G.edge_state(u,v) != m.G.BLOCKED:
                m.G.set_edge_weight(u,v,base_map.G.weight(u,v))

    for m in supermaps:
        m.update_cost(m.G.goal)

    return supermaps

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
    info = {'agree': [], 'merged': None} # dictionary for storing info to write in file later

    for i,m in enumerate(supermaps):
        in_agreement, new_info = m.agrees_with(new_map)
        if in_agreement:
            m.n += 1
            info['agree'].append(i)
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
        logger.info("Added new map")
    elif map_merge != None:
        # merge
        logger.info("Merging to map {}".format(n_map_merge))
        logger.info(new_info)
        map_merge.updateG(update)
        info['merged'] = n_map_merge
    else:
        logger.info("No map added")

    return supermaps, info

