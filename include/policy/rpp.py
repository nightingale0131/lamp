"""
Written by: Florence Tsang
Creation date: 2018-08-14

Compatible with Python 2.7

Contains main RPP solvers and additional functions
"""

#!/usr/bin/env python2
import logging
logger = logging.getLogger(__name__)
# import timing # measures runtime
import sys
import math
import networkx as nx

from collections import deque # queue class
from copy import deepcopy

import tgraph
# from gridgraph import GridGraph 
from classes import Cost, Node, Outcome, Observation
# from dijkstra import dijkstra
from utility import isclose, euclidean_distance
import costfns as cf

def solve_RPP(M, p, features, start, goal, robot_range=None, costfn=1):
    """ This is the modified version for LRPP 
        What's changed:
            - M: list of Map class objects (supermaps)
            - p: list of probabilities of each supermap
            - features: union of all features in each supermap (for future when we aren't
              using just edges as observations)
            - start: start vertex, should match key in M[i].G
            - goal:  goal vertex, should match key in M[i].G 
            - robot_range (optional): range of robot, only needed if using costfn3
            - costfn: sets which costfn to use [1,2,3]
    """
    logger.info('Running solve_RPPv2')
    if len(M) != len(p): sys.exit('solve_RPP failed: # supermaps and # prob mismatch.') 

    """ Setup graphs ------------------------------------------
        - all subgraphs need to have the same vertices
        - M=[M0, M1, ...]
        - assume M[0] is the master graph (all vertices defined)
    """
    environments=[int(n) for n in range(len(M))]
    base_map = M[0] # this is used pretty often

    # if any of the subgraph probabilities are 0, that shouldn't even be in the initial
    # belief
    init_Y=[i for i in environments if p[i] != 0]

    logger.info("Generating policy... -------------")
    policy=[Node(init_Y,start), Node(init_Y)]
    policy[0].add_outcome(policy[1])
    Q=deque([(init_Y,start,policy[-1])]) # [-1] means last element of list

    while len(Q) > 0:
        (Y,v,new_node)=Q.popleft()
        logger.info('Start of a new Q loop for {}'.format(v))

        # check if it is possible for robot to go to goal based on what it knows
        can_terminate=True
        for i in Y:
            if M[i].get_cost(goal, v) != float('inf'):
                can_terminate=False
                break

        if can_terminate == True:
            logger.info('Cannot reach goal')
            new_node.add_leg("no goal")
        else:
            # calculate P(Xy=i) of each i in Y
            p_sum=0
            p_Xy={}
            for i in Y:
                p_sum+=p[i]
            logger.debug('p_sum = {:.3f}'.format(p_sum))

            for i in Y:
                p_Xy[i]=p[i]/p_sum
                logger.info('p_Xy[{}] = {:.3f}'.format(i,p_Xy[i]))

            # update known graph & calc transition costs from v
            knownG = get_knownG(features, M, Y)
            logger.info("Calculating c_knownG")
            cost, paths = nx.single_source_dijkstra( knownG, v, weight =
            'weight')
            # c_knownG = Cost(dijkstra(knownG, knownG.known_weight, v))
            c_knownG = Cost(cost, paths)

            # compute R ----------------------------------------
            R, D = useful_features( features, M, p_Xy, c_knownG, Y, goal, robot_range,
                    costfn )
            # R - [(O1,v1), (O2, v2), ...]
            # D - {v1: cost_v1, v2: cost_v2, ...}
            logger.debug('R = {}'.format(str(R)))

            # find minO -------------------------------------------------------
            logger.info("Finding minO...")
            if len(R) == 0:
                # If R is empty, then it's cheaper to go to the goal than to go
                #   anywhere else
                logger.info('Add goal to policy')
                new_node.add_leg(goal,None,c_knownG.paths[goal])
            else:
                # else find the best observation (min of eqn 10), add tiebreaker
                minScore = float('inf') # result of eqn 10
                for item in R:
                    # calculate entropy of each observation
                    (obsv, v) = item
                    logger.debug('Calculating entropy of ({},{})'.format(obsv.E,v))

                    H = 0 # negated conditional entropy
                    for outcome in obsv.outcomes:
                        h = 0
                        for i in outcome.Yo:
                            # add conditional prob of supermaps that have mapped obsv
                            # p_cond -> P(Xy=i|Eo=E)
                            p_cond=p_Xy[i]/outcome.p
                            logger.debug('  p_cond of {} = {:.3f}'.format(i,p_cond))
                            if p_cond == 0:
                                h += 0 # log(0) returns NaN
                            else:
                                h += p_cond*math.log(p_cond)

                            for i in outcome.unknown:
                               # add conditional prob of supermaps that have not mapped
                               #    obsv 
                               p_cond = p_Xy[i]
                               logger.debug(' p_cond of {} = {:.3f}'.format(i,p_cond))
                               if p_cond == 0:
                                    h += 0 # log(0) returns NaN
                               else:
                                    h += p_cond*math.log(p_cond)

                        H += outcome.p*h

                    H = -H
                    score = D[v]*H
                    logger.info('({},{})  Entropy: {:.4f}  Score: {:.4f}'
                            .format(obsv.E, v, H,score))
                    if score < minScore:
                        minScore = score
                        minO = (obsv, v, D[v])

                    # in the event of multiple observations score = 0, select (O, v) w/ 
                    #   least expected cost to goal if we go to v
                    if score == 0 and minScore == 0:
                        if D[v] < minO[2]:
                            minO = (obsv, v, D[v])

                (min_o, min_u, min_cost) = minO
                logger.info("MinO: ({}, {})".format(min_o.E, min_u))

                (u1, u2) = min_o.E # assuming feature is an edge
                if u1 == min_u: unblocked_u = u2
                elif u2 == min_u: unblocked_u = u1
                else: 
                    raise Exception("min_u ({}) is neither u1 ({}) or u2 ({})!"
                        .format(min_u, u1, u2))

                minObservation = deepcopy(min_o)

                # set node observation, state, and path
                new_node.add_leg(min_u,minObservation,c_knownG.paths[min_u])

                # Add new children nodes and add those to the queue
                # For LAMP, there's only two possible outcomes: blocked/unblocked
                for outcome in min_o.outcomes:
                    policy.append(Node(outcome.new_belief()))
                    new_node.add_outcome(policy[-1])

                    if outcome.state == base_map.G.BLOCKED:
                        # starting vertex is beginning of edge
                        Q.append((outcome.new_belief(),min_u,policy[-1]))
                    elif outcome.state == base_map.G.UNBLOCKED:
                        Q.append((outcome.new_belief(),unblocked_u,policy[-1]))
                    else:
                        raise Exception("State of outcome is not blocked or unblocked!")

    return policy

def get_knownG(features, supermaps, belief):
    logger.info("Updating knownG...")
    base_map = supermaps[0]
    # knownG=type(base_map.G.graph)(base_map.G) # use same type as Map.G
    knownG=nx.Graph()
    knownG.add_nodes_from(base_map.G.vertices())
    # all feature states are unknown

    # Compare features for all supermaps
    # Features should be same type as keys in dict returned by supermaps.G.observe
    known_features = {}
    for feature in features:
        # logger.debug("Checking {}".format(feature))

        # If feature state is blocked in one supermap but unblocked in another supermap in
        # the belief, then it can't be in knownG.
        # If feature state is unknown in all beliefs, then add it to knownG so I can use
        # it
        state = None
        is_known = True
        for i in belief:
            new_state = supermaps[i].feature_state(feature)

            if new_state != base_map.G.UNKNOWN:
                if state == None: state = new_state
                if state != None and new_state != state: is_known = False

        # if is_known: known_features[feature] = state
        (a,b) = feature
        knownG.add_edge(a,b)
        if is_known and state != base_map.G.BLOCKED: 
            # assuming feature is an edge that looks like (a,b)
            knownG[a][b]['weight'] = base_map.G.weight(a,b) 
        else:
            knownG[a][b]['weight'] = float('inf')

    # logger.debug("known features = {}".format(known_features))
    logger.debug(nx.info(knownG))
    return knownG

def useful_features( features, supermaps, p_Xy, c_knownG, belief, goal, robot_range,
        costfn ):
    """
    Compute Rv
    Calculate set of reachable and constructive observations
    """

    # determine which features are constructive (compare feature subsets)-------
    logger.info('Checking constructive features...')
    constrO = [] # list of observation objects w/ constructive feature

    for feature in features:
        # init Observation object to store outcomes of feature
        obsv = Observation(feature)

        # vars for dealing with features that are UNKNOWN in some supermaps
        unknown_i = []
        has_unknowns = False
        outcome_norm = 0

        for i in belief:
            is_new_outcome = True
            possible_state = supermaps[i].feature_state(feature)

            if possible_state == supermaps[i].G.UNKNOWN:
                # if the feature state is unknown in supermap[i]
                unknown_i.append(i)
                has_unknowns = True
            else:
                for outcome in obsv.outcomes:
                    if outcome.state == possible_state:
                        outcome.Yo.append(i)
                        outcome.p += p_Xy[i]
                        is_new_outcome = False
                if is_new_outcome == True and p_Xy[i] != 0:
                    obsv.outcomes.append(Outcome(possible_state, [i], p_Xy[i]))

                outcome_norm += p_Xy[i] # for normalizing outcome probability

        # normalize outcome probabilities to exclude unknowns
        if has_unknowns == True:
            for outcome in obsv.outcomes:
                outcome.unknown.extend(unknown_i)
                outcome.p = outcome.p/outcome_norm

        if len(obsv.outcomes) > 1:
            # observation is only constructive if there is more than 1
            # outcome
            constrO.append(deepcopy(obsv))
            msg = "{}:\n".format(obsv.E)
            for outcome in obsv.outcomes:
                msg += "\t{}\n".format(outcome)
            logger.info(msg)

    # determine which vertices are reachable (eq 9)-----------------------------
    logger.info('Checking reachable vertices with eqn 9...')
    reachable = {}  # dict: {v: cost_to_v, u: cost_to_u,...}

    for v in supermaps[0].G.vertices():
        # determine reachable vertices
        try:
            cost_to_v = c_knownG.cost[v]
        except KeyError:
            cost_to_v = float('inf') # same issue as line 266

        if cost_to_v != float('inf'): reachable[v] = cost_to_v

    logger.info('reachable v = {}'.format(reachable))

    # combine the two to decide which (e,u) pairs are feasible.
    # if e in (e,u) isn't visible for ALL maps in the belief
    #    then (e,u) is not feasible
    R=[] # list of constructive and reachable observations: (O, v)
    D={} # dict of expected travel cost of observation where we go to v first
    for v, cost_v in reachable.items():
        for o in constrO:
            is_viewable = True 

            for i in belief:
                try:
                    if v not in supermaps[i].feature_viewable_from(o.E):
                        is_viewable = False
                        break
                except KeyError:
                    # if feature doesn't exist/can't be viewed in supermap, then e is not
                    # a feasible observation
                    is_viewable = False
                    break

            if is_viewable:
                (u1,u2) = o.E
                edge_cost = supermaps[0].G.weight(u1,u2)
                if u1 == v:
                    start_u = u1
                    end_u = u2
                elif u2 == v:
                    start_u = u2
                    end_u = u1

                # Select cost function
                if costfn == 1:
                    travel_cost = cf.costfn1(reachable[v], start_u, goal, belief, supermaps,
                            p_Xy)
                elif costfn == 2:
                    travel_cost = cf.costfn2(reachable[v], start_u, end_u, goal, o.outcomes,
                            supermaps, p_Xy) 
                elif costfn == 3:
                    travel_cost = cf.costfn3()

                logger.info("Comparing known cost to goal: {:.3f} to cost of {}: {:.3f}"
                        .format(c_knownG.cost[goal], v, travel_cost))
                if (c_knownG.cost[goal] == float('inf') or 
                        (c_knownG.cost[goal] > travel_cost and not 
                        isclose(c_knownG.cost[goal], travel_cost))):
                    # need this because isclose(inf, 10) returns True
                    # to reach this point cost_v must be a finite number
                    R.append((o,v))
                    D[v] = travel_cost 

    logger.debug('D = {}'.format(D))
    logger.info("Completed calculating set of constructive and reachable obsv pairs!")
    return R, D
