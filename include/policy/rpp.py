"""
Written by: Florence Tsang
Creation date: 2018-08-14

Compatible with Python 2.7

Contains main RPP solvers and additional functions
"""

#!/usr/bin/env python2
import logging
logger = logging.getLogger(__name__)
import timing # measures runtime
import sys
import math
import networkx as nx

from collections import deque # queue class
from copy import deepcopy

import tgraph
from gridgraph import GridGraph 
from classes import Cost, Node, Outcome, Observation
# from dijkstra import dijkstra
from utility import isclose, euclidean_distance

def get_knownG(features, supermaps, belief):
    """
        Inputs: list of all possible features in list of supermaps
                Supermap list M = [M0, M1, ...]

        Output: knownG - same type as supermaps.G, updated for this node
    """
    logger.info("Updating knownG...")
    base_map = supermaps[0]
    # knownG=type(base_map.G.graph)(base_map.G) # use same type as Map.G
    knownG=nx.Graph()
    knownG.add_nodes_from(base_map.G.vertices())
    # all feature states are unknown

    # compare features for all supermaps
    # features should be same type as keys in dict returned by supermaps.G.observe
    known_features = {}
    for feature in features:
        # logger.debug("Checking {}".format(feature))
        state = None
        is_known = True
        for i in belief:
            new_state = supermaps[i].feature_state(feature)

            if new_state != base_map.G.UNKNOWN:
                if state == None: state = new_state
                if state != None and new_state != state: is_known = False

        # if feature state is blocked in one supermap but unblocked in another supermap in
        # the belief, then it can't be in knownG.
        # if feature state is unknown in all beliefs, then add it to knownG so I can use
        # it

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

def solve_RPP(M, p, features, start, goal):
    """ This is the modified version for LRPP
        What's changed:
            - G is M = [(Eb,Eu,n),...] (supermap set from LRPP)
              Similar to G, no modifications should be made to this data here!
            - cost_to_goal is calculated outside of this solver, by LRPP, and updated
            - list of features is calculated outside of solver
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
            R, D = useful_features( features, M, p_Xy, c_knownG, Y, goal )
            # R - [(O1,v1), (O2, v2), ...]
            # D - {v1: cost_v1, v2: cost_v2, ...}
            logger.info('R = {}'.format(R))

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
                    logger.info('Entropy: {:.4f}  Score: {:.4f}'.format(H,score))
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

                minObservation = deepcopy(min_o)

                # set node observation, state, and path
                new_node.add_leg(min_u,minObservation,c_knownG.paths[min_u])

                # Add new children nodes and add those to the queue
                for outcome in min_o.outcomes:
                    policy.append(Node(outcome.new_belief()))
                    new_node.add_outcome(policy[-1])
                    Q.append((outcome.new_belief(),min_u,policy[-1]))

    return policy

def useful_features( features, supermaps, p_Xy, c_knownG, belief, goal ):
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
    reachable_v = {}  # dict: {v: cost_v, u: cost_u,...}

    for v in supermaps[0].G.vertices():
        # calculate expected cost
        expected_cost = 0
        for i in belief:
            # nx.dijkstra doesn't return nodes in clusters that are disconnected
            # so if it's not in get_cost, then just set cost_to_goal as infinity (temp
            # workaround, hopefully there's a better solution
            try:
                cost_to_goal = supermaps[i].get_cost(goal, v)
            except KeyError:
                cost_to_goal = float('inf')
            if cost_to_goal < float('inf'):
                # is this still a valid assumption? In LRPP, the cost is not zero
                # if it is a no goal, it has to check via reactive planner, so
                # it's actually very expensive.
                # Does it make sense to ignore the cost of a no goal when
                # calculating the policy? I think so, if there is a supermap with
                # no possible path to goal, it'll just inflate the cost of EVERY
                # observation in constrO. Unless I can somehow calculate the
                # actual cost of the reactive planner to determine no goal, then I
                # can pick better observations to minimize that path
                expected_cost += cost_to_goal*p_Xy[i]

        try:
            cost_to_v = c_knownG.cost[v]
        except KeyError:
            cost_to_v = float('inf') # same issue as line 266

        cost_v = cost_to_v + expected_cost

        logger.debug("Comparing known cost to goal: {:.3f} to cost of {}: {:.3f}"
                .format(c_knownG.cost[goal], v, cost_v))
        if cost_v == float('inf'): continue # v is not reachable if cost is infinite
        if c_knownG.cost[goal] == float('inf'):
            # need this because isclose(inf, 10) returns True
            # to reach this point cost_v must be a finite number
            reachable_v[v] = cost_v
        elif c_knownG.cost[goal] > cost_v and not isclose(c_knownG.cost[goal], cost_v):
            reachable_v[v]= cost_v

    logger.debug('reachable_v = {}'.format(reachable_v))

    # combine the two to decide which (e,u) pairs are feasible.
    # if e in (e,u) isn't visible for ALL maps in the belief
    #    (e,u) is not feasible
    R=[] # list of constructive and reachable observations: (O, v)
    for v, cost_v in reachable_v.items():
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

            if is_viewable: R.append((o,v))

    logger.info("Completed calculating set of constructive and reachable obsv pairs!")
    return R, reachable_v
