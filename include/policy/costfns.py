'''
Created by: Florence Tsang
Creation date: Oct 30, 2019

Different cost functions to try
'''
import logging
logger = logging.getLogger(__name__)
from shapely.geometry import LineString
import utility as util

def costfn1(known_cost_to_v, v, goal, belief, supermaps, p_Xy):
    # assumes edge state can be viewed from either endpt
    exp_cost = 0

    for i in belief:
        # nx.dijkstra doesn't return nodes in clusters that are disconnected
        # so if it's not in get_cost, then just set cost_to_goal as infinity (temp
        # workaround, hopefully there's a better solution
        try:
            cost_in_i = supermaps[i].get_cost(v, goal)
        except KeyError:
            cost_in_i = float('inf')

        logger.debug("  cG_{}({}) = {:.3f}".format(i, v, cost_in_i))

        if cost_in_i < float('inf'):
            '''
            In RPP, the robot will stop at the observation once it believes there is
            no path to goal, but in LRPP, since we do not know all possible environment
            configurations, if the policy determines there is no path to
            goal, the robot still has to exhaustively check every possible path to
            goal.

            Does it make sense to ignore the cost of a no goal when
            calculating the policy? 
            I think so, if there is a supermap with
            no possible path to goal, it'll just inflate the cost of EVERY
            observation in constrO. Unless I can somehow calculate the
            actual cost of the reactive planner to determine no goal, then I
            can pick better observations to minimize that path
            '''
            exp_cost += cost_in_i*p_Xy[i]

    return known_cost_to_v + exp_cost

def costfn2(known_cost_to_v, v, u, goal, outcomes, supermaps, p_Xy):
    '''
    Assumes you need to traverse entire edge to observe state if unblocked, but can
    view blocked edges from an endpoint
    Assumes union of belief of all outcomes = belief
    v - beginning of edge
    u - end of edge
    goal - goal vertex
    outcomes - list of Outcome class objects
    supermaps - list of Map class objects
    p_Xy - p(X = i|Y), probability of being in map i given the belief Y
    '''
    edge_cost = supermaps[0].G.weight(v, u)

    # currently it's only possible to have 2 outcomes
    for outcome in outcomes:
        if outcome.state == supermaps[0].G.BLOCKED:
            blocked_p = outcome.p 
            blocked_exp_cost = expected_cost_outcome(goal, v,
                    outcome, supermaps, p_Xy) 
            # calc expected cost from start_u
        elif outcome.state == supermaps[0].G.UNBLOCKED:
            unblocked_p = outcome.p
            # calc expected cost from end_u
            unblocked_exp_cost = expected_cost_outcome(goal, u,
                    outcome, supermaps, p_Xy) 

    return (known_cost_to_v + blocked_p*blocked_exp_cost +
            unblocked_p*(unblocked_exp_cost + edge_cost))

def costfn3(known_cost_to_v, v, u, goal, knownG, outcomes, supermaps, p_Xy, robot_range):
    '''
    Takes into account that submaps are convex, uses that to give better cost estimate.
    Supermaps[i].G must be TGraph class object!
    '''
    tg = supermaps[0].G # use this to get polygon, all of the supermaps should be identical
    edge_est = tg.min_mid_dist(v,u)
    unblocked_est = max(0, edge_est - (robot_range/2.0)) # Exp travel along edge before
                                                         # obsving edge is unblocked 
    line = LineString([tg.pos(v), tg.pos(u)])
    midpt = line.centroid
    o_pt = line.interpolate(unblocked_est)

    vlist = tg.get_vertices_in_polygon(tg.get_polygon(v,u))

    # Currently it's only possible to have 2 outcomes
    # All the variables should return something, because if all other vertices are
    # blocked, it should return v as ub and uu
    for outcome in outcomes:
        if outcome.state == tg.BLOCKED:
            blocked_p = outcome.p 
            blocked_exp_cost, ub = min_exp_cost(midpt, v, goal, vlist,
                    outcome, supermaps, p_Xy)
        elif outcome.state == tg.UNBLOCKED:
            unblocked_p = outcome.p
            # calc expected cost from end_u
            unblocked_exp_cost, uu = min_exp_cost(o_pt, v, goal, vlist,
                    outcome, supermaps, p_Xy)

    exp_cost = (known_cost_to_v + blocked_p*blocked_exp_cost +
            unblocked_p*unblocked_exp_cost)

    return (uu, ub, exp_cost) 

def min_exp_cost(est_loc, vs, goal, ulist, outcome, supermaps, p_Xy): 
    '''
    est_loc - shapely Point class object
    vs      - vertex that robot started observation from
    goal    - vertex
    ulist   - list of vertices
    outcome - Outcome class object
    supermaps - list of Map class objects
    p_Xy - p(X = i|Y), probability of being in map i given the belief Y

    Return min_exp_cost, min_u
    '''
    tg = supermaps[0].G
    min_u = None
    min_expcost = float('inf')

    logger.info("Calculating min exp cost for {}".format(list(est_loc.coords)))
    for u in ulist:
        dist = util.euclidean_distance((est_loc.x, est_loc.y), tg.pos(u))
        expcost = dist + expected_cost_outcome(u, goal, outcome, supermaps, p_Xy)
        logger.info("  ({},{}): dist={:.2f}, expcost={:.2f}".format(vs, u, dist, expcost))

        # check that edge is not blocked in any i of the outcome's belief
        if vs == u:
            if min_u == None or expcost < min_expcost:
                logger.info("  Set as min")
                min_u = u
                min_expcost = expcost
        else:
            for i in outcome.Yo:
                if supermaps[i].G.edge_state(vs, u) == tg.BLOCKED:
                    break
            else:
                if min_u == None or expcost < min_expcost:
                    logger.info("  Set as min")
                    min_u = u
                    min_expcost = expcost

    assert (min_u != None), ("min_u = None! This is not right! Check logs.")

    return min_expcost, min_u

def expected_cost_outcome(v, u, outcome, supermaps, p_Xy):
    # calculate expected cost based on outcome probabilities
    exp_cost = 0
    for i in outcome.Yo:
        try:
            cost_in_i = supermaps[i].get_cost(u, v)
        except KeyError:
            cost_in_i = float('inf')
        if cost_in_i < float('inf'):
            logger.debug("cost_in_{}={:.2f}".format(i, cost_in_i))
            exp_cost += cost_in_i*p_Xy[i]/outcome.p

    for i in outcome.unknown:
        try:
            cost_in_i = supermaps[i].get_cost(u, v)
        except KeyError:
            cost_in_i = float('inf')
        if cost_in_i < float('inf'):
            logger.debug("cost_in_{}={:.2f}".format(i, cost_in_i))
            exp_cost += cost_in_i*p_Xy[i]

    logger.debug("exp_cost from {} to {}: {:.2f}".format(u,v,exp_cost))

    return exp_cost
