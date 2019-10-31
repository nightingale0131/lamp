'''
Created by: Florence Tsang
Creation date: Oct 30, 2019

Different cost functions to try
'''
import logging
logger = logging.getLogger(__name__)

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

def costfn3(known_cost_to_v, v, u, goal, outcomes, supermaps, p_Xy, robot_range):
    '''
    Takes into account that submaps are convex, uses that to give better cost estimate.
    Supermaps[i].G must be TGraph class object!
    '''

    return

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
