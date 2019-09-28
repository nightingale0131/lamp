"""
Written by: Florence Tsang
Creation date: 2018-08-11
"""

#!/usr/bin/python
import logging
import networkx as nx
logger = logging.getLogger(__name__)

class Node(object):
    """ policy node
    """
    def __init__(self,belief,state=None,observation=None,path=None):
        self.Y = belief
        self.s = state
        self.opair = observation # observation object 
        self.path = path    # shortest path from previous parent state to current state

        # pointers to children nodes
        # this is to allow flexibility for a non-binary tree later
        # For compatibility with VREP script, 1: yes, 2: no
        self.children = []

    def add_leg(self,state,observation=None,path=None):
        """ if node is set to goal or no goal """
        self.s = state
        self.opair = observation
        self.path = path
        # modify path to include edge being observed

    def add_outcome(self, node):
        self.children.append(node)
        """ belief of the new node must match one of the world outcomes of the observation
        """
    def print_policy(self):
        """ Prints tree starting with 'self' as the root 
            Goes root, left, right 
        """
        try:
            feature = self.opair.E
        except AttributeError:
            # if there's no observation assigned, set to None
            feature = None

        # print('{:>8}: {:20}   {:20}    {}'.format(self.s, feature, self.Y, self.path))
        msg = '\n{:>8}: {:20}   {:20}    {}'.format(self.s, feature, self.Y, self.path)
        for node in self.children:
            msg += node.print_policy()

        return msg

    def next_node(self, feature_state=None):
        """ returns the next node based on the state of feature
        """
        if feature_state == None:
            return self.children[0]
        else:
            for i, outcome in enumerate(self.opair.outcomes):
                if feature_state == outcome.state:
                    return self.children[i]

        logger.error("{} cannot be in state {}".format(self.opair.E, feature_state))
        return None

class Observation(object):
    """ an observation
        Put into a class to make room for additional properties that may be needed
    """
    def __init__(self,feature):
        self.E=feature  # feature to be observed
        self.outcomes = [] # should be type outcome

class Outcome(object):
    """ an outcome """
    def __init__(self,state=None,Yo=[],p=0):
        self.state = state # possible state of feature after being observed
        self.Yo=Yo # subgraphs where feature e was observed to be at "state", subset of existing Y
        self.unknown = [] # subgraphs where state of feature is unknown
        self.p=p # probability of outcome

    def __str__(self):
        return "p = {:.3f} Eo = {} Yo = {}".format(self.p, self.state, self.Yo + self.unknown)

    def new_belief(self):
        return self.Yo + self.unknown

class Cost(object):
    """ structure for recording min cost from a fixed vertex to all the other vertices in
        a graph
        I could potentially get rid of this and just directly use the dict if I need to do
        some optimization
    """
    def __init__(self, cost, paths):
        self.cost = cost
        self.paths = paths

class Map(object):
    """
        General wrapper class for different types of maps (grids, graphs).
        Any class will do, as long as it has the following attributes:
            - UNKNOWN # repr unknown state of feature
            - vertices()
            - weight(v)
            - observe(v, range)
            - neighbours(v)
            - state(e)
            - _visibility(
        The class must have the idea of vertices (states that the robot can move between)
        and features (some discretization of what the robot can sense)

        This is a really dumb class that makes everything more complicated than it needs
        to be. But removing it is difficult.
    """
    def __init__( self, G ):
        self.G = G # G - processed sensor data from robot 
        self.n = 1 # int, number of times map was encountered
        self._cost = {}  # dict of Cost type objects, estimated cost to some vertex
        self._features = {} # dict of features, {e: [set(v1,v2...), state of e], ...}
        self.update_all_feature_states()
        self.update_cost(G.goal)

    def update_cost(self, v):
        cost, path = nx.single_source_dijkstra( self.G.graph, self.G.goal, weight = 'weight' ) 
        self._cost[v] = Cost(cost, path) 

    def get_cost(self, v, u):
        # min cost from v to u, assuming either v or u has been calculated already
        if v in self._cost.keys(): return self._cost[v].cost[u]
        elif u in self._cost.keys(): return self._cost[u].cost[v]
        else: raise Exception('Cost from {} to {} has not been calculated yet!'.format(v,u))

    def update_all_feature_states( self ):
        # using G.observe, determine list of features, what state they are in, and what
        #   vertex they can be sensed (in G at a range of obs_range)

        self._features.clear()
        for v in self.G.graph.nodes():
            observation = self.G.observe(v) # only returns list of visible edges

            for e in observation:
                (a,b) = e
                state = self.G.graph[a][b]['state']
                key = None
                if (a,b) in self._features.keys(): key = (a,b)
                elif (b,a) in self._features.keys(): key = (b,a)
                else:
                    self._features[e] = [ {v}, state ]

                if key != None:
                    if self._features[key][1] == state:
                        self._features[key][0].add(v)
                    else: raise Exception('Conflicting states of feature {}'.format(e))

    def feature_state( self, feature):
        if feature not in self.features():
            logger.error("{} not a feature in current map!".format(feature))
            return -1
        return self._features[feature][1]

    def feature_viewable_from(self, feature):
        # returns set of vertices where feature is viewable from
        return self._features[feature][0]

    def features(self):
        # returns set of features observable in this map
        return set(self._features.keys())

    def agrees_with(self, m):
        # returns a bool (whether maps agree or not) and if they agree, a dict of new
        # feature, state info to update the self with. Otherwise empty dict is returned.
        # - doesn't deal w/ the case where a feature exists in one map but not the other,
        # assumes that the same features will be listed in _features for all maps

        # M is another Map type object, should check G attr is the same type
        if type(self.G) != type(m.G): 
            raise Exception("Maps don't contain the same sensor data type!")

        new_info = {} 
        for e, value in self._features.items():
            (a,b) = e 
            state = value[1]

            # hacky way to make sure both orientations of edge feature is checked
            if m.feature_state((a,b)) != self.G.UNKNOWN:
                mstate = m.feature_state((a,b))
            if m.feature_state((b,a)) != self.G.UNKNOWN:
                mstate = m.feature_state((b,a))

            logger.debug("Checking state of {}: self = {}, newmap = {}"
                    .format(e, state, mstate))
            if state == self.G.UNKNOWN and mstate != self.G.UNKNOWN:
                # if feature state is unknown in self but known in M, this is new info wrt
                # self
                new_info[e] = m.G.edge_info(e[0],e[1])
            elif state != mstate and mstate != self.G.UNKNOWN:
                # if feature is known in both maps and they are different
                return False, {}

        return True, new_info

    def updateG(self, new_info):
        # updates only sensor information with new information
        # new_info = {feature: state, feature: state, ...}
        self.G.update(new_info)
        self.update_all_feature_states()
