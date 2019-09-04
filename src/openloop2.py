#!/usr/bin/env python
__author__ = 'fiorellasibona' # repurposed code
"""
v2: Works with tgraph
"""
import logging
logger = logging.getLogger(__name__) 
import rospy, rospkg 
import math 
import os
import networkx as nx
import cv2

import actionlib
from policy import tgraph
from policy import utility as util
from nav_msgs.msg import OccupancyGrid, Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, Polygon
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from policy.srv import CheckEdge
from policy.msg import EdgeUpdate

PADDING = 0.3 # must be greater than xy_goal_tolerance

class MoveBaseSeq():
    def __init__(self, base_graph, path=None, policy=None):
        # base_graph - base graph rep of map needed for graph edges
        # path - list of vertices [v1, v2, ...]
        rospy.init_node('move_base_seq')
        self.base_graph = base_graph
        self.pos = self.base_graph.pos('s')
        self.path_blocked = False # flag for path being blocked, calls reactive algo
        self.observe = False # flag for making an observation
        self.at_final_goal = False # flag for reaching final destination

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.plan_subscriber = rospy.Subscriber("move_base/NavfnROS/plan", Path,
                                                self.plan_callback, queue_size=1,
                                                buff_size=2**24)
        self.edge_subscriber = rospy.Subscriber("policy/edge_update", EdgeUpdate,
                                                self.edge_callback, queue_size=5)

        self.posearray_publisher = rospy.Publisher("waypoints", PoseArray, queue_size=1)
        
        if policy == None:
            self.path = path
        else:
            self.node = policy[0].next_node()
            self.path = self.node.path

        self.set_new_path(self.path, self.base_graph) # sets pose_seq and goal_cnt
        self.vnext = self.path[1]
        self.vprev = self.path[0] 

        # connect to move_base node
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements...")
        self.movebase_client()

    def update_current_edge(self):
        self.vprev = self.vnext
        self.vnext = self.path[self.goal_cnt]

    def active_cb(self):
        rospy.loginfo("Goal pose " + str(self.goal_cnt) + " is now being processed by the Action server...")

    def feedback_cb(self, feedback):
        # print current pose at each feedback
        # feedback is MoveBaseFeedback type msg
        pose = feedback.base_position.pose
        xy_goal_tolerance = rospy.get_param('/move_base/TrajectoryPlannerROS/xy_goal_tolerance')

        rospy.loginfo("Feedback for goal " + str(self.goal_cnt) + ":\n" +
                      self.print_pose_in_euler(pose))
        rospy.loginfo("vprev = {}, vnext = {}".format(self.vprev, self.vnext))

        # check if robot is close enough to send next goal
        position = feedback.base_position.pose.position
        (x,y) = (position.x, position.y)

        # set this to be available to all class methods
        self.pos = (x,y)

        curr_goal = self.pose_seq[self.goal_cnt].position
        (gx,gy) = (curr_goal.x, curr_goal.y)

        dist_to_curr_goal = util.euclidean_distance((x,y), (gx,gy))

        if dist_to_curr_goal < xy_goal_tolerance and (self.goal_cnt < len(self.pose_seq) - 1):
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " reached!")
            if self.vprev != self.vnext:
                self.base_graph.set_edge_state(self.vprev, self.vnext, tgraph.UNBLOCKED)
            self.goal_cnt += 1
            self.vprev = self.vnext
            self.vnext = self.path[self.goal_cnt]
            self.set_and_send_next_goal()
        elif dist_to_curr_goal < xy_goal_tolerance and (self.goal_cnt == len(self.pose_seq) - 1):
            rospy.loginfo("Reached end of path")
            self.base_graph.set_edge_state(self.vprev, self.vnext, tgraph.UNBLOCKED)
            # observe and select next node based on state of observed feature
            if self.observe == False and self.going_to_final_goal() != True:
                self.observe == True
                edge_to_observe = self.node.opair.E
                self.make_observation(edge_to_observe)
            # if feature is unknown move to reactive b/c it is not visible when it
            # should've been

    def make_observation(self, edge):
        # returns state of edge in current LiveMap
        # OBSOLETE - doesn't work with tgraph
        (u,v) = edge
        self.LiveMap._edge_check(edge)
        feature_state = self.LiveMap.graph[u][v]['state']
        rospy.loginfo("State of observed edge {}: {}".format(edge, feature_state))

        # selecting next node in tree and setting path
        self.node = self.node.next_node(feature_state)
        rospy.loginfo("next path: {}".format(self.node.path))
        self.path = self.node.path # update path
        self.set_new_path(self.path, self.base_map) # update pose seq
        self.set_and_send_next_goal()

    def going_to_final_goal(self):
        if self.path[self.goal_cnt] == 'g': return True
        else: return False

    def done_cb(self, status, result):
        # refer to http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            # The goal received a cancel request after it started executing
            # and has since completed its execution (Terminal State)
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + 
                " received a cancel request after it started executing, finished execution!")

        if status == 3:
            # The goal was achieved successfully by the action server (Terminal State)
            rospy.loginfo("Done Status 3")
            if self.going_to_final_goal() == True:
                print(self.base_graph.edges(data='state'))
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            # The goal was aborted during execution by the action server due
            # to some failure (Terminal State)
            rospy.loginfo("Goal pose " + str(self.goal_cnt) +
                " was aborted by the Action Server")
            if not self.path_blocked:
                self.path_blocked = True
                # set edge to be blocked
                vnext = self.path[self.goal_cnt]
                vcurr = self.path[self.goal_cnt - 1]
                self.base_graph.set_edge_state(vnext, vcurr, tgraph.BLOCKED)
                self.base_graph.set_edge_weight(vnext, vcurr,float('inf'))

                self.replan()
                self.path_blocked = False
            return

        if status == 5:
            # The goal was rejected by the action server without being processed,
            # because the goal was unattainable or invalid (Terminal State)
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + 
                " has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt) +
                " rejected, shutting down!")
            return

        if status == 8:
            # The goal received a cancel request before it started executing
            # and was successfully cancelled (Terminal State)
            rospy.loginfo("Goal pose "+str(self.goal_cnt) + 
                " received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        self.set_and_send_next_goal()
        rospy.spin()

    def set_and_send_next_goal(self):
        next_goal = MoveBaseGoal()
        next_goal.target_pose.header.frame_id = "map"
        next_goal.target_pose.header.stamp = rospy.Time.now()
        next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose\n" +
                self.print_pose_in_euler(self.pose_seq[self.goal_cnt]) + 
            " to Action server")
        self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)

    def set_new_path(self, path, graph, at_first_node = True):
        points = list() 
        for v in path:
            (x,y) = graph.pos(v)
            new_point = (x,y,0.0)
            points.append(new_point)

        yaweulerangles_seq = self.calc_headings(points)

        # if not at_first_node:
            # yaweulerangles_seq[0] = yaweulerangles_seq[1] # can change if this causes problems
        rospy.loginfo('New path: ')
        for i, angle in enumerate(yaweulerangles_seq):
            rospy.loginfo("  {}: {:.2f}".format(path[i], math.degrees(angle)))

        quat_seq = list()
        self.pose_seq = list() # list of poses, combo of Point and Quaternion
        self.goal_cnt = 1 if at_first_node else 0
        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle))))
        n = 0
        for point in points:
            self.pose_seq.append(Pose(Point(*point), quat_seq[n]))
            n += 1

    def conv_to_PoseArray(self, poseArray):
        poses = PoseArray()
        poses.header.frame_id = 'map'
        poses.poses = [pose for pose in poseArray]
        return poses

    def calc_headings(self, path):
        # calc goal heading at each waypoint of path
        # for now just use angle of line segment
        # assume path starts with current location of robot

        yaweulerangles_seq = [1.0]
        i = 1
        while i < len(path): # start at 2nd element
            (x1, y1, z1) = path[i - 1]
            (x2, y2, z2) = path[i]

            # calc heading
            heading = math.atan2((y2 - y1), (x2 - x1))
            yaweulerangles_seq.append(heading)
            i += 1

        if len(path) > 1:
            yaweulerangles_seq[0] = yaweulerangles_seq[1] # can change if this causes problems

        return yaweulerangles_seq

    def print_pose_in_euler(self, pose):
        quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)

        msg = (" x: " + str(pose.position.x) + 
               "\n y: " + str(pose.position.y) + 
               "\n yaw:"+ str(math.degrees(yaw)))

        # only print x,y and yaw
        return msg

    def plan_callback(self, data):
        # constantly checking returned rospath. If it exits the reg mark path as blocked
        # extracting needed data

        if not (self.goal_cnt == 0 or self.path_blocked):
            # if goal_cnt == 0, this means we are returning to the prev vertex and we
            # don't need to check if path is in polygon. Maybe have smarter behaviour that
            # retraces steps? It might still not be what I want in light of new info
            # though
            vnext = self.path[self.goal_cnt]
            vcurr = self.path[self.goal_cnt - 1]
            rospath = to_2d_path(data)

            rospy.loginfo("Checking edge ({},{})".format(vcurr, vnext))
            rospy.logdebug("Path: {}".format(util.print_coord_list(rospath)))

            curr_edge_state = self.base_graph.check_edge_state(vcurr, vnext, rospath,
                                                               PADDING, False)
            rospy.loginfo("result of check_edge_state: {}".format(curr_edge_state))

            if curr_edge_state == tgraph.BLOCKED:
                # recalculate path on base_graph
                self.path_blocked = True

            if self.path_blocked:
                rospy.loginfo("Path is blocked!")
                self.client.cancel_goals_at_and_before_time(rospy.get_rostime())
                self.replan()

        self.posearray_publisher.publish(self.conv_to_PoseArray(self.pose_seq))

    def replan(self):
        rospy.loginfo("Replanning on graph...")
        start = self.path[max(0,self.goal_cnt - 1)]
        dist, paths = nx.single_source_dijkstra(self.base_graph.graph, start, 'g')

        if dist['g'] == float('inf'):
            print(self.base_graph.edges(data='state'))
            rospy.loginfo("Cannot go to goal! Stopping node.")
            rospy.signal_shutdown("Cannot go to goal! Stopping node.")
            return # path_blocked = True from now until shutdown

        self.path = paths['g']

        # if robot is in the same submap as first edge, go straight to second node
        curr_poly = self.base_graph.get_polygon(self.path[0], self.path[1])
        if self.base_graph.in_polygon(self.pos, curr_poly):
            self.path.pop(0)
        
        self.set_new_path(self.path, self.base_graph, at_first_node = False)
        self.vnext = self.path[self.goal_cnt]
        self.set_and_send_next_goal()
        self.path_blocked = False

    def edge_callback(self, data):
        # updates edges
        u = data.u
        if u.isdigit(): u = int(u)

        v = data.v
        if v.isdigit(): v = int(v)

        if data.state != tgraph.UNKNOWN:
            self.base_graph.set_edge_state(u,v,data.state)

        if data.state == tgraph.BLOCKED:
            self.base_graph.set_edge_weight(u,v,float('inf'))

            # if blocked edge is in path, replan
            if not self.path_blocked:
                for i, v_i in enumerate(self.path):
                    if i == 0: continue
                    if (self.path[i-1] == u and v_i == v) or (self.path[i-1] == v and v_i == u):
                        # remember to check both directions
                        self.path_blocked = True
                        self.client.cancel_goals_at_and_before_time(rospy.get_rostime())
                        rospy.loginfo("Path is blocked!")
                        self.replan()
                        break

    def check_edge_client(self, u, v, polygon):
        # u,v - vertices in tgraph
        # polygon - shapely polygon
        # OBSOLETE

        # convert vertices -> coordinates -> Point
        ux, uy = self.base_graph.pos(u)
        u_pt = Point(ux, uy, 0)
        vx, vy = self.base_graph.pos(v)
        v_pt = Point(vx, vy, 0)

        # inflate polygon and convert shapely polygon -> ros Polygon
        polygon = polygon.buffer(PADDING)
        ros_polygon = Polygon([Point(p[0],p[1],0) for p in polygon.exterior.coords])

        # call service
        rospy.wait_for_service('check_edge')
        try:
            check_edge = rospy.ServiceProxy('check_edge', CheckEdge)
            result = check_edge(u_pt, v_pt, ros_polygon)
            return result.result
        except rospy.ServiceException, e:
            print("Service call failed: {}".format(e))


def to_pose_stamped(x,y,z):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    # don't care about orientation
    pose.pose.orientation = Quaternion(*(quaternion_from_euler(0,0,0)))

    return pose

def to_2d_path(rospath):
    # rospath - ros path message, seq of stamped poses
    path = []

    for stamped_pose in rospath.poses:
        path.append((stamped_pose.pose.position.x,
                     stamped_pose.pose.position.y))

    return path

if __name__ == '__main__':
    # load map and determine graph
    rospack = rospkg.RosPack()
    pkgdir = rospack.get_path('policy')
    graph = nx.read_yaml(pkgdir + '/tests/tristan_maze_tgraph.yaml')
    poly_dict = tgraph.polygon_dict_from_csv(pkgdir + '/tests/tristan_maze_polygons.csv')

    logging.basicConfig(filename = pkgdir + '/openloop_tgraph_debug.log', filemode='w', level=logging.INFO)

    map0 = tgraph.TGraph(graph, poly_dict)

    # run dijkstra
    dist, paths = nx.single_source_dijkstra(map0.graph, 's', 'g')

    if dist['g'] == float('inf'):
        print("No path to goal found, ending program.")
    else:
        # give path to MoveBaseSeq
        path = paths['g']
        try:
            MoveBaseSeq(map0, path=path)
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation finished.")
