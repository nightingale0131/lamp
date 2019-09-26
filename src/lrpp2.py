#!/usr/bin/env python
__author__ = 'fiorellasibona' # repurposed code
import logging
logger = logging.getLogger(__name__) 
import rospy, rospkg 
import math 
import os, glob
import networkx as nx
import numpy as np
import cv2
from copy import copy

import actionlib
from policy.classes import Map
from policy import utility as util
from policy import rpp, tgraph
from policy import mapfilters as mf
from policy.generate_obstacles import spawn_obstacles, delete_obstacles

from nav_msgs.msg import OccupancyGrid, Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseWithCovariance
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, Twist, Vector3
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from policy.msg import EdgeUpdate, CurrEdge
from std_msgs.msg import Empty as rosEmpty

from std_srvs.srv import *
from gazebo_msgs.srv import *

PADDING = 0.3 # must be greater than xy_goal_tolerance
PKGDIR = rospkg.RosPack().get_path('policy')
# MAP = 'tristan_maze'
MAP = 'test_large'
RESULTSFILE = PKGDIR + "/results/" + MAP + "/lrpp_results.dat"

class LRPP():
    def __init__(self, base_graph, polygon_dict, T=1):
        # seq - list of vertices [v1, v2, ...]
        # base_map is map type
        rospy.init_node('move_base_seq')

        self.base_graph = base_graph
        self.poly_dict = polygon_dict

        base_tgraph = tgraph.TGraph(self.base_graph, self.poly_dict)
        (x,y) = base_tgraph.pos('s')
        self.pos = (x,y)
        self.travelled_dist = 0

        # setup initial start pose for publishing
        self.start_pose = Pose(Point(x,y,0.0),
                # Quaternion(*(quaternion_from_euler(0, 0, 1.57))))
                Quaternion(*(quaternion_from_euler(0, 0, 3.14))))
        # self.gaz_pose = (2, -4, 1.57) # (x, y, yaw (rad)), tristan
        self.gaz_pose = (18.5, 18.5, 3.14) # (x, y, yaw (rad)), test_large

        # set all edges to UNBLOCKED for initial map
        for (u,v) in base_tgraph.edges():
            base_tgraph.set_edge_state(u,v,base_tgraph.UNBLOCKED)
        self.base_map = Map(base_tgraph)
        self.features = self.base_map.features()
        self.M = [self.base_map] # initialize map storage
        self.T = T  # number of tasks to execute
        self.tcount = 1 # current task being executed

        self.path_blocked = False # flag for path being blocked, calls reactive algo
        self.at_final_goal = False # flag for reaching final destination
        self.entered_openloop = False
        self.mode = None # what kind of policy to follow (policy, openloop, naive)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.posearray_publisher = rospy.Publisher("waypoints", PoseArray, queue_size=1)
        self.v_publisher = rospy.Publisher("policy/current_edge", CurrEdge, queue_size=10)
        self.amcl_publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped,
                queue_size=10, latch=True)

        self.start_task("policy")
        rospy.spin()

    def start_task(self, mode):
        rospy.loginfo("Starting task in {} mode".format(mode))
        self.mode = mode
        self.check_mode()

        # create blank tgraph to fill in (make sure nothing is carried over!)
        self.curr_graph = tgraph.TGraph(self.base_graph, self.poly_dict)
        self.path_blocked = False
        self.entered_openloop = False

        if mode == "policy":
            rospy.loginfo("Calculating policy for task {}...".format(self.tcount))
            self.p = update_p_est(self.M, self.tcount) 
            self.policy = rpp.solve_RPP(self.M, self.p, self.features, 's', 'g')
            rospy.loginfo(self.policy[0].print_policy())

        # set robot location
        (x,y) = self.curr_graph.pos('s')
        self.pos = (x,y) 

        if mode == "policy":
            # set robot to follow policy
            self.node = self.policy[0].next_node()
            self.vprev = self.node.path[0] 
            self.set_new_path(self.node.path) # sets pose_seq and goal_cnt

        elif mode == "openloop":
            # set robot to go straight into openloop
            self.entered_openloop = True
            self.node = None
            self.vprev = 's'
            dist, paths = nx.single_source_dijkstra(self.curr_graph.graph, 's', 'g')
            self.set_new_path(paths['g'])

        elif mode == "naive":
            self.node = None
            self.vprev = 's'
            self.vnext = 'g'
            self.path = ['g']
            self.goal_cnt = 0
            (gx,gy) = self.curr_graph.pos('g')

            # modify pose_seq directly instead of setting path
            self.pose_seq = [Pose(Point(gx,gy,0.0), 
                Quaternion(*(quaternion_from_euler(0,0,0))))]

        # setup task environment
        rospy.loginfo("Setting up environment...")

        # Cancel all goals to move_base
        self.client.cancel_goals_at_and_before_time(rospy.get_rostime())

        self.set_robot_pose(*(self.gaz_pose)) # Move jackal back to beginning in gazebo
        rospy.sleep(1)
        self.set_amcl_pose() # Set initial guess for amcl back to start 

        # Set up any obstacles if starting a new task
        if mode == "policy":
            self.del_cmd = spawn_obstacles()

        # wait for input before sending goals to move_base
        # raw_input('Check that amcl localized properly!\
        # Press any key to begin execution of task {}'.format(self.tcount))
        rospy.sleep(5)

        # reset costmap using service /move_base/clear_costmaps
        self.clear_costmap_client()
        self.reset_travel_dist()
        rospy.sleep(2) # give some time to make sure costmap is cleared before starting

        # double check connection with move base
        self.check_connection()
        rospy.loginfo("Status of action client: {}".format(self.client.simple_state))
        self.client.simple_state = 0 # set back to pending

        # setup subscribers
        if mode == "policy" or mode == "openloop":
            self.plan_subscriber = rospy.Subscriber("move_base/GlobalPlanner/plan", Path,
                                                    self.plan_callback, queue_size=1,
                                                    buff_size=2**24)
            self.edge_subscriber = rospy.Subscriber("policy/edge_update", EdgeUpdate,
                                                    self.edge_callback, queue_size=5)

        # set timer
        self.task_start_time = rospy.Time.now()
        # run set_and_send_next goal with path
        self.set_and_send_next_goal()

    def finish_task(self):
        task_time = rospy.Time.now() - self.task_start_time

        # open results file
        f = open(RESULTSFILE, "a")

        if self.mode == "policy" or self.mode == "openloop":
            # unsubscribe from plan and map
            self.plan_subscriber.unregister()
            self.edge_subscriber.unregister()

        if self.mode == "policy":
            next_mode = "openloop"
            # run map filter
            self.save_map_and_filter()

            # write
            f.write("\nFinished executing task {}".format(self.tcount))
            f.write("\nEntered openloop: {}".format(self.entered_openloop))

            f.write(self.policy[0].print_policy())

            # print states
            f.write("\n\n Map states")
            for edge in self.base_graph.edges():
                line = "\n{:<10}".format(edge) 
                u,v = edge
                for m in self.M:
                    line += "{:3}".format(m.G.edge_state(u,v))

                f.write(line)

            # print weights
            f.write("\n\n Map weights")
            for edge in self.base_graph.edges():
                line = "\n{:<10}".format(edge) 
                u,v = edge
                for m in self.M:
                    line += "{:8.3f}".format(m.G.weight(u,v))

                f.write(line)

            f.write("\n\n   Mode    Distance travelled (m)  Task Completion Time (h:mm:ss)")
            f.write("\nPolicy      {:9.3f}               {}"
                    .format(self.travelled_dist, util.secondsToStr(task_time.to_sec())))

        elif self.mode == "openloop":
            next_mode = "naive"
            f.write("\nOpenloop    {:9.3f}               {}"
                    .format(self.travelled_dist, util.secondsToStr(task_time.to_sec())))

        elif self.mode == "naive":
            next_mode = "policy"
            f.write("\nNaive       {:9.3f}               {}"
                    .format(self.travelled_dist, util.secondsToStr(task_time.to_sec())))
            f.write("\n==============================")

            # clear all non-static obstacles
            delete_obstacles(self.del_cmd)

            # increment task counter
            self.tcount +=1
            # check if we need to keep going
            if self.tcount == self.T + 1:
                self.shutdown()

        f.close()
        self.start_task(next_mode)

    def check_connection(self):
        # make sure there is a connection to move_base node
        rospy.loginfo("Waiting for move_base action server...")
        rospy.loginfo("shutdown? {}".format(rospy.is_shutdown()))
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements...")

    def save_map_and_filter(self):
        rospy.loginfo("Checking map from task {}".format(self.tcount))
        n = len(self.M)
        
        rospy.loginfo("Saved map: \n" + str(self.curr_graph))
        # copy current tgraph 
        new_map = Map(copy(self.curr_graph))

        # now do comparisons and map merging??
        self.M = mf.filter1(self.M, new_map)

        # do no comparisons and just save the new map lol
        # self.M.append(new_map)

    def shutdown(self):
        # print maps
        rospy.loginfo("Shutting down...")
        rospy.signal_shutdown("Finished tasks.")

    def active_cb(self):
        rospy.loginfo("Goal pose " + str(self.goal_cnt) + " is now being processed by the Action server...")

        # update which edge robot is traversing
        if self.goal_cnt > 0:
            self.vprev = self.path[self.goal_cnt - 1]
        self.vnext = self.path[self.goal_cnt]

    def feedback_cb(self, feedback):
        # print current pose at each feedback
        # feedback is MoveBaseFeedback type msg
        pose = feedback.base_position.pose
        xy_goal_tolerance = rospy.get_param('/move_base/TrajectoryPlannerROS/xy_goal_tolerance')

        rospy.logdebug("Feedback for goal " + str(self.goal_cnt) + ":\n" +
                      self.print_pose_in_euler(pose))
        rospy.loginfo("vprev = {}, vnext = {}".format(self.vprev, self.vnext))
        self.v_publisher.publish(str(self.vprev), str(self.vnext))

        # check if robot is close enough to send next goal
        position = feedback.base_position.pose.position
        (x,y) = (position.x, position.y)

        # update distance travelled and current position
        self.travelled_dist += util.euclidean_distance((x,y), self.pos)
        self.pos = (x,y)

        curr_goal = self.pose_seq[self.goal_cnt].position
        (gx,gy) = (curr_goal.x, curr_goal.y)

        dist_to_curr_goal = util.euclidean_distance((x,y), (gx,gy))

        if dist_to_curr_goal < xy_goal_tolerance and (self.goal_cnt < len(self.pose_seq) - 1):
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " reached!")
            if self.vprev != self.vnext:
                self.curr_graph.set_edge_state(self.vprev, self.vnext, self.base_map.G.UNBLOCKED)
            self.goal_cnt += 1

            self.set_and_send_next_goal()

        elif self.move_to_next_node():
            # if node under observation is no longer unknown, move to next node
            # selecting next node in tree and setting path
            (u,v) = self.node.opair.E 
            state = self.curr_graph.edge_state(u,v)
            self.node = self.node.next_node(state)
            self.set_new_path(self.node.path) # update pose seq
            self.set_and_send_next_goal()

        elif dist_to_curr_goal < xy_goal_tolerance and (self.goal_cnt == len(self.pose_seq) - 1):
            rospy.loginfo("Reached end of path")
            # if move_to_next_node hasn't been activated, it must've reached the goal,
            # otherwise there is an error in the code

    def move_to_next_node(self):
        # check if observation has been satisfied
        if self.node != None and self.node.opair != None:
            (u,v) = self.node.opair.E 
            state = self.curr_graph.edge_state(u,v)
            if  state != self.base_map.G.UNKNOWN:
                if state == self.base_map.G.UNBLOCKED: 
                    rospy.loginfo("Edge ({},{}) is UNBLOCKED! Moving to next node...".format(u,v))
                else: 
                    rospy.loginfo("Edge ({},{}) is BLOCKED! Moving to next node...".format(u,v))
                return True
            else:
                return False
        return False

    def going_to_final_goal(self):
        if self.path[self.goal_cnt] == self.base_map.G.goal: return True
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
                rospy.loginfo("Final goal pose reached!")
                self.finish_task()
            return

        if status == 4:
            # The goal was aborted during execution by the action server due
            # to some failure (Terminal State)
            # ASSUMPTION: there's always a path to goal
            # Failure is most likely due to localization error
            rospy.loginfo("Goal pose " + str(self.goal_cnt) +
                " was aborted by the Action Server")

            if self.mode == "naive":
                self.clear_costmap_client()
                # attempt to go to goal again
                self.set_and_send_next_goal()
            elif not self.path_blocked:
                self.path_blocked = True
                # set edge to be blocked
                vnext = self.path[self.goal_cnt]
                vcurr = self.path[self.goal_cnt - 1]
                self.curr_graph.set_edge_state(vnext, vcurr, self.base_map.G.BLOCKED)
                self.curr_graph.set_edge_weight(vnext, vcurr,float('inf'))

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


    def set_and_send_next_goal(self):
        next_goal = MoveBaseGoal()
        next_goal.target_pose.header.frame_id = "map"
        next_goal.target_pose.header.stamp = rospy.Time.now()
        next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose\n" +
                self.print_pose_in_euler(self.pose_seq[self.goal_cnt]) + 
            " to Action server")
        self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)

    def set_new_path(self, path):
        self.path = path
        self.modify_assigned_path()

        points = list() 
        for i in path:
            (x,y) = self.base_map.G.pos(i)
            new_point = (x,y,0.0)
            points.append(new_point)
            # print(new_point)

        yaweulerangles_seq = self.calc_headings(points)
        # if not at_first_node:
            # yaweulerangles_seq[0] = yaweulerangles_seq[1] # can change if this causes problems
        rospy.loginfo('New path: ')
        for i, angle in enumerate(yaweulerangles_seq):
            rospy.loginfo("  {}: {:.2f}".format(path[i], math.degrees(angle)))

        quat_seq = list()
        self.pose_seq = list() # list of poses, combo of Point and Quaternion
        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle))))
        n = 0
        for point in points:
            self.pose_seq.append(Pose(Point(*point), quat_seq[n]))
            n += 1

    def modify_assigned_path(self):
        # do any needed online fixes to the path to adapt to not knowing where
        # observations will be made
        # set goal count in here as well

        # if robot is observing an edge, append edge to end of path
        if self.node != None and self.node.opair != None:
            edge_to_observe = self.node.opair.E

            (u,v) = edge_to_observe
            if self.path[-1] == u: self.path.append(v)
            elif self.path[-1] == v: self.path.append(u)

        # if robot is in the same submap as first edge, go straight to second vertex in
        # path
        if len(self.path) > 1:
            curr_poly = self.curr_graph.get_polygon(self.path[0], self.path[1])
            if self.curr_graph.in_polygon(self.pos, curr_poly):
                self.path[0] = self.vprev
                self.goal_cnt = 1
            else:
                self.goal_cnt = 0
        else:
            self.goal_cnt = 0

        # add modification to beginning of path if robot is closer to another point on the path


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

        vnext = self.vnext
        vcurr = self.vprev

        if len(data.poses) > 0:
            plan_dest = (data.poses[-1].pose.position.x, data.poses[-1].pose.position.y)

            if not (vnext == vcurr or self.path_blocked or 
                    util.euclidean_distance(plan_dest, self.curr_graph.pos(vnext)) > 0.25):
                # Conditions explained:
                # 1. if vnext == vcurr, robot is going back to where it came from, and I assume
                # it can take the way it came to get back
                # 2. if path_blocked = true, this means it's in the middle of replanning so
                # vnext is being changed
                # 3. if the final destination of given plan is not vnext, don't check
                # 4. if path is empty don't check

                rospath = to_2d_path(data)

                rospy.loginfo("Checking edge ({},{})".format(vcurr, vnext))
                rospy.logdebug("Path: {}".format(util.print_coord_list(rospath)))

                curr_edge_state = self.curr_graph.check_edge_state(vcurr, vnext, rospath,
                                                                   PADDING, False)
                rospy.loginfo("result of check_edge_state: {}".format(curr_edge_state))

                if (curr_edge_state == self.base_map.G.BLOCKED and
                    not self.edge_under_observation(vnext, vcurr)):
                    # recalculate path on curr_graph
                    self.path_blocked = True

                if self.path_blocked:
                    rospy.loginfo("Path is blocked!")
                    self.client.cancel_goals_at_and_before_time(rospy.get_rostime())
                    self.replan()

        self.posearray_publisher.publish(self.conv_to_PoseArray(self.pose_seq))

    def replan(self):
        rospy.loginfo("In openloop, replanning on graph...")
        rospy.sleep(1) # ensure goals set during this function are after the cancel time

        self.entered_openloop = True
        self.node = None
        start = self.vprev
        dist, paths = nx.single_source_dijkstra(self.curr_graph.graph, start, 'g')

        if dist['g'] == float('inf'):
            print(self.curr_graph.edges(data='state'))
            rospy.loginfo("Cannot go to goal! Ending task.")
            self.finish_task()
            return # path_blocked = True from now until shutdown

        path = paths['g']

        self.set_new_path(path)
        self.set_and_send_next_goal()
        self.path_blocked = False

    def edge_callback(self, data):
        # updates edges
        u = data.u
        if u.isdigit(): u = int(u)

        v = data.v
        if v.isdigit(): v = int(v)

        if data.state != self.base_map.G.UNKNOWN:
            # TODO: have some 'average' of states before setting it, there appears to be the
            # occasional false blocks/unblocks
            self.curr_graph.set_edge_state(u,v,data.state)

        if data.state == self.base_map.G.BLOCKED:
            self.curr_graph.set_edge_weight(u,v,float('inf'))

            # if robot is not already replanning and the blocked edge is not under observation, 
            if not (self.path_blocked or self.edge_under_observation(u,v)):
                for i, v_i in enumerate(self.path):
                    if i == 0: continue
                    if (self.path[i-1] == u and v_i == v) or (self.path[i-1] == v and v_i == u):
                        # remember to check both directions
                        self.path_blocked = True
                        self.client.cancel_goals_at_and_before_time(rospy.get_rostime())
                        rospy.loginfo("Path is blocked!")
                        self.replan()
                        break

    def edge_under_observation(self, u, v):
        if self.node != None and self.node.opair != None:
            (ou, ov) = self.node.opair.E 
            if (ou == u and ov == v) or (ou == v and ov == u):
                return True

        return False

    def reset_travel_dist(self):
        self.travelled_dist = 0

    def check_mode(self):
        assert (self.mode == "policy" or self.mode == "openloop" or self.mode == "naive"), (
            "Mode '{}' is not valid!".format(self.mode))

    def clear_costmap_client(self):
        # service client for clear_costmaps
        rospy.wait_for_service('move_base/clear_costmaps')
        try:
            clear_costmap = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
            clear_costmap()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

    def set_robot_pose(self, x, y, yaw):
        # service client for setting robot pose in gazebo
        model_state = ModelState("jackal",
                Pose(Point(x,y,1),Quaternion(*(quaternion_from_euler(0,0,yaw)))),
                Twist(Vector3(0,0,0), Vector3(0,0,0)), "map")

        rospy.wait_for_service('gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
            set_model_state(model_state)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

    def set_amcl_pose(self):
        # set initial position for amcl
        init_pose = PoseWithCovarianceStamped()
        init_pose.pose.pose = self.start_pose
        init_pose.pose.covariance = np.zeros(36)
        init_pose.header.stamp = rospy.Time.now()
        init_pose.header.frame_id = "map"

        self.amcl_publisher.publish(init_pose)

    def get_robot_pose(self, model):
        # service client for getting robot pose in gazebo
        rospy.wait_for_service('gazebo/get_model_state')
        try:
            get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
            state = get_model_state(model, "world")
            return (state.pose.position.x, state.pose.position.y)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

def to_2d_path(rospath):
    # rospath - ros path message, seq of stamped poses
    path = []

    for stamped_pose in rospath.poses:
        path.append((stamped_pose.pose.position.x,
                     stamped_pose.pose.position.y))

    return path

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

if __name__ == '__main__':
    # load nxgraph and polygon information
    graph = nx.read_yaml(PKGDIR + '/maps/' + MAP + '/' + MAP + '_tgraph.yaml')
    poly_dict = tgraph.polygon_dict_from_csv(PKGDIR + '/maps/' + MAP + '/' + MAP + '_polygons.csv')

    # setup logging
    logging.basicConfig(level=logging.INFO)

    # set number of tasks
    ntasks = 5

    # run LRPP
    try:
        LRPP(graph, poly_dict, T=ntasks)
    except rospy.ROSInterruptException:
        rospy.loginfo("Finished simulation.")

    # add some test data collection like:
    #   - graph data, states, weights
