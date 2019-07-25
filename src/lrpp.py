#!/usr/bin/env python
__author__ = 'fiorellasibona' # repurposed code
import logging
logger = logging.getLogger(__name__) 
import rospy, rospkg 
import math 
import os, glob
import networkx as nx
import cv2

import actionlib
from policy.gridgraph import GridGraph, LiveGridGraph
from policy.classes import Map
from policy import utility as util
from policy import rpp, timing

from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class LRPP():
    def __init__(self, base_map, mapdir, T=1):
        # seq - list of vertices [v1, v2, ...]
        # base_map is map type
        rospy.init_node('move_base_seq')

        self.base_map = base_map.G
        self.features = base_map.features()
        self.M = [base_map] # initialize map storage
        self.T = T  # number of tasks to execute
        self.tcount = 1 # current task being executed
        self.LiveMap = None # filled in by map_callback
        self.mapdir = mapdir

        self.path_blocked = False # flag for path being blocked, calls reactive algo
        self.observe = False # flag for making an observation
        self.at_final_goal = False # flag for reaching final destination

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.posearray_publisher = rospy.Publisher("waypoints", PoseArray, queue_size=1)

        self.start_task()
        rospy.spin()

    def start_task(self):
        rospy.loginfo("Calculating policy for task {}...".format(self.tcount))
        # calculate policy
        self.p = update_p_est(self.M, self.tcount) 
        policy = rpp.solve_RPP(self.M, self.p, self.features, self.base_map.start,
                self.base_map.goal)
        rospy.loginfo(policy[0].print_policy())
        # setup policy in set_new_path
        self.node = policy[0].next_node()
        self.path = self.node.path

        self.set_new_path(self.path, self.base_map) # sets pose_seq and goal_cnt

        # wait for input before sending goals to move_base
        raw_input('Press any key to begin execution of task {}'.format(self.tcount))

        # double check connection with move base
        self.check_connection()
        rospy.loginfo("Status of action client: {}".format(self.client.simple_state))
        self.client.simple_state = 0 # set back to pending

        # setup map_subscriber
        self.map_subscriber = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1, buff_size=2**24)
        # self.map_subscriber = rospy.Subscriber("move_base/global_costmap/costmap", OccupancyGrid, self.map_callback, queue_size=1, buff_size=2**24)

        # run set_and_send_next goal with path
        self.set_and_send_next_goal()

    def finish_task(self):
        # wait until status == 3 and final goal pose reached before executing
        # unsubscribe from map
        self.map_subscriber.unregister()
        # run map filter
        self.save_map_and_filter()
        # shut down cartographer and move_base
        os.system("rosnode kill /cartographer_node /cartographer_occupancy_grid_node /move_base")
        # increment task counter
        self.tcount +=1
        # check if we need to keep going
        if self.tcount == self.T + 1:
            self.shutdown()
    
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
        filepath = mapdir + "{:02d}".format(n) # assuming t < 100
        os.system("rosrun map_server map_saver -f " + filepath)

        gridgraph = GridGraph(filepath + ".pgm", filepath + ".yaml", goal,
                refmap=self.base_map)
        new_map = Map(gridgraph)

        # now do comparisons and map merging??
        # M = filter(M,new_map)

        # do no comparisons and just save the new map lol
        self.M.append(new_map)

    def shutdown(self):
        rospy.loginfo("Shutting down...")
        rospy.signal_shutdown("Finished tasks.")

    def active_cb(self):
        rospy.loginfo("Goal pose " + str(self.goal_cnt) + " is now being processed by the Action server...")

    def feedback_cb(self, feedback):
        # print current pose at each feedback
        # feedback is MoveBaseFeedback type msg
        pose = feedback.base_position.pose
        rospy.loginfo("Feedback for goal " + str(self.goal_cnt) + ":\n" +
                      self.print_pose_in_euler(pose))

        # check if robot is close enough to send next goal
        position = feedback.base_position.pose.position
        (x,y) = (position.x, position.y)

        curr_goal = self.pose_seq[self.goal_cnt].position
        (gx,gy) = (curr_goal.x, curr_goal.y)

        dist_to_curr_goal = util.euclidean_distance((x,y), (gx,gy))

        if dist_to_curr_goal < 0.5 and (self.goal_cnt < len(self.pose_seq) - 1):
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " reached!")
            self.goal_cnt += 1
            self.set_and_send_next_goal()
        elif dist_to_curr_goal < 0.25 and (self.goal_cnt == len(self.pose_seq) - 1):
            rospy.loginfo("Reached end of path")
            # observe and select next node based on state of observed feature
            if self.observe == False and self.going_to_final_goal() != True:
                self.observe == True
                edge_to_observe = self.node.opair.E
                self.make_observation(edge_to_observe)
            # if feature is unknown move to reactive b/c it is not visible when it
            # should've been

    def make_observation(self, edge):
        # returns state of edge in current LiveMap
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
        if self.path[self.goal_cnt] == self.base_map.goal: return True
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
                self.start_task()
            return

        if status == 4:
            # The goal was aborted during execution by the action server due
            # to some failure (Terminal State)
            rospy.loginfo("Goal pose " + str(self.goal_cnt) +
                " was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose " + str(self.goal_cnt) +
                " aborted, shutting down!")
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

    def set_new_path(self, path, gridgraph, at_first_node = True):
        points = list() 
        for i in path:
            (x,y) = gridgraph.pos(i)
            new_point = (x,y,0.0)
            points.append(new_point)
            print(new_point)

        yaweulerangles_seq = self.calc_headings(points)
        # if not at_first_node:
            # yaweulerangles_seq[0] = yaweulerangles_seq[1] # can change if this causes problems
        rospy.loginfo('Angle seq: ')
        for angle in yaweulerangles_seq:
            rospy.loginfo(angle)
        quat_seq = list()
        self.pose_seq = list() # list of poses, combo of Point and Quaternion
        self.goal_cnt = 0
        # self.goal_cnt = 1 if at_first_node else 0
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

    def map_callback(self, data):
        # constantly checking if edge is blocked
        if self.path_blocked == False:
            LiveMap = LiveGridGraph(data, self.base_map, robot_width=0.5)
            self.LiveMap = LiveMap # this may cause timing issues
            # cv2.imwrite('liveMap.jpg', LiveMap.occ_grid)

            # follow path
            for i in range(max(1, self.goal_cnt), len(self.path)):
                u = self.path[max(0,i-1)]
                v = self.path[max(1,i)]
                LiveMap._edge_check((u,v))
                (x1,y1) = LiveMap.pos(u)
                (x2,y2) = LiveMap.pos(v)
                rospy.loginfo("Probability of edge ({:.3f},{:.3f}),({:.3f},{:.3f}): {:.3f}"
                              .format(x1, y1, x2, y2, LiveMap.graph[u][v]['prob']))
                if LiveMap.graph[u][v]['state'] == LiveMap.BLOCKED:
                    self.path_blocked = True;
                    rospy.loginfo("Path is blocked!")
                    self.client.cancel_goals_at_and_before_time(rospy.get_rostime())
                    if i == self.goal_cnt:
                        self.goal_cnt = self.goal_cnt - 1
                    break

            if self.path_blocked:
                # cv2.imwrite('liveMap.jpg', LiveMap.occ_grid)
                #LiveMap._collision_check()
                start = self.path[max(0,self.goal_cnt)]
                came_from, cost_so_far = util.a_star_search(LiveMap, start, LiveMap.goal, check_edges=True)
                try:
                    self.path = util.reconstruct_path(came_from, start, LiveMap.goal)
                except KeyError:
                    rospy.loginfo("Cannot go to goal! Ending task.")
                    self.client.cancel_all_goals()
                    self.finish_task()
                    self.start_task()
                    return # path_blocked = True from now until shutdown

                self.set_new_path(self.path, LiveMap, at_first_node = False)
                self.set_and_send_next_goal()
                self.path_blocked = False

        self.posearray_publisher.publish(self.conv_to_PoseArray(self.pose_seq))

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

def import_base_map(folder, goal):
    # imports only 00 pgm/yaml file as base map
    # assumes they are all for the same environment and start/goal is the same

    for pgm_path in sorted(glob.glob(folder + "/*.pgm")):

        # get yaml file as well
        (root, ext) = os.path.splitext(pgm_path)
        filename = os.path.basename(root)
        yaml_path = root + ".yaml"
        print(pgm_path)
        print(yaml_path)

        if filename == '00':
            # assume 00 is the zero map
            gridgraph = GridGraph(pgm_path, yaml_path, goal, graph_res=1.5) 
            base_map = Map(gridgraph)
            timing.log("Finished visibility check")
            print("Imported base map.")
            break

    return base_map 


if __name__ == '__main__':
    # specify start and goal
    start = (0.0, 0.0)
    #goal = (4.0, -4.0) #for robohub
    # goal = (-8.0, 4.5)
    # goal = (8, 0) # for maze
    goal = (8.5, 8.0) # for maze
    # goal = (0.0,8.5)
    # goal = (-6.0, 3.7)
    # goal = (0.0, -20.0)
    ntasks = 5
    # load map and determine graph
    rospack = rospkg.RosPack()
    pkgdir = rospack.get_path('policy')
    mapdir = pkgdir + '/maps/'

    # setup logging for rpp
    logging.basicConfig(filename = pkgdir + '/lrpp_multi_debug.log', filemode='w', level=logging.INFO)
    # import maps
    base_map = import_base_map(mapdir, goal)
    base_map.G.show_img()

    # run LRPP
    try:
        LRPP(base_map, mapdir, T=ntasks)
    except rospy.ROSInterruptException:
        rospy.loginfo("Finished simulation.")

