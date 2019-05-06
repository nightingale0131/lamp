#!/usr/bin/env python
__author__ = 'fiorellasibona' # repurposed code
import logging
logger = logging.getLogger(__name__) 
import rospy, rospkg 
import math 
import os
import networkx as nx
import cv2

import actionlib
from policy.gridgraph import GridGraph, LiveGridGraph
from policy import utility as util
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class MoveBaseSeq():
    def __init__(self, path, gridgraph):
        # seq - list of vertices [v1, v2, ...]
        # gridgraph - base grid graph map needed for graph edges
        rospy.init_node('move_base_seq')
        self.base_map = gridgraph;
        self.path = path
        self.path_blocked = False

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.map_subscriber = rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        self.posearray_publisher = rospy.Publisher("waypoints", PoseArray, queue_size=1)

        self.set_new_path(path, self.base_map) # sets pose_seq and goal_cnt

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

    def active_cb(self):
        rospy.loginfo("Goal pose " + str(self.goal_cnt) + " is now being processed by the Action server...")

    def feedback_cb(self, feedback):
        # print current pose at each feedback
        rospy.loginfo("Feedback for goal " + str(self.goal_cnt) + ": " + str(feedback))

    def done_cb(self, status, result):
        # refer to http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            # The goal received a cancel request after it started executing
            # and has since completed its execution (Terminal State)
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + 
                "received a cancel request after it started executing, finished execution!")

        if status == 3:
            # The goal was achieved successfully by the action server (Terminal State)
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " reached")
            self.goal_cnt += 1
            if self.goal_cnt < len(self.pose_seq):
                self.set_and_send_next_goal()
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
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

    def movebase_client(self):
        self.set_and_send_next_goal()
        rospy.spin()

    def set_and_send_next_goal(self):
        next_goal = MoveBaseGoal()
        next_goal.target_pose.header.frame_id = "map"
        next_goal.target_pose.header.stamp = rospy.Time.now()
        next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose " +
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
        euler = euler_from_quaternion(quaternion)

        return str(pose.position) + "[" + str(euler) + "]"

    def map_callback(self, data):
        # constantly checking if edge is blocked
        if self.path_blocked == False:
            LiveMap = LiveGridGraph(data, self.base_map, robot_width=0.5)
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
                    break
                # ^ causes client to flip between sending new goal and cancelling

            if self.path_blocked:
                cv2.imwrite('liveMap.jpg', LiveMap.occ_grid)
                LiveMap._collision_check()
                start = self.path[max(0,self.goal_cnt - 1)]
                came_from, cost_so_far = util.a_star_search(LiveMap, start, LiveMap.goal)
                try:
                    self.path = util.reconstruct_path(came_from, start, LiveMap.goal)
                except KeyError:
                    rospy.loginfo("Cannot go to goal! Stopping node.")
                    rospy.signal_shutdown("Cannot go to goal! Stopping node.")
                    return # path_blocked = True from now until shutdown

                self.set_new_path(self.path, LiveMap, at_first_node = False)
                self.set_and_send_next_goal()
                self.path_blocked = False

        self.posearray_publisher.publish(self.conv_to_PoseArray(self.pose_seq))

if __name__ == '__main__':
    # specify start and goal
    start = (0.0, 0.0)
    goal = (-8.0, 4.5)
    # goal = (-6.0, 3.7)
    # load map and determine graph
    rospack = rospkg.RosPack()
    pkgdir = rospack.get_path('policy')
    mapdir = pkgdir + '/maps/'
    pgm0 = mapdir + 'simple1.pgm'
    yaml0 = mapdir + 'simple1.yaml'

    logging.basicConfig(filename = pkgdir + '/debug.log', filemode='w', level=logging.INFO)

    map0 = GridGraph(pgm0, yaml0, goal, graph_res=1.5, robot_width=0.5)
    print(nx.info(map0.graph))
    # nx.write_adjlist(map0.graph, pkgdir + '/src/map0.adjlist')

    # run a* on custom map object
    came_from, cost_so_far = util.a_star_search(map0, map0.start, map0.goal)
    path = util.reconstruct_path(came_from, map0.start, map0.goal)

    # give path to MoveBaseSeq
    try:
        MoveBaseSeq(path, map0)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

    # save pgm/yaml file of map
    os.system("rosrun map_server map_saver -f " + mapdir + "/test")
