#!/usr/bin/env python
__author__ = 'fiorellasibona' # repurposed code
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class MoveBaseSeq():
    def __init__(self, path):
        # seq - list of coordinates [(x1, y1, z1), (x2, y2, z1)]
        rospy.init_node('move_base_seq')
        points = list(path)
        yaweulerangles_seq = self.calc_headings(points)
        rospy.loginfo('Angle seq: ')
        for angle in yaweulerangles_seq:
            rospy.loginfo(angle)
        raw_input('Press enter to continue')

        quat_seq = list()
        self.pose_seq = list() # list of poses, combo of Point and Quaternion
        self.goal_cnt = 1 # assume first pose is current pose

        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle))))

        n = 0
        for point in points:
            self.pose_seq.append(Pose(Point(*point), quat_seq[n]))
            n += 1

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

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

        return yaweulerangles_seq

    def print_pose_in_euler(self, pose):
        quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
        euler = euler_from_quaternion(quaternion)

        return str(pose.position) + "[" + str(euler) + "]"


if __name__ == '__main__':
    path = [(0.0, 0.0, 0.0), (-2.0, -1.0, 0.0), (-4.0, -2.0, 0.0)]
    try:
        MoveBaseSeq(path)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
