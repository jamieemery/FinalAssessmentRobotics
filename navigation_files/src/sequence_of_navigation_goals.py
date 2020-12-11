#!/usr/bin/env python

import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from TSP_solutions import TSP
from our_node import Node


class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        p_seq = [8,19.6,0,15,19.6,0,21,19.6,0,8,8.2,0,15,8.2,0,22,8.2,0]
        # refilling point's position
        refill_p = [32.2,14,0]
        self.refill_yaw = Quaternion(*(quaternion_from_euler(0, 0, 90*math.pi/180, axes='sxyz')))
        self.refill_p = Pose(Point(*refill_p),self.refill_yaw)
        # starting point's position
        start_p = [4.85,14.4,0]
        self.start_yaw = Quaternion(*(quaternion_from_euler(0, 0, 90*math.pi/180, axes='sxyz')))
        self.start_p = Pose(Point(*refill_p),self.start_yaw)
        # amount of spray
        self.spray_counter = 100
        # flag to check if it's been to the refilling point
        self.flag = 0
        # Only yaw angle required (no rotations around x and y axes) in deg:
        yea_seq = [90,90,90,-90,-90,-90]
        #List of goal quaternions:
        quat_seq = list()
        #List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0
        for yawangle in yea_seq:
            #Unpacking the quaternion tuple and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [p_seq[i:i+n] for i in range(0, len(p_seq), n)]
        rospy.loginfo(str(points))
        for point in points:
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1
        self.spray_needed = [37, 23, 54, 20, 20, 19]

        i=0
        self.combine_pose_spray=[]
        for array in self.spray_needed:
            i += 1
            self.combine_pose_spray.append([self.pose_seq[i-1], self.spray_needed[i-1]])

        #rospy.loginfo(str(self.pose_seq))
        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        #wait = self.client.wait_for_server(rospy.Duration(5.0))
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Calculating optimal path for visiting the goals...")
        notVisited = list()
        # transform points to Node objects
        for point in self.pose_seq:
            notVisited.append(Node(point.position.x,point.position.y,point.position.z,point.orientation))
        start = Node(self.start_p.position.x,self.start_p.position.y,self.start_p.position.z,self.start_p.orientation)
        solution = TSP()
        # TSP's greedy algorithm implementation
        # seq_nodes = solution.greedyTSPAlgorithm(start, notVisited)
        seq_nodes = solution.bruteTSPAlgorithm(notVisited)
        # turn Node objects back to Pose objects for the Action server to use
        self.pose_seq = list()
        for node in seq_nodes:
            point = [node.x,node.y,node.z]
            self.pose_seq.append(Pose(Point(*point),node.theta))
        rospy.loginfo(str(self.pose_seq))
        # assign spray amount needed to every goal, randomly for this trial

        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def organised_spray_needed (self, table):     # sort the spray_needed according to the new goal pose
        for row in self.combine_pose_spray:
            if row[0]==table:
                return row[1]

    def active_callback(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_callback(self, feedback):
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_callback(self, status, result):
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
	        self.goal_cnt += 1
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            if self.flag:
                rospy.loginfo("Ready for business again!")
                self.flag = 0
            else:
                self.goal_cnt += 1
                rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
                rospy.loginfo("Spraying area!")
                self.spray_counter -= self.organised_spray_needed(self.pose_seq[self.goal_cnt-1])
            if self.spray_counter >= self.organised_spray_needed(self.pose_seq[self.goal_cnt]) and self.goal_cnt < len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_callback, self.active_callback, self.feedback_callback)
            elif self.spray_counter < self.organised_spray_needed(self.pose_seq[self.goal_cnt]) and self.goal_cnt < len(self.pose_seq):
                rospy.loginfo("Spray amount is not enough, heading to refilling station!")
                self.flag = 1
                self.spray_counter = 100
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = self.refill_p
                self.client.send_goal(goal, self.done_callback)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
	        self.goal_cnt += 1
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
	        self.goal_cnt += 1
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
	        self.goal_cnt += 1
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
    #for pose in pose_seq:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")