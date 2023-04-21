#!/usr/bin/env python3
import math
import rospy
import tf2_ros
import actionlib
import tf2_geometry_msgs
from robp_msgs.msg import DutyCycles
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, PoseArray, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
import tf

class path_planner():
    def __init__(self):
        rospy.init_node('path_tracker_client')
        self.path_client = actionlib.SimpleActionClient('path_tracker', MoveBaseAction)
        self.path_client.wait_for_server()


        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # self.goal = MoveBaseGoal()
        # self.goal.target_pose.header.frame_id = "map"
        # self.goal.target_pose.header.stamp = rospy.Time.now()
        # self.goal.target_pose.pose.position.x = 1.0
        # self.goal.target_pose.pose.position.y = 0.0
        # self.goal.target_pose.pose.position.z = 0.0
        # self.goal.target_pose.pose.orientation.x = 0.0
        # self.goal.target_pose.pose.orientation.y = 0.0
        # self.goal.target_pose.pose.orientation.z = 0.0
        # self.goal.target_pose.pose.orientation.w = 1.0
        # self.path_client.send_goal(self.goal)
        # self.path_client.wait_for_result()
        # print("Path tracker finished")


    def goal_callback(self, msg):
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = msg.pose.position.x
        self.goal.target_pose.pose.position.y = msg.pose.position.y
        self.goal.target_pose.pose.position.z = msg.pose.position.z
        self.goal.target_pose.pose.orientation.x = msg.pose.orientation.x
        self.goal.target_pose.pose.orientation.y = msg.pose.orientation.y
        self.goal.target_pose.pose.orientation.z = msg.pose.orientation.z
        self.goal.target_pose.pose.orientation.w = msg.pose.orientation.w
        self.path_client.send_goal(self.goal)
        self.path_client.wait_for_result()
        rospy.loginfo("Goal received")


if __name__ == '__main__':
    try:
        pp = path_planner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
