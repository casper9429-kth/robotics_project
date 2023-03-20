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

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = 1.0
        self.goal.target_pose.pose.position.y = 0.0
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.0
        self.goal.target_pose.pose.orientation.z = 0.0
        self.goal.target_pose.pose.orientation.w = 1.0
        self.path_client.send_goal(self.goal)
        self.path_client.wait_for_result()
        print("Path tracker finished")

if __name__ == '__main__':
    try:
        pp = path_planner()
    except rospy.ROSInterruptException:
        pass
