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

        # Subscribers
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)


    def goal_callback(self, msg):
        rospy.loginfo("Goal received")
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = msg.header.stamp
        self.goal.target_pose.pose.position.x = msg.pose.position.x
        self.goal.target_pose.pose.position.y = msg.pose.position.y
        self.goal.target_pose.pose.position.z = msg.pose.position.z
        self.goal.target_pose.pose.orientation.x = msg.pose.orientation.x
        self.goal.target_pose.pose.orientation.y = msg.pose.orientation.y
        self.goal.target_pose.pose.orientation.z = msg.pose.orientation.z
        self.goal.target_pose.pose.orientation.w = msg.pose.orientation.w
        self.path_client.send_goal(self.goal)
        self.path_client.wait_for_result()


if __name__ == '__main__':
    try:
        pp = path_planner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
