#!/usr/bin/env python3

import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
import tf2_ros 
import tf2_geometry_msgs



class explorer():
    def __init__(self):
        
        # Init node
        rospy.init_node('explorer')

        # subscribers
        self.map_sub = rospy.Subscriber('/map', PoseStamped, self.map_callback) # change to the topic that the map is published on

        # publishers
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10) # change to the topic to what the path_planner subscribes to

        # Parameters
        self.map_coords = []
        self.nearest_goal = None

        # tf stuff
        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(2)
        print('Tf2 stuff initialized')

        # Position and orientation of the robot
        self.pose = PoseStamped()
        self.pose_in_map = PoseStamped()
        # Position and orientation of the robot in the base_link frame
        self.pose.header.frame_id = 'base_link'
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 0.0


    def map_callback(self, msg: PoseStamped):
        # extract the map coords from the message
        # self.map_coords = np.array([msg.pose.position.x, msg.pose.position.y]) # change the msg to what the map is published as and extract x and y coords
        self.map_coords = np.array([(3,4), (1,2), (5,6), (7,8)])
        print('Map received')
        self.transforms()
        self.find_goal()


    def transforms(self):   
        stamp = self.pose.header.stamp  
        try:                                    # lookup_transform('target frame','source frame', time.stamp, rospy.Duration(0.5))
            transform_base_link_2_map = self.tfBuffer.lookup_transform('map','base_link', stamp,rospy.Duration(0.5))               # odom or map
            self.pose_in_map = tf2_geometry_msgs.do_transform_pose(self.pose, transform_base_link_2_map)   
            self.point = np.array([self.pose_in_map.pose.position.x, self.pose_in_map.pose.position.y])
            # self.point = np.array([1,1])
            print('Transform found')
        except:
            print('No transform found')


    def find_goal(self):
        self.nearest_goal = None
        # Add a way to only look at the points that is unexplored (value of -1 with a neighbour of 0)


        # calculate the nearest point to the robot
        for idx, data in enumerate(self.map_coords):
            distances = np.linalg.norm(data-self.point)
            if self.nearest_goal is None:
                self.nearest_goal = data
            elif np.linalg.norm(distances) < np.linalg.norm(self.nearest_goal-self.point):
                self.nearest_goal = data
            print(data)
        # print(self.nearest_goal)
        self.publish_goal()


    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = self.nearest_goal[0]
        goal.pose.position.y = self.nearest_goal[1]
        goal.pose.position.z = 0.0
        # goal.pose.orientation.x = 0.0
        # goal.pose.orientation.y = 0.0     # it is commented so that the robot keeps its orientation
        # goal.pose.orientation.z = 0.0
        # goal.pose.orientation.w = 0.0
        self.goal_pub.publish(goal)
        print('Goal published')



if __name__ == '__main__':
    explorer = explorer()
    rospy.spin()