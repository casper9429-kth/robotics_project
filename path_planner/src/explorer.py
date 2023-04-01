#!/usr/bin/env python3

import math
import rospy
import random
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
import tf2_ros 
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool
from mapping.msg import GridMapMsg



class explorer():
    def __init__(self):
        
        # Init node
        rospy.init_node('explorer')
        
        # tf stuff
        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(1)
        print('Tf2 stuff initialized')
        self.rate = rospy.Rate(10)
        # subscribers
        self.grid_map_sub = rospy.Subscriber("/map/GridMap", GridMapMsg, self.map_callback)
        
        ########## test subscriber ##########
        # self.bool_sub = rospy.Subscriber('/bool', Bool, self.map_callback)

        # publishers
        self.goal_pub = rospy.Publisher('/start_and_goal', PoseStamped, queue_size=10) # change to the topic to what the path_planner subscribes to
        #self.start_and_goal_pub = rospy.Publisher('/start_and_goal', PoseStamped, queue_size=10)

        # Parameters
        self.map_coords = []
        self.nearest_goal = None
        self.timer = rospy.Time.now() 

        # Position and orientation of the robot
        self.pose = PoseStamped()
        self.pose_in_map = PoseStamped()
        self.pose_in_gridmap = PoseStamped()

        # Position and orientation of the robot in the base_link frame
        self.pose.header.frame_id = 'base_link'
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 0.0

        
    ########## test callbacks ##########
    # def map_callback(self, msg):
    #     Array = []
    #     for i in range(10):
    #         array = [random.choice([-1, 0, 1]) for _ in range(10)]
    #         Array.append(array)
    #     self.map_coords = Array
    #     # print(self.map_coords)
    #     self.transforms()
    #     self.find_goal()

    # # test transform
    # def transforms(self):
    #     stamp = self.pose.header.stamp  
    #     try:                                   
    #         transform_base_link_2_map = self.tfBuffer.lookup_transform('map','base_link', stamp,rospy.Duration(0.5)) # lookup_transform('target frame','source frame', time.stamp, rospy.Duration(0.5))
    #         self.pose_in_map = tf2_geometry_msgs.do_transform_pose(self.pose, transform_base_link_2_map)
    #         self.point = np.array([1,4])
            
    #     except:
    #         print('No transform found')
    ########################################

    # dummy function to test if the path_planner communication is working properly
    def test_path_planner_communication(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = 1.0
        goal.pose.position.y = 1.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0     # it is commented so that the robot keeps its orientation
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        #print(f'goal: {goal}')
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = rospy.Time.now()
        start.pose.position.x = 0.0
        start.pose.position.y = 0.0
        start.pose.position.z = 0.0
        start.pose.orientation.x = 0.0
        start.pose.orientation.y = 0.0     # it is commented so that the robot keeps its orientation
        start.pose.orientation.z = 0.0
        start.pose.orientation.w = 1.0
        #print(f'start: {start}')
        start_and_goal = Path()
        start_and_goal.header.frame_id = 'map'
        start_and_goal.header.stamp = rospy.Time.now()




        start_and_goal.poses.append(start)
        start_and_goal.poses.append(goal)
        #print(start_and_goal)

        self.start_and_goal_pub.publish(start_and_goal)
        rospy.loginfo('sent start and goal to path_planner')

    def map_callback(self, msg: GridMapMsg):
        # extract the map cells and their values from the message 
        #   rospy.loginfo('Explorer: enter callback')
        self.map_coords = msg.data
        self.map_resolution = msg.resolution
        self.bbminx = msg.bbminx
        self.bbminy = msg.bbminy
        self.bbmaxx = msg.bbmaxx
        self.bbmaxy = msg.bbmaxy
        self.t_stamp = msg.header.stamp
        self.origo_index_i = msg.origo_index_i
        self.origo_index_j = msg.origo_index_j
        if rospy.Time.now() - self.timer > rospy.Duration(0.5):
            self.timer = rospy.Time.now()
            self.transforms()
            #self.find_goal()
            self.find_goal()

    def transforms(self):   
        stamp = self.pose.header.stamp  
        try:                                   
            transform_base_link_2_map = self.tfBuffer.lookup_transform('map','base_link', self.t_stamp,rospy.Duration(0.5)) # lookup_transform('target frame','source frame', time.stamp, rospy.Duration(0.5))
            self.pose_in_map = tf2_geometry_msgs.do_transform_pose(self.pose, transform_base_link_2_map)
            
            # calculates in what cell the robot is   
            delta_i = self.pose_in_map.pose.position.x/self.map_resolution
            delta_j = self.pose_in_map.pose.position.y/self.map_resolution
            self.pose_in_gridmap = self.pose_in_map
            self.pose_in_gridmap.pose.position.x = self.origo_index_i+delta_i
            self.pose_in_gridmap.pose.position.y = self.origo_index_j+delta_j
            self.position_in_gridmap = np.array([self.pose_in_gridmap.pose.position.x, self.pose_in_gridmap.pose.position.y])

        except:
            print('No transform found')


    def find_goal(self):
        self.nearest_goal = None
        self.cells = []

        # extracts the cells that are unknown space a.k.a float32[] data
        for i , data_i in enumerate(self.map_coords):
            for j, data_j in enumerate(data_i.data):
                if int(data_j) == -1:
                    self.cells.append([i,j])

        #self.map_coords[i].data[j] == -1
        # calculate the nearest point to the robot
        self.cells = np.array(self.cells)
        #print(self.position_in_gridmap)
        distances = self.cells - self.position_in_gridmap
        min_distance_index = np.argmin(np.linalg.norm(distances, axis=1))
        self.nearest_goal = self.cells[min_distance_index]
    
        self.publish_goal()


    def publish_goal(self):         #transforms the goal to the map frame
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.t_stamp
        goal.pose.position.x = self.bbminx + (self.nearest_goal[0])*self.map_resolution
        goal.pose.position.y = self.bbminy + (self.nearest_goal[1])*self.map_resolution
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0     # it is commented so that the robot keeps its orientation
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.0
        self.goal_pub.publish(goal)
        # print(goal.pose.position.x, goal.pose.position.y)
        # print('Goal published')

    def publish_start_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.t_stamp
        goal.pose.position.x = self.bbminx + (self.nearest_goal[0])*self.map_resolution
        goal.pose.position.y = self.bbminy + (self.nearest_goal[1])*self.map_resolution
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0     # it is commented so that the robot keeps its orientation
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.0

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.t_stamp
        start.pose.position.x= self.pose_in_map.pose.position.x
        start.pose.position.y= self.pose_in_map.pose.position.y
        start.pose.position.z = 0.0
        start.pose.orientation.x = 0.0
        start.pose.orientation.y = 0.0     # it is commented so that the robot keeps its orientation
        start.pose.orientation.z = 0.0


        start_and_goal = Path()
        start_and_goal.header.frame_id = 'map'
        start_and_goal.header.stamp = self.t_stamp

        start_and_goal.poses.append(start)
        start_and_goal.poses.append(goal)
        #self.start_and_goal_pub.publish(start_and_goal)
        rospy.loginfo('Explorer: Start and goal published')
        #print(f'Explorer: start: {start.pose.position.x}, {start.pose.position.y}')
        
        #print(f'Explorer: goal: {goal.pose.position.x}, {goal.pose.position.y}')
        
    def spin(self):
        while not rospy.is_shutdown():
            #self.test_path_planner_communication()
            self.rate.sleep()



        print('Start published')
if __name__ == '__main__':
    # Needs to be this way because otherwise the node will not be able to publish correctly
    explorer = explorer()
    explorer.spin()
        
