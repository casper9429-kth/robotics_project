#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
import tf2_ros 
import tf2_geometry_msgs

from mapping.msg import GridMapMsg
from std_srvs.srv import Trigger, TriggerResponse


class Explorer():
    def __init__(self):
        
        # Init node
        rospy.init_node('explorer')
        
        # tf stuff
        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(0.5) # TODO: check if this is necessary

        # subscribers
        self.grid_map_sub = rospy.Subscriber("/map/GridMap", GridMapMsg, self.map_callback)

        # publishers
        # self.goal_pub = rospy.Publisher('/start_and_goal', PoseStamped, queue_size=10) # TODO: change to the topic to what the path_planner subscribes to
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10) # TODO: change to the topic to what the path_planner subscribes to

        # Parameters
        self.map_coords = []
        self.nearest_goal = None
        self.is_running = False
        self.timer = rospy.Time.now() 

        # Position and orientation of the robot
        self.pose = PoseStamped()
        self.pose_in_map = PoseStamped()
        self.pose_in_gridmap = PoseStamped()
        
        # Services
        self.start_service = rospy.Service('/explorer/start', Trigger, self.start_callback)
        self.stop_service = rospy.Service('/explorer/stop', Trigger, self.stop_callback)

    def start_callback(self, req):
        self.is_running = True
        return TriggerResponse(success=True, message='Started')
    
    def stop_callback(self, req):
        self.is_running = False
        return TriggerResponse(success=True, message='Stopped')

    def map_callback(self, msg: GridMapMsg):
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
            self.find_goal()
            if self.nearest_goal is not None and self.is_running:
                self.publish_goal()

    def transforms(self):   
        stamp = self.pose.header.stamp  
        try:                                   
            transform_base_link_2_map = self.tfBuffer.lookup_transform('map','base_link', self.t_stamp,rospy.Duration(0.5)) 
            self.pose_in_map = tf2_geometry_msgs.do_transform_pose(self.pose, transform_base_link_2_map)
            delta_i = self.pose_in_map.pose.position.x/self.map_resolution
            delta_j = self.pose_in_map.pose.position.y/self.map_resolution
            self.pose_in_gridmap.pose.position.x = self.origo_index_i+delta_i
            self.pose_in_gridmap.pose.position.y = self.origo_index_j+delta_j
            self.position_in_gridmap = np.array([self.pose_in_gridmap.pose.position.x, self.pose_in_gridmap.pose.position.y])

        except:
            print('No transform found')

    def find_goal(self):
        self.nearest_goal = None
        self.cells = []

        for i , data_i in enumerate(self.map_coords):
            for j, data_j in enumerate(data_i.data):
                if int(data_j) == -1:
                    self.cells.append([i,j])
                    
        if len(self.cells) > 0:
            self.cells = np.array(self.cells)
            distances = self.cells - self.position_in_gridmap
            min_distance_index = np.argmin(np.linalg.norm(distances, axis=1))
            self.nearest_goal = self.cells[min_distance_index]

    def publish_goal(self):      
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.t_stamp
        goal.pose.position.x = self.bbminx + (self.nearest_goal[0])*self.map_resolution
        goal.pose.position.y = self.bbminy + (self.nearest_goal[1])*self.map_resolution
        self.goal_pub.publish(goal)


if __name__ == '__main__':
    explorer = Explorer()
    rospy.spin()
        
