#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
import numpy as np
import colorsys

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
from  math import pi
from aruco_msgs.msg import MarkerArray
import tf


class Detection:
    def __init__(self):
        # Initialize node
        rospy.loginfo('Initializing detection node')
        rospy.init_node('detection')
        
        # Create subscriber 
        self.sub_goal = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.cloud_callback)
        
        # Create a publisher to the aruco markers
        self.pub_red = rospy.Publisher('/camera/depth/color/red_points', PointCloud2,queue_size=1)
        self.pub_turqoise = rospy.Publisher('/camera/depth/color/turqoise_points', PointCloud2,queue_size=1)
        
        
        # Robot parameters
        self.ticks_per_rev = 3072
        self.wheel_r = 0.04921
        self.base = 0.3 
        self.update_rate = 1

        # Create rate var
        self.rate = rospy.Rate(self.update_rate)

        # Create variables
        self.red_points = 0
        self.turqoise_points = 0

        # turqoise and red points list
        self.turqoise_points_list = []
        self.red_points_list = []
        self.turqoise_points_list_map = []
        self.red_points_list_map = []
        
        # Mean coord of the turqoise and red points
        self.mean_red = np.array([0,0,0])
        self.mean_turqoise = np.array([0,0,0])
        
        # Define listener
        self.listener = tf.TransformListener()
        
        # Define tf  r
        
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(1200))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        
        # Run loop
        self.run()
                
    def run(self):
        """
        Main loop
        """
        # Run loop
        rospy.loginfo('Running detection node main loop')
        while not rospy.is_shutdown():
            
            
            # Save new red points in map frame
            try:
                red_point_coord = [self.get_coord_in_map_frame(coord["time"],"red_ball") for coord in self.red_points_list]
            except:
                red_point_coord = []
                rospy.loginfo('Red points: %s', len(red_point_coord))

            self.red_points_list_map += red_point_coord
            self.red_points_list = []

            # Save new turqoise points in map frame
            try:
                turqoise_point_coord = [self.get_coord_in_map_frame(coord["time"],"turqoise_square") for coord in self.turqoise_points_list]
            except:
                turqoise_point_coord = []
                rospy.loginfo('Turqoise points: %s', len(turqoise_point_coord))
            self.turqoise_points_list_map += turqoise_point_coord
            self.turqoise_points_list = []
            
            # Only work on the last 30 values
            

            # If there are red points, calculate the mean and remove outliers
            if self.red_points_list_map:
                
                if len(self.red_points_list_map)<30:
                    red_point_coord = self.red_points_list_map
                else:
                    red_point_coord = self.red_points_list_map[-30:]
                    
                red_point_coord = np.array(red_point_coord)
                red_point_mean = np.mean(red_point_coord,axis=0)
                red_point_sigma = np.std(red_point_coord,axis=0)
                # outlier removal, one std
                red_point_coord = red_point_coord[np.abs(red_point_coord[:,0]-red_point_mean[0])<1*red_point_sigma[0]]
                red_point_coord = red_point_coord[np.abs(red_point_coord[:,1]-red_point_mean[1])<1*red_point_sigma[1]]
                red_point_coord = red_point_coord[np.abs(red_point_coord[:,2]-red_point_mean[2])<1*red_point_sigma[2]]
                red_point_mean = np.mean(red_point_coord,axis=0)
                
                rospy.loginfo('Red points: %s', red_point_mean)
                self.update_tf_frame("red_ball_mean",red_point_mean,"map")

            # If there are turqoise points, calculate the mean and remove outliers
            if self.turqoise_points_list_map:
                #rospy.loginfo('Turqoise points: %s', self.turqoise_points)

                if len(self.turqoise_points_list_map)<30:
                    turqoise_point_coord = self.turqoise_points_list_map            
                else: 
                    turqoise_point_coord = self.turqoise_points_list_map[-30:]
                    
                turqoise_point_coord = np.array(turqoise_point_coord)
                turqoise_point_mean = np.mean(turqoise_point_coord,axis=0)
                turqoise_point_sigma = np.std(turqoise_point_coord,axis=0)
                # outlier removal, one std
                turqoise_point_coord = turqoise_point_coord[np.abs(turqoise_point_coord[:,0]-turqoise_point_mean[0])<1*turqoise_point_sigma[0]]
                turqoise_point_coord = turqoise_point_coord[np.abs(turqoise_point_coord[:,1]-turqoise_point_mean[1])<1*turqoise_point_sigma[1]]
                turqoise_point_coord = turqoise_point_coord[np.abs(turqoise_point_coord[:,2]-turqoise_point_mean[2])<1*turqoise_point_sigma[2]]
                turqoise_point_mean = np.mean(turqoise_point_coord,axis=0)
                
                rospy.loginfo("Turqoise points: %s", turqoise_point_mean)
                self.update_tf_frame("turqoise_square_mean",turqoise_point_mean,"map")


            # Run at rate
            self.rate.sleep()    


    def get_coord_in_map_frame(self,time,frame_name="camera_link"):
        """
        Get coord in map frame, given time and frame
        """
        to_frame = "map"
        
        # Transform to map frame using tf2
        transform = self.tfBuffer.lookup_transform(to_frame,frame_name,time,rospy.Duration(0.005))
        
        x = transform.transform.translation.x 
        y = transform.transform.translation.y 
        z = transform.transform.translation.z 
        return x,y,z                
        
        
    def update_tf_frame(self,object_name,coord,frame_name,time=None):
        """
        Update the tf frame
        """
        if time == None:
            time = rospy.Time.now()
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = time
        t.header.frame_id = frame_name
        t.child_frame_id = object_name
        t.transform.translation.x = coord[0]
        t.transform.translation.y = coord[1]
        t.transform.translation.z = coord[2]
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)

        
    def cloud_callback(self, msg: PointCloud2):
        # Convert between ROS -> Open3D
        cloud = o3drh.rospc_to_o3dpc(msg)

        # Filter points
        legal_points = cloud.select_by_index([i for i, c in enumerate(cloud.colors) if self.is_color_legal(c) ]) # Pre-filtering
        points_turqoise = legal_points.select_by_index([i for i, c in enumerate(legal_points.colors) if self.is_color_turqoise(c) ]) # Filter turqoise
        points_red = legal_points.select_by_index([i for i, c in enumerate(legal_points.colors) if self.is_color_red(c) ])  # Filter red
        
        # Paint unique colors to the points
        points_turqoise.paint_uniform_color([0, 1, 1])
        points_red.paint_uniform_color([1, 0, 0])
        
        
        # Count the number of points
        self.red_points = len(points_red.colors) 
        self.turqoise_points = len(points_turqoise.colors)

        # If there are more than 5 points, calculate the mean coord: Turqoise
        if self.turqoise_points > 5:        
            # Get the coord of the turqoise points in relation to the robot
            turqoise_points_coord = np.asarray(points_turqoise.points)
            # Get the mean coord of the points
            self.mean_turqoise = np.mean(turqoise_points_coord, axis=0)

            new_coord = {"coord":self.mean_turqoise,"time":msg.header.stamp}
            self.turqoise_points_list.append(new_coord)

            # Publish the red and turqoise points to its topic
            out_msg_turqoise = o3drh.o3dpc_to_rospc(points_turqoise)
            out_msg_turqoise.header = msg.header
            self.pub_turqoise.publish(out_msg_turqoise)
            self.update_tf_frame("turqoise_square",self.mean_turqoise,"camera_depth_optical_frame",time = msg.header.stamp)                
            
            
        # If there are more than 5 points, calculate the mean coord: Red
        if self.red_points > 5:
            # Get the coord of the red points in relation to the robot
            red_points_coord = np.asarray(points_red.points)
            # Get the mean coord of the points
            self.mean_red = np.mean(red_points_coord, axis=0)

            new_coord = {"coord":self.mean_red,"time":msg.header.stamp}
            self.red_points_list.append(new_coord)
                

            out_msg_red = o3drh.o3dpc_to_rospc(points_red)
            out_msg_red.header = msg.header
            self.pub_red.publish(out_msg_red)
            self.update_tf_frame("red_ball",self.mean_red,"camera_depth_optical_frame",time = msg.header.stamp)

        

        
    
    def is_color_legal(self,rgb):
        """Determine if color is allowed"""
        if rgb[0] > 0.9 and rgb[1] < 0.5 and rgb[2] < 0.5:
            return True
        elif rgb[0] >= 0 and rgb[1] < 0.5 and rgb[2] < 0.45 and rgb[0] < 0.05 and rgb[1] > 0.4 and rgb[2] > 0.35:
            return True
        else:
            return False        
    def is_color_turqoise(self,rgb):
        """Determine is turqoise"""
        if rgb[0] >= 0 and rgb[1] < 0.5 and rgb[2] < 0.45 and rgb[0] < 0.05 and rgb[1] > 0.4 and rgb[2] > 0.35:
            return True
        else:
            return False

    def is_color_red(self,rgb):
        """Determine if color is red"""
        if rgb[0] > 0.9 and rgb[1] < 0.5 and rgb[2] < 0.5:
            return True
        else:
            return False

        
if __name__ == '__main__':
    new_obj = Detection()
    
    
    
