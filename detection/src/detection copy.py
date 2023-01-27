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
        threshold_red = 100
        threshold_turqoise = 100
        rospy.loginfo('Running detection node main loop')
        while not rospy.is_shutdown():
            
            try:
                red_point_coord = [self.get_coord_in_map_frame(coord["time"],"red_ball") for coord in self.red_points_list]
            except:
                red_point_coord = []
                rospy.loginfo('Red points: %s', len(red_point_coord))

            self.red_points_list_map += red_point_coord
            self.red_points_list = []

            #turqoise_point_coord = [self.get_coord_in_map_frame(coord) for coord in self.turqoise_points_list]

            try:
                turqoise_point_coord = [self.get_coord_in_map_frame(coord["time"],"turqoise_square") for coord in self.turqoise_points_list]
            except:
                turqoise_point_coord = []
                rospy.loginfo('Turqoise points: %s', len(turqoise_point_coord))
            self.turqoise_points_list_map += turqoise_point_coord
            self.turqoise_points_list = []

            if self.red_points_list_map:
                
                red_point_coord = self.red_points_list_map
                red_point_coord = np.array(red_point_coord)
                red_point_mean = np.mean(red_point_coord,axis=0)
                red_point_sigma = np.std(red_point_coord,axis=0)
                # outlier removal 95% of the points
                red_point_coord = red_point_coord[np.abs(red_point_coord[:,0]-red_point_mean[0])<2*red_point_sigma[0]]
                red_point_coord = red_point_coord[np.abs(red_point_coord[:,1]-red_point_mean[1])<2*red_point_sigma[1]]
                red_point_coord = red_point_coord[np.abs(red_point_coord[:,2]-red_point_mean[2])<2*red_point_sigma[2]]
                red_point_mean = np.mean(red_point_coord,axis=0)
                
                self.update_tf_frame("red_ball_mean",red_point_mean,"map")

            if self.turqoise_points_list_map:
                #rospy.loginfo('Turqoise points: %s', self.turqoise_points)

                turqoise_point_coord = self.turqoise_points_list_map                
                turqoise_point_coord = np.array(turqoise_point_coord)
                turqoise_point_mean = np.mean(turqoise_point_coord,axis=0)
                turqoise_point_sigma = np.std(turqoise_point_coord,axis=0)
                # outlier removal 95% of the points
                turqoise_point_coord = turqoise_point_coord[np.abs(turqoise_point_coord[:,0]-turqoise_point_mean[0])<2*turqoise_point_sigma[0]]
                turqoise_point_coord = turqoise_point_coord[np.abs(turqoise_point_coord[:,1]-turqoise_point_mean[1])<2*turqoise_point_sigma[1]]
                turqoise_point_coord = turqoise_point_coord[np.abs(turqoise_point_coord[:,2]-turqoise_point_mean[2])<2*turqoise_point_sigma[2]]
                turqoise_point_mean = np.mean(turqoise_point_coord,axis=0)
                
                self.update_tf_frame("turqoise_square_mean",turqoise_point_mean,"map")



            self.rate.sleep()    


    def get_coord_in_map_frame(self,time,frame_name="camera_link"):
        """
        Get coord in map frame, given time and frame
        """
        to_frame = "map"
        
        # Transform to map frame using tf2
        transform = self.tfBuffer.lookup_transform(to_frame,frame_name,time,rospy.Duration(0.005))
        
        x = transform.transform.translation.x #+ coord["coord"][2]
        y = transform.transform.translation.y #- coord["coord"][1]
        z = transform.transform.translation.z #- coord["coord"][0]
        return x,y,z                
        
        # Define pose stamp and transform to map
        ps = PoseStamped()
        ps.header.stamp = coord["time"]
        ps.header.frame_id = frame_name
        ps.pose.position.x =  coord["coord"][2]
        ps.pose.position.y = -coord["coord"][1]
        ps.pose.position.z = -coord["coord"][0]
        q = tf_conversions.transformations.quaternion_from_euler(0,0,0)
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]

        self.listener.waitForTransform(frame_name, to_frame,coord["time"],rospy.Duration(0.05) )#rospy.Time(), rospy.Duration(4.0))
        ps_map = self.listener.transformPose("map",ps)
        return ps_map.pose.position.x,ps_map.pose.position.y,ps_map.pose.position.z
    
        
        

        
        
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
    
        #legal_points = cloud.select_by_index([i for i, c in enumerate(cloud.colors) if self.is_color_legal(c) ]) #c[0] > 0.8 ])
        points_turqoise = cloud.select_by_index([i for i, c in enumerate(cloud.colors) if self.is_color_turqoise(c) ]) #c[0] > 0.8 ])
        points_red = cloud.select_by_index([i for i, c in enumerate(cloud.colors) if self.is_color_red(c) ]) #c[0] > 0.8 ])
        
        # Set one color to all points
        #points_red.paint_uniform_color([1, 0, 0])
        #points_turqoise.paint_uniform_color([0, 1, 1])
        
        
        self.red_points = len(points_red.colors)#sum([1 for i, c in enumerate(points_red.colors)  ]) 
        self.turqoise_points = len(points_turqoise.colors)#sum([1 for i, c in enumerate(points_turqoise.colors)  ])

        if self.turqoise_points > 10:        
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
            self.update_tf_frame("turqoise_square",[self.mean_turqoise[2],-self.mean_turqoise[1],-self.mean_turqoise[0]],"camera_link",time = msg.header.stamp)                
            

        if self.red_points > 10:
            # Get the coord of the red points in relation to the robot
            red_points_coord = np.asarray(points_red.points)
            # Get the mean coord of the points
            self.mean_red = np.mean(red_points_coord, axis=0)

            # Append to the lists
            new_coord = {"coord":self.mean_red,"time":msg.header.stamp}
            self.red_points_list.append(new_coord)

            out_msg_red = o3drh.o3dpc_to_rospc(points_red)
            out_msg_red.header = msg.header
            self.pub_red.publish(out_msg_red)
            self.update_tf_frame("red_ball",[self.mean_red[2],-self.mean_red[1],-self.mean_red[0]],"camera_link",time = msg.header.stamp)
        

        

        
    
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
    
    def rgb_to_hsv(self,rgb):
        """
        input rgb array of shape (3,) with values in [0,1]
        return hsv array of shape (3,) with values in [0,1]
        """
        r,g,b = rgb
        h,s,v = colorsys.rgb_to_hsv(r,g,b)
        return np.array([h,s,v])

    def det_red_or_not(self,rgb):
        """
        input rgb array of shape (3,) with values in [0,1]
        return True if red, False if not red
        """
        hsv = self.rgb_to_hsv(rgb)
        h,s,v = hsv
        if h > 0.98 or h < 0.02:
            return True
        else:
            return False
        
        
if __name__ == '__main__':
    new_obj = Detection()
    
    
    
