#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import  PoseArray, Pose, TransformStamped
from tf2_geometry_msgs import PoseStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
from  math import pi
import tf
from sensor_msgs.msg import Imu
import numpy as np
from nav_msgs.msg import Odometry,OccupancyGrid
from robp_msgs.msg import DutyCycles
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Bool
from collections import defaultdict
from sensor_msgs.msg import PointCloud2
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
import pcl_ros
from tf import TransformListener
from math import atan2
# import gaussian_filter1d  
from scipy.ndimage.filters import gaussian_filter1d
from scipy.signal import argrelextrema
from grid_map_pkg.grid_map import GridMap
from mapping.msg import GridMapMsg
# Mapping node

## Gridmap
### Resolution
### Contour of geofence rectange offseted outwards

## Camera
### Everything obstacles
### Map down to 2d
### Update map
### If above threshold, add to map
### Update map if below threshold

## Robot Pose
### Look up base_link in map frame (tf)


#### Dev 
# 1. Create a map grid from geofence and resolution
# 2. Subscribe to base_link and display it in map
# 3. Vizualize map in rviz





class Mapping():
    def __init__(self):
        """ Put the node name here, and description of the node"""
        rospy.init_node('mapping')

        # Subscribers 
        self.geo_fence_sub = rospy.Subscriber("/geofence/pose_array", PoseArray, self.callback_geofence)   
        self.sub_goal = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.cloud_callback)
        
        # Publisher
        self.OccupancyGrid_pub = rospy.Publisher("/occupancy_grid/walls", OccupancyGrid, queue_size=10)
        # Publish GridMapMsg 
        self.grid_map_pub = rospy.Publisher("/map/GridMap", GridMapMsg, queue_size=1)


        # Define rate
        self.update_rate = 20 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 

        # Tf 
        self.tf_buffer = tf2_ros.Buffer()
        self.br = tf2_ros.TransformBroadcaster()
        self.listner = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_listner = TransformListener()

        # Resolution
        self.resolution = 0.05 # [m]

        # time of latest cloud callback
        self.t_latest_cloud = rospy.Time.now()
        # time treshold for cloud callback
        self.t_treshold = rospy.Duration(0.5)
        
        # Robot pose in map frame [x,y,theta]
        self.robot_pose = [0,0,0]
        
        # define grid_map
        self.grid_map = GridMap(self.resolution)


    def cloud_callback(self, msg: PointCloud2):
        #rospy.loginfo("##################")

        if rospy.Time.now() - self.t_latest_cloud < self.t_treshold:
            #rospy.loginfo("cloud callback too fast")
            return
        

        t1 = rospy.Time.now().to_sec()


        # Convert ROS -> Open3D
        cloud = o3drh.rospc_to_o3dpc(msg)
        cropped = cloud.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=np.array([-100.0, -0.4, -100.0]), max_bound=np.array([100.0, 0.075, 1.5 ])))
        cropped = cropped
        
        # Downsample the point cloud to 1/10 of resolution 
        ds_cloud = cropped.voxel_down_sample(voxel_size=self.resolution/10)


        # # Convert Open3D -> NumPy
        points = np.asarray(ds_cloud.points)
        #colors = np.asarray(ds_cloud.colors)

        
        # import points in to grid map         
        if len(points) == 0:
            return
                        
        new_points = np.zeros((len(points),2))
        new_points[:,0] = points[:,2]
        new_points[:,1] = -points[:,0]
        points = new_points
        # Count number of identical points and save as dict

        self.grid_map.import_point_cloud_rays(points)
        
        return



    def callback_geofence(self, msg):
        """Save geofence coordinates in map frame 2D and find bounding box of geofence in form [x_min, x_max, y_min, y_max]"""
        self.grid_map.update_geofence_and_boundingbox(msg)



    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """
        # update robot pose in map frame, self.robvo_pose = [x,y,theta]
        try:
            map_base_link = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time()) # TransformStamped
            self.grid_map.update_robot_pose(map_base_link)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)
                

        # if geofence is give
        Occupancy_Grid = self.grid_map.get_OccupancyGrid()
        if Occupancy_Grid != None:
            self.OccupancyGrid_pub.publish(Occupancy_Grid)
            
            
        # Publish grid map
        print("Publishing grid map")
        grid_map = self.grid_map.get_GridMapMsg()
        if grid_map != None:
            self.grid_map_pub.publish(self.grid_map.get_GridMapMsg()) 
        

    def run(self):
        """
        Run the node. 
        Don't change anything here, change main instead.
        """
        
        # Run as long as node is not shutdown
        while not rospy.is_shutdown():
            self.main()
            self.rate.sleep()


if __name__ == "__main__":

    node=Mapping()
    node.run()
