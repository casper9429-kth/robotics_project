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
import json
from mapping.msg import GridMapMsg
from mapping.msg import Array_float
from std_msgs.msg import Float32
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


class GridMap():
    def __init__(self,resolution=0.05):
        self.resolution = resolution
        
        # map grid (tuples)
        self.map_grid = None#defaultdict(lambda: -1)
        self.contour_mask = None
        # values
        self.occupied = 1
        self.unkown = -1
        self.free = 0
        self.wall = 2
        
        # Geofence coord 
        self.given_geofence = False
        self.geofence_list = []

        
        # Bounding box
        self.bounding_box = [0,0,0,0]
        
        # Robot pose
        self.robot_pose_time = 0
        self.robot_pose = [0,0,0]
        
        # 

    
    def get_GridMapMsg(self):
        """
        Get GridMapMsg
        Containing:
        * header
        * 2d array of gridmap
        """
        # Create GridMapMsg
        grid_map_msg = GridMapMsg()
        
        # Fill header
        if self.robot_pose_time == 0:
            rospy.loginfo("No robot pose recieved yet")
            return None
        
        if self.given_geofence == False:
            rospy.loginfo("No geofence recieved yet")
            return None
        
        grid_map_msg.header.stamp = self.robot_pose_time
        grid_map_msg.header.frame_id = "map"


        # Get raw numpy array of map
        map_data = self.get_grid_map_array()
        # Convert to list
        map_data = map_data.tolist()

        # For each row, convert to Array
        map_data_array = []
        for i,row in enumerate(map_data):
            array = Array_float()
            array.data = row
            map_data_array.append(array)

        
        # Fill grid map
        grid_map_msg.data = map_data_array

        # Fill bounding box
        grid_map_msg.resolution = self.resolution
        grid_map_msg.bbminx = self.bounding_box[0]
        grid_map_msg.bbmaxx = self.bounding_box[1]
        grid_map_msg.bbminy = self.bounding_box[2]
        grid_map_msg.bbmaxy = self.bounding_box[3]
        # Get index of origo
        i = int((self.bounding_box[0])/self.resolution)
        j = int((self.bounding_box[2])/self.resolution)
        grid_map_msg.origo_index_i = i
        grid_map_msg.origo_index_j = j 

        # Return grid map msg
        return grid_map_msg
    



    def update_geofence_and_boundingbox(self,msg):
        """Update geofence coordinates and bounding box, takes pose array message as input, aslo sets given_geofence to true, if geofence is given will remove old geofence from map"""
        
        if self.given_geofence == True:
            rospy.loginfo("Geofence already given")
            return
        # save new geofence
        self.geofence_list = [[pose.position.x, pose.position.y] for pose in msg.poses] 



        # Find bounding box of geofence 
        self.bounding_box = [self.geofence_list[0][0], self.geofence_list[0][0], self.geofence_list[0][1], self.geofence_list[0][1]]


        for x,y in self.geofence_list:
            if x < self.bounding_box[0]:
                self.bounding_box[0] = x
            if x > self.bounding_box[1]:
                self.bounding_box[1] = x
            if y < self.bounding_box[2]:
                self.bounding_box[2] = y
            if y > self.bounding_box[3]:
                self.bounding_box[3] = y

        # Expand bounding box by resolution on all sides
        self.bounding_box[0] -= self.resolution*30
        self.bounding_box[1] += self.resolution*30
        self.bounding_box[2] -= self.resolution*30
        self.bounding_box[3] += self.resolution*30


    

        # Create numpy array of bounding box
        self.map_grid = np.ones([int(((self.bounding_box[1]-self.bounding_box[0])/self.resolution)),int(((self.bounding_box[3]-self.bounding_box[2])/self.resolution))])*self.unkown
        self.map_grid = self.map_grid.astype(int)
        print("#######################")
        print("#######################")
        print("#######################")
        print(self.map_grid.shape)
        print("#######################")
        print("#######################")
        print("#######################")
        
                
        # Change all cordiantes to grid coordinates if they are on our outside of the geofence polygon
        # arange but exclude last value
        for i in range(self.map_grid.shape[0]):
            for j in range(self.map_grid.shape[1]):
                x = self.resolution*i + self.bounding_box[0]
                y = self.resolution*j + self.bounding_box[2]
                if not self.is_point_in_polygon(x,y,self.geofence_list):
                    self.map_grid[i,j] = self.occupied

        self.contour_mask = self.map_grid.copy()


        # Set given geofence to true
        self.given_geofence = True


    def is_point_in_polygon(self,x,y,poly):
        """Check if point is in polygon"""
        n = len(poly)
        inside =False

        p1x,p1y = poly[0]
        for i in range(n+1):
            p2x,p2y = poly[i % n]
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x,p1y = p2x,p2y

        return inside
    
    
    def update_robot_pose(self,msg):
        """Update robot pose, takes transformed stamped message as input"""
        self.robot_pose_time = msg.header.stamp
        self.robot_pose_new = [msg.transform.translation.x, msg.transform.translation.y, tf.transformations.euler_from_quaternion([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])[2]]

        self.robot_pose = self.robot_pose_new
        
    
    
    
    def get_grid_map_raw(self):
        """Return map grid, if not given geofence, return None"""
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None

        
        return self.map_grid
    
    def get_grid_map_array(self):
        """Return map grid as array, if not given geofence, return None"""
        # Retrun map grid as array if not given geofence, return None
        if self.given_geofence == False:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        #map_array = np.zeros([int((self.bounding_box[1]-self.bounding_box[0])/self.resolution+1),int((self.bounding_box[3]-self.bounding_box[2])/self.resolution+1)])
        #for i,x in enumerate(np.arange(self.bounding_box[0], self.bounding_box[1], self.resolution)):
        #    for j,y in enumerate(np.arange(self.bounding_box[2], self.bounding_box[3], self.resolution)):
        #        map_array[i,j] = self.map_grid[i,j]
                
        return self.map_grid
    
    def get_index_of_pos(self,x,y):
        """Return index of position in map grid, if not given geofence, return None"""
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        x_index = int((x-self.bounding_box[0])/self.resolution)
        y_index = int((y-self.bounding_box[2])/self.resolution)
        
        return x_index, y_index
    
        
        
    def get_pos_of_index(self,i,j):
        """Return position of index in map grid, if not given geofence, return None"""
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        x = self.bounding_box[0] + i*self.resolution
        y = self.bounding_box[2] + j*self.resolution
        
        return x, y

    def get_value_of_index(self,i,j):
        """Return value of index in map grid, if not given geofence, return None"""
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        # if out of bounds, return 1
        if i < 0 or i > int((self.bounding_box[1]-self.bounding_box[0])/self.resolution) or j < 0 or j > int((self.bounding_box[3]-self.bounding_box[2])/self.resolution):
            return 1
        
        return self.map_grid[i,j]
    
    def get_value_of_pos(self,x,y):
        """Return value of position in map grid, if not given geofence, return None"""
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        i,j = self.get_index_of_pos(x,y)
        
        return self.get_value_of_index(i,j)

    def set_value_of_index(self,i,j,value):
        """Set value of index in map grid, if not given geofence, return None"""
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        # check for tuple in geo_fence_index_list numpy array
        if self.map_grid[i,j] != 2:
            self.map_grid[i,j] = value

        
    
    def set_value_of_pos(self,x,y,value):
        """Set value of position in map grid, if not given geofence, return None"""
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        i,j = self.get_index_of_pos(x,y)
        
        self.set_value_of_index(i,j,value)

    def set_value_of_pose_array(self,pose_array,count,threshold=0):
        for c,pose in zip(count,pose_array):
            if c > threshold:
                self.set_value_of_pos(pose[0],pose[1],1)            
            
    
    def import_point_cloud_rays(self,pointcloud,range_max = 1.5,x=None,y=None,theta=None):
        """
        Import point cloud and set values in map grid
        pointcloud should be numpy Nx2 array
        will assume frame of latest given robot pose        
        """

        # if no geofence given, return
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        # get robot pose in map frame
        if x == None or y == None or theta == None:
            x = self.robot_pose[0]
            y = self.robot_pose[1]
            theta = self.robot_pose[2]
        
        # add x and y to pointcloud
        pointcloud[:,0] = pointcloud[:,0] 
        pointcloud[:,1] = pointcloud[:,1] 
        
                
        # calc a range and angle for each point in pointcloud
        ranges = np.sqrt(np.sum(pointcloud**2,axis=1))
        
        # Discritize ranges
        #ranges = np.floor(ranges/(10*self.resolution))*self.resolution*10
        ranges[ranges > range_max] = range_max
        #ranges = np.floor(ranges/self.resolution)*self.resolution
        angle = np.arctan2(pointcloud[:,1],pointcloud[:,0])

        # map angle from -pi to pi
        #angle = np.mod(angle+np.pi,2*np.pi)-np.pi        

        
        if len(angle) == 0:
            return



        # Create array of possible angles
        arc_length = range_max*1
        steps_in_ang = int(arc_length/self.resolution) # might need to be bigger
        d_ang = 1
        ang_res = d_ang/steps_in_ang # d_ang = 1        
        # Create angle array of possible angle indexes
        ang_ind_list = (np.floor((angle + 0.5)/ang_res)).astype(int)
        ang_ind_list[ang_ind_list < 0] = 0
        ang_ind_list[ang_ind_list > steps_in_ang-1] = steps_in_ang-1
        
        # Create rays
        rays_ind = np.stack((ang_ind_list,ranges),axis=1)
        
        # Filter unique rays
        rays, counts = np.unique(rays_ind,axis=0,return_counts=True)
        rays = rays[counts > 1]
        index = rays[:,0].astype(int)
        ranges = rays[:,1]
        # Ray array
        rays = []
        for i in range(steps_in_ang):
            rang = ranges[index==i]
            if len(rang) > 0:
                range_min = np.min(rang)
            else:
                range_min = 0
            ang = -0.5 + i*ang_res
            rays.append([ang,range_min]) 
            
            
        
        # make rays np array
        rays = np.array(rays)
        
        # offset rays by robot theta
        rays[:,0] = rays[:,0] + theta
        

        # Make all points inbetween robot and ray 0, make end of ray 1
        map_grid = self.map_grid.copy() # maybe need to add a copy here
        for ray in rays:
            # make end point of ray 1 
            ang = ray[0]
            if ray[1] == 0:
                ray_end = range_max
            else:
                ray_end = ray[1]
            new_x = x + ray_end*np.cos(ang)
            new_y = y + ray_end*np.sin(ang)
            #self.set_value_of_pos(new_x,new_y,1)
            
            # make all points inbetween 0
            # get points inbetween
            dx = new_x - x
            dy = new_y - y
            
            # get number of points
            n = int(2*(np.sqrt(dx**2 + dy**2)/self.resolution))
            
            # get points
            xs = np.linspace(x,new_x,n)
            ys = np.linspace(y,new_y,n)
            
            xs = ((xs-self.bounding_box[0])/self.resolution).astype(int)
            ys = ((ys-self.bounding_box[2])/self.resolution).astype(int)
            map_grid[(xs[:],ys[:])] = self.free
                
                
                
        for ray in rays:
            # make end point of ray 1 
            if ray[1] == 0:
                continue
            ray_end = ray
            new_x = x + ray_end[1]*np.cos(ray_end[0])
            new_y = y + ray_end[1]*np.sin(ray_end[0])
            new_x = int((new_x-self.bounding_box[0])/self.resolution)
            new_y = int((new_y-self.bounding_box[2])/self.resolution)
            map_grid[new_x,new_y] = self.occupied

        # Apply mask of geofence to make sure no points outside of geofence are 0
        map_grid[self.contour_mask==self.occupied] = self.occupied
        self.map_grid = map_grid
            

        

        return            
    

    def get_OccupancyGrid(self):
        """
        Return OccupancyGrid message of map grid, if not given geofence, return None
        """
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = self.map_grid.shape[0]#int(((self.bounding_box[1]-self.bounding_box[0])/self.resolution) +1)
        occupancy_grid.info.height = self.map_grid.shape[1]#int(((self.bounding_box[3]-self.bounding_box[2])/self.resolution) +1)
        occupancy_grid.info.origin.position.x = self.bounding_box[0]
        occupancy_grid.info.origin.position.y = self.bounding_box[2]
        occupancy_grid.info.origin.position.z = 0
        occupancy_grid.info.origin.orientation.x = 0
        occupancy_grid.info.origin.orientation.y = 0
        occupancy_grid.info.origin.orientation.z = 0
        occupancy_grid.info.origin.orientation.w = 1
    


        
        occupancy_grid.data = []
        #for j in range(occupancy_grid.info.height):
        #    for i in range(occupancy_grid.info.width):
        #        occupancy_grid.data.append(self.get_value_of_index(i,j))
    
        occupancy_grid.data = self.map_grid.flatten('F').tolist()
        return occupancy_grid
        

