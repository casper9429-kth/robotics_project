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
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

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
    def __init__(self,resolution):
        self.resolution = resolution
        
        # map grid (tuples)
        self.map_grid = defaultdict(lambda: -1)
        
        # Geofence coord 
        self.given_geofence = False
        self.geofence_list = []
        
        # Bounding box
        self.bounding_box = [0,0,0,0]
        
        # Robot pose
        self.robot_pose_time = 0
        self.robot_pose = [0,0,0]
        
        # 

    def update_geofence_and_boundingbox(self,msg):
        """Update geofence coordinates and bounding box, takes pose array message as input, aslo sets given_geofence to true, if geofence is given will remove old geofence from map"""
        
        # save new geofence
        self.geofence_list_new = [[pose.position.x, pose.position.y] for pose in msg.poses] 
    
        # if old geofence == new geofence, do nothing
        if self.geofence_list == self.geofence_list_new:
            return

        # if old geofence is given, remove it from the map
        if self.given_geofence:
            for i,x in enumerate(np.arange(self.bounding_box[0], self.bounding_box[1], self.resolution)):
                for j,y in enumerate(np.arange(self.bounding_box[2], self.bounding_box[3], self.resolution)):
                    if not self.is_point_in_polygon(x,y,self.geofence_list):
                        self.map_grid[(i,j)] = -1

        # save new geofence
        self.geofence_list = self.geofence_list_new 

        # Find bounding box of geofence 
        if len(self.geofence_list) > 0:
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
        

        # Change all cordiantes to grid coordinates if they are on our outside of the geofence polygon
        for i,x in enumerate(np.arange(self.bounding_box[0], self.bounding_box[1], self.resolution)):
            for j,y in enumerate(np.arange(self.bounding_box[2], self.bounding_box[3], self.resolution)):
                if not self.is_point_in_polygon(x,y,self.geofence_list):
                    self.map_grid[(i,j)] = 1

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

        # add robot pose to map
        if self.robot_pose != self.robot_pose_new:
            self.set_value_of_pos(self.robot_pose[0],self.robot_pose[1],-1)
            self.robot_pose = self.robot_pose_new
            self.set_value_of_pos(self.robot_pose[0],self.robot_pose[1],1)
        
    
    
    
    def get_grid_map_raw(self):
        """Return map grid, if not given geofence, return None"""
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None

        return self.map_grid
    
    def get_grid_map_array(self):
        """Return map grid as array, if not given geofence, return None"""
        # Retrun map grid as array if not given geofence, return None
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        map_array = np.zeros([int((self.bounding_box[1]-self.bounding_box[0])/self.resolution),int((self.bounding_box[3]-self.bounding_box[2])/self.resolution)])
        for i,x in enumerate(np.arange(self.bounding_box[0], self.bounding_box[1], self.resolution)):
            for j,y in enumerate(np.arange(self.bounding_box[2], self.bounding_box[3], self.resolution)):
                map_array[i,j] = self.map_grid[(i,j)]
                
        return map_array
    
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
        
        return self.map_grid[(i,j)]
    
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
        
        self.map_grid[(i,j)] = value
    
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
            
    
    def import_point_cloud(self,pointcloud):
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
        x = self.robot_pose[0]
        y = self.robot_pose[1]
        theta = self.robot_pose[2]
        
        # add x and y to pointcloud
        pointcloud[:,0] = pointcloud[:,0] 
        pointcloud[:,1] = pointcloud[:,1] 
        
        
        # calc a range and angle for each point in pointcloud
        ranges = np.sqrt(np.sum(pointcloud**2,axis=1))
        angle = np.arctan2(pointcloud[:,1],pointcloud[:,0])
        # shift angle to be relative to robot
        angle = angle 

        # Get min and max
        minangle = np.min(angle)
        maxangle = np.max(angle)
        minrange = np.min(ranges)
        maxrange = np.max(ranges)


        # create discretized rays
        resolution_ang = 80
        angle = ((angle-minangle)/resolution_ang).astype(int)
        resolution_rang = self.resolution
        ranges = ((ranges-minrange)/resolution_rang).astype(int)
        

        # create rays
        rays = np.zeros(len(ranges))
        # fill rays with angle and range as each element
        rays = np.stack((angle,ranges),axis=1)
        # 

        # get unique ray indices
        # flatten rays
        rays, counts = np.unique(rays,axis=0,return_counts=True)
        min_ang_index = np.min(rays[:,0])
        max_ang_index = np.max(rays[:,0])
        min_range_index = np.min(rays[:,1])
        max_range_index = np.max(rays[:,1])
        
        
        # create rays array
        d_ang_index = (max_ang_index-min_ang_index+1).astype(int)
        d_range_index = (max_range_index-min_range_index+1).astype(int)
        rays_array = np.zeros((d_ang_index,d_range_index))
        rays_array[(rays[:,0],rays[:,1])] = counts[:]

        
        rays = []
        # iter over each col in rays array, make everyth
        for i in range(d_ang_index):            
            # find first max index in col
            theta = i*resolution_ang+minangle
            index = np.argmax(rays_array[i,:])
            r = index*resolution_rang+minrange
            rays.append([r,theta])

        # rays to numpy array
        rays = np.array(rays)
        
        # transform to robots point of view
        rays = np.stack((rays[:,0]*np.cos(rays[:,1]+theta),rays[:,0]*np.sin(rays[:,1]+theta)),axis=1)
        
        # add to map grid
        for ray in rays:
            self.set_value_of_pos(ray[0]+x,ray[1]+y,1) 
        
        # add ray to map grid, if within
        
                
        
        return
        # transform angle and range to index
        angle = ((angle-minangle)).astype(int)
        ranges = ((ranges-minrange)).astype(int)

        rays = np.zeros((delta_angle,2))
        rays[:,0] = ranges
        rays[:,1] = angle

        # count unique rays
        rays, counts = np.unique(rays,axis=0,return_counts=True)
        
        # create rays array
        ray

        # fill rays array with range and angle
        
        
        
        # fill rays array with range and angle
        ra
        # tran

        # 
        rays[:,0] = ranges
        rays[:,1] = angle
        
        
        





        # cluster points into rays with resolution of 1 degree
        # calc x,y of each point in pointcloud in map frame 
        x = x + ranges*np.cos(angle+theta)
        y = y + ranges*np.sin(angle+theta) 

        # discretize x,y to map grid 
        x_index = np.floor((x-self.bounding_box[0])/self.resolution).astype(int)
        y_index = np.floor((y-self.bounding_box[2])/self.resolution).astype(int)
        
        # set values in map grid
        self.map_grid[(x_index,y_index)] = 1
        
        # get index of robot pose in map grid
        robot_x_index = int((self.robot_pose[0]-self.bounding_box[0])/self.resolution)
        robot_y_index = int((self.robot_pose[1]-self.bounding_box[2])/self.resolution)
        
        # set robot position in map grid to 0
        self.map_grid[(robot_x_index,robot_y_index)] = 0
        
        # return map grid
        return self.map_grid
        

        # calc x,y of each point in pointcloud in map frame 
        x = x + ranges*np.cos(angle+theta)
        y = y + ranges*np.sin(angle+theta) 

        # 


        # discretize x,y to map grid 
        x_index = np.floor((x-self.bounding_box[0])/self.resolution).astype(int)
        y_index = np.floor((y-self.bounding_box[2])/self.resolution).astype(int)
        



        # get index of robot pose in map grid
        robot_x_index = int((self.robot_pose[0]-self.bounding_box[0])/self.resolution)
        robot_y_index = int((self.robot_pose[1]-self.bounding_box[2])/self.resolution)
        
        # S
    

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
        occupancy_grid.info.width = int((self.bounding_box[1]-self.bounding_box[0])/self.resolution)
        occupancy_grid.info.height = int((self.bounding_box[3]-self.bounding_box[2])/self.resolution)
        occupancy_grid.info.origin.position.x = self.bounding_box[0]
        occupancy_grid.info.origin.position.y = self.bounding_box[2]
        occupancy_grid.info.origin.position.z = 0
        occupancy_grid.info.origin.orientation.x = 0
        occupancy_grid.info.origin.orientation.y = 0
        occupancy_grid.info.origin.orientation.z = 0
        occupancy_grid.info.origin.orientation.w = 1
    


        
        occupancy_grid.data = []
        
        for j in range(occupancy_grid.info.height):
            for i in range(occupancy_grid.info.width):

                occupancy_grid.data.append(self.get_value_of_index(i,j))
        
        return occupancy_grid
        



class Mapping():
    def __init__(self):
        """ Put the node name here, and description of the node"""
        rospy.init_node('mapping')

        # Subscribers 
        self.geo_fence_sub = rospy.Subscriber("/geofence/pose_array", PoseArray, self.callback_geofence)   
        self.sub_goal = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.cloud_callback)
        
        # Publisher
        self.OccupancyGrid_pub = rospy.Publisher("/occupancy_grid/walls", OccupancyGrid, queue_size=10)

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
        rospy.loginfo("##################")

        if rospy.Time.now() - self.t_latest_cloud < self.t_treshold:
            rospy.loginfo("cloud callback too fast")
            return
        


        t1 = rospy.Time.now().to_sec()


        # Convert ROS -> Open3D
        cloud = o3drh.rospc_to_o3dpc(msg)
        # cropped = cloud.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=np.array([-100.0, -100.0, 0.0]), max_bound=np.array([100.0, 100.0, 0.9 ])))
        cropped = cloud
        
        # Downsample the point cloud to 1/10 of resolution 
        ds_cloud = cropped.voxel_down_sample(voxel_size=self.resolution/5)


        rospy.loginfo("downsmaple done %s",  rospy.Time.now().to_sec() - t1)


        # # Convert Open3D -> NumPy
        points = np.asarray(ds_cloud.points)
        #colors = np.asarray(ds_cloud.colors)

        
        # import points in to grid map 
                
        if len(points) == 0:
            return
        
        # Get bbox
        #minx = np.min(points[:,0])
        #maxx = np.max(points[:,0])
        #miny = np.min(points[:,1])
        #maxy = np.max(points[:,1])
        #bbox = [minx, maxx, miny, maxy]    
                
        points = points[:,:2]
        # Count number of identical points and save as dict

        # unique, counts = np.unique(points, axis=0, return_counts=True)

        # Transform into points 
        # unique_points = np.zeros((len(unique),2))
        # unique_points[:,0] = unique[:,0]*self.resolution + minx
        # unique_points[:,1] = unique[:,1]*self.resolution + miny

        # rospy.loginfo("math done %s",  rospy.Time.now().to_sec() - t1)

        self.grid_map.import_point_cloud(points)
        
        rospy.loginfo("Gridmap updated %s", rospy.Time.now().to_sec() - t1)        
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
