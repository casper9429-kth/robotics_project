#!/usr/bin/env python3
import heapq
import rospy
from functools import total_ordering
import numpy as np
import math
import actionlib
import move_base_msgs.msg as mb
import sys
from collections import defaultdict
from dataclasses import dataclass,field
from occupancy_grid import Occupancy_grid

from numba import jit
from typing import Dict,Tuple
from scipy.interpolate import CubicSpline
from queue import PriorityQueue
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from mapping.msg import GridMapMsg
#from mapping.grid_map.grid_map import GridMap


@total_ordering
@dataclass
class Node:
    x: float
    y: float
    goal: tuple
    g: float = field(init=False)
    h: float = field(init=False)
    f: float = field(init=False)
    parent: object = field(default=None)
    #children: list = field(default_factory=list, init=False)

    # Makes sure that the stuff to be initalized is. 
    # Needed since dataclasses cant handle initalizing functions
    def __post_init__(self):
        if self.parent == None:
            self.g = 0
        else:
            par_pos =[pos for pos in self.parent.position()]
            self.g = self.parent.g + math.dist([self.x,self.y],par_pos)
        self.h = self.dist_to_goal(self.goal)
        self.f = self.g + self.h

    def __lt__(self,other):
        return self.f < other.f
    """def __eq__(self, other):
        return self.f == other.f"""

    def dist_to_goal(self, goal):
        return math.dist((self.x, self.y), goal)
    
    def set_g(self):
        self.g = self.parent.f
    
    def position(self):
        return (self.x,self.y)
    
    def as_array(self):
        return np.array([self.x, self.y, self.goal[0], self.goal[1], self.g, self.h, self.f])




class GridMap():
    def __init__(self,resolution):
        self.resolution = resolution
        
        # map grid (tuples)
        self.map_grid = defaultdict(lambda: -1)
        
        # values
        self.occupied = 1
        self.unkown = -1
        self.free = 0
        self.wall = 2
        
        # Geofence coord 
        self.given_geofence = False
        self.geofence_list = []
        self.geo_fence_index_dict = defaultdict(lambda: 0)

        
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
                        self.map_grid[(i,j)] = self.unkown

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
                    self.map_grid[(i,j)] = self.wall


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
        
        # check for tuple in geo_fence_index_list numpy array
        if self.map_grid[(i,j)] != 2:
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
        will assume frame of latest given robot pose,
        
        Buggs:
        * if robot doesn't see a wall, there is no rays and the empty space will not be changed from unkown to free        
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

        # round to resolution
        resolution_ang = np.pi/100
        angle = np.floor(angle/resolution_ang)*resolution_ang
        resolution_rang = self.resolution/10
        ranges = np.floor(ranges/resolution_rang)*resolution_rang
        
        # map angle from -pi to pi
        angle = np.mod(angle+np.pi,2*np.pi)-np.pi        


        # Create rays
        rays = np.stack((angle,ranges),axis=1)

        # Filter unique rays
        rays, counts = np.unique(rays,axis=0,return_counts=True)



        angle_range_dict = defaultdict(list)
        for ray,count in zip(rays,counts):
            ray = [ray[0],ray[1],count]
            angle_range_dict[ray[0]].append(ray)


        rays = [] 
        count_threshold = 0
        for key in angle_range_dict.keys():
            ray_list = angle_range_dict[key]
            # find first highest count ray in list
            # find max valu
            ray_list = np.array(ray_list)

            # find shortest ray with count > count_threshold
            ray_list = ray_list[ray_list[:,2] > count_threshold]
            ray_list = ray_list[ray_list[:,1].argsort()]
            ray = ray_list[0]            
            
            

            #index = np.argmax(ray_list[:,2])
            #ray = ray_list[index]            
            if ray[2] > count_threshold:
                rays.append(ray[0:2])

        # make rays np array
        rays = np.array(rays)
        

        # offset rays by robot theta
        rays[:,0] = rays[:,0] + theta
        
        # default dict to store points 
        points_to_add = defaultdict()


        # Make all points inbetween robot and ray 0, make end of ray 1
        for ray in rays:
            # make end point of ray 1 
            ray_end = ray
            new_x = x + ray_end[1]*np.cos(ray_end[0])
            new_y = y + ray_end[1]*np.sin(ray_end[0])
            #self.set_value_of_pos(new_x,new_y,1)
            
            # make all points inbetween 0
            # get points inbetween
            dx = new_x - x
            dy = new_y - y
            
            # get number of points
            n = int(2*(np.sqrt(dx**2 + dy**2)/self.resolution))
            
            # get points
            xs = np.linspace(x,new_x,n)[0:-1]
            ys = np.linspace(y,new_y,n)[0:-1]
        
            
            # set values of points inbetween
            for i in range(len(xs)-1):
                x1 = xs[i]
                x2 = xs[i+1]
                y1 = ys[i]
                y2 = ys[i+1]               

                p1 = self.get_index_of_pos(x1,y1)
                p2 = self.get_index_of_pos(x2,y2)
                p3 = self.get_index_of_pos(x1,y2)
                p4 = self.get_index_of_pos(x2,y1)
                points_to_add[p1] = self.free
                points_to_add[p2] = self.free
                points_to_add[p3] = self.free
                points_to_add[p4] = self.free
                
                
                
        for ray in rays:
            # make end point of ray 1 
            ray_end = ray
            new_x = x + ray_end[1]*np.cos(ray_end[0])
            new_y = y + ray_end[1]*np.sin(ray_end[0])
            p1 = self.get_index_of_pos(new_x,new_y)
            points_to_add[p1] = self.occupied
            #self.set_value_of_pos(new_x,new_y,1)
            

        for p in points_to_add.keys():
            self.set_value_of_index(p[0],p[1],points_to_add[p])
            #self.set_value_of_pos(p[0],p[1],points_to_add[p])        

        return        

    def import_point_cloud_rays(self,pointcloud):
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

        # round to resolution
        resolution_ang = np.pi/100
        angle = np.floor(angle/resolution_ang)*resolution_ang
        resolution_rang = self.resolution/10
        ranges = np.floor(ranges/resolution_rang)*resolution_rang
        
        # map angle from -pi to pi
        angle = np.mod(angle+np.pi,2*np.pi)-np.pi        


        # Create rays
        rays = np.stack((angle,ranges),axis=1)

        # Filter unique rays
        rays, counts = np.unique(rays,axis=0,return_counts=True)



        angle_range_dict = defaultdict(list)
        for ray,count in zip(rays,counts):
            ray = [ray[0],ray[1],count]
            angle_range_dict[ray[0]].append(ray)


        rays = [] 
        count_threshold = 0
        for key in angle_range_dict.keys():
            ray_list = angle_range_dict[key]
            # find first highest count ray in list
            # find max valget_value_of_posu
            ray_list = np.array(ray_list)

            # find shortest ray with count > count_threshold
            ray_list = ray_list[ray_list[:,2] > count_threshold]
            ray_list = ray_list[ray_list[:,1].argsort()]
            ray = ray_list[0]            
            
            

            #index = np.argmax(ray_list[:,2])
            #ray = ray_list[index]            
            if ray[2] > count_threshold:
                rays.append(ray[0:2])

        # make rays np array
        rays = np.array(rays)
        
        # min theta and max theta
        print("min theta: ",np.min(rays[:,0]))
        print("max theta: ",np.max(rays[:,0]))
        

        # offset rays by robot theta
        rays[:,0] = rays[:,0] + theta
        
        # default dict to store points 
        points_to_add = defaultdict()


        # Make all points inbetween robot and ray 0, make end of ray 1
        for ray in rays:
            # make end point of ray 1 
            ray_end = ray
            new_x = x + ray_end[1]*np.cos(ray_end[0])
            new_y = y + ray_end[1]*np.sin(ray_end[0])
            #self.set_value_of_pos(new_x,new_y,1)
            
            # make all points inbetween 0
            # get points inbetween
            dx = new_x - x
            dy = new_y - y
            
            # get number of points
            n = int(2*(np.sqrt(dx**2 + dy**2)/self.resolution))
            
            # get points
            xs = np.linspace(x,new_x,n)[0:-1]
            ys = np.linspace(y,new_y,n)[0:-1]
        
            
            # set values of points inbetween
            for i in range(len(xs)-1):
                x1 = xs[i]
                x2 = xs[i+1]
                y1 = ys[i]
                y2 = ys[i+1]               

                p1 = self.get_index_of_pos(x1,y1)
                p2 = self.get_index_of_pos(x2,y2)
                p3 = self.get_index_of_pos(x1,y2)
                p4 = self.get_index_of_pos(x2,y1)
                points_to_add[p1] = self.free
                points_to_add[p2] = self.free
                points_to_add[p3] = self.free
                points_to_add[p4] = self.free
                
                
                
        for ray in rays:
            # make end point of ray 1 
            ray_end = ray
            new_x = x + ray_end[1]*np.cos(ray_end[0])
            new_y = y + ray_end[1]*np.sin(ray_end[0])
            p1 = self.get_index_of_pos(new_x,new_y)
            points_to_add[p1] = self.occupied
            #self.set_value_of_pos(new_x,new_y,1)
            

        for p in points_to_add.keys():
            self.set_value_of_index(p[0],p[1],points_to_add[p])
            #self.set_value_of_pos(p[0],p[1],points_to_add[p])        

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
        


class A_star():

    def __init__(self):
        #self.client = actionlib.SimpleActionClient('path_tracker', move_base_msgs.msg.MoveBaseAction)
        self.map_coords = None
        self.map_resolution = None
        self.bbminx = None
        self.bbminy = None
        self.bbmaxx = None
        self.bbmaxy = None
        self.t_stamp = None
        self.origo_index_i = None
        self.origo_index_j = None

        self.iterations = 500
        self.goal_reached = Bool()
        self.goal_reached.data = False
        # subscriber 
        self.sub_map = rospy.Subscriber("/map/GridMap", GridMapMsg, self.map_callback)
        self.sub_goal_reached = rospy.Subscriber('/path_tracker/feedback', Bool, self.goal_reached_callback)
        #self.sub_start_and_goal = rospy.Subscriber("/start_and_goal", PoseArray, self.start_and_goal_callback)
        

    def goal_reached_callback(self,msg): # TODO figure it out
        self.goal_reached = msg.data

    def map_callback(self,msg):
        self.map_resolution = msg.resolution
        self.bbminx = msg.bbminx
        self.bbminy = msg.bbminy
        self.bbmaxx = msg.bbmaxx
        self.bbmaxy = msg.bbmaxy
        self.t_stamp = msg.header.stamp
        self.origo_index_i = msg.origo_index_i
        self.origo_index_j = msg.origo_index_j
        self.map_coords = msg.data # TODO look at explorer find goal and see how to get the map

    def get_index_of_pos(self,x,y):
        """Return index of position in map grid, if not given geofence, return None"""
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        x_index = int((x-self.bbminx)/self.map_resolution)
        y_index = int((y-self.bbminy)/self.map_resolution)
        
        return x_index, y_index
    
    def get_value_of_index(self,i,j):
        """Return value of index in map grid, if not given geofence, return None"""
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        # if out of bounds, return 1

        if i < 0 or i > int((self.bbmaxx-self.bbminx)/self.map_resolution) or j < 0 or j > int((self.bbmaxy-self.bbminy)/self.map_resolution):
            return 1
        
        return self.map_coords[i].data[j]
    
    def get_value_of_pos(self,x,y):
        """Return value of position in map grid, if not given geofence, return None"""
        if not self.given_geofence:
            rospy.logwarn("No geofence given, but trying to get map grid")
            return None
        
        i,j = self.get_index_of_pos(x,y)
        
        return self.get_value_of_index(i,j)
        
    def distance(self,node1: Node,node2:Node):
        return math.dist(node1.position(),node2.position())

    def reconstruct_path(self,node: Node):
        pathlist = []
        while node.parent is not None:  # found target
            print('got in while')
            pathlist.append(node)
            node = node.parent
        pathlist.append(node) 
        path = [(node.x,node.y) for node in pathlist]
        path.reverse()
        return path
    

    
    # checks if inbounds according to the paramethes of the map
    def is_in_bounds(self,node):
        if self.bbminy < node.y < self.bbmaxy:
            if self.bbminx < node.x < self.bbmaxx:
                return True
        else:
            return False
        #self.map.is_point_in_polygon(new_x,new_y,self.map.geofence_list):

    #TODO implement dx,dy 
    def generate_neighbours(self,node):
        neighbourlist = {}
        buffer = 0.1
        #walllist = []
        dx = self.map.resolution # 5 cm
        dy = self.map.resolution
        for longditude in range(-1,2,1):
            for latitude in range(-1,2,1):
                new_x = node.x + dx*longditude
                new_y = node.y + dy*latitude
                neighbour = Node(new_x, new_y, parent = node, goal= node.goal)
                if self.is_in_bounds(neighbour):
                    #print(f'send in coord {new_x,new_y}')
                    #index_x,index_y = self.map.get_index_of_pos(new_x,new_y)
                    #print(f'cell value {cell.value}, {cell.position()}')
                    if self.get_value_of_pos(new_x,new_y)>=0.8: # will always work due to checking inbounds
                        #walllist.append((neighbour.position()))
                        neighbour.g = np.inf
                        neighbour.f = neighbour.g + neighbour.h
                    
                    neighbourlist[neighbour.position()] = neighbour
                    

        """for wall in walllist:
            print(f'wall is {wall}')"""
                    
                
        return neighbourlist


    def path(self,start,goal):
        # usually min heap
        goal = (goal.pose.position.x,goal.pose.position.y)
        start = (start.pose.position.x,start.pose.position.y)
        heap = []
        heapq.heapify(heap)
        openset = {}
        closedset = {}
        #heapify(openset)
        #best_path = None
        start_node = Node(x=start[0],y=start[1],goal=goal)
        openset[start_node.position()] = start_node
        heapq.heappush(heap,(start_node.f,start_node))

        # Controls the amount of max iterations before cancel
        #maxiter = 15000
        iter = 0
        
        while heapq and iter < self.iterations:
            #current = min(openset, key=lambda cordinates: openset[cordinates].f)
            current = heapq.heappop(heap)
            current = current[1]
            #if current.position() == (1.0,1.0):
            #print(openset)
            #print(current.position())
            #print(current.g)
            #print('\n')
            #print(current)
            #currentlist.append(current.position())
            if (current.x,current.y) == goal:
                return True,self.reconstruct_path(current)
            
            closedset[start_node.position()] = start_node
            
            neighbours = self.generate_neighbours(current)
            
            for neighbour in neighbours:
                neighbournode= neighbours[neighbour]

                if neighbournode.position() in closedset:
                    continue
                else:
                    closedset[neighbournode.position()] = neighbournode
                
                if neighbournode.position() in openset:
                    sameCoords = {key: openset[key] for key in openset.keys()
                                        & {(neighbournode.x, neighbournode.y)}}
                    nodes = sameCoords.values()
                    #print(f'nodes = {nodes}')
                    #print(nodes)
                    for node in nodes:
                        if neighbournode.g >= node.g:
                            pass
                        else:
                            heapq.heappush(heap,(neighbournode.f,neighbournode))
                            openset[neighbournode.position()] = neighbournode
                else: 
                    openset[neighbournode.position()] = neighbournode
                    heapq.heappush(heap,(neighbournode.f,neighbournode))
            iter +=1
            #print(iter)
        #print('no path found')
        #print(f'iter = {iter}')
        #print(f'openset length = {len(openset)}')
        print(' at the end')
        return False, self.reconstruct_path(current)
    
    def path_smoothing(self,path):
        # check 3 points at a time
        # if the change in dx and dy is the same, remove the middle point
        # if the change in dx and dy is not the same, keep the middle point
        path_length = len(path)
        #print(f'path_length = {path_length}')
        points_to_remove = []
        for point in range( 1, path_length-1):
            #print(f'point = {path[point]}')
            dx1 = (path[point][0] - path[point-1][0])
            dy1 = (path[point][1] - path[point-1][1])
            dx2 = (path[point+1][0] - path[point][0])
            dy2 = (path[point+1][1] - path[point][1])
            #print(f'dx1 = {dx1}, dy1 = {dy1}, dx2 = {dx2}, dy2 = {dy2}')
            if dx1 == dx2 and dy1 == dy2:
                points_to_remove.append(point)
        points_to_remove.reverse()

        #print(f'points_to_remove = {points_to_remove}')
        for point in points_to_remove:
            path.pop(point)
        #print(f'smoothened path = {path}')
        return path

class Path_Planner():
    def __init__(self) -> None:
        rospy.init_node('path_planner')
        #server
        #self.client = actionlib.SimpleActionClient('path_tracker', mb.MoveBaseAction)
        #print('waiting for server')
        #self.client.wait_for_server(rospy.Duration(0.5))
        #print('server found')

        #subscribers
        self.sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.start_and_goa_sub = rospy.Subscriber('/start_and_goal', Path, self.start_and_goal_callback)

        #publishers
        self.move_to_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        self.start = PoseStamped()
        self.goal = PoseStamped()
        
        # might need to change in the future
        self.rate = rospy.Rate(1)

        self.path_planner = A_star()
        self.last_msg = PoseStamped()
        
        print('path planner node started')
        #rospy.roslog('path planner node started')
############################################ Callbacks ############################################

    def start_and_goal_callback(self,msg):
        self.start = msg.poses[0]
        self.goal = msg.poses[1]
        self.main()

    def goal_callback(self,goal):
        self.goal = goal
        self.main() # this might be bad programming

    def map_callback(self,msg):
        self.map_resolution = msg.resolution
        self.bbminx = msg.bbminx
        self.bbminy = msg.bbminy
        self.bbmaxx = msg.bbmaxx
        self.bbmaxy = msg.bbmaxy
        self.t_stamp = msg.header.stamp
        self.origo_index_i = msg.origo_index_i
        self.origo_index_j = msg.origo_index_j
        self.path_planner.map = msg.data # TODO look at explorer find goal and see how to get the map
        
############################################ Main ############################################

    def tranform_path_to_posestamped(self,path):#fuzzy controller
        path_list = []
        """path_tosend = Path
        path_tosend.header.frame_id = "map"
        path_tosend.header.stamp = rospy.Time.now()"""

        for point in path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            path_list.append(pose)

        end = path_list[-1]
        end.pose.orientation = self.goal.pose.orientation
        path_list[-1] = end
        #path_tosend.poses = path_list

        return path_list

    """
    rosmsg show move_base_msgs/MoveBaseActionGoal

    std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
    actionlib_msgs/GoalID goal_id
    time stamp
    string id
    move_base_msgs/MoveBaseGoal goal
    geometry_msgs/PoseStamped target_pose
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        geometry_msgs/Pose pose
        geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
        geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w

    """
    def send_path(self,path): #TODO  fix to publish
        path_list = self.tranform_path_to_posestamped(path)
        #print('paths transformed')
        #print(path_list)
        
        if self.last_msg != path_list[0]:
            self.move_to_pub.publish(path_list[0])
            self.last_msg = path_list[0]
            #print('published')
        
        """for pose_stamped in path_list:
            goal = mb.MoveBaseGoal()
            goal.target_pose = pose_stamped
            client.send_goal(goal,done_cb=self.done_cb,feedback_cb=self.feedback_cb)
            client.wait_for_result() # result callback 
            print(client.get_result())"""
        

    """def done_cb(self,status,result):
        print('done')
        print(f'status is {status}')
        print(f'result is {result}')

        
    def feedback_cb(self,feedback):
        print('feedback')
        print(feedback)"""

    def main(self):
        #goal = (10,10)
        #self.path_planner.map = self.get_map()
        #print(self.goal)
        status,path = self.path_planner.path(self.start,self.goal)
        path = self.path_planner.path_smoothing(path)
        print(path)
        self.send_path(path)
        #print('path sent')


    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            self.rate.sleep()


if __name__ == "__main__":
    path_planner = Path_Planner()
    path_planner.spin()




