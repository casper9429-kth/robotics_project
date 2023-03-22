import heapq
import rospy
from functools import total_ordering
import numpy as np
import math
import actionlib
import move_base_msgs.msg as mb
import sys

from dataclasses import dataclass,field
from occupancy_grid import Occupancy_grid

from numba import jit
from typing import Dict,Tuple
from scipy.interpolate import CubicSpline
from queue import PriorityQueue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from mapping.grid_map.grid_map import GridMap


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


class A_star():

    def __init__(self):
        #self.client = actionlib.SimpleActionClient('path_tracker', move_base_msgs.msg.MoveBaseAction)
        self.map = None

    def distance(self,node1: Node,node2:Node):
        return math.dist(node1.position(),node2.position())

    def reconstruct_path(self,node: Node):
        pathlist = []
        while node.parent is not None:  # found target
            pathlist.append(node)
            node = node.parent
        pathlist.append(node) 
        path = [(node.x,node.y) for node in pathlist]
        path.reverse()
        return path
    
    # checks if inbounds according to the paramethes of the map
    """def isinbounds(self,node):
        # retrives the limits for the map
        #TODO improve this for different maps
        limits = self.map.limits
        xmin = limits[0]
        xmax = limits[1]
        ymin = limits[2]
        ymax = limits[3]
        if node.x < xmin or node.x > xmax:
            #print('out of x bounds')
            return False
        if node.y < ymin or node.y > ymax:
            #print('out of y bounds')
            return False
        return True"""
    
    def is_in_bounds(self,node,buffer):
        for longditude in range(-1,2,1):
            for latitude in range(-1,2,1):
                new_x = node.x + buffer*longditude
                new_y = node.y + buffer*latitude
                if not self.map.is_point_in_polygon(new_x,new_y,self.map.geofence_list):
                    return False
        return True
        #self.map.is_point_in_polygon(new_x,new_y,self.map.geofence_list):

    #TODO implement dx,dy 
    def generate_neighbours(self,node):
        neighbourlist = {}
        buffer = 0.5
        #walllist = []
        dx = self.map.resolution # 5 cm
        dy = self.map.resolution
        #dx = map.dx
        #dy = map.dy
        for longditude in range(-1,2,1):
            for latitude in range(-1,2,1):
                new_x = node.x + dx*longditude
                new_y = node.y + dy*latitude
                neighbour = Node(new_x, new_y, parent = node, goal= node.goal)
                #if self.isinbounds(neighbour):
                if self.is_in_bounds(neighbour,buffer):
                    #print(f'send in coord {new_x,new_y}')
                    #index_x,index_y = self.map.get_index_of_pos(new_x,new_y)
                    #print(f'cell value {cell.value}, {cell.position()}')
                    #cell = self.map
                    if self.map.get_value_of_pos(new_x,new_y)>=0.8: # will always work due to checking inbounds
                        #walllist.append((neighbour.position()))
                        neighbour.g = np.inf
                        neighbour.f = neighbour.g + neighbour.h
                    
                    neighbourlist[neighbour.position()] = neighbour
                    

        """for wall in walllist:
            print(f'wall is {wall}')"""
                    
                
        return neighbourlist


    def path(self,goal):
        # usually min heap
        start = self.map.robot_pos[0:2] # gets the start position from the map
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
        maxiter = 15000
        iter = 0
        
        while heapq and iter < maxiter:
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
                return self.reconstruct_path(current)
            
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
        return False, self.reconstruct_path(current)


    def path_smoothing_cubline_split(self,path):
        x_path = [path[i][0] for i in range(len(path))]
        y_path = [path[i][1] for i in range(len(path))]
        cubic_spline = CubicSpline(x_path,y_path)
        x = np.linspace(path[0][0],path[0][-1],1000)
        y = cubic_spline(x)
        return np.array([x,y]).T

    # straight line split path smoothing
    def path_smoothing_straightline_split(self,path):
        x_path = [path[i][0] for i in range(len(path))]
        y_path = [path[i][1] for i in range(len(path))]
        x = np.linspace(path[0][0],path[0][-1],1000)
        y = np.interp(x,x_path,y_path)
        return np.array([x,y]).T

class server_manger():
    def __init__(self) -> None:
        rospy.init_node('server_manger')
        self.client = actionlib.SimpleActionClient('path_tracker', mb.msg.MoveBaseAction)
        self.client.wait_for_server()


    def get_map():
        resolution = 0.05
        map = Gridmap(resolution=resolution)
        return map

    def tranform_path_to_posestamped(self,path, end_pose):
        path_list = []
        path_tosend = Path
        path_tosend.header.frame_id = "map"
        path_tosend.header.stamp = rospy.Time.now()

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
        end.pose.orientation = end_pose.pose.orientation
        path_list[-1] = end
        path_tosend.poses = path_list
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
    def send_path(self,client,path,endpostion):
        path_list = self.tranform_path_to_posestamped(path,endpostion)
        for pose_stamped in path_list:
            goal = mb.msg.MoveBaseGoal()
            goal.goal.target_pose = pose_stamped
            client.send_goal(goal)
            client.wait_for_result() # result callback 
            print(client.get_result())

    def main(self):
        goal = (10,10)
        path_planner = A_star()
        path_planner.map = self.get_map()
        path = path_planner.path(goal)
        self.send_path(self.client,path,goal)


if __name__ == "__main__":
    



