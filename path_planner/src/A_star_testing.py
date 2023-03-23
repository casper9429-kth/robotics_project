#!/usr/bin/env python3
import heapq
import rospy
from functools import total_ordering
import numpy as np
import math
import actionlib
import move_base_msgs.msg

from dataclasses import dataclass,field
from occupancy_grid import Occupancy_grid
import cProfile
import pstats
from numba import jit
from typing import Dict,Tuple
import Cython
from scipy.interpolate import CubicSpline
from queue import PriorityQueue

#map = Occupancy_grid(20,20)
currentlist = [] # List for debugging can be removed


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

# Example usage
class A_star_prototype():

    def __init__(self):
        #self.client = actionlib.SimpleActionClient('path_tracker', move_base_msgs.msg.MoveBaseAction)
        self.map = Occupancy_grid(20,20)

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
    def isinbounds(self,node):
        """xmin = -map.x+1
        ymin = -map.y+1
        xmax = map.x -1
        ymax = map.y -1"""
        # retrives the limits for the map
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
        return True
    #TODO implement dx,dy 
    def generate_neighbours(self,node): # will cause issues with smaller dx,dx in occupancy_grid
        neighbourlist = {}
        buffer = 0.5
        walllist = []
        dx = 1
        dy = 1
        #dx = map.dx
        #dy = map.dy
        min_pos = None
        min_f = np.inf
        for longditude in range(-1,2,1):
            #print(f'longditude {longditude}')
            
            #print(f'i = {i}')
            for latitude in range(-1,2,1):
                #print(f'lattitude {latitude}')
                new_x = node.x + dx*longditude
                new_y = node.y + dy*latitude
                
                neighbour = Node(new_x, new_y, parent = node, goal= node.goal)
                if latitude != 0 and longditude != 0:
                    if self.isinbounds(neighbour):

                        #print(f'send in coord {new_x,new_y}')
                        cell = self.map.check_pos(neighbour.position())
                        #print(f'cell value {cell.value}, {cell.position()}')
                        if cell.value >=0.8: # will always work due to checking inbounds
                            #walllist.append((neighbour.position()))
                            neighbour.g = np.inf
                            neighbour.f = neighbour.g + neighbour.h
                        
                        if neighbour.f < min_f:
                            min_f = neighbour.f
                            min_pos = neighbour.position()
                        
                        neighbourlist[neighbour.position(),neighbour.f] = neighbour
                    

        """for wall in walllist:
            print(f'wall is {wall}')"""
                    
                
        return neighbourlist, min_f,min_pos

    def return_current(self,openset):
        return min(openset, key=lambda cordinates: openset[cordinates].f)

    def path(self,start,goal):
        # usually min heap
        openset = {}
        closedset = {}
        #best_path = None
        start_node = Node(x=start[0],y=start[1],goal=goal)
        openset[start_node.position(),start_node.f] = start_node

        # Controls the amount of max iterations before cancel
        maxiter = 20000
        iter = 0

        # Used for optimization
        min_f = start_node.f
        min_pos = start_node.position()
        
        while openset and iter < maxiter:
            #current = min(openset,key=)
            print(f'min pos {min_pos}, min f {min_f}')
            print(openset.keys())
            print('\n')
            current = openset[min_pos,min_f]
            del openset[min_pos,min_f]
            #current = min(openset, key=lambda cordinates: openset[cordinates].f)
            #current = return_current_numba(openset)
            #current = openset.pop((current,current.f))
            #if current.position() == (1.0,1.0):
            #print(openset)
            #print(current.position())
            #print(current.g)
            #print('\n')
            
            currentlist.append(current.position())
            if (current.x,current.y) == goal:
                return self.reconstruct_path(current)
            
            closedset[start_node.position()] = start_node
            
            neighbours, temp_min_f,temp_min_pos = self.generate_neighbours(current)

            min_f = temp_min_f
            min_pos = temp_min_pos

            for neighbour in neighbours:
                neighbournode = neighbours[neighbour]
                #neighbournode = neighbour
                #print(neighbour)
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
                            openset[neighbournode.position(),neighbournode.f] = neighbournode
                else: openset[neighbournode.position(),neighbournode.f] = neighbournode
            iter +=1
            #print(iter)

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

class A_star():

    def __init__(self):
        #self.client = actionlib.SimpleActionClient('path_tracker', move_base_msgs.msg.MoveBaseAction)
        self.map = Occupancy_grid(20,20)

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
    def isinbounds(self,node):
        """xmin = -map.x+1
        ymin = -map.y+1
        xmax = map.x -1
        ymax = map.y -1"""
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
        return True
    
    #TODO implement dx,dy 
    def generate_neighbours(self,node):
        neighbourlist = {}
        buffer = 0.5
        #walllist = []
        dx = 0.05 # 5 cm
        dy = 0.05 
        #dx = map.dx
        #dy = map.dy
        min_pos = None
        for longditude in range(-1,2,1):
            for latitude in range(-1,2,1):
                new_x = node.x + dx*longditude
                new_y = node.y + dy*latitude
                neighbour = Node(new_x, new_y, parent = node, goal= node.goal)
                if self.isinbounds(neighbour):

                    #print(f'send in coord {new_x,new_y}')
                    cell = self.map.check_pos(neighbour.position())
                    #print(f'cell value {cell.value}, {cell.position()}')
                    if cell.value >=0.8: # will always work due to checking inbounds
                        #walllist.append((neighbour.position()))
                        neighbour.g = np.inf
                        neighbour.f = neighbour.g + neighbour.h
                    
                    neighbourlist[neighbour.position()] = neighbour
                    

        """for wall in walllist:
            print(f'wall is {wall}')"""
                    
                
        return neighbourlist


    def path(self,start,goal):
        # usually min heap
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
            currentlist.append(current.position())
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




"""Bezier Curve Smoothing:
def path_smoothing_bezier(path):
    x_path = [path[i][0] for i in range(len(path))]
    y_path = [path[i][1] for i in range(len(path))]
    x = np.linspace(path[0][0],path[0][-1],1000)
    y = Bezier.bezier_curve(x,x_path,y_path)
    return np.array([x,y]).T"""



def main():
    goalvar = 2
    if goalvar == 1:
        start = (-1682.0,-1682.0)
        goal = (1682.0,1682.0)
    elif goalvar == 2:
        start = (0.0,0.0)
        goal = (1.0,2.0)
    #print(f'goal = {goal[0]}')
    path_planner = A_star_test()
    #path_planner = A_star()
    
    # Obstacle manegment
    obs_1 = tuple((float(x-2),-2.0) for x in range(5))
    print(f'obstacle {obs_1}')
    obs_2 = tuple((float(x-4),1.0) for x in range(9))
    print(f'obstacle {obs_2}')
    obs_3 = tuple((0.0,float(y+2)) for y in range(2))
    print(f'obstacle {obs_3}')
    obstacle_tuple = obs_1+obs_2+obs_3
    for obs in obstacle_tuple:
        status = path_planner.map.set_obstacle((obs))


    #object management
    obj = (-4.0,-4.0)
    stat = path_planner.map.add_object(obj,'animal') # returns True of False to make sure it works 
    print('got here')
    #map.print_pos()

    def test_A_star():
        path = path_planner.path(start,goal)
        print(path)
        #print(currentlist)
        #smooth_path = path_smoothing(path)
        if goalvar == 2:
            path_planner.map.print_grid(path)

    # Gives stats for the algorithm
    def return_stats():
        with cProfile.Profile() as pr:
            test_A_star()
            #test_heap_Astar()
        stats = pstats.Stats(pr)
        stats.sort_stats(pstats.SortKey.TIME)
        stats.print_stats()
    #test_A_star()
    return_stats()


if __name__ == '__main__':
    main()


