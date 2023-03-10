#!/usr/bin/env python3
from heapq import heapify, heappush, heappop
import rospy
import numpy as np
import math
from dataclasses import dataclass,field
from occupancy_grid import Occupancy_grid
import cProfile
import pstats

map = Occupancy_grid(2000,2000)
currentlist = [] # List for debugging can be removed


@dataclass(order=True)
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

    def dist_to_goal(self, goal):
        return math.dist((self.x,self.y,),goal)
    
    def set_g(self):
        self.g = self.parent.f
    
    def position(self):
        return (self.x,self.y)
    
def distance(node1: Node,node2:Node):
    return math.dist(node1.position(),node2.position())


def reconstruct_path(node: Node):
    pathlist = []
    while node.parent is not None:  # found target
        pathlist.append(node)
        node = node.parent
    pathlist.append(node) 
    path = [(node.x,node.y) for node in pathlist]
    path.reverse()
    return path

# checks if inbounds according to the paramethes of the map
def isinbounds(node):
    """xmin = -map.x+1
    ymin = -map.y+1
    xmax = map.x -1
    ymax = map.y -1"""
    # retrives the limits for the map
    limits = map.limits
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
def generate_neighbours(node): # will cause issues with smaller dx,dx in occupancy_grid
    neighbourlist = {}
    buffer = 0.5
    walllist = []
    dx = 1
    dy = 1
    #dx = map.dx
    #dy = map.dy
    for longditude in range(-1,2,1):
        #print(f'i = {i}')
        for latitude in range(-1,2,1):
            new_x = node.x + dx*longditude
            new_y = node.y + dy*latitude
            neighbour = Node(new_x, new_y, parent = node, goal= node.goal)
            if isinbounds(neighbour):

                #print(f'send in coord {new_x,new_y}')
                cell = map.check_pos(neighbour.position())
                #print(f'cell value {cell.value}, {cell.position()}')
                if cell.value >=0.8: # will always work due to checking inbounds
                    walllist.append((neighbour.position()))
                    neighbour.g = np.inf
                    neighbour.f = neighbour.g + neighbour.h

                
                neighbourlist[neighbour.position()] = neighbour
                

    """for wall in walllist:
        print(f'wall is {wall}')"""
                
            
    return neighbourlist

def A_star(start,goal,map):
    # usually min heap
    openset = {}
    closedset = {}
    #heapify(openset)
    #best_path = None
    start_node = Node(x=start[0],y=start[1],goal=goal)
    openset[start_node.position()] = start_node

    # Controls the amount of max iterations before cancel
    maxiter = 10000
    iter = 0
    
    while openset and iter < maxiter:
        current = min(openset, key=lambda cordinates: openset[cordinates].f)
        current = openset.pop(current)
        #if current.position() == (1.0,1.0):
        #print(openset)
        #print(current.position())
        #print(current.g)
        #print('\n')
        
        currentlist.append(current.position())
        if (current.x,current.y) == goal:
            return reconstruct_path(current)
        
        closedset[start_node.position()] = start_node
        
        neighbours = generate_neighbours(current)
        
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
                        openset[neighbournode.position()] = neighbournode
            else: openset[neighbournode.position()] = neighbournode
        iter +=1
        #print(iter)

    return False, reconstruct_path(current)

def main():
    start = (-1678,-1678.0)
    goal = (1678,1678.0)

    # Obstacle manegment
    obs_1 = tuple((float(x-2),-2.0) for x in range(5))
    print(f'obstacle {obs_1}')
    obs_2 = tuple((float(x-4),1.0) for x in range(9))
    print(f'obstacle {obs_2}')
    obs_3 = tuple((0.0,float(y+2)) for y in range(2))
    print(f'obstacle {obs_3}')
    obstacle_tuple = obs_1+obs_2+obs_3
    for obs in obstacle_tuple:
        status = map.set_obstacle((obs))


    #object management
    obj = (-4.0,-4.0)
    stat = map.add_object(obj,'animal') # returns True of False to make sure it works 
    print('got here')
    #map.print_pos()

    def test_A_star():
        path = A_star(start,goal,map)
        print(path)
        #print(currentlist)
        #map.print_grid(path)

    # Gives stats for the algorithm
    def return_stats():
        with cProfile.Profile() as pr:
            test_A_star()
        stats = pstats.Stats(pr)
        stats.sort_stats(pstats.SortKey.TIME)
        stats.print_stats()
    test_A_star()
    #return_stats()



if __name__ == '__main__':
    main()


