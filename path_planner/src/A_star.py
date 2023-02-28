#!/usr/bin/env python3
from heapq import heapify, heappush, heappop
import rospy
import numpy as np
import math
from dataclasses import dataclass,field
from occupancy_grid import Occupancy_grid

map = Occupancy_grid(20,20)
currentlist = []




@dataclass(order=True)
class Node:
    x: float
    y: float
    goal: tuple
    g: float = field(init=False)
    h: float = field(init=False)
    f: float = field(init=False)
    parent: object = field(default=None)
    children: list = field(default_factory=list, init=False)

    def __post_init__(self):
        if self.parent == None:
            self.g = 0
        else:
            self.g = self.parent.g + math.dist((self.x,self.y,),self.parent.position())
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
    while node.parent.parent is not None:  # found target
        pathlist.append(node)
        node = node.parent
    path = [(node.x,node.y) for node in pathlist]
    path.reverse()
    return path


def isinbounds(node):
    xmin = 0
    ymin = 0
    xmax = map.x
    ymax = map.y
    if node.x < xmin or node.x > xmax:
        #print('out of x bounds')
        return False
    if node.y < ymin or node.y > ymax:
        #print('out of y bounds')
        return False
    return True

def generate_neighbours(node):
    neighbourlist = {}
    buffer = 0.5
    for longditude in range(-1,2,1):
        #print(f'i = {i}')
        for latitude in range(-1,2,1):
            neighbour = Node(x = node.x + longditude,y = node.y + latitude,parent = node, goal= node.goal)
            if map.grid[longditude][latitude].value == 1:
                neighbour.g = np.inf 
            if isinbounds(node):
                neighbourlist[neighbour.position] = neighbour
    return neighbourlist

def A_star(start,goal,map):
    # usually min heap
    openset = {}
    closedset = {}
    #heapify(openset)
    #best_path = None
    start_node = Node(x=start[0],y=start[1],goal=goal)
    openset[start_node.position()] = start_node
    maxiter = 1000

    iter = 0
    
    while openset and iter < maxiter:
        current = min(openset, key=lambda cordinates: openset[cordinates].f)
        current = openset.pop(current)
        #print(openset)
        #print(current.position())
        #print(current.h)
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
        print(iter)
    return False

def main():
    start = (20,0)
    goal = (10,10)
    #map.add_obstacle(3,(-15,-11),'east')
    print('got here')
    path = A_star(start,goal,map)
    print(path)
    print(currentlist)
    map.print_grid(path)

if __name__ == '__main__':
    main()


