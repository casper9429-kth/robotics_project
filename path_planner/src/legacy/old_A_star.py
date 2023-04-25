#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Erik Skog}
# {Erskog@kth.se}
import math
from math import atan2
import numpy as np
from dubins import *

class Node():
    def __init__(self,car,x,y,parent):
        self.x=x
        self.y=y
        self.parent=parent
        if self.parent is None:
            self.g = 0
        else:
            self.g = self.parent.g + vecdist(x, y, self.parent.x, self.parent.y)
        self.heuristic = vecdist(x, y, car.xt, car.yt)
        self.f = self.g + self.heuristic

    def __str__(self) -> str:
        return f'x,y : {self.x}, {self.y}, \ng,h: {self.g}, {self.heuristic}\nf: {self.f}'
    def __lt__(self,other):
        return self.f <other.f
        
    def __eq__(self,other: object) -> bool:
        if self.x == other.x and self.y == other.y:
            return True
        else: return False
    
        
    def getParent(self):
        return self.parent

    def position(self):
        return self.x, self.y

def recounstructPath(node):
    pathlist = []
    while node.parent.parent is not None:  # found target
        pathlist.append(node)
        node = node.parent
    path = [(node.x,node.y) for node in pathlist]
    path.reverse()
    return path

def generatemap(car):
    # establish bounderies
    dx = 0.2
    dy = 0.2
    xdist = (car.xub - car.xlb)/dx
    x = np.arange(car.xlb, car.xub, dx)
    y = np.arange(car.ylb, car.yub, dy)
    # find obstacles and give nodes g = inf
    # generate grid
    # use aStar on grid
    return x,y,dx,dy

def generateNeighbours(node,dx,dy,car):
    obstaclelist = car.obs
    neighbourlist = {}
    buffer = 0.5
    for longditude in range(-1,2,1):
        #print(f'i = {i}')
        for latitude in range(-1,2,1):
            neighbour = Node(car,node.x + dx*longditude,node.y + dy*latitude,node)
            for obstacle in obstaclelist:
                if vecdist(neighbour.x,neighbour.y,obstacle[0],obstacle[1])<(obstacle[2]+buffer):
                    neighbour.g = np.inf
            if isinbounds(car, node.x,node.y):
                neighbourlist[(node.x + dx*longditude,node.y + dy*latitude)] = neighbour
    return neighbourlist



def aStar3(car):
    visitedlist = {}
    openlist = {}
    xcoords, ycoords, dx, dy = generatemap(car)
    startnode = Node(car,car.x0,car.y0,None)
    openlist[startnode.position()] = startnode

    while openlist:
        #print(len(openlist))
        bestcoordinates = min(openlist, key=lambda cordinates: openlist[cordinates].f)
        #pop best value
        current = openlist.pop(bestcoordinates)
        #print(f'current: {current}')
        visitedlist[current.position()] = current
        # check if we at goal.
        if iswithintarget(car, current.x, current.y):
            path = recounstructPath(current)
            print('Found Path')
            #print(path)
            return path
        # generate neighbours
        neighbours = generateNeighbours(current,dx,dy,car)
        #print(neighbours.keys())
        #print(f'antal genrerade neighbours {len(neighbours)}')
        #print(f'längdskit {(neighbours)}')
        for neighbour in neighbours:
            neighbournode= neighbours[neighbour] # get node objectobject
            if isinbounds(car,neighbournode.x,neighbournode.y):
                #print(f'openlist = {len(openlist)}')
                #print(f'neighbournode= {(neighbournode)}')
                if (neighbournode.x,neighbournode.y) in visitedlist:
                    #print('in visitedlist')
                    #print(neighbournode)
                    pass
                else:
                    if (neighbournode.x, neighbournode.y) in openlist:
                        sameCoords = {key: openlist[key] for key in openlist.keys()
                                      & {(neighbournode.x, neighbournode.y)}}
                        nodes = sameCoords.values()
                        #print(f'nodes = {nodes}')
                        #print(nodes)
                        for node in nodes:
                            if neighbournode.g > node.g:
                                pass
                            else:
                                 #print('addednode to openlist')
                                 openlist[neighbournode.position()] = neighbournode
                    else: openlist[neighbournode.position()] = neighbournode
            #else: print('out of bounds')

    print(f'Found no path')
    return

def vecdist(x1,y1,x2,y2): # potential source of error
    dist = np.sqrt((x1-x2)**2 + (y1-y2)**2)
    return dist

def iswithintarget(car,x,y):
    if vecdist(x,y,car.xt,car.yt) < 1.4:
        return True
    else:
        return False



def isinbounds(car,x,y):
    xmin ,xmax = car.xlb,car.xub
    ymin ,ymax = car.ylb,car.yub
    #out of bounds check
    if x < xmin or x > xmax:
        #print('out of x bounds')
        return False
    if y < ymin or y > ymax:
        #print('out of y bounds')
        return False
    return True


def checkPhi(phi):
    if phi < -math.pi / 4:
        return  -math.pi / 4
    if phi > math.pi / 4:
        return math.pi / 4
    else: return phi

def solution(car):
    # 1. ta ut närmaste koordinaten på den planerade vägen
    # 2. ansätt dessa till nya koordinater

    # 3. phi = np.arctan2(idealx-currentx,idealy-currenty)-theta
    # 4. se till att phi är inom intervallet.
    # 5. kör step
    # 6. kör tills framme vid målet
    ''' <<< write your code below >>> '''
    dt = 0.1
    path = aStar3(car)
    controls = [0.0]

    times = [0.0, dt]
    x_curr = car.x0
    y_curr = car.y0
    theta_curr = 0
    phi = 0

    while not iswithintarget(car,x_curr,y_curr) or isinbounds(car,x_curr,y_curr):

        sorted_coords= list(filter(lambda pathnodes: vecdist(x_curr,y_curr,  pathnodes[0],  pathnodes[1]) < 1.3, path))
        #sorted_coords= list(filter(lambda pathnodes: np.hypot(x_curr - pathnodes[0], y_curr - pathnodes[1]) < 1.3, path))
        #print(sorted_coords)
        """sorted_coords = [math.hypot(x_curr - value[0], y_curr - value[1]) for index,value in enumerate(path)]
        lowest_score = min(sorted_coords)
        lowest_index = sorted_coords.index(lowest_score)
        ideal_path = path[lowest_index]

        if lowest_score < 1.5:
            print('Not anything low enough')
            break"""

        if len(sorted_coords) == 0:
            print('End of nodes')
            break

        ideal_path = sorted_coords[-1]
        phi = phi + np.arctan2(ideal_path[1] - y_curr, ideal_path[0] - x_curr) - theta_curr

        phi = checkPhi(phi)
        controls.append(phi)
        times.append(times[-1] + dt)

        x_curr, y_curr, theta_curr = step(car, x_curr, y_curr, theta_curr, phi, dt)
    return controls, times