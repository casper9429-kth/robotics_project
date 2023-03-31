import rospy
from dataclasses import dataclass
import random 
import math


@dataclass
class Node():
    x: float
    y: float
    obstacle: bool
    children: Node
    parent: Node
    cost: float

    def lowest_cost(self):
        best_child = 1e15
        for child in children:
            if child.cost < best_child:
                best_child = child
        return best_child


class RRT_star():
    def __init__(self) -> None:
        self.r= 10
        self.nodes = []
        self.grid = None
        self.path = []
        self.max_iter = 100


    

    def generate_random_postion(self):
        return Node(0,0,0,None,None,0,0)
    
    def nearest_node(self,new_node: Node):
        best_node = None
        best_dist = 1e15
        for node in self.nodes:
            if self.node_dist(new_node,node) > best_dist:
                best_node = node
        return best_node,best_dist

    def node_dist(self,node_1: Node,node_2: Node):
        return math.hypot(node_1.x,node_1.y,node_2.x,node_2.y)
    # sets parent child relation and the cost of the child 
    # might cause problems
    def chain(self,parent: Node, child:Node):
        parent.children.append(child) 
        child.cost = parent.cost + self.node_dist(parent,child)

    def find_neighbors(self,new_node: Node):
        """neighbors = []
        for node in self.nodes:
            if self.node_dist(new_node,node)< r:
                neighbors.append(node)"""
        return [node for node in self.nodes if self.node_dist(new_node,node) <self.r]
    
    def best_path(self):
        pass
    
    def algorithm(self):
        for iter in range(self.max_iter):
            new_node = self.generate_random_postion()
            if not new_node.obstacle:
                closest_node,new_node_cost = self.nearest_node(new_node)
                neighbors = self.find_neighbors()
                link = self.Chain(closest_node,new_node)
                local_best = []
                for node in neighbors: # might need to be otomized later
                    if new_node_cost + self.node_dist(new_node,node) < node.cost:
                        self.chain(node,new_node)
                        self.path.append(node)
                        # need to figure out how to update best path
            else: pass
        return 
"""
Rad = r
G(V,E) //Graph containing edges and vertices
For itr in range(0…n)
    Xnew = RandomPosition()
    If Obstacle(Xnew) == True, try again
    Xnearest = Nearest(G(V,E),Xnew)
    Cost(Xnew) = Distance(Xnew,Xnearest)
    Xbest,Xneighbors = findNeighbors(G(V,E),Xnew,Rad)
    Link = Chain(Xnew,Xbest)
    For x’ in Xneighbors
        If Cost(Xnew) + Distance(Xnew,x’) < Cost(x’)
            Cost(x’) = Cost(Xnew)+Distance(Xnew,x’)
            Parent(x’) = Xnew
            G += {Xnew,x’}
    G += Link 
Return G
"""