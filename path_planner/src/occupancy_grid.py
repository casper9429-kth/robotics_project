#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass,field
import cProfile
import pstats

@dataclass
class Cell:
    # value = 0 free 
    # value = 1 Wall
    # value = 3 means robot
    x: float
    y: float
    value: float = 0
    threshold: float = 0.6
    label: str = None

    def deprication(self):
        pass

    def position(self):
        return (self.x,self.y)

class Occupancy_grid():
    def __init__(self,x,y) -> None:
        self.amount_of_squares = 100
        self.x = x
        self.y = y
        self.dx = 1
        self.dy = 1
        self.label_list =  ['cube','ball','animal']
        self.labels = np.array([self.label_list.index(label) for label in self.label_list])
        self.semantic_map = {
            'wall': 1.0,
            'cube': 0.3,
            'animal': 0.2,
            'ball' : 0.7
            # might need more labels
            }

        self.grid = self.create_grid()
        self.limits = self.return_bounderys()

    def create_grid(self):
        grid = [[Cell(float(i),float(j)) for i in range(-self.x,self.x,self.dx)] for j in range(-self.y,self.y,self.dy)]
        #print(grid)
        #for i in range(2):
        #    grid[4][i].value = 1
        return grid
    
    def print_pos(self):
        x_list = [self.grid[i] for i in range(len(self.grid))]
        for i in range(len(x_list)):
            newlist = [cell[i].position() for cell in x_list]
            print(newlist)
        #print(newlist)

    def return_bounderys(self):
        xmin = 0
        xmax = 0
        ymin = 0
        ymax = 0
        for row in self.grid:
            for cell in row:
                pos = cell.position()
                #print(f'pos is {pos}')
                xmin = min(pos[0],xmin)
                xmax = max(pos[0],xmax)

                ymin = min(pos[1],ymin)
                ymax = max(pos[1],ymax)

        #bound =[xmin,xmax,ymin,ymax]
        #print(f'bound is {bound}')
        return [xmin,xmax,ymin,ymax]


    def get_slope(self,points: tuple):
        x1,y1 = points[0][0],points[0][1]
        x2,y2 = points[1][0],points[1][1]
        k= (y1-y2)/(x1-x2)
        return k
        

    def define_workspace(self,points: list):
        # find vectors
        # determine the lines
        # assign values ---> use self.add_obstacles
        
        pass
        
    #TODO need to be optimized Is currently the thing thats slows it down the most
    def check_pos(self,pos: tuple): 
        #print(f'pos first is {pos}')
        if len(pos)>2:
            raise ValueError('Too many variables')
        try:
            #print(f'x index is {int(pos[0])+self.x}')
            #print(f'y index is {pos[1]+self.y}')
            return self.grid[(int(pos[1])+self.y)][(int(pos[0])+self.x)]
        except:
            print(f'found no matching position for {pos}')


    def set_obstacle(self,pos:tuple):
        for row in self.grid:
            for cell in row:
                if cell.position() == pos:
                    cell.value = 1
                    cell.label = 'wall'
                    return True
        return False
            
    def add_object(self,pos: tuple,label: str):
        if label in self.label_list:
            for row in self.grid:
                for cell in row:
                    if cell.position() == pos:
                        cell.value = 1
                        cell.label = label
                        return True
            return False
        else:# might want to raise error here
            return False
        

    #prints a little grid
    def print_grid(self, path = None):
        for row in self.grid:
            for cell in (row):
                #print(f'j is {j}\n')
                #cell = row[j]
                if path:
                    for coord in path:
                        if (cell.x,cell.y) == coord:
                            print('O',end='')
                            pass
                    if  cell.value > cell.threshold:
                        if cell.label == 'wall':
                            print(' X ',end='')
                        elif cell.label in self.label_list:
                            print('A',end='')
                    else:
                        print(' - ' ,end='')
                #print(cell)
                else:
                    if cell.value > cell.threshold:
                        if cell.label == 'wall':
                            print(' X ',end='')
                        elif cell.label in self.label_list:
                            print('A ',end='')
                    else:
                        print(' - ' ,end='')
            print('\n')
        print('')

    # not yet working 
    def plot(self):
        #plt.plot(x,y,'o')
        for x in range(-self.x,self.x):
            for y in range(-self.y,self.y):
                cell = self.grid[x][y]
                if cell.value == 1:
                    plt.plot(cell.x,cell.y,'o',)   
        plt.ylim([-5,5])
        plt.xlim([-5,5])
        plt.show()
        
if __name__ == '__main__':
    map = Occupancy_grid(5,5)
    map.print_pos()
    print(map.grid[9][3])
    print(map.check_pos((4.0, -2.0)))
