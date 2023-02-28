#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass,field

@dataclass
class Cell:
    # value = 0 free 
    # value = 1 Wall
    # value = 3 means robot
    x: float
    y: float
    value: float = 0
    threshold: float = 0.6

    def deprication(self):
        pass

class Occupancy_grid():
    def __init__(self,x,y) -> None:
        self.amount_of_squares = 100
        self.x = x
        self.y = y

        self.grid = self.create_grid()

    def create_grid(self):
        grid = [[Cell(i,j) for i in range(-self.x,self.x,1)] for j in range(-self.y,self.y,1)]
        #print(grid)
        #for i in range(2):
        #    grid[4][i].value = 1
        return grid
        
    def add_obstacle(self,length,start: tuple,direction):
        north = 0
        east = 0
        west = 0
        south = 0
        x_start  = start[0]
        y_start  = start[1]
        if direction == 'north':
            north = length
        if direction == 'east':
            east = length
        if direction == 'west':
            west = length
        if direction == 'south':
            south = length

        if length > self.x or length > self.y:
            print('Length longer then allowed')
        elif x_start > self.x or y_start > self.y:
            print(' out of the area')
        else:
            if direction == 'north':
                for y in range(length):
                    self.grid[x_start-y][y_start].value =1
            if direction == 'south':
                for y in range(length):
                    self.grid[x_start+y][y_start].value = 1
            if direction == 'west':
                for x in range(length):
                    self.grid[x_start][y_start-x].value =1
            if direction == 'east':
                for x in range(length):
                    self.grid[x_start][y_start+x].value = 1

    #prints a little grid
    def print_grid(self, path = None):
        for i in range(-self.x,self.x):
            for j in range(-self.y,self.y):
                cell = self.grid[i][j]
                if path:
                    for coord in path:
                        if (cell.x,cell.y) == coord:
                            print('O ',end='')
                #print(cell)
                if  cell.value > cell.threshold:
                    print('X ',end='')
                else:
                    print('-', end=' ')
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
    occupancy = Occupancy_grid(5,5)
    #grid.plot()
    print(range(-occupancy.x,occupancy.x))
 
    occupancy.add_obstacle(3,(5,5),'east')
    occupancy.add_obstacle(3,(7,5),'west')
    occupancy.add_obstacle(4,(10,10),'north')
    occupancy.print_grid()
    #print(lst[1:2])
