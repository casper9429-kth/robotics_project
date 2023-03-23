
import math
import heapq
import sys

from A_star import Node

lst = [0,1,2]
print(lst[0:2])
sys.path.append('/home/robotics/catkin_ws/src/path_planner/src')
import mapping
from mapping import Gridmap
print(sys.path)
map = Gridmap()
print(map)