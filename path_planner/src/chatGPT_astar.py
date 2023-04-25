import matplotlib.pyplot as plt
import numpy as np
import heapq
import random
from A_star_testing import Node
import math
import time


# Define constants
size = 400
l = size-1
n = int(size*3)
START = random.randint(0,l),random.randint(0,l)
GOAL = random.randint(0, l), random.randint(0, l)
#START = 0,0
#GOAL = 1999, 1999
ROWS = size
COLS = size
#OBSTACLES = [(1, 1), (2, 1), (3, 1), (3, 2), (3, 3), (2, 3), (1, 3)]
OBSTACLES = [(random.randint(0, l),random.randint(0,l)) for i in range(n)]
COLORS = ['r', 'b', 'g', 'c', 'm', 'y', 'k']


# Define the heuristic function (Euclidean distance)


class A_star():

    def __init__(self):
        # self.client = actionlib.SimpleActionClient('path_tracker', move_base_msgs.msg.MoveBaseAction)
        pass
        #self.map = Occupancy_grid(2000, 2000)

    def distance(self, node1: Node, node2: Node):
        return math.dist(node1.position(), node2.position())

    def reconstruct_path(self, node: Node):
        pathlist = []
        while node.parent is not None:  # found target
            pathlist.append(node)
            node = node.parent
        pathlist.append(node)
        path = [(node.x, node.y) for node in pathlist]
        path.reverse()
        return path

    # checks if inbounds according to the paramethes of the map
    def isinbounds(self, node):
        """xmin = -map.x+1
        ymin = -map.y+1
        xmax = map.x -1
        ymax = map.y -1"""
        # retrives the limits for the map
        # TODO improve this for different maps
        limits = (0, size-1, 0, size-1)
        xmin = limits[0]
        xmax = limits[1]
        ymin = limits[2]
        ymax = limits[3]
        if node.x < xmin or node.x > xmax:
            # print('out of x bounds')
            return False
        if node.y < ymin or node.y > ymax:
            # print('out of y bounds')
            return False
        return True

    # TODO implement dx,dy
    def generate_neighbours(self, node):
        neighbourlist = {}
        buffer = 1
        # walllist = []
        dx = 1  # 5 cm
        dy = 1
        # dx = map.dx
        # dy = map.dy

        for longditude in range(-1, 2, 1):
            for latitude in range(-1, 2, 1):
                new_x = node.x + dx*longditude
                new_y = node.y + dy*latitude
                neighbour = Node(new_x, new_y, parent=node, goal=node.goal)
                if self.isinbounds(neighbour):

                    if (neighbour.x, neighbour.y) in OBSTACLES:
                        # walllist.append((neighbour.position()))
                        neighbour.g = np.inf
                        neighbour.f = neighbour.g + neighbour.h

                    neighbourlist[neighbour.position()] = neighbour

        """for wall in walllist:
            print(f'wall is {wall}')"""

        return neighbourlist

    def path(self, start, goal):
        # usually min heap
        heap = []
        heapq.heapify(heap)
        openset = {}
        closedset = {}
        # heapify(openset)
        # best_path = None
        start_node = Node(x=start[0], y=start[1], goal=goal)
        openset[start_node.position()] = start_node
        heapq.heappush(heap, (start_node.f, start_node))

        # Controls the amount of max iterations before cancel
        maxiter = 15000
        iter = 0

        while heapq and iter < maxiter:
            # current = min(openset, key=lambda cordinates: openset[cordinates].f)
            current = heapq.heappop(heap)
            current = current[1]
            openset.pop(current.position())
            # if current.position() == (1.0,1.0):
            # print(openset)
            # print(current.position())
            # print(current.g)
            # print('\n')
            # print(current)
            # print(len(openset))

            # currentlist.append(current.position())
            if (current.x, current.y) == goal:
                return True, self.reconstruct_path(current)

            closedset[start_node.position()] = start_node

            neighbours = self.generate_neighbours(current)

            for neighbour in neighbours:
                neighbournode = neighbours[neighbour]

                if neighbournode.position() in closedset:
                    continue
                else:
                    closedset[neighbournode.position()] = neighbournode

                if neighbournode.position() in openset:
                    sameCoords = {key: openset[key] for key in openset.keys()
                                  & {(neighbournode.x, neighbournode.y)}}
                    nodes = sameCoords.values()
                    # print(f'nodes = {nodes}')
                    # print(nodes)
                    for node in nodes:
                        if neighbournode.g >= node.g:
                            pass
                        else:
                            heapq.heappush(
                                heap, (neighbournode.f, neighbournode))
                            openset[neighbournode.position()] = neighbournode
                else:
                    openset[neighbournode.position()] = neighbournode
                    heapq.heappush(heap, (neighbournode.f, neighbournode))
            iter += 1
            # print(iter)
        # print('no path found')
        # print(f'iter = {iter}')
        # print(f'openset length = {len(openset)}')
        return False, self.reconstruct_path(current)


    def path_smoothing(self, path):
        # check 3 points at a time
        # if the change in dx and dy is the same, remove the middle point
        # if the change in dx and dy is not the same, keep the middle point
        path_length = len(path)
        print(f'path_length = {path_length}')
        points_to_remove = []
        for point in range(1, path_length-1):
            # print(f'point = {path[point]}')
            dx1 = (path[point][0] - path[point-1][0])
            dy1 = (path[point][1] - path[point-1][1])
            dx2 = (path[point+1][0] - path[point][0])
            dy2 = (path[point+1][1] - path[point][1])
            print(f'dx1 = {dx1}, dy1 = {dy1}, dx2 = {dx2}, dy2 = {dy2}')
            if dx1 == dx2 and dy1 == dy2:
                points_to_remove.append(point)
        points_to_remove.reverse()

        print(f'points_to_remove = {points_to_remove}')
        for point in points_to_remove:
            path.pop(point)
        # print(f'smoothened path = {path}')
        return path


def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

# Define the A* algorithm function


def astar(start, goal, obstacles):
    # Initialize the priority queue
    pq = []
    heapq.heappush(pq, (0, start))
    # Initialize the cost and came_from dictionaries
    cost = {start: 0}
    came_from = {start: None}
    # Start the search loop
    while pq:
        current_cost, current_pos = heapq.heappop(pq)
        # Check if we've reached the goal
        if current_pos == goal:
            path = []
            while current_pos:
                path.append(current_pos)
                current_pos = came_from[current_pos]
            return path[::-1]
        # Check neighbors
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            neighbor_pos = (current_pos[0] + dx, current_pos[1] + dy)
            if 0 <= neighbor_pos[0] < ROWS and 0 <= neighbor_pos[1] < COLS and neighbor_pos not in obstacles:
                new_cost = cost[current_pos] + \
                    heuristic(current_pos, neighbor_pos)
                if neighbor_pos not in cost or new_cost < cost[neighbor_pos]:
                    cost[neighbor_pos] = new_cost
                    priority = new_cost + heuristic(neighbor_pos, goal)
                    heapq.heappush(pq, (priority, neighbor_pos))
                    came_from[neighbor_pos] = current_pos
    return None

# Define the visualization function


def visualize_grid(start, goal, obstacles, paths):
    # Create the grid
    grid = np.zeros((ROWS, COLS))
    for obs in obstacles:
        grid[obs[0], obs[1]] = 1
    # Set the start and goal points
    grid[start[0], start[1]] = 2
    grid[goal[0], goal[1]] = 3
    # Set the path
    for i, path in enumerate(paths):
        if path:
           # print(path)
            for pos in path:
                #print(pos)
                grid[pos[0], pos[1]] = 4 + i%2
    # Visualize the grid
    plt.imshow(grid, cmap=plt.cm.Dark2_r, interpolation='nearest')
    plt.xticks(range(COLS))
    plt.yticks(range(ROWS))
    plt.show()


# Run the A* algorithm and visualize the results
def test_runtime(n):
    times = [[], []]
    paths = []
    
    for i in range(n):
        print(f'iteration {i}')
        start_time = time.time()
        path = astar(START, GOAL, OBSTACLES)
        end_time = time.time()
        diff1 = end_time - start_time
        times[0].append(diff1)
        paths.append(path)

        PathPlanner = A_star()
        start_time = time.time()
        status, path = PathPlanner.path(START, GOAL)
        end_time = time.time()
        diff2 = end_time - start_time
        times[1].append(diff2)

        paths.append(path)
        print(f'diff1 = {diff1}, diff2 = {diff2}')
    return times, paths

times, paths = test_runtime(20)
print(f'avrg time for A* = {np.mean(times[0])}')
print(f'avrg time for my A* = {np.mean(times[1])}')
print(f'A* is {np.mean(times[0])/np.mean(times[1])} times faster than my A*')
print(f'Start = {START}, Goal = {GOAL}')
visualize_grid(START, GOAL, OBSTACLES, paths)
