#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from std_msgs.msg import Header
from queue import Queue

from mapping.grid_map.grid_map import GridMap
from mapping.msg import GridMap as GridMapMsg

from GridMapMsg.msg import GridMap as GridMapMsg


class WaveFrontDetector:

    def __init__(self):
        

        # subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback) # change to the topic that the map is published on
        self.OccupancyGrid_pub = rospy.Subscriber("/occupancy_grid/walls", OccupancyGrid, queue_size=10)
        self.grid_map_pub = rospy.Subscriber("/map/GridMap", GridMapMsg, queue_size=1)

        # publishers
        self.frontiers_pub = rospy.Publisher('/frontiers', PointStamped, queue_size=10)

        self.visited = set()

    def map_callback(self, msg):

# Header header
# Array_float[] data
# float32 resolution
# float32 bbminx
# float32 bbminy
# float32 bbmaxx
# float32 bbmaxy
# float32 origo_index_i
# float32 origo_index_j

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        data = msg.data

        queue_m = Queue()
        for i in range(width):
            for j in range(height):
                if data[i + j*width] == -1:
                    point = Point(i*resolution, j*resolution, 0)
                    self.mark(point, "Map-Open-List")
                    queue_m.put(point)

        while not queue_m.empty():
            p = queue_m.get()
            if self.is_marked(p, "Map-Close-List"):
                continue
            if self.is_frontier(p, msg):
                queue_f = Queue()
                new_frontier = []
                queue_f.put(p)
                self.mark(p, "Frontier-Open-List")
                while not queue_f.empty():
                    q = queue_f.get()
                    if self.is_marked(q, ["Map-Close-List", "Frontier-Close-List"]):
                        continue
                    if self.is_frontier(q, msg):
                        new_frontier.append(q)
                        for w in self.get_neighbors(q, width, height):
                            if not self.is_marked(w, ["Frontier-Open-List", "Frontier-Close-List", "Map-Close-List"]):
                                queue_f.put(w)
                                self.mark(w, "Frontier-Open-List")
                        self.mark(q, "Frontier-Close-List")
                self.publish_frontiers(new_frontier)
                for nf in new_frontier:
                    self.mark(nf, "Map-Close-List")
            for v in self.get_neighbors(p, width, height):
                if not self.is_marked(v, ["Map-Open-List", "Map-Close-List"]) and self.has_open_space_neighbor(v, msg):
                    queue_m.put(v)
                    self.mark(v, "Map-Open-List")
            self.mark(p, "Map-Close-List")

    def is_frontier(self, point, msg):
        width = msg.info.width
        data = msg.data

        if not self.is_marked(point, "Map-Open-List"):
            return False

        for neighbor in self.get_neighbors(point, width, msg.info.height):
            if data[neighbor.x + neighbor.y*width] == -1:
                return True

        return False

    def has_open_space_neighbor(self, point, msg):
        width = msg.info.width
        data = msg.data

        for neighbor in self.get_neighbors(point, width, msg.info.height):
            if data[neighbor.x + neighbor.y*width] == 0:
                return True

        return False

    def get_neighbors(self, point, width, height):
        neighbors = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                neighbor = Point(point.x + i, point.y + j, 0)
                if neighbor.x >= 0 and neighbor.x < width and neighbor.y >= 0 and neighbor