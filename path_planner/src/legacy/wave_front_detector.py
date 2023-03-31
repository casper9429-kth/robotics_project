#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from std_msgs.msg import Header
from queue import Queue
from mapping.msg import GridMap as GridMapMsg


class WaveFrontDetector:

    def __init__(self):
        

        # subscribers
        self.grid_map_sub = rospy.Subscriber("/map/GridMap", GridMapMsg, self.map_callback)

        # publishers
        self.frontiers_pub = rospy.Publisher('/frontiers', PointStamped, queue_size=10)

        self.visited = set()

    def map_callback(self, msg: GridMapMsg):
        self.map_coords = msg.header.data
        self.map_resolution = msg.resolution
        self.bbminx = msg.bbminx
        self.bbminy = msg.bbminy
        self.bbmaxx = msg.bbmaxx
        self.bbmaxy = msg.bbmaxy
        self.origo_index_i = msg.origo_index_i
        self.origo_index_j = msg.origo_index_j

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
                if neighbor.x >= 0 and neighbor.x < width and neighbor.y >= 0 and neighbor.y < height:
                    neighbors.append(neighbor)
        return neighbors
    
    def publish_frontiers(self, frontiers):
        for frontier in frontiers:
            point = PointStamped()
            point.header.frame_id = "map"
            point.header.stamp = rospy.Time.now()
            point.point = frontier
            self.frontiers_pub.publish(point)

    def mark(self, point, label):
        self.visited.add((point.x, point.y, label))

    def is_marked(self, point, label):
        if isinstance(label, str):
            return (point.x, point.y, label) in self.visited
        else:
            for l in label:
                if (point.x, point.y, l) in self.visited:
                    return True
            return False
        
if __name__ == '__main__':
    rospy.init_node('wave_front_detector')
    WaveFrontDetector()
    rospy.spin()