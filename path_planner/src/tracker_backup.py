#!/usr/bin/env python3

import rospy
import random
import math
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class RRTExplorer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('rrt_explorer')
        
        # Subscribe to the map and click events topics
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/clicked_point', PointStamped, self.goal_callback)

        # Setup the visualization publisher
        self.pub_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.marker_id = 0
        
        # Wait for the map to be received
        self.map_received = False
        while not self.map_received:
            rospy.sleep(0.1)

        # Define the search space boundaries
        self.min_x = self.map.info.origin.position.x
        self.max_x = self.map.info.origin.position.x + self.map.info.width * self.map.info.resolution
        self.min_y = self.map.info.origin.position.y
        self.max_y = self.map.info.origin.position.y + self.map.info.height * self.map.info.resolution

        # Define the RRT parameters
        self.epsilon = 0.1         # maximum distance between two consecutive points in the RRT tree
        self.max_iterations = 5000 
        self.goal_reached_threshold = 0.1

        # Define the tree
        self.tree = {}

        # Define the goal position
        self.goal = None

    def map_callback(self, map):
        self.map = map
        self.map_received = True

    def goal_callback(self, point):
        # If the goal position is within the search space
        if self.min_x <= point.point.x <= self.max_x and self.min_y <= point.point.y <= self.max_y:
            self.goal = point.point
            rospy.loginfo('New goal position: (%f, %f)', self.goal.x, self.goal.y)

            # Reset the tree and start the RRT exploration
            self.tree = {0: {'position': (self.min_x, self.min_y), 'parent': None}}
            self.marker_id = 0
            self.explore()

    def explore(self):
        # Loop until the goal is reached or the maximum number of iterations is reached
        for i in range(1, self.max_iterations):
            # Generate a random point in the search space
            if random.random() < self.epsilon:
                x = self.goal.x
                y = self.goal.y
            else:
                x = random.uniform(self.min_x, self.max_x)
                y = random.uniform(self.min_y, self.max_y)

            # Find the nearest node in the tree
            nearest_node = None
            nearest_dist = float('inf')
            for j in self.tree:
                dist = math.sqrt((self.tree[j]['position'][0] - x)**2 + (self.tree[j]['position'][1] - y)**2)
                if dist < nearest_dist:
                    nearest_node = j
                    nearest_dist = dist
                
            # Compute the new node position
            new_x = self.tree[nearest_node]['position'][0] + 0.1 * (x - self.tree[nearest_node]['position'][0])
            new_y = self.tree[nearest_node]['position'][1] + 0.1 * (y - self.tree[nearest_node]['position'][1])

            # Check if the new node is in collision
            if self.is_in_collision(new_x, new_y):
                continue

            # Add the new node to the tree
            self.tree[i] = {'position': (new_x, new_y), 'parent': nearest_node}

            # Publish the new node
            self.publish_node(new_x, new_y, 0, 1, 0)

            # Check if the goal is reached
            if math.sqrt((self.goal.x - new_x)**2 + (self.goal.y - new_y)**2) < self.goal_reached_threshold:
                rospy.loginfo('Goal reached!')
                break

        # Publish the tree
        self.publish_tree()

    def is_in_collision(self, x, y):
        # Convert the point to a map index
        i = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        j = int((y - self.map.info.origin.position.y) / self.map.info.resolution)

        # Check if the point is in collision
        if self.map.data[j * self.map.info.width + i] != 0:
            return True
        else:
            return False
        
    def publish_node(self, x, y, r, g, b):
        # Create a marker message
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'rrt_explorer'
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        # Publish the marker
        self.pub_marker.publish(marker)

        # Increment the marker id
        self.marker_id += 1

    def publish_tree(self):
        # Loop through the tree
        for i in self.tree:
            # Skip the root node
            if self.tree[i]['parent'] is None:
                continue

            # Create a marker message
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'rrt_explorer'
            marker.id = self.marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.points.append(PointStamped(header=marker.header, point=Point(x=self.tree[i]['position'][0], y=self.tree[i]['position'][1], z=0.1)).point)
            marker.points.append(PointStamped(header=marker.header, point=Point(x=self.tree[self.tree[i]['parent']]['position'][0], y=self.tree[self.tree[i]['parent']]['position'][1], z=0.1)).point)

            # Publish the marker
            self.pub_marker.publish(marker)

            # Increment the marker id
            self.marker_id += 1

if __name__ == '__main__':
    try:
        RRTExplorer()
    except rospy.ROSInterruptException:
        pass




                