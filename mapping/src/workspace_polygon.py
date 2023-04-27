#!/usr/bin/env python3
import rospy
from rospy import Subscriber, Service, Publisher
import tf2_ros
import tf2_geometry_msgs
from math import inf
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from mapping.srv import CheckPolygon, CheckPolygonResponse, CheckPolygonRequest


class Polygon:
    def __init__(self):
        rospy.init_node("polygon")

        # Parameters
        self.rate = rospy.Rate(10)
        self.workspace = None
        self.goal = None

        # Subscribers
        self.goal_subscriber = Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # Services
        self.start_service = rospy.Service("/inside/workspace", CheckPolygon, self.is_in_workspace_callback)

    def goal_callback(self, msg: PoseStamped):
        self.goal = msg
        self.goal.header.frame_id = 'map'

    def is_in_workspace_callback(self, req):
        self.segments = self.generate_line_segments(req)
        self.point_array = [(point.position.x, point.position.y) for point in req]
        return self.contains(self.goal.pose.position) 

    def contains(self, point):
        return self.is_point_in_polygon(point.x, point.y, self.point_array)

    def is_point_in_polygon(self,x,y,poly):
        """Check if point is in polygon"""
        n = len(poly)
        inside =False

        p1x,p1y = poly[0]
        for i in range(n+1):
            p2x,p2y = poly[i % n]
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x,p1y = p2x,p2y

        return inside

    def generate_line_segments(self, point_array):
        segments = []
        point_array.append(point_array[0])
        for i in range(len(point_array)-1):
            x1 = point_array[i].position.x
            y1 = point_array[i].position.y
            x2 = point_array[i+1].position.x
            y2 = point_array[i+1].position.y
            segments.append(LineSegment(x1,y1,x2,y2))
        return segments


class LineSegment:
    def __init__(self,x1,y1,x2,y2) -> None:
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.k = self.get_slope()
        self.m = self.get_y_intercept()
        self.is_horizontal = False
        self.is_vertical = False
    
    def get_slope(self):
        try:
            return (self.y2-self.y1)/(self.x2-self.x1)
        except ZeroDivisionError:
            self.is_vertical = True
            return inf

    def get_y_intercept(self):
        return self.y1 - self.get_slope()*self.x1

    def get_x_intercept(self,y):
        try:
            return (y-self.m)/self.k
        except ZeroDivisionError:
            self.is_horizontal = True
            return inf

    def ray_intersect(self, ray_start):
        """
        Returns True if the ray intersects the line segment, False otherwise.
        The ray starts at ray_start and extends to the right.
        """
        if self.is_vertical:
            return ray_start.x < self.x1
        elif self.is_horizontal:
            return False
        else:
            is_between_y = not (ray_start.y < self.y1 and ray_start.y < self.y2 or ray_start.y > self.y1 and ray_start.y > self.y2)
            x_to_the_left = ray_start.x < self.get_x_intercept(ray_start.y)
            return is_between_y and x_to_the_left
        
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
        

if __name__ == "__main__":
    polygon = Polygon()
    polygon.run()

