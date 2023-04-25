from math import atan2, hypot

import rospy
import tf2_ros
from rospy import Service, Publisher, Subscriber


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
            return math.inf

    def get_y_intercept(self):
        return self.y1 - self.get_slope()*self.x1

    def get_x_intercept(self,y):
        try:
            return (y-self.m)/self.k
        except ZeroDivisionError:
            self.is_horizontal = True
            return math.inf

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
        

class Polygon:
    def __init__(self, pose_array: PoseArray):
        self.segments = self.generate_line_segments(pose_array)
        self.point_array = [(point.position.x, point.position.y) for point in pose_array] 
    def contains(self, point):
        """
        Returns True if the point is inside the polygon, False otherwise.
        The point is something that has an x and y attribute, like a Point.
        """
        # The point is inside the polygon if an imaginary ray starting at the point
        # intersects the polygon an odd number of times.
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


class PathTracker:
    def __init__(self):
        rospy.init_node('path_tracker')
        
        self.workspace = None
        self.goal = None
        
        # TF2
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        
        # Services
        self.is_running = False
        self.start_service = Service('/path_tracker/start', Trigger, self.start_callback)
        self.cancel_service = Service('/path_tracker/cancel', Trigger, self.cancel_callback)
        self.is_running_service = Service('/path_tracker/is_running', BoolSrv, self.is_running_callback)
        
        # Publishers
        self.cmd_vel_publisher = Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.workspace_subscriber = Subscriber('/workspace_poses/pose_array', PoseArray, self.workspace_callback)
        self.goal_subscriber = Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

    def start_callback(self, req):
        self.is_running = True
        return TriggerResponse(success=True, message='Started')
    
    def cancel_callback(self, req):
        self.is_running = False
        return TriggerResponse(success=True, message='Cancelled')
    
    def is_running_callback(self, req):
        return BoolSrvResponse(self.is_running)

    def goal_callback(self, msg: PoseStamped):
        self.goal = msg
        self.goal.header.stamp = rospy.Time(0) # always use the latest transform
        self.goal.header.frame_id = 'map'

    def workspace_callback(self, msg: PoseArray):
        self.workspace = Polygon(msg.poses)

    def calculate_cmd_twist(self):
        goal_in_base_link = None
        try:
            goal_in_base_link = self.buffer.transform(self.goal, 'base_link')
        except: # TODO: catch only relevant exceptions
            return
        
        angle_to_goal = atan2(self.goal_in_base_link.pose.position.y,self.goal_in_base_link.pose.position.x)
        distance = hypot(self.goal_in_base_link.pose.position.x,self.goal_in_base_link.pose.position.y)
        robot_theta = tf.transformations.euler_from_quaternion([self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w])[2] 
        goal_orientation = tf.transformations.euler_from_quaternion([self.goal_in_base_link.pose.orientation.x, self.goal_in_base_link.pose.orientation.y, self.goal_in_base_link.pose.orientation.z, self.goal_in_base_link.pose.orientation.w])[2]       
        dtheta = goal_orientation - robot_theta
    
    
    
    
    
    
    
    
    # return self.workspace is not None and self.workspace.contains(pose.position)
