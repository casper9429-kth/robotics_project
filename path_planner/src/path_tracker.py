from math import atan2, hypot, inf

import rospy
import tf2_geometry_msgs  # this import is needed for tf2_ros.transform to work
import tf2_ros
from rospy import Service, Publisher, Subscriber
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from std_srvs.srv import Trigger, TriggerResponse
from path_planner.srv import Bool, BoolResponse

from tf.transformations import euler_from_quaternion


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

        # Parameters
        self.close_to_goal_threshold = rospy.get_param('~close_to_goal_threshold', 0.3)
        self.in_goal_threshold = rospy.get_param('~in_goal_threshold', 0.03)
        self.angular_threshold = rospy.get_param('~angular_threshold', 0.1)
        self.orientation_threshold = rospy.get_param('~orientation_threshold', 0.1)

        self.fast_linear_speed = rospy.get_param('~fast_linear_speed', 0.2)
        self.slow_linear_speed = rospy.get_param('~slow_linear_speed', 0.1)
        self.angular_speed = rospy.get_param('~angular_speed', 0.7)

        self.move_duration = rospy.get_param('~move_duration', 0.5)
        self.observation_duration = rospy.get_param('~observation_duration', 0.5)
        
        # State
        self.rate = rospy.Rate(10)
        self.workspace = None
        self.goal = None
        self.is_running = False
        self.is_observing = False
        self.move_timer = None
        self.observation_timer = None
        
        # TF2
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        rospy.sleep(0.5)
        
        # Services
        self.start_service = Service('/path_tracker/start', Trigger, self.start_callback)
        self.cancel_service = Service('/path_tracker/cancel', Trigger, self.cancel_callback)
        self.is_running_service = Service('/path_tracker/is_running', Bool, self.is_running_callback)
        
        # Publishers
        self.cmd_vel_publisher = Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.workspace_subscriber = Subscriber('/workspace_poses/pose_array', PoseArray, self.workspace_callback)
        self.goal_subscriber = Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

    def start_callback(self, req):
        self.start()
        return TriggerResponse(success=True, message='Started')
    
    def cancel_callback(self, req):
        self.stop()
        return TriggerResponse(success=True, message='Cancelled')
    
    def is_running_callback(self, req):
        return BoolResponse(self.is_running)

    def goal_callback(self, msg: PoseStamped):
        self.goal = msg
        # self.goal.header.stamp = rospy.Time(0) # always use the latest transform
        self.goal.header.frame_id = 'map'

    def workspace_callback(self, msg: PoseArray):
        self.workspace = Polygon(msg.poses)

    def calculate_cmd_twist(self):
        goal_in_base_link = None
        try:
            goal_in_base_link = self.buffer.transform(self.goal, 'base_link')
        except: # TODO: catch only relevant exceptions
            print(self.buffer.can_transform('base_link', 'map', rospy.Time(0)))
            return Twist()
        
        angle_to_goal = atan2(goal_in_base_link.pose.position.y, goal_in_base_link.pose.position.x)
        distance_to_goal = hypot(goal_in_base_link.pose.position.x, goal_in_base_link.pose.position.y)
        goal_orientation = euler_from_quaternion([goal_in_base_link.pose.orientation.x,
                                                  goal_in_base_link.pose.orientation.y,
                                                  goal_in_base_link.pose.orientation.z,
                                                  goal_in_base_link.pose.orientation.w])[2]

        cmd_twist = Twist()
        
        if distance_to_goal > self.in_goal_threshold:
            if angle_to_goal > self.angular_threshold:
                cmd_twist.angular.z = self.angular_speed
            elif angle_to_goal < -self.angular_threshold:
                cmd_twist.angular.z = -self.angular_speed
            elif distance_to_goal > self.close_to_goal_threshold:
                cmd_twist.linear.x = self.fast_linear_speed
            else:
                cmd_twist.linear.x = self.slow_linear_speed
        else:
            if goal_orientation > self.orientation_threshold:
                cmd_twist.angular.z = self.angular_speed
            elif goal_orientation < -self.orientation_threshold:
                cmd_twist.angular.z = -self.angular_speed
            else:
                self.stop()
                # TODO: publish a message that the goal has been reached
        
        return cmd_twist
    
    def goal_in_workspace(self):
        return self.workspace and self.goal and self.workspace.contains(self.goal.pose.position)
    
    def start(self):
        self.is_running = True
        self.move_timer = rospy.Timer(rospy.Duration(self.move_duration),
                                      self.move_timer_callback,
                                      oneshot=True)
        
    def stop(self):
        self.is_running = False
        if self.move_timer:
            self.move_timer.shutdown()
        if self.observation_timer:
            self.observation_timer.shutdown()
        self.cmd_vel_publisher.publish(Twist())
    
    def move_timer_callback(self, event):
        self.is_observing = True
        self.observation_timer = rospy.Timer(rospy.Duration(self.observation_duration),
                                             self.observation_timer_callback,
                                             oneshot=True)
        
    def observation_timer_callback(self, event):
        self.is_observing = False
        self.move_timer = rospy.Timer(rospy.Duration(self.move_duration),
                                      self.move_timer_callback,
                                      oneshot=True)
    
    def run(self):
        while not rospy.is_shutdown():
            if self.is_running: # and self.goal_in_workspace(): TODO: workspace is buggy
                cmd_twist = Twist() if self.is_observing else self.calculate_cmd_twist()
                self.cmd_vel_publisher.publish(cmd_twist)
            self.rate.sleep()
            
          
if __name__ == '__main__':
    node = PathTracker()
    node.run()
    