#!/usr/bin/env python3
import math
import rospy
import tf2_ros
import actionlib
import tf2_geometry_msgs
from robp_msgs.msg import DutyCycles
# from aruco_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, PoseArray, Pose, Point 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
import tf
import time

class LineSegment():
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
        

class Polygon():
    def __init__(self, point_array) -> None:
        self.segments = self.generate_line_segments(point_array)
        self.point_array = [(point.position.x, point.position.y) for point in point_array] 
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
        

class PathTracker():
    def __init__(self):
        rospy.init_node('path_tracker')
        print('path_tracker node initalized')
        self.robot_frame = 'base_link'
        self.rate = rospy.Rate(10)

        # Position and orientation of the robot
        self.pose = PoseStamped()
        self.pose_in_map = PoseStamped()
        # Position and orientation of the robot in the base_link frame
        self.pose.header.frame_id = self.robot_frame
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 0.0
        
        
        # Flag that a goal has been received
        self.goal_received = False


        # Goal position and orientation
        self.goal = PoseStamped()
        self.goal_in_base_link = PoseStamped()
        self.goal_theta_in_odom = 0
        
        # To control the robots movement
        self.move = Twist()
        self.acceleration = 0.05
        self.max_speed = 0.4
        self.max_angle = 0.1
        self.angle_speed = 0.2
        self.deceleration_distance = 0.0
        self.in_goal_tolerance = 0.03
        self.orientaion_tolerance = 0.1
        self.stop_time = 1 # change to affect how long you want to stop
        self.duration = 2 # change to affect how long you want to drive before stopping
        self.start_time = False # internal variable to keep track of time

        # Used when you want the robot to follow an aruco marker    (not fully implemented yet)
        # self.aruco = Marker()

        # tf stuff
        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(2)
        print('Path Tracker: Tf2 stuff initialized')

        self.polygon = None

        #publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_reached_pub = rospy.Publisher('/goal_reached', Bool, queue_size=10)


        # subscribers
        self.fence_sub = rospy.Subscriber('/workspace_poses/pose_array', PoseArray, self.fence_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)  
        # self.aruco_sub = rospy.Subscriber('/aruco/markers/transformed_pose', Marker, self.aruco_callback)  
        print('Path tracker: Subscribers initalized')

   
        

   
    # To get the position of the goal
    def goal_callback(self, msg:PoseStamped):
        self.goal.pose.position = msg.pose.position                 # 2D Nav goal in rviz is in odom frame
        self.goal.pose.orientation = msg.pose.orientation
        self.goal_received = True
        rospy.loginfo('Tracker: recived goal')

    def fence_callback(self, msg:PoseArray):
        self.polygon = Polygon(msg.poses)


    def check_if_in_fence(self, pose: Pose):
        # Transform pose to map frame
        if self.polygon:
            return self.polygon.contains(pose.position)
        else:
            #raise Exception('No fence set')
            return None


    
    def transforms(self):   
        
        stamp = self.pose.header.stamp  
        try:                                    # lookup_transform('target frame','source frame', time.stamp, rospy.Duration(0.5))
            transform_map_2_base_link = self.tfBuffer.lookup_transform('base_link','map', stamp,rospy.Duration(0.5))     # give goal in base link frame
            self.goal_in_base_link= tf2_geometry_msgs.do_transform_pose(self.goal, transform_map_2_base_link)   
        except:
            print('Path tracker: No transform found')
                        

    def pure_pursuit(self):
            k = 0.4
            L = 0.30 
            z= 0.3
            
            angle_to_goal =  1 * math.atan2(self.goal_in_base_link.pose.position.y,self.goal_in_base_link.pose.position.x)
            distance = 1 * math.hypot(self.goal_in_base_link.pose.position.x,self.goal_in_base_link.pose.position.y)
            robot_theta = tf.transformations.euler_from_quaternion([self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w])[2] 
            goal_orientation = tf.transformations.euler_from_quaternion([self.goal_in_base_link.pose.orientation.x, self.goal_in_base_link.pose.orientation.y, self.goal_in_base_link.pose.orientation.z, self.goal_in_base_link.pose.orientation.w])[2]       
            dtheta = goal_orientation - robot_theta
            print(f'goal pos {self.goal_in_base_link.pose.position}')
            v=k*distance
            w=2*v*math.sin(angle_to_goal)/L
            print(w)
            self.move.linear.x = v
            self.move.linear.x  = max(self.move.linear.x ,0.0)
            self.move.linear.x  = min(self.move.linear.x ,self.max_speed) # max speed
            
            
            self.move.angular.z = w
            self.move.angular.z = max(self.move.angular.z ,0.0)
            self.move.angular.z = min(self.move.angular.z ,self.max_angle) # max angular speed
            
            
            
            if distance<= self.in_goal_tolerance:
                if abs(dtheta) >= self.orientaion_tolerance:
                    self.move.linear.x = 0.0
                    if dtheta >= 0:
                        self.move.angular.z = self.angle_speed
                        # print('rotating left')
                    elif dtheta < 0:
                        self.move.angular.z = -self.angle_speed
                        # print('rotating right')
                else:
                    self.move.linear.x = 0.0
                    self.move.angular.z = 0.0
                    print('Goal orientation reached')
                    
                    
            self.cmd_pub.publish(self.move)


        
     # Calculate the direction the robot should go
    def math(self):
        angle_to_goal =  1 * math.atan2(self.goal_in_base_link.pose.position.y,self.goal_in_base_link.pose.position.x)
        distance = 1 * math.hypot(self.goal_in_base_link.pose.position.x,self.goal_in_base_link.pose.position.y)
        robot_theta = tf.transformations.euler_from_quaternion([self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w])[2] 
        goal_orientation = tf.transformations.euler_from_quaternion([self.goal_in_base_link.pose.orientation.x, self.goal_in_base_link.pose.orientation.y, self.goal_in_base_link.pose.orientation.z, self.goal_in_base_link.pose.orientation.w])[2]       
        dtheta = goal_orientation - robot_theta
        #print(f'distance is {distance}')

        if distance > self.in_goal_tolerance:

            if angle_to_goal >= self.max_angle:
                self.move.linear.x = 0.0
                self.move.angular.z = self.angle_speed 
                # print('turning left')

            elif angle_to_goal <= -self.max_angle:
                self.move.linear.x = 0.0
                self.move.angular.z = -self.angle_speed 
                # print('turning right')

            else:
                self.move.linear.x = self.velocity_controller(distance)

                self.move.angular.z = 0.0
                
        else:
            #print('Goal reached')
            #print(f'Theta {dtheta}')
            if abs(dtheta) >= self.orientaion_tolerance:
                self.move.linear.x = 0.0
                if dtheta >= 0:
                    self.move.angular.z = self.angle_speed 
                    #print('rotating left')
                elif dtheta < 0:
                    self.move.angular.z = -self.angle_speed
                    #print('rotating right')
            else:
                self.move.linear.x = 0.0
                self.move.angular.z = 0.0
                self.goal_reached_pub.publish(True)
                print('Goal orientation reached')

        self.cmd_pub.publish(self.move)   
        
    def check_elapsed_time(self):
        stop = False
        if self.start_time == False:
            self.start_time = self.perf_counter()
        if self.start_time - time.perf_counter() > self.duration:
            stop = True
            self.start_time = False
        return stop 
        

    # This function is used to control the velocity of the robot
    def velocity_controller(self,distance):
        # This is the distance the robot needs to stop from current velocity'
        stop = self.check_elapsed_time() # chekcs if it is time to stop
        self.deceleration_distance = 0.5 * self.move.linear.x**2 / self.acceleration
        
        if distance <= self.deceleration_distance:
            self.move.linear.x -= self.acceleration
            self.move.linear.x  = max(self.move.linear.x ,0.0)
            #print('Slowing down')

        # Makes it so that the robot always want to accelerate to a max speed of 0.5
        else:
            self.move.linear.x += self.acceleration
            self.move.linear.x  = min(self.move.linear.x ,self.max_speed) # max speed
            #print('Moving forward')
        # if time to stop thjen no movement
        if stop: 
            self.linear.x = 0.0
        return self.move.linear.x


    def spin(self):
        if not self.goal_received:
            return

        if self.check_if_in_fence(self.goal.pose): 
            #print('In fence')
            self.transforms()
            self.math()
            if self.check_elapsed_time():
                rospy.sleep(self.stop_time)
        else:
            print('Path tracker: Goal Pose not inside workspace')
            self.move.linear.x = 0.0
            self.move.angular.z = 0.0
            self.cmd_pub.publish(self.move)

                
    def main(self):
        try:
            while not rospy.is_shutdown():
                self.spin()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass



if __name__ == '__main__':
    node = PathTracker()
    node.main()