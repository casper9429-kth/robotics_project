#!/usr/bin/env python3
import math
import rospy
import tf2_ros
import actionlib
import tf2_geometry_msgs
from robp_msgs.msg import DutyCycles
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, PoseArray, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
import tf
### NEW ###
from std_srvs.srv import Trigger, TriggerResponse
from path_planner.srv import Bool, BoolResponse
###########

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

    def contains(self, point):
        """
        Returns True if the point is inside the polygon, False otherwise.
        The point is something that has an x and y attribute, like a Point.
        """
        # The point is inside the polygon if an imaginary ray starting at the point
        # intersects the polygon an odd number of times.
        intersections = 0
        for segment in self.segments:
            if segment.ray_intersect(point):
                intersections += 1
        return intersections % 2 == 1

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
        rospy.init_node('path_tracker_server_brain')
        print('path_tracker_server_brain node initalized')
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

        ### NEW ###
        # Services
        self.is_running = False
        self.start_service = rospy.Service('/path_tracker/start', Trigger, self.start_callback)
        self.cancel_service = rospy.Service('/path_tracker/cancel', Trigger, self.cancel_callback)
        self.is_running_service = rospy.Service('/path_tracker/is_running', Bool, self.is_running_callback)
        ###########

        # Goal position and orientation
        self.goal = PoseStamped()
        self.goal_in_base_link = PoseStamped()
        self.goal_theta_in_odom = 0
        
        # To control the robots movement
        self.move = Twist()
        self.acceleration = 0.05
        self.max_speed = 0.6
        self.max_angle = 0.1
        self.angle_speed = 0.7
        self.deceleration_distance = 0.0
        self.in_goal_tolerance = 0.02
        self.orientaion_tolerance = 0.05

        # Used when you want the robot to follow an aruco marker    (not fully implemented yet)
        self.aruco = Marker()

        # tf stuff
        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(2)
        print('Tf2 stuff initialized')

        self.polygon = None

        #publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # self.duty_pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=10)

        # subscribers
        self.fence_sub = rospy.Subscriber('/workspace_poses/pose_array', PoseArray, self.fence_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)  
        # self.aruco_sub = rospy.Subscriber('/aruco/markers/transformed_pose', Marker, self.aruco_callback)  
        print('Subscribers initalized')

    # To get the position of the goal from camera
    # def aruco_callback(self, msg:Marker):
        # self.aruco.header = msg.header
        # self.aruco.id = msg.id
        # self.aruco.pose.pose.position = msg.pose.pose.position
        # self.aruco.pose.pose.orientation = msg.pose.pose.orientation


        self.path_tracker_server = actionlib.SimpleActionServer('path_tracker', MoveBaseAction, execute_cb=self.execute_cb, auto_start=False)
        self.path_tracker_server.start()
        print('Action server started')
        
    ### NEW ###
    def start_callback(self, req):
        self.is_running = True
        return TriggerResponse(success=True, message='Started')
    
    def cancel_callback(self, req):
        self.is_running = False
        return TriggerResponse(success=True, message='Cancelled')
    
    def is_running_callback(self, req):
        return BoolResponse(self.is_running)
    ###########

    def execute_cb(self, goal):
        self.goal = goal.target_pose            # target_pose is a PosedStamped
        print('Goal recieved')
        self.path_tracker_server.set_succeeded()
        #self.main()
        

   
    # To get the position of the goal
    def goal_callback(self, msg:PoseStamped):
        self.goal.pose.position = msg.pose.position                 # 2D Nav goal in rviz is in odom frame
        self.goal.pose.orientation = msg.pose.orientation
        # print(self.goal)
        

    def fence_callback(self, msg:PoseArray):
        self.polygon = Polygon(msg.poses)


    def check_if_in_fence(self, pose: Pose):
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
            print('No transform found')
                        

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
            
            
            self.move.angular.z= w
            self.move.angular.z  = max(self.move.angular.z ,0.0)
            self.move.angular.z  = min(self.move.angular.z ,self.max_angle) # max angular speed
            
            
            
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
        ### NEW ###
        if not self.is_running:
            return
        ###########
        
        angle_to_goal =  1 * math.atan2(self.goal_in_base_link.pose.position.y,self.goal_in_base_link.pose.position.x)
        distance = 1 * math.hypot(self.goal_in_base_link.pose.position.x,self.goal_in_base_link.pose.position.y)
        robot_theta = tf.transformations.euler_from_quaternion([self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w])[2] 
        goal_orientation = tf.transformations.euler_from_quaternion([self.goal_in_base_link.pose.orientation.x, self.goal_in_base_link.pose.orientation.y, self.goal_in_base_link.pose.orientation.z, self.goal_in_base_link.pose.orientation.w])[2]       
        dtheta = goal_orientation - robot_theta

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
            # print('Goal reached')
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
                ### NEW ###
                self.is_running = False
                ###########
                # print('Goal orientation reached')
        
        self.cmd_pub.publish(self.move)   
        



    # This function is used to control the velocity of the robot
    def velocity_controller(self,distance):
        # This is the distance the robot needs to stop from current velocity
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
        return self.move.linear.x


    def spin(self):
        #print(self.goal.pose)
        #print(self.check_if_in_fence(self.goal.pose))
        if self.check_if_in_fence(self.goal.pose): #
            rospy.loginfo('In fence')
            self.transforms()
            self.math()
        else:
            rospy.loginfo('Goal Pose not inside workspace')
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
