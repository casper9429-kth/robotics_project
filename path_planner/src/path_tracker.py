#!/usr/bin/env python3
from math import atan2, hypot
import rospy
import tf2_geometry_msgs  # this import is needed for tf2_ros.transform to work
import tf2_ros
from rospy import Service, Publisher, Subscriber, ServiceProxy
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from nav_msgs.msg import Path 
from std_srvs.srv import Trigger, TriggerResponse
from path_planner.srv import Bool, BoolResponse
from mapping.srv import CheckPolygon, CheckPolygonRequest
from tf.transformations import euler_from_quaternion


class PathTracker:
    def __init__(self):
        rospy.init_node('path_tracker')

        # Parameters
        self.close_to_goal_threshold = rospy.get_param('~close_to_goal_threshold', 0.3)
        self.in_goal_threshold = rospy.get_param('~in_goal_threshold', 0.03)
        self.angular_threshold = rospy.get_param('~angular_threshold', 0.16)
        self.orientation_threshold = rospy.get_param('~orientation_threshold', 0.1)

        self.fast_linear_speed = rospy.get_param('~fast_linear_speed', 0.2)
        self.slow_linear_speed = rospy.get_param('~slow_linear_speed', 0.1)
        self.angular_speed = rospy.get_param('~angular_speed', 0.7)

        self.move_duration = rospy.get_param('~move_duration', 1.5)
        self.observation_duration = rospy.get_param('~observation_duration', 0.5)
        
        # State
        self.rate = rospy.Rate(10)
        self.goal = None
        self.polygon_response = None
        self.is_running = False
        self.is_observing = False
        self.move_timer = None
        self.observation_timer = None
        self.path = Path()
        
        # TF2
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        rospy.sleep(0.5) # TODO: Check if this is necessary
        
        # Services
        self.start_service = Service('/path_tracker/start', Trigger, self.start_callback)
        self.stop_service = Service('/path_tracker/stop', Trigger, self.stop_callback)
        self.is_running_service = Service('/path_tracker/is_running', Bool, self.is_running_callback)
        
        # Service Proxies
        rospy.wait_for_service("/workspace/is_inside")
        self.is_inside_workspace = ServiceProxy("/workspace/is_inside", CheckPolygon)

        # Publishers
        self.cmd_vel_publisher = Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        self.path_sub = rospy.Subscriber('/path',Path,self.path_callback)

    def start_callback(self, req):
        self.start()
        return TriggerResponse(success=True, message='Started')
    
    def stop_callback(self, req):
        self.stop()
        return TriggerResponse(success=True, message='Stopped')
    
    def is_running_callback(self, req):
        return BoolResponse(self.is_running)

    def path_callback(self,msg:Path):
        self.path = msg
        self.goal = self.path.poses[0]
        # TODO check if needed
        self.goal.header.frame_id = 'map'
    
    
    def set_next_goal(self):
        if self.path.poses:
            self.goal = self.path.poses.pop(0)

    def calculate_cmd_twist(self):
        goal_in_base_link = None
        try:
            goal_in_base_link = self.buffer.transform(self.goal, 'base_link')
        except: # TODO: catch only relevant exceptions
            # print(self.buffer.can_transform('base_link', 'map', rospy.Time(0)))
            return Twist()
        
        angle_to_goal = atan2(goal_in_base_link.pose.position.y, goal_in_base_link.pose.position.x)
        distance_to_goal = hypot(goal_in_base_link.pose.position.x, goal_in_base_link.pose.position.y)
        goal_orientation = euler_from_quaternion([goal_in_base_link.pose.orientation.x,
                                                  goal_in_base_link.pose.orientation.y,
                                                  goal_in_base_link.pose.orientation.z,
                                                  goal_in_base_link.pose.orientation.w])[2]

        cmd_twist = Twist()

        #         
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
                self.set_next_goal()
                if len(self.path.poses) == 0:
                    self.stop()
                # TODO: publish a message that the goal has been reached
        
        return cmd_twist

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
            
            if self.goal and self.is_running:
                goal_point = self.goal.pose.position.x, self.goal.pose.position.y
                if self.is_inside_workspace(*goal_point).success:
                    cmd_twist = Twist() if self.is_observing else self.calculate_cmd_twist()
                    self.cmd_vel_publisher.publish(cmd_twist)
            self.rate.sleep()
            
          
if __name__ == '__main__':
    node = PathTracker()
    node.run()
    