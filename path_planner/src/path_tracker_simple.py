#!/usr/bin/env python3
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from robp_msgs.msg import DutyCycles
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist


class path_tracker():
    def __init__(self):
        rospy.init_node('path_tracker')
        print('path_tracker node initalized')
        self.robot_frame = 'base_link'
        self.rate = rospy.Rate(10)
        self.pose = PoseStamped()
        self.pose_in_map = PoseStamped()
        self.goal = PoseStamped()
        self.goal_in_base_link = PoseStamped()
        self.aruco = Marker()
        # self.move = DutyCycles()
        self.move = Twist()
        self.pose.header.frame_id = self.robot_frame

        # tf stuff
        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        print('Tf2 stuff initialized')

        # Position and orientation of the robot in the base_link frame
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0
        self.pose.pose.orientation.w = 0

        #publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.duty_pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=10)

        # subscribers
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)  
        # self.aruco_sub = rospy.Subscriber('/aruco/markers/transformed_pose', Marker, self.aruco_callback)  
        print('Subscribers initalized')

    # To get the position of the goal from camera
    # def aruco_callback(self, msg:Marker):
        # self.aruco.header = msg.header
        # self.aruco.id = msg.id
        # self.aruco.pose.pose.position = msg.pose.pose.position
        # self.aruco.pose.pose.orientation = msg.pose.pose.orientation


    # To get the position of the goal
    def goal_callback(self, msg:PoseStamped):
        rospy.sleep(2)              # wait for the tf to be ready
        self.goal.pose.position = msg.pose.position
        self.goal.pose.orientation = msg.pose.orientation

        

    # give goal in base link frame
    def robots_location_in_map(self):
        
        stamp = self.pose.header.stamp  
        try:                                    # lookup_transform('target frame','source frame', time.stamp, rospy.Duration(0.5))
            transform_map_2_base_link = self.tfBuffer.lookup_transform(self.robot_frame,'map', stamp,rospy.Duration(0.5)) # The transform that relate map frame to base link frame
            self.goal_in_base_link= tf2_geometry_msgs.do_transform_pose(self.goal, transform_map_2_base_link)
        except:
            print('No transform found')
            pass
            
            # return None

        
     # Calculate the direction the robot should go
    def math(self):
        in_goal_tolerance = 0.1
        angle =  1 *   math.atan2(self.goal_in_base_link.pose.position.y,self.goal_in_base_link.pose.position.x)
        distance = 1 * math.hypot(self.goal_in_base_link.pose.position.x,self.goal_in_base_link.pose.position.y)


        if distance > in_goal_tolerance:
            if abs(angle) >0.1:
                self.move.linear.x = 0.0
                self.move.angular.z = angle # might need to be negative
                print('turning left')
            elif distance > self.move.linear.x:
                self.move.linear.x += 0.01
                self.move.angular.z = 0.0
                print('moving forward')
            elif distance < 1:
                self.move.linear.x -= 0.05
                self.move.angular.z = 0.0
                print('Slowing down')
        else:
            self.move.linear.x = 0.0
            self.move.angular.z = 0.0
            print('Goal reached') 
        self.move.angular.z  = max(self.move.angular.z ,-0.5)
        self.move.angular.z  = min(self.move.angular.z ,0.5)
        self.move.linear.x  = max(self.move.linear.x ,-0.5)
        self.move.linear.x  = min(self.move.linear.x ,0.5)
        self.cmd_pub.publish(self.move)
                

    # send goal to move_base which allows it to be shown in rviz
    def send_goal(self):
        self.goal.header.frame_id = 'map'
        self.goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(self.goal)


    def spin(self):
        self.robots_location_in_map()
        self.send_goal()
        directions = self.math()
        # self.publish_twist(directions)
               
    
    def main(self):
        try:
            while not rospy.is_shutdown():
                self.spin()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    node = path_tracker()
    node.main()