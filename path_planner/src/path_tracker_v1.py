#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
import tf2_ros
import tf2_geometry_msgs
from aruco_msgs.msg import MarkerArray, Marker



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

        self.rate = rospy.Rate(10)
        #publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # subscribers
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)  # To get the position of the goal
        self.goal_sub = rospy.Subscriber('/aruco/markers/transformed_pose', Marker, self.aruco_callback)  # To get the position of the goal from camera
        print('Subscribers initalized')

    # Getting the position of the goal
    def aruco_callback(self, msg:Marker):
        self.aruco.header = msg.header
        self.aruco.header.frame_id = 'map'
        self.aruco.id = msg.id
        # self.goal.header.frame_id = 'map'
        # self.goal.header.stamp = msg.header.stamp
        # self.goal.pose.position = msg.pose.pose.position
        # self.goal.pose.orientation = msg.pose.pose.orientation

    def goal_callback(self, msg:PoseStamped):
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

    def robots_location_in_base_link(self):
        stamp = self.pose.header.stamp  
        try:                                    # lookup_transform('target frame','source frame', time.stamp, rospy.Duration(0.5))
            transform_map_2_base_link = self.tfBuffer.lookup_transform('map',self.robot_frame, stamp,rospy.Duration(0.5)) # The transform that relate map frame to base link frame
            self.pose_in_map= tf2_geometry_msgs.do_transform_pose(self.pose, transform_map_2_base_link)
        except:
            print('No transform found')
            pass
            
            # return None
        
     # Calculate the direction the robot should go
    def math(self):
        in_goal_tolerance = 0.12
        turn =  1 *   math.atan2(self.goal_in_base_link.pose.position.y,self.goal_in_base_link.pose.position.x)
        forward = 1 * math.hypot(self.goal_in_base_link.pose.position.x,self.goal_in_base_link.pose.position.y)

        if math.hypot(self.goal_in_base_link.pose.position.x,self.goal_in_base_link.pose.position.y) < in_goal_tolerance:
            forward = 0
            turn = 0
            print('Goal reached')
        turn = max(turn,-0.5)
        turn = min(turn,0.5)
        forward = max(forward,-0.5)
        forward = min(forward,0.5)
        #turn  = max(turn,-1)
        #turn  = min(turn,1)
        #forward = max(forward,-1)
        #forward = min(forward,1)
        #print('turn: ',turn)
        #print('forward: ',forward)
        return [forward,turn]

    # publish twist message
    def publish_twist(self, msg):
        message = Twist()
        message.linear.x = msg[0]
        message.angular.z = msg[1]
        self.cmd_pub.publish(message)

    # send goal to move_base which allows it to be shown in rviz
    def send_goal(self):
        self.goal.header.frame_id = 'map'
        self.goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(self.goal)


    def spin(self):
        self.robots_location_in_map()
        directions = self.math()
        self.publish_twist(directions)
        print(self.pose_in_map.pose.position)
        self.send_goal()
    
                    
    
    def main(self):
        try:
            while not rospy.is_shutdown():
                self.spin()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass


    def visualize_path(self):
        # poselist = [(self.pose.position.x, pose.position.y, pose.position.z) for pose in self.pose_array.poses]
        poselist = [(self.pose_in_map.pose.position.x, self.pose_in_map.pose.position.y, self.pose_in_map.pose.position.z), (self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z)]

        mark = Marker()
        mark.header.frame_id = "map"
        mark.header.stamp = rospy.Time.now()
        mark.type = mark.LINE_STRIP
        mark.action = mark.ADD
        mark.scale.x = 0.1
        mark.color.a = 1.0
        mark.color.r = 1.0
        mark.color.g = 0.0
        mark.color.b = 0.0
        mark.pose.orientation.w = 1.0
        mark.pose.position.x = 0.0#poselist[0][0]
        mark.pose.position.y = 0.0#poselist[0][1]
        mark.pose.position.z = 0.0#poselist[0][2]

        mark.id = 0
        mark.lifetime = rospy.Duration(0)
        mark.points = [Point(x=x, y=y, z=z) for x, y, z in poselist]
        mark.points.append(Point(x=poselist[0][0], y=poselist[0][1], z=poselist[0][2]))

        self.marker_pub.publish(mark)
        
        
    def transform_btm(self):   
        
        stamp = self.pose.header.stamp  
        try:                                    # lookup_transform('target frame','source frame', time.stamp, rospy.Duration(0.5))
            transform_map_2_base_link = self.tfBuffer.lookup_transform('map','base_link', stamp,rospy.Duration(0.5))     # give goal in base link frame
            self.pose_in_map = tf2_geometry_msgs.do_transform_pose(self.pose, transform_map_2_base_link)   
        except:
            print('No transform found')

if __name__ == '__main__':
    node = path_tracker()
    node.main()