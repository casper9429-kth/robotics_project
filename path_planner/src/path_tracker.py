#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
import tf2_ros
import tf2_geometry_msgs

# Recives poses from path_planner and transforms them to the robot frame

class path_tracker:
    def __init__(self):
        rospy.init_node('path_tracker')
        print('path_tracker node initalized')
        self.path = []
        self.rate = rospy.Rate(10)
        # subscribers
        self.path_sub = rospy.Subscriber('/path', PoseStamped, self.path_callback)
        self.transform_sub = rospy.Subscriber('/transform', TransformStamped, self.transform_callback)
        self.robot_frame = 'base_link'
        print('Subscribers initalized')
        
        #publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # tf stuff
        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        print('Tf2 stuff initialized')

    def path_callback(self, msg):
        self.path.append(msg)

    def transform_callback(self, msg):
        pass

    def transform_to_robot_frame(self, pose):
        stamp = pose.header.stamp  
        transform = self.tfBuffer.lookup_transform(self.robot_frame,'map',stamp,rospy.Duration(0.5)) # The transform that relate map fram to base link frame
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        return transformed_pose

        

    def math(self,pose):
        turn =  3 * math.atan2(pose.point.y,pose.point.x)
        forward = 1 * math.hypot(pose.point.y,pose.point.x)
        return [forward,turn]

    def publish_twist(self, msg):
        message = Twist()
        message.linear.x = msg[0]
        message.angular.z = msg[1]
        self.cmd_pub.publish(message)


    def spin(self):
        if len(self.path) > 0:
            pose = self.path.pop(0)
            pose = self.transform_to_robot_frame(pose)
            twist = self.math(pose)
            self.publish_twist(twist)
        
        
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