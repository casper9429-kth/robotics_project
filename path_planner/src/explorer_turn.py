#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool


class explorer_turn():

    def __init__(self):

        #Init node
        rospy.init_node('explorer_turn')

        # subscribers
        self.bool_sub = rospy.Subscriber('/bool', Bool, self.bool_callback)
        
        #publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # To control the robots movement
        self.move = Twist()
        self.max_angle = 0.1
        self.angle_speed = math.pi/4
        self.in_goal_tolerance = 0.02
        self.rate = rospy.Rate(10)
        for i in range(50):
            self.rate.sleep()


    def bool_callback(self, msg: Bool):
        self.switch = msg.data
        if msg.data:
            self.turn360()
        else:
            self.move.angular.z = 0
            self.cmd_pub.publish(self.move)

    def turn360(self):

        angle_rotated = 0
        self.move.angular.z = self.angle_speed
        
        while angle_rotated < math.pi*4 and self.switch:
            self.cmd_pub.publish(self.move)
            rospy.sleep(0.1)
            angle_rotated += abs(self.move.angular.z) * 0.1 # need to be tuned to the real robot
            print(angle_rotated)
        
        self.switch = False    
        self.move.angular.z = 0
        self.cmd_pub.publish(self.move)


if __name__ == '__main__':
    explorer = explorer_turn()
    rospy.spin()