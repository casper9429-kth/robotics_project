#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped, Twist


class explorer_turn():

    def __init__(self):

        #Init node
        rospy.init_node('explorer_turn')

        
        #subscribers
        self.map_sub = rospy.Subscriber('/map', PoseStamped, self.map_callback) # change to the topic that the map is published on

        
        #publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # To control the robots movement
        self.move = Twist()
        self.max_angle = 0.1
        self.angle_speed = 0.8
        self.in_goal_tolerance = 0.02
        
        self.turn360()


    def turn360(self):

        angle_rotated = 0
        self.move.angular.z = self.angle_speed
        
        while angle_rotated < math.pi*2:
            self.cmd_pub.publish(self.move)
            rospy.sleep(0.1)
            angle_rotated += abs(self.move.angular.z) * 0.1 # need to be tuned to the real robot
            # print(angle_rotated)
            
        self.move.angular.z = 0
        self.cmd_pub.publish(self.move)


if __name__ == '__main__':
    explorer = explorer_turn()
    rospy.spin()