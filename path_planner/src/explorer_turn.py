#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from rospy import Service
from std_srvs.srv import Trigger


class ExplorerTurn():

    def __init__(self):

        #Init node
        rospy.init_node('explorer_turn')
        
        #subscribers
        self.map_sub = rospy.Subscriber('/map', PoseStamped, self.map_callback) # change to the topic that the map is published on
        
        #publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # To control the robots movement
        self.move = Twist()
        self.angle_speed = 0.8

        # Service to explore
        self.turn360_service = Service('/explore', Trigger, self.turn)
        rospy.spin()

    def turn(self):
        self.move.angular.z = self.angle_speed
        self.cmd_pub.publish(self.move)
        # self.move.angular.z = 0
        # self.cmd_pub.publish(self.move)


if __name__ == '__main__':
    explorer = ExplorerTurn()
