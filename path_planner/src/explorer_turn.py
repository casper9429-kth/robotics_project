#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from rospy import Service
from std_srvs.srv import Trigger, TriggerResponse


class ExplorerTurn():

    def __init__(self):

        #Init node
        rospy.init_node('explorer_turn')
        
        #publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # To control the robots movement
        self.move = Twist()
        self.angle_speed = 0.4

        # Service to explore
        self.turn360_service = Service('/explore', Trigger, self.turn)
        rospy.spin()

    def turn(self, _):
        self.move.angular.z = self.angle_speed
        self.cmd_pub.publish(self.move)
        # self.move.angular.z = 0
        # self.cmd_pub.publish(self.move)
        return TriggerResponse(True, "Turning")


if __name__ == '__main__':
    explorer = ExplorerTurn()
