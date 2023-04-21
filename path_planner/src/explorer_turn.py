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
        self.i = 0
        self.angle_speeds = [0.6] * 7 + [0.0] * 11

        # Service to explore
        self.turn360_service = Service('/explore', Trigger, self.turn)
        rospy.spin()

    def turn(self, _):
        self.move.angular.z = self.angle_speeds[self.i]
        self.i = (self.i + 1) % len(self.angle_speeds)
        self.cmd_pub.publish(self.move)
        # self.move.angular.z = 0
        # self.cmd_pub.publish(self.move)
        return TriggerResponse(True, "Turning")


if __name__ == '__main__':
    explorer = ExplorerTurn()
