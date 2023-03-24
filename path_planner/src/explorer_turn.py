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
<<<<<<< HEAD
        self.map_sub = rospy.Subscriber('/map', PoseStamped, self.map_callback) # change to the topic that the map is published on
=======
        # self.map_sub = rospy.Subscriber('/map', PoseStamped, self.map_callback) # change to the topic that the map is published on

>>>>>>> 94bf2a08e14e8070f64d8afc9921f3984022c241
        
        #publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # To control the robots movement
        self.move = Twist()
<<<<<<< HEAD
        self.angle_speed = 0.8

        # Service to explore
        self.turn360_service = Service('/explore', Trigger, self.turn)
        rospy.spin()

    def turn(self):
        self.move.angular.z = self.angle_speed
=======
        self.max_angle = 0.1
        self.angle_speed = math.pi/4 # need to be tuned to the real robot
        self.in_goal_tolerance = 0.02
        self.rate = rospy.Rate(10)
        for i in range(50):
            self.rate.sleep()
        self.turn360()
        
     
    def turn360(self):

        angle_rotated = 0
        self.move.angular.z = self.angle_speed
        
        while angle_rotated < math.pi*4:
            self.cmd_pub.publish(self.move)
            rospy.sleep(0.1)
            angle_rotated += abs(self.move.angular.z) * 0.1 # need to be tuned to the real robot
            print(angle_rotated)
            
        self.move.angular.z = 0
>>>>>>> 94bf2a08e14e8070f64d8afc9921f3984022c241
        self.cmd_pub.publish(self.move)
        # self.move.angular.z = 0
        # self.cmd_pub.publish(self.move)


if __name__ == '__main__':
    explorer = ExplorerTurn()
