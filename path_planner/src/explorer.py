#!/usr/bin/env python3

import rospy
import random
import math
from geometry_msgs.msg import PointStamped


class explorer:
    def __init__(self):
        """ Put the node name here, and description of the node"""
        rospy.init_node('explorer')

        # Subscribers 
        self.sub_topic = rospy.Subscriber("topic", type, self.callback_topic)
        
        # Publisher
        self.message_pub = rospy.Publisher("topic", type, queue_size=10)

        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 
        
        # Paramethers HERE
        
    ###### All your callbacks here ######

    