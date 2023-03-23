#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped, Twist


class explorer():
    def __init__(self):
        
        # Init node
        rospy.init_node('explorer')

        # subscribers
        self.map_sub = rospy.Subscriber('/map', PoseStamped, self.map_callback) # change to the topic that the map is published on