
#!/usr/bin/env python
import rospy
from robp_msgs.msg import DutyCycles
from robp_msgs.msg import Encoders
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from math import pi
import numpy as np
import scipy
import json
import os
from collections import defaultdict


class cmd_vel_to_motors:
    def __init__(self):
        rospy.init_node('cmd_vel_to_motors')
        
        # Initial sleep to allow roscore to start
        rospy.sleep(2)
        
        # Robot specific parameters
        self.ticks_per_rev = 3072
        self.wheel_radius = 0.04921
        self.base = 0.3 

        
        # Define the publiser to the /motor/duty_cycles topic
        self.duty_cycle_pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=1)
        
        # Define the subscriber to the /cmd_vel topic
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Deine the rate at which the node will run        
        self.rate = rospy.Rate(20)
    
        # Define some variables that the twist callback will use
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
    
    
        # Define a DutyCycles message 
        self.duty_cycle_msg = DutyCycles()
        self.duty_cycle_msg.duty_cycle_left = 0.0
        self.duty_cycle_msg.duty_cycle_right = 0.0

    
    
        self.run()
    
    def cmd_vel_callback(self, msg):
        """
        This node subscribes to the /cmd_vel topic and converts the linear and angular velocity
        It then updates the internal variables that are used to publish the duty cycle message
        """
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    
    def run(self):
        while not rospy.is_shutdown():
            # Convert the desired linear and angular velocity to the desired wheel velocities
            v_left, v_right = self.transform_v_omega_to_v_left_v_right(self.linear_velocity, self.angular_velocity)
            v_right = v_right + 0.01 * np.sign(v_right)
            # Limit the duty cycle to be between -1 and 1
            if np.abs(v_left) > 1 or np.abs(v_right) > 1:
                v_left = v_left / np.abs(v_left)
                v_right = v_right / np.abs(v_right)

            # Update the duty cycle message
            self.duty_cycle_msg.duty_cycle_left = v_left
            self.duty_cycle_msg.duty_cycle_right = v_right
            
            # Publish the duty cycle message
            self.duty_cycle_pub.publish(self.duty_cycle_msg)
            
            
            self.rate.sleep()

    def transform_v_omega_to_v_left_v_right(self, v, omega):
        """Transforms the desired linear and angular velocity to the desired wheel velocities, angular velocity is in rad/s, speed in m/s"""
        v_right = (2*v + self.base * omega)/2
        v_left = (2*v - self.base * omega)/2
        return v_left, v_right


    def transform_v_left_v_right_to_v_omega(self, v_left, v_right):
        """Transforms the desired wheel velocities to the desired linear and angular velocity, angular velocity is in rad/s, speed in m/s"""
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self.base
        return v, omega



if __name__ == '__main__':
    new_object = cmd_vel_to_motors()