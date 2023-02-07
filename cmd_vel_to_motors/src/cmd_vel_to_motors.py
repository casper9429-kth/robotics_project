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


class PIDController:
    # Initialize the PID controller with the proportional, integral, and derivative gains
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = 0.0
        self.previous_error = 0.0
    # Update the PID controller with the new error and dt (time elapsed since the last update)
    def update(self, error, dt):
        # Integrate the error over time
        self.error_sum += error * dt
        # Calculate the derivative of the error
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        # Calculate the output of the PID controller as a weighted sum of the error, the integral of the error, and the derivative of the error
        return self.kp * error + self.ki * self.error_sum + self.kd * derivative


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
        self.encoder_sub = rospy.Subscriber('/motor/encoders',Encoders,self.encoder_callback)
        
        # Deine the rate at which the node will run        
        self.rate = rospy.Rate(20)
    
        # Define some variables that the twist callback will use
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
    
    
        # Define a DutyCycles message 
        self.duty_cycle_msg = DutyCycles()
        self.duty_cycle_msg.duty_cycle_left = 0.0
        self.duty_cycle_msg.duty_cycle_right = 0.0

        # Integral variables
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.right_dt = 0.0
        self.left_dt = 0.0

        #PIDS
        self.pid_right = None
        self.pid_left = None

        self.run()
    
    def endocer_callback(self,msg):
        time = rospy.get_time()
        # tics per second
        tic_spd_left = msg.delta_encoder_left / msg.delta_time_left     #(time - self.time_enco)
        tic_spd_right = msg.delta_encoder_right / msg.delta_time_right            #/ (time - self.time_enco)
        self.time_enco = time
        # m/tics 
        self.wheel_radius
        circ = 2*self.wheel_radius*pi
        tics = self.ticks_per_rev
        m_tic = circ/tics 
        
        # tic/s * m/tic = m/s
        v_left = tic_spd_left * m_tic *1000
        v_right = tic_spd_right * m_tic *1000

        # save in internal var
        self.v_right_enco = v_right 
        self.v_left_enco = v_left
        # save dt for each wheel
        self.left_dt = msg.delta_time_left
        self.right_dt = msg.delta_time_right
        # append integral
        self.integral_right = self.integral_right + (self.v_right_des - self.v_right_enco)
        self.integral_left = self.integral_left + (self.v_left_des - self.v_left_enco)

        


    def cmd_vel_callback(self, msg):
        """
        This node subscribes to the /cmd_vel topic and converts the linear and angular velocity
        It then updates the internal variables that are used to publish the duty cycle message
        """
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    
    def run(self):
        
        self.initalize_PID()
        
        while not rospy.is_shutdown():
            # Convert the desired linear and angular velocity to the desired wheel velocities
           #v_left, v_right = self.transform_v_omega_to_v_left_v_right(self.linear_velocity, self.angular_velocity)

            v_right ,v_left = self.PID()
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
        v_right = v + self.base * omega
        v_left = v - self.base * omega
        return v_left, v_right



    def transform_v_left_v_right_to_v_omega(self, v_left, v_right):
        """Transforms the desired wheel velocities to the desired linear and angular velocity, angular velocity is in rad/s, speed in m/s"""
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self.base
        return v, omega

    # Initializes the PID class for each wheel
    def initalize_PID(self):
        # K_P
        K_P_r = 0.1
        K_P_l = 0.1  

        #K_I
        K_I_r = 0.01
        K_I_l = 0.01

        #K_D
        K_D_r = 0.005
        K_D_l = 0.005

        self.pid_right = PIDController(K_P_r,K_I_r,K_D_r)
        self.pid_left = PIDController(K_P_l,K_I_l,K_D_l)

    # returns PID value for each controller
    def PID(self):
        v_left_desiered, v_right_desired = self.transform_v_omega_to_v_left_v_right(self.linear_velocity,self.angular_velocity)
        err_right = v_right_desired-self.v_right_enco
        err_left = v_left_desiered-self.v_left_enco
        
        # PID code for each controller
        

        v_right = self.pid_right.update(err_right,self.right_dt)
        v_left = self.pid_left.update(err_left,self.left_dt)
        
        return v_right,v_left

if __name__ == '__main__':
    new_object = cmd_vel_to_motors()