#!/usr/bin/env python3
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
import skfuzzy as fuzz
from skfuzzy import control as ctrl
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
        try:
            derivative = (error - self.previous_error) / dt
        except:
            derivative = 0
        self.previous_error = error
        # Calculate the output of the PID controller as a weighted sum of the error, the integral of the error, and the derivative of the error
        return self.kp * error + self.ki * self.error_sum + self.kd * derivative


class cmd_vel_to_motors:
    def __init__(self):
        rospy.init_node('cmd_vel_to_motors')
        print('cmd_vel_to_motors node initalized')
        # Initial sleep to allow roscore to start
        rospy.sleep(2)
        
        # Robot specific parameters
        self.ticks_per_rev = 3072
        self.wheel_radius = 0.04921
        self.base = 0.3       

        
        # Define the publiser to the /motor/duty_cycles topic
        self.duty_cycle_pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=1)
        
        # Define some variables that the twist callback will use
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Define left v and right v
        self.v_left_enco = 0.0
        self.v_right_enco = 0.0

        # Define the subscriber to the /cmd_vel topic
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.encoder_sub = rospy.Subscriber('/motor/encoders',Encoders,self.encoder_callback)
        
        # Deine the rate at which the node will run        
        self.rate = rospy.Rate(20)

        # Define the fuzzy input variables (error and error rate)
        self.error = ctrl.Antecedent(np.arange(-10, 11, 1), 'error')
        self.error_rate = ctrl.Antecedent(np.arange(-10, 11, 1), 'error_rate')

        # Define the fuzzy output variables (left wheel and right wheel speeds)
        self.left_wheel = ctrl.Consequent(np.arange(-100, 101, 1), 'left_wheel')
        self.right_wheel = ctrl.Consequent(np.arange(-100, 101, 1), 'right_wheel')

        # Define a DutyCycles message 
        self.duty_cycle_msg = DutyCycles()
        self.duty_cycle_msg.duty_cycle_left = 0.0
        self.duty_cycle_msg.duty_cycle_right = 0.0

        self.run()
    
    def encoder_callback(self,msg):
        time = rospy.get_time()
        f = 20
        # tics per second
        tic_spd_left = msg.delta_encoder_left / msg.delta_time_left               #(time - self.time_enco)
        tic_spd_right = msg.delta_encoder_right / msg.delta_time_right            #/ (time - self.time_enco)
        self.time_enco = time
        # m/tics 
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
        #self.integral_right = self.integral_right + (self.v_right_des - self.v_right_enco)
        #self.integral_left = self.integral_left + (self.v_left_des - self.v_lef v_left = 2*pi*f*msg.delta_time_left / self.ticks_per_rev
        v_right = 2*pi*f*msg.delta_time_right / self.ticks_per_rev
        

        


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

            v_right, v_left = self.PID()
            
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

    def fuzzy_PID(self):
        v_left_desiered, v_right_desired = self.transform_v_omega_to_v_left_v_right(self.linear_velocity,self.angular_velocity)
        err_right = v_right_desired-self.v_right_enco
        err_left = v_left_desiered-self.v_left_enco

        v_actual,omega_actual = self.transform_v_left_v_right_to_v_omega(self.v_left_enco,self.v_right_enco)
        
        #Define the membership functions for the "left_wheel" and "right_wheel" output variables
        self.left_wheel['fast_backward'] = fuzz.trimf(self.left_wheel.universe, [-100, -100, -50])
        self.left_wheel['slow_backward'] = fuzz.trimf(self.left_wheel.universe, [-100, -50, 0])
        self.left_wheel['stop'] = fuzz.trimf(self.left_wheel.universe, [-50, 0, 50])
        self.left_wheel['slow_forward'] = fuzz.trimf(self.left_wheel.universe, [0, 50, 100])
        self.left_wheel['fast_forward'] = fuzz.trimf(self.left_wheel.universe, [50, 100, 100])

        self.right_wheel['fast_backward'] = fuzz.trimf(self.right_wheel.universe, [-100, -100, -50])
        self.right_wheel['slow_backward'] = fuzz.trimf(self.right_wheel.universe, [-100, -50, 0])
        self.right_wheel['stop'] = fuzz.trimf(self.right_wheel.universe, [-50, 0, 50])
        self.right_wheel['slow_forward'] = fuzz.trimf(self.right_wheel.universe, [0, 50, 100])
        self.right_wheel['fast_forward'] = fuzz.trimf(self.right_wheel.universe, [50, 100, 100])


        # Define the fuzzy rules
        rule1 = ctrl.Rule(self.error['negative_large'] & self.error_rate['negative_large'], (self.left_wheel['fast_backward'], self.right_wheel['fast_backward']))
        rule2 = ctrl.Rule(self.error['negative_large'] & self.error_rate['negative_small'], (self.left_wheel['slow_backward'], self.right_wheel['fast_backward']))
        rule3 = ctrl.Rule(self.error['negative_large'] & self.error_rate['zero'], (self.left_wheel['stop'], self.right_wheel['fast_backward']))
        rule4 = ctrl.Rule(self.error['negative_large'] & self.error_rate['positive_small'], (self.left_wheel['stop'], self.right_wheel['slow_backward']))
        rule5 = ctrl.Rule(self.error['negative_large'] & self.error_rate['positive_large'], (self.left_wheel['stop'], self.right_wheel['fast_backward']))

        self.controller = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, ...])
        self.fuzzy_pid = ctrl.ControlSystemSimulation(self.controller)

        # Set the left and right wheel speeds
        left_wheel_speed = self.fuzzy_pid.output['left_wheel']
        right_wheel_speed = self.fuzzy_pid.output['right_wheel']

        # Update the last error
        self.last_error = error
if __name__ == '__main__':
    new_object = cmd_vel_to_motors()