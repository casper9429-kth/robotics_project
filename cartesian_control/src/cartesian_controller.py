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
# def of used msgs types



class cartesian_controller:
    def __init__(self):
        self.autotune = False
        # init a node
        rospy.init_node('cartesian_controller', anonymous = True)

        self.pid_path = os.path.join(os.path.expanduser('~'), 'dd2419_ws', 'pid.json')
        # init a sleep rate
        self.r = rospy.Rate(20)

        # init a publisher to the /motor/duty_cycles topic
        # used to publish duty cycle values from the pid
        self.duty_cycle_pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=1)

        # init a subscriber to the /motor/encoders topic
        # used to read encoder values 
        self.encoders_sub = rospy.Subscriber('/motor/encoders', Encoders, self.callback_encoders)

        # init a subscriber to the /cmd_vel topic
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.callback_twist)

        # ticks per rev: 360
        # Wheel radius (r): 0.0352 m
        # Base (b): 0.23 m
        self.ticks_per_rev = 360
        self.wheel_radius = 0.0352
        self.base = 0.23

        # Encoder var, will be updated by callback encoder
        self.v_left_enco = 0.0
        self.v_right_enco = 0.0

 
        # Disired velocity, will be updated by callback twist
        self.v_des = 0.0
        self.omega_des = 0.0

        # Desired wheel velocity, will be updated in run main loop
        self.v_left_des = 0.0
        self.v_right_des = 0.0


        # Time since last twist message
        self.time_twist = rospy.get_time()

        try:        
            with open(self.pid_path,"r") as json_file:
                pid = json.load(json_file)
                self.Kp_v_right = pid['Kp_v_right']
                self.Ki_v_right = pid['Ki_v_right']
                self.Kd_v_right = pid['Kd_v_right']
                self.Kp_v_left = pid['Kp_v_left']
                self.Ki_v_left = pid['Ki_v_left']
                self.Kd_v_left = pid['Kd_v_left']
                rospy.loginfo("Cartesian Controller: Loaded pid values from file, Kp_v_right: %f, Ki_v_right: %f, Kd_v_right: %f, Kp_v_left: %f, Ki_v_left: %f, Kd_v_left: %f", self.Kp_v_right, self.Ki_v_right, self.Kd_v_right, self.Kp_v_left, self.Ki_v_left, self.Kd_v_left)
        except:
            # Pid
            self.Kp_v_right = 1.0#0.95
            self.Ki_v_right = 0.0#0.7
            self.Kd_v_right = 0.0#0.0
            self.Kp_v_left =  1.0#1.2
            self.Ki_v_left =  0.0#0.8
            self.Kd_v_left =  0.0#0.0
            rospy.loginfo("Cartesian Controller: No pid file found, using default values, Kp_v_right: %f, Ki_v_right: %f, Kd_v_right: %f, Kp_v_left: %f, Ki_v_left: %f, Kd_v_left: %f", self.Kp_v_right, self.Ki_v_right, self.Kd_v_right, self.Kp_v_left, self.Ki_v_left, self.Kd_v_left)


        

        self.run()


    def run(self,): 
        """
        Main loop of the node
        * transforms desired velocity to desired wheel velocity
        * publishes duty cycle message to /motor/duty_cycles with wheel velocities
        * gets wheel velocities from encoder feedback and PI control
        """

        # check if desired speed is to high and warns user
        if self.v_left_des > 1.0 or self.v_right_des > 1.0:
            rospy.loginfo("Cartesian Controller: Desired speed for wheel is too high, max is 0.5 m/s for realiable operation")

        # Create duty cycle message object
        # create inital duty cycle message object, will be used to publish to duty cycle topic
        duty_cycle_msg = DutyCycles()
        duty_cycle_msg.duty_cycle_left = 0.0
        duty_cycle_msg.duty_cycle_right = 0.0


        
        while not rospy.is_shutdown(): # insted of while True, makes sure ros dies when supposed to

            
            
            
            # Edge case tp stop robot and not get oscillations from PI control. Used when no new twist message is received or when desired speed is 0 
            if ((rospy.get_time() - self.time_twist  ) > 0.5):
                duty_cycle_msg.duty_cycle_left = 0.0
                duty_cycle_msg.duty_cycle_right = 0.0
                self.duty_cycle_pub.publish(duty_cycle_msg)
                self.integral_left = 0.0 # reset integral in pi control 
                self.integral_right = 0.0 # reset integral in pi control
                self.derivative_left = []
                self.derivative_right = []        
                self.r.sleep()
                continue

            
            
            # Calculate desired wheel velocity
            self.v_left_des,self.v_right_des = self.transform_v_omega_to_v_left_v_right(self.v_des, self.omega_des)

            # calulate omega 
            v_enco,omega_enco = self.transform_v_left_v_right_to_v_omega(self.v_left_enco, self.v_right_enco)



            if self.v_des == 0 and self.omega_des == 0 and self.v_left_enco < 0.05 and self.v_right_enco < 0.05:
                duty_cycle_msg.duty_cycle_left = 0
                duty_cycle_msg.duty_cycle_right = 0
                self.integral_left = 0.0
                self.integral_right = 0.0
                self.derivative_left = []
                self.derivative_right = []

            else:
                duty_cycle_msg.duty_cycle_left = self.PID_left(self.v_left_des, self.v_left_enco)
                duty_cycle_msg.duty_cycle_right = self.PID_right(self.v_right_des, self.v_right_enco)



            self.duty_cycle_pub.publish(duty_cycle_msg)
        
        

            # sleep
            self.r.sleep()
            



    def transform_v_omega_to_v_left_v_right(self, v, omega):
        """Transforms the desired linear and angular velocity to the desired wheel velocities, angular velocity is in rad/s, speed in m/s"""
        # v = (v_left + v_right) / 2
        # omega = (v_right - v_left) / 2b
        # v_right = v + b * omega 
        # v_left = v - b * omega
        
        v_right = v + self.base * omega
        v_left = v - self.base * omega
        return v_left, v_right
            


    
    def transform_v_left_v_right_to_v_omega(self, v_left, v_right):
        """Transform v_left and v_right to v and omega"""
        # v = (v_left + v_right) / 2
        # omega = (v_right - v_left) / 2b
        
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / (2*self.base)
        return v, omega


        
                            
    # define callback functions
    def callback_encoders(self,data):
        """calc wheel speed from encoders"""
        # Encoder data

        # tics per second
        tic_spd_left = data.delta_encoder_left / data.delta_time_left
        tic_spd_right = data.delta_encoder_right / data.delta_time_right
        
        # m/tics 
        self.wheel_radius
        circ = 2*self.wheel_radius*pi
        tics = self.ticks_per_rev
        m_tic = circ/tics 
        
        # tic/s * m/tic = m/s
        v_left = tic_spd_left * m_tic*1000
        v_right = tic_spd_right * m_tic*1000 

        # save in internal var
        self.v_right_enco = v_right
        self.v_left_enco = v_left

        
    def callback_twist(self,data):        
        # set time stamp
        self.time_twist = rospy.get_time()
        
        # save in internal var
        self.v_des = data.linear.x
        self.omega_des = data.angular.z
        
        



if __name__ == '__main__':
    new_obj = cartesian_controller()
    
    