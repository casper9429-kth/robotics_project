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

class autotune:
    def __init__(self):
        self.autotune = False
        # init a node
        rospy.init_node('autotune', anonymous = True)

        self.pid_path = os.path.join(os.path.expanduser('~'), 'dd2419_ws', 'pid.json')
        # init a sleep rate
        self.r = rospy.Rate(10)

        # init a publisher to the /motor/duty_cycles topic
        # used to publish duty cycle values from the pid
        self.duty_cycle_pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=1)

        # init a subscriber to the /motor/encoders topic
        # used to read encoder values 
        self.encoders_sub = rospy.Subscriber('/motor/encoders', Encoders, self.callback_encoders)



        # ticks per rev: 360
        # Wheel radius (r): 0.0352 m
        # Base (b): 0.23 m
        self.ticks_per_rev = 360
        self.wheel_radius = 0.0352
        self.base = 0.23

        # Encoder var, will be updated by callback encoder
        self.v_left_enco = 0.0
        self.v_right_enco = 0.0

        # Integral var, will be updated by PI control
        self.integral_left = 0.0
        self.integral_right = 0.0
        # Derivative var, will be updated by PID control
        self.derivative_left = []
        self.derivative_right = []
        
        self.error_list_v_right = []
        self.integral_v_right = 0
        self.derivative_v_right = []
        
        self.error_list_v_left = []
        self.integral_v_left = 0
        self.derivative_v_left = []
 
        # Disired velocity, will be updated by callback twist
        self.v_des = 0.5
        self.omega_des = 0.0

        # Desired wheel velocity, will be updated in run main loop
        self.v_left_des = 0.0
        self.v_right_des = 0.0


        # Time since last twist message
        #self.time_twist = rospy.get_time()

        try:        
            with open(self.pid_path,"r") as json_file:
                pid = json.load(json_file)
                self.Kp_v_right = pid['Kp_v_right']
                self.Ki_v_right = pid['Ki_v_right']
                self.Kd_v_right = pid['Kd_v_right']
                self.Kp_v_left = pid['Kp_v_left']
                self.Ki_v_left = pid['Ki_v_left']
                self.Kd_v_left = pid['Kd_v_left']
                rospy.loginfo("Loaded pid values from file, Kp_v_right: %f, Ki_v_right: %f, Kd_v_right: %f, Kp_v_left: %f, Ki_v_left: %f, Kd_v_left: %f", self.Kp_v_right, self.Ki_v_right, self.Kd_v_right, self.Kp_v_left, self.Ki_v_left, self.Kd_v_left)
        except:
            # Pid
            self.Kp_v_right = 1.0#0.95
            self.Ki_v_right = 0.0#0.7
            self.Kd_v_right = 0.0#0.0
            self.Kp_v_left =  1.0#1.2
            self.Ki_v_left =  0.0#0.8
            self.Kd_v_left =  0.0#0.0
            rospy.loginfo("No pid file found, using default values, Kp_v_right: %f, Ki_v_right: %f, Kd_v_right: %f, Kp_v_left: %f, Ki_v_left: %f, Kd_v_left: %f", self.Kp_v_right, self.Ki_v_right, self.Kd_v_right, self.Kp_v_left, self.Ki_v_left, self.Kd_v_left)


        
        # Pid list
        self.PID_v_right_list = []
        self.PID_v_left_list = []
        self.PID_list = []
        
        # Error list
        self.error_list_v_left = []
        self.error_list_v_right = []
        self.error_list_omega = []
        self.error_list_v = []


        self.sample_size = 60
        if self.autotune:
            
            # Pre tune
            self.pre_tune(10)
            
            # Not worth it , takes to much time. But will explore more local min/max
            # Hard tune
            #p = [self.Kp_v_left, self.Ki_v_left, self.Kd_v_left, self.Kp_v_right, self.Ki_v_right, self.Kd_v_right]
            #self.twiddle(p,0.2,5,0.7,2)
            
            # Soft tune
            p = [self.Kp_v_left, self.Ki_v_left, self.Kd_v_left, self.Kp_v_right, self.Ki_v_right, self.Kd_v_right]
            self.twiddle(p,0.1,5)


            # Super soft tune
            p = [self.Kp_v_left, self.Ki_v_left, self.Kd_v_left, self.Kp_v_right, self.Ki_v_right, self.Kd_v_right]
            self.twiddle(p,0.1,3,0.05,0)

            
            
            # save pid values into json file
            pid = defaultdict()
            pid['Kp_v_right'] = self.Kp_v_right
            pid['Ki_v_right'] = self.Ki_v_right
            pid['Kd_v_right'] = self.Kd_v_right
            pid['Kp_v_left'] = self.Kp_v_left
            pid['Ki_v_left'] = self.Ki_v_left
            pid['Kd_v_left'] = self.Kd_v_left
            
            # open file at specified path and write pid values, if it exist it will be overwritten
            with open(self.pid_path, 'w') as outfile:
                json.dump(pid, outfile)
            print('saved pid values')
        else: 
            self.run()


    def run(self,iteration=10): 
        """
        Main loop of the node
        * transforms desired velocity to desired wheel velocity
        * publishes duty cycle message to /motor/duty_cycles with wheel velocities
        * gets wheel velocities from encoder feedback and PI control
        """

        # check if desired speed is to high and warns user
        if self.v_left_des > 0.5 or self.v_right_des > 0.5:
            rospy.loginfo("Desired speed for wheel is too high, max is 0.5 m/s for realiable operation")

        # Create duty cycle message object
        # create inital duty cycle message object, will be used to publish to duty cycle topic
        duty_cycle_msg = DutyCycles()
        duty_cycle_msg.duty_cycle_left = 0.0
        duty_cycle_msg.duty_cycle_right = 0.0


        # main loop
        t = 0#rospy.get_time()
        
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.derivative_left = []
        self.derivative_right = []        
        
        while not rospy.is_shutdown(): # insted of while True, makes sure ros dies when supposed to

            
            if t <5: # Sleep
                self.omega_des = 0.0#np.random.uniform(-0.3,0.3) 
                self.v_des = 0.5#np.random.uniform(0.1,0.5)
            
            t += 1                
            
            # Outerloop pid
            
            # Edge case tp stop robot and not get oscillations from PI control. Used when no new twist message is received or when desired speed is 0 
            #if ((rospy.get_time() - self.time_twist  ) > 0.5):
            #    duty_cycle_msg.duty_cycle_left = 0.0
            #    duty_cycle_msg.duty_cycle_right = 0.0
            #    self.duty_cycle_pub.publish(duty_cycle_msg)
            #    self.integral_left = 0.0 # reset integral in pi control 
            #    self.integral_right = 0.0 # reset integral in pi control
            #    self.derivative_left = []
            #    self.derivative_right = []        
            #    self.r.sleep()
            #    continue

            
            
            # Calculate desired wheel velocity
            self.v_left_des,self.v_right_des = self.transform_v_omega_to_v_left_v_right(self.v_des, self.omega_des)

            # calulate omega 
            v_enco,omega_enco = self.transform_v_left_v_right_to_v_omega(self.v_left_enco, self.v_right_enco)

            # Calculate omega error and append to list
            self.error_list_omega.append(self.omega_des - omega_enco)
            self.error_list_v.append(self.v_des - v_enco)


            self.error_list_v_left.append(self.v_left_des - self.v_left_enco)
            self.error_list_v_right.append(self.v_right_des - self.v_right_enco)
            
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
            


    def pre_tune(self,iteration=10): 
        """Code to do a pre tune using ziegler nichols method"""

        # check if desired speed is to high and warns user
        if self.v_left_des > 0.5 or self.v_right_des > 0.5:
            rospy.loginfo("Desired speed for wheel is too high, max is 0.5 m/s for realiable operation")

        # Create duty cycle message object
        # create inital duty cycle message object, will be used to publish to duty cycle topic
        duty_cycle_msg = DutyCycles()
        duty_cycle_msg.duty_cycle_left = 0.0
        duty_cycle_msg.duty_cycle_right = 0.0


        t = 0#rospy.get_time()
        
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.derivative_left = []
        self.derivative_right = []        
        
        while not rospy.is_shutdown(): # insted of while True, makes sure ros dies when supposed to



            if (len(self.PID_v_right_list) > iteration) and (len(self.PID_v_left_list) > iteration):
                omega_sorted = sorted(self.PID_list, key=lambda x: x[2])[0]
                
                
                self.Kp_v_left = omega_sorted[0][0]
                self.Ki_v_left = omega_sorted[0][1]
                self.Kd_v_left = omega_sorted[0][2]
                self.Kp_v_right = omega_sorted[1][0]
                self.Ki_v_right = omega_sorted[1][1]
                self.Kd_v_right = omega_sorted[1][2]
                
                rospy.loginfo("Kp_left: %s, Ki_left: %s, Kd_left: %s", self.Kp_v_left,self.Ki_v_left,self.Kd_v_left)
                rospy.loginfo("Kp_right: %s, Ki_right: %s, Kd_right: %s", self.Kp_v_right,self.Ki_v_right,self.Kd_v_right)

                
                return

            if t <5: # Sleep
                self.omega_des = 0.0#np.random.uniform(-0.3,0.3) 
                self.v_des = 0.0#np.random.uniform(0.1,0.5)

            elif t < 35: # Run forward
                self.omega_des = 0.0#np.random.uniform(-0.5,0.5)
                self.v_des = 0.5#np.random.uniform(0.1,0.5)

            elif t<40:
                self.omega_des = 0.0
                self.v_des = 0.0
            elif t < 70: # Run forward
                self.omega_des = 0.0#np.random.uniform(-0.5,0.5)
                self.v_des = -0.5#np.random.uniform(0.1,0.5)

            elif t<75:
                self.omega_des = 0.0
                self.v_des = 0.0

            else:
                # Update omega sqrt error
                # clean omega
                omega_sqrt_error = (np.sqrt(np.mean(np.square(self.error_list_omega))))
                v_sqrt_error = (np.sqrt(np.mean(np.square(self.error_list_v))))
                pid_1 = [self.Kp_v_left,self.Ki_v_left,self.Kd_v_left]
                pid_2 = [self.Kp_v_right,self.Ki_v_right,self.Kd_v_right]
                self.error_list_omega = []
                self.error_list_v = []
                error_tot = np.sqrt((omega_sqrt_error**2 + v_sqrt_error**2)/2)
                self.PID_list.append([pid_1,pid_2,error_tot])
                rospy.loginfo("PID error v: %f, omega: %f, total: %f", v_sqrt_error,omega_sqrt_error,error_tot)
                
                self.auto_tune_v_left(0)
                self.auto_tune_v_right(0)
                
                t = -1
            
            t += 1                

            
            
            # Calculate desired wheel velocity
            self.v_left_des,self.v_right_des = self.transform_v_omega_to_v_left_v_right(self.v_des, self.omega_des)

            # calulate omega 
            v_enco,omega_enco = self.transform_v_left_v_right_to_v_omega(self.v_left_enco, self.v_right_enco)

            # Calculate omega error and append to list
            self.error_list_omega.append(self.omega_des - omega_enco)
            self.error_list_v.append(self.v_des - v_enco)


            self.error_list_v_left.append(self.v_left_des - self.v_left_enco)
            self.error_list_v_right.append(self.v_right_des - self.v_right_enco)
            
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
            

    def twiddle_run(self,p):
        """
        This function is run by the twiddle algorithm to find the best PID parameters
        """

        p1 = p[0:3]
        p2 = p[3:6]

        # check if desired speed is to high and warns user
        if self.v_left_des > 0.5 or self.v_right_des > 0.5:
            rospy.loginfo("Desired speed for wheel is too high, max is 0.5 m/s for realiable operation")

        # Create duty cycle message object
        # create inital duty cycle message object, will be used to publish to duty cycle topic
        duty_cycle_msg = DutyCycles()
        duty_cycle_msg.duty_cycle_left = 0.0
        duty_cycle_msg.duty_cycle_right = 0.0

        # Reset pid
        self.derivative_left = []
        self.derivative_right = []
        self.integral_left = 0
        self.integral_right = 0
        error_list_v_left = []
        error_list_v_right = []


        # main loop
        t = 0
        
        while not rospy.is_shutdown(): # insted of while True, makes sure ros dies when supposed to




            if t <5: # Sleep
                self.omega_des = 0.0#np.random.uniform(-0.3,0.3) 
                self.v_des = 0.0#np.random.uniform(0.1,0.5)

            elif t < 45: # Run forward
                self.omega_des = 0#np.random.uniform(-0.5,0.5)
                self.v_des = 0.5#np.random.uniform(0.1,0.5)

            elif t<50:
                self.omega_des = 0.0
                self.v_des = 0.0
            elif t < 90: # Run forward
                self.omega_des = 0#np.random.uniform(-0.5,0.5)
                self.v_des = -0.5#np.random.uniform(0.1,0.5)

            elif t<95:
                self.omega_des = 0.0
                self.v_des = 0.0

            else:
                # calc sqrt mean error
                
                omega_sqrt_error = 2*(np.sqrt(np.mean(np.square(self.error_list_omega))))
                v_sqrt_error = (np.sqrt(np.mean(np.square(self.error_list_v))))
                pid_1 = [self.Kp_v_left,self.Ki_v_left,self.Kd_v_left]
                pid_2 = [self.Kp_v_right,self.Ki_v_right,self.Kd_v_right]
                self.error_list_omega = []
                self.error_list_v = []
                error_tot = np.sqrt((omega_sqrt_error**2 + v_sqrt_error**2)/2)
                
                
                return error_tot
            
            t += 1                

            # Outerloop pid
            
            
            
            
            # Calculate desired wheel velocity
            self.v_left_des,self.v_right_des = self.transform_v_omega_to_v_left_v_right(self.v_des, self.omega_des)

            error_list_v_left.append((self.v_left_des - self.v_left_enco)**2)
            error_list_v_right.append((self.v_right_des - self.v_right_enco)**2)
            
            # calulate omega 
            v_enco,omega_enco = self.transform_v_left_v_right_to_v_omega(self.v_left_enco, self.v_right_enco)

            # Calculate omega error and append to list
            self.error_list_omega.append(self.omega_des - omega_enco)
            self.error_list_v.append(self.v_des - v_enco)

            
            
            
            if self.v_des == 0 and self.omega_des == 0 and self.v_left_enco < 0.05 and self.v_right_enco < 0.05:
                duty_cycle_msg.duty_cycle_left = 0
                duty_cycle_msg.duty_cycle_right = 0
                self.integral_left = 0.0
                self.integral_right = 0.0
                self.derivative_left = []
                self.derivative_right = []

            else:
                duty_cycle_msg.duty_cycle_left = self.PID_left(self.v_left_des, self.v_left_enco,p1)
                duty_cycle_msg.duty_cycle_right = self.PID_right(self.v_right_des, self.v_right_enco,p2)

            self.duty_cycle_pub.publish(duty_cycle_msg)
        
        

            # sleep
            self.r.sleep()    
        
    def twiddle(self, p, tol=0.1, iter_out = 10,dp_val = 0.1,aggresive = 0):
        """
        Implements the twiddle algorithm for tuning the PID controllers, optimes for indirect control of the robot omega and v
        aggresive = 0: low aggresive, 1: medium aggresive, 2: high aggresive
        dp_val = 0.1: initial value for dp. Higher value will make the algorithm more aggresive
        default values will make a semi fine tune of the PID controllers
        """
        dp = [dp_val for i in range(0,6)]#[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

        if aggresive == 0:
            low_c = 0.95
            high_c = 1.05
        elif aggresive == 1:
            low_c = 0.9
            high_c = 1.1
        elif aggresive == 2:
            low_c = 0.8
            high_c = 1.2


        best_error = self.twiddle_run(p)
        best_p = p
        count = -1
        while np.mean(dp)*3 > tol and count < iter_out:
            count += 1
            for i in range(len(p)):
                p[i] += dp[i]
                error = self.twiddle_run(p)
                rospy.loginfo("twiddle error: %f", error)

                if error < best_error:
                    best_error = error
                    best_p = p
                    dp[i] *= high_c
                    rospy.loginfo("twiddle for state 1 improved")
                else:
                    p[i] -= 2 * dp[i]
                    error = self.twiddle_run(p)
                    rospy.loginfo("twiddle error 1: %f", error)

                    if error < best_error:
                        best_error = error
                        best_p = p
                        dp[i] *= high_c
                    else:
                        p[i] += dp[i]
                        dp[i] *= low_c


        p = best_p
        self.Kp_v_left = p[0]
        self.Ki_v_left = p[1]
        self.Kd_v_left = p[2]
        self.Kp_v_right = p[3]
        self.Ki_v_right = p[4]
        self.Kd_v_right = p[5]
        rospy.loginfo("Twiddle finish, Kp1: %f, Ki1: %f, Kd1: %f, Kp2: %f, Ki2: %f, Kd2: %f", self.Kp_v_left,self.Ki_v_left,self.Kd_v_left,self.Kp_v_right,self.Ki_v_right,self.Kd_v_right)
        rospy.loginfo("Twiddle finish, best error: %f", best_error)
        return 

    def auto_tune_v_left(self,error):
        """Continously tune v_left PID using Ziegler-Nichols method, used by Ziegler-Nichols method"""
        # Calculate error for each sample
        # Check if enough samples have been collected
        if len(self.error_list_v_left) < self.sample_size:
            return 
        # Calculate the oscillation period
        peak_indices = scipy.signal.find_peaks(self.error_list_v_left)[0]
        if len(peak_indices) > 1:
            period = peak_indices[-1] - peak_indices[-2]
        else:
            self.error_list_v_left = []
            return
        # Check if the period is valid, if not, clear the error list and return
        if period < 2 or period > self.sample_size/2:
            self.error_list_v_left = []
            return
        error_pow = np.sqrt(sum([x**2 for x in self.error_list_v_left]))/len(self.error_list_v_left)
        self.PID_v_left_list.append([self.Kp_v_left, self.Ki_v_left, self.Kd_v_left,error_pow])

        #Calculate Ziegler-Nichols tuning parameters
        Ku = 4 * max(self.error_list_v_left) / (np.pi * period)
        Pu = period * self.r.sleep_dur.to_sec()
        self.Kp_v_left = 0.6 * Ku
        self.Ki_v_left = 2 * self.Kp_v_left / Pu
        self.Kd_v_left = self.Kp_v_left * Pu / 8
        # Reset error list and integral
        print("Old v_left error sum: {}".format(error_pow)) 
        # Replace with more efficent one liner       
        self.error_list_v_left = []
        self.integral_left = 0
        self.derivative_left = []
        # Print new tuning parameters

        print("New v_left PID tuning parameters: Kp = {}, Ki = {}, Kd = {}".format(self.Kp_v_left, self.Ki_v_left, self.Kd_v_left))

    def auto_tune_v_right(self,error):
        """Auto tune v_right PID using Ziegler-Nichols method"""
        # Calculate error for each sample
        if len(self.error_list_v_right) < self.sample_size:
            return 
        # Calculate the oscillation period
        peak_indices = scipy.signal.find_peaks(self.error_list_v_right)[0]
        if len(peak_indices) > 1:
            period = peak_indices[-1] - peak_indices[-2]
        else:
            self.error_list_v_right = []
            return
        # Check if the period is valid, if not, clear the error list and return
        if period < 2 or period > self.sample_size/2:
            self.error_list_v_right = []
            return
        error_pow = np.sqrt(sum([x**2 for x in self.error_list_v_right]))/len(self.error_list_v_right)
        self.PID_v_right_list.append([self.Kp_v_right, self.Ki_v_right, self.Kd_v_right,error_pow])

        #Calculate Ziegler-Nichols tuning parameters
        Ku = 4 * max(self.error_list_v_right) / (np.pi * period)
        Pu = period * self.r.sleep_dur.to_sec()
        self.Kp_v_right = 0.6 * Ku
        self.Ki_v_right = 2 * self.Kp_v_right / Pu
        self.Kd_v_right = self.Kp_v_right * Pu / 8
        # Reset error list and integral
        print("Old v_right error sum: {}".format(error_pow)) 
        # Replace with more efficent one liner       
        self.error_list_v_right = []
        self.integral_right = 0
        self.derivative_right = []
        # Print new tuning parameters

        print("New v_right PID tuning parameters: Kp = {}, Ki = {}, Kd = {}".format(self.Kp_v_right, self.Ki_v_right, self.Kd_v_right))
    def transform_v_omega_to_v_left_v_right(self, v, omega):
        """Transforms the desired linear and angular velocity to the desired wheel velocities, angular velocity is in rad/s, speed in m/s"""
        v_right = v + self.base * omega
        v_left = v - self.base * omega
        return v_left, v_right
            
    def PID_left(self,des_spd,cur_spd,p=None):
        """PID controller for PI_left extended by derivative term""" 
        # PID param
        if p == None:
            K_p = self.Kp_v_left #1.2
            K_i = self.Ki_v_left #0.8
            K_d = self.Kd_v_left #0.0
        else:
            K_p = p[0]
            K_i = p[1]
            K_d = p[2]


        # Integral
        self.integral_left = self.integral_left + (des_spd - cur_spd)
        self.derivative_left.append(des_spd - cur_spd)

        # Derivative
        if len(self.derivative_left) >= 2:
            derivative_left = (self.derivative_left[-1] - self.derivative_left[-2]) / self.r.sleep_dur.to_sec()
        else:
            derivative_left = 0

        # Y 
        y =  K_p * (des_spd - cur_spd) + K_i * self.integral_left + K_d * derivative_left

        # overload protection
        if y > 1:
            y = 1
        if y < -1:
            y = -1

        return y
    


    
    def PID_right(self,des_spd,cur_spd,p=None):
        """PID controller for PI_right extended by derivative term""" 
        # PID param
        if p == None:
            K_p = self.Kp_v_right#0.95
            K_i = self.Ki_v_right #0.7
            K_d = self.Kd_v_right#0.001
        else:
            K_p = p[0]
            K_i = p[1]
            K_d = p[2]

        # Integral
        self.integral_right = self.integral_right + (des_spd - cur_spd)
        self.derivative_right.append(des_spd - cur_spd)

        # Derivative
        if len(self.derivative_right) >= 2:
            derivative_right = (self.derivative_right[-1] - self.derivative_right[-2]) / self.r.sleep_dur.to_sec()
        else:
            derivative_right = 0

        # Y 
        y =  K_p * (des_spd - cur_spd) + K_i * self.integral_right + K_d * derivative_right

        # overload protection
        if y > 1:
            y = 1
        if y < -1:
            y = -1


        return y
    
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

        

        
        
        
        



if __name__ == '__main__':
    new_obj = autotune()
    
    