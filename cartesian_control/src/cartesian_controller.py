import rospy
from robp_msgs.msg import DutyCycles
from robp_msgs.msg import Encoders
from geometry_msgs.msg import Twist
from math import pi
import numpy as np


class cartesian_controller:
    def __init__(self):
        
        # init a node
        rospy.init_node('cartesian_controller', anonymous = True)

        # init a sleep rate
        self.r = rospy.Rate(20)

        # init a publisher to the /motor/duty_cycles topic
        # used to publish duty cycle values from the pid
        self.duty_cycle_pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=1)

        # init a subscriber to the /motor/encoders topic
        # used to read encoder values 
        self.encoders_sub = rospy.Subscriber('/motor/encoders', Encoders, self.callback_encoders)

        # init a subscriber to the /motor_controller/twist topic
        # used to read twist values or the robot
        self.twist_sub = rospy.Subscriber('/motor_controller/twist', Twist, self.callback_twist)

        # ticks per rev: 360
        # Wheel radius (r): 0.0352 m
        # Base (b): 0.23 m
        self.ticks_per_rev = 3072
        self.wheel_radius = 0.04921
        self.base = 0.3 


        # Encoder var, will be updated by callback encoder
        self.v_left_enco = 0.0
        self.v_right_enco = 0.0


        # Disired velocity, will be updated by callback twist
        self.v_des = 0.0
        self.omega_des = 0.0

        # Desired wheel velocity, will be updated in run main loop
        self.v_left_des = 0.0
        self.v_right_des = 0.0

        # Time since last encoder message
        self.time_enco = rospy.get_time()
        
        # Time since last twist message
        self.time_twist = rospy.get_time()


        # Pid left
        self.Kp_left = 1
        self.Ki_left = 0.5
        self.Kd_left = 0
        self.integral_left = 0.0
        self.derivative_left = []
        
        # Pid right
        self.Kp_right = 1
        self.Ki_right = 0.5
        self.Kd_right = 0
        self.integral_right = 0.0
        self.derivative_right = []

        # PID omega
        self.Kp_omega = 1
        self.Ki_omega = 0.5
        self.Kd_omega = 0
        self.integral_omega = 0.0
        self.derivative_omega = []
        
        # PID v
        self.Kp_v = 1
        self.Ki_v = 0.5
        self.Kd_v = 0
        self.integral_v = 0.0
        self.derivative_v = []
        





        # choose if to pid each wheel seperatly, to pid v and omega or not to pid at all
        self.run() # no pid
        #self.run_vl_vr() # pid each wheel seperatly based on encoder feedback
        #self.run_v_omega() # pid v and omega based on encoder feedback


    def run(self):
        """
        Main loop of the node, alt 1
        No pid control, just transforms desired velocity to desired wheel velocity
        And publishes duty cycle message to /motor/duty_cycles with wheel velocities 
        """
        
        
        # Create duty cycle message object
        # create inital duty cycle message object, will be used to publish to duty cycle topic
        duty_cycle_msg = DutyCycles()
        duty_cycle_msg.duty_cycle_left = 0.0
        duty_cycle_msg.duty_cycle_right = 0.0
        
        while not rospy.is_shutdown():
            # Convert the desired linear and angular velocity to the desired wheel velocities
            v_left, v_right = self.transform_v_omega_to_v_left_v_right(self.v_des, self.omega_des)

            # Limit the duty cycle to be between -1 and 1
            if np.abs(v_left) > 1 or np.abs(v_right) > 1:
                v_left = v_left / np.abs(v_left)
                v_right = v_right / np.abs(v_right)

            # Update the duty cycle message
            duty_cycle_msg.duty_cycle_left = v_left
            duty_cycle_msg.duty_cycle_right = v_right
            
            # Publish the duty cycle message
            self.duty_cycle_pub.publish(self.duty_cycle_msg)
            
            
            self.rate.sleep()



    def run_vl_vr(self):
        """
        Main loop of the node, alt 2
        * transforms desired v,omega to desired wheel velocity
        * gets wheel velocities from encoder feedback and PID control for each wheel seperatly
        * publishes duty cycle message to /motor/duty_cycles with wheel velocities
        """
        
        # Create duty cycle message object
        # create inital duty cycle message object, will be used to publish to duty cycle topic
        duty_cycle_msg = DutyCycles()
        duty_cycle_msg.duty_cycle_left = 0.0
        duty_cycle_msg.duty_cycle_right = 0.0

        # main loop
        while not rospy.is_shutdown():
            
            # Used when no new twist message is received for 0.5 seconds
            if ((rospy.get_time() - self.time_twist  ) > 0.5):
                duty_cycle_msg.duty_cycle_left = 0.0
                duty_cycle_msg.duty_cycle_right = 0.0
                self.duty_cycle_pub.publish(duty_cycle_msg)
                self.r.sleep()
                continue
                    
            # Get desired wheel velocity
            v_left_des, v_right_des = self.transform_v_omega_to_v_left_v_right(self.v_des, self.omega_des)
            
            # Calculate error
            error_left = v_left_des - self.v_left_enco
            error_right = v_right_des - self.v_right_enco
            
            # Calculate PID
            left_pid =self.PID_left(error_left)
            right_pid = self.PID_right(error_right)
            
            # Do fail checks, bad solution
            threshold = 1
            if np.abs(left_pid) > threshold:  left_pid = left_pid/np.abs(left_pid)
            if np.abs(left_pid) > threshold:  right_pid = right_pid/np.abs(right_pid)
                
            if self.v_des == 0 and np.abs(left_pid) < 0.05 and self.omega_des == 0 and np.abs(right_pid) < 0.05:
                left_pid = 0
                right_pid = 0


            duty_cycle_msg.duty_cycle_left = left_pid
            duty_cycle_msg.duty_cycle_right = right_pid

            # publish duty cycle message
            self.duty_cycle_pub.publish(duty_cycle_msg)
            self.r.sleep()
            
            
            
            


    def run_v_omega(self):
        """
        Main loop of the node, alt 3
        * transforms encoder feedback to desired v and omega
        * PID control of v and omega
        * transform pid v and omega to pid vl and vr
        * publishes duty cycle message to /motor/duty_cycles with wheel velocities
        """

        # Create duty cycle message object
        # create inital duty cycle message object, will be used to publish to duty cycle topic
        duty_cycle_msg = DutyCycles()
        duty_cycle_msg.duty_cycle_left = 0.0
        duty_cycle_msg.duty_cycle_right = 0.0


        
        # main loop
        while not rospy.is_shutdown(): # insted of while True, makes sure ros dies when supposed to



            # Used when no new twist message is received for 0.5 seconds
            if ((rospy.get_time() - self.time_twist  ) > 0.5):
                duty_cycle_msg.duty_cycle_left = 0.0
                duty_cycle_msg.duty_cycle_right = 0.0
                self.duty_cycle_pub.publish(duty_cycle_msg)
                self.r.sleep()
                continue



            
            # Get current velocity and omega from encoders
            v_enco, omega_enco = self.transform_v_left_v_right_to_v_omega(self.v_left_enco, self.v_right_enco)
            
            # Calculate error
            v_error = self.v_des - v_enco
            omega_error = self.omega_des - omega_enco

            # Calculate pid
            v_pid = self.PID_v(v_error)
            omega_pid = self.PID_omega(omega_error)


            self.v_left_des,self.v_right_des = self.transform_v_omega_to_v_left_v_right(v_pid, omega_pid)
            
            
            # set threshold for duty cycle, to not get to high values
            threshold = 1
            if np.abs(self.v_left_des) > threshold or np.abs(self.v_right_des) > threshold:
                self.v_left_des = self.v_left_des/np.abs(self.v_left_des)
                self.v_right_des = self.v_right_des/np.abs(self.v_right_des)


            # If desired speed is 0, and robot speed is close to 0, stop robot
            if self.v_des == 0 and np.abs(v_enco) < 0.05 and self.omega_des == 0 and np.abs(omega_enco) < 0.05:
                self.v_left_des = 0
                self.v_right_des = 0
            duty_cycle_msg.duty_cycle_left = self.v_left_des
            duty_cycle_msg.duty_cycle_right = self.v_right_des

            # publish duty cycle message
            self.duty_cycle_pub.publish(duty_cycle_msg)
            
            # sleep
            self.r.sleep()



    def PID_left(self, error,p=None):
        """PID controller for left wheel"""
        # PID param
        if p != None:
            K_p = p[0]
            K_i = p[1]
            K_d = p[2]
        else:
            K_p = self.Kp_left
            K_i = self.Ki_left
            K_d = self.Kd_left
            
        # Integral
        self.integral_left = self.integral_left + error
        self.derivative_left.append(error)

        # Derivative
        if len(self.derivative_left) >= 2:
            derivative_left = (self.derivative_left[-1] - self.derivative_left[-2]) / self.r.sleep_dur.to_sec()
        else:
            derivative_left = 0

        # PID
        pid = K_p * error + K_i * self.integral_left + K_d * derivative_left
        return pid


    def PID_right(self, error,p=None):
        """PID controller for right wheel"""
        # PID param
        if p != None:
            K_p = p[0]
            K_i = p[1]
            K_d = p[2]
        else:
            K_p = self.Kp_right
            K_i = self.Ki_right
            K_d = self.Kd_right
        # Integral
        self.integral_right = self.integral_right + error
        self.derivative_right.append(error)

        # Derivative
        if len(self.derivative_right) >= 2:
            derivative_right = (self.derivative_right[-1] - self.derivative_right[-2]) / self.r.sleep_dur.to_sec()
        else:
            derivative_right = 0

        # PID
        pid = K_p * error + K_i * self.integral_right + K_d * derivative_right
        return pid
    

    def PID_v(self, error,p=None):
        """PID controller for v"""
        # PID param
        if p != None:
            K_p = p[0]
            K_i = p[1]
            K_d = p[2]
        else:
            K_p = self.Kp_v#1.0 # 1.0
            K_i = self.Ki_v#1.0 # 0.5
            K_d = self.Kd_v#0.000001
        # Integral
        self.integral_v = self.integral_v + error
        self.derivative_v.append(error)

        # Derivative
        if len(self.derivative_v) >= 2:
            derivative_v = (self.derivative_v[-1] - self.derivative_v[-2]) / self.r.sleep_dur.to_sec()
        else:
            derivative_v = 0

        # Y 
        y =  K_p * error + K_i * self.integral_v + K_d * derivative_v


        return y
    
    def PID_omega(self, error,p=None):
        """PID controller for omega"""
        #New omega PID tuning parameters: Kp = 0.10716170717716685, Ki = 1.0716170717716684, Kd = 0.0026790426794291713
        #Old omega error sum: -0.01301851845077856

        # PID param
        if p != None:
            K_p = p[0]
            K_i = p[1]
            K_d = p[2]
        else:
            K_p = self.Kp_omega#1.0 # 1.0
            K_i = self.Ki_omega#1.0 # 0.5
            K_d = self.Kd_omega#0.000001

        # Integral
        self.integral_omega = self.integral_omega + error
        self.derivative_omega.append(error)

        # Derivative
        if len(self.derivative_omega) >= 2:
            derivative_omega = (self.derivative_omega[-1] - self.derivative_omega[-2]) / self.r.sleep_dur.to_sec()
        else:
            derivative_omega = 0

        # Y 
        y =  K_p * error + K_i * self.integral_omega + K_d * derivative_omega


        return y
    
        
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

                            
    # define callback functions
    def callback_encoders(self,data):
        """calc wheel speed from encoders"""

        time = rospy.get_time()
        # tics per second
        tic_spd_left = data.delta_encoder_left / data.delta_time_left     #(time - self.time_enco)
        tic_spd_right = data.delta_encoder_right / data.delta_time_right            #/ (time - self.time_enco)
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
        
        

    def callback_twist(self,data):        
        # set time stamp
        self.time_twist = rospy.get_time()
        
        # save in internal var
        self.v_des = data.linear.x
        self.omega_des = data.angular.z
                
        



if __name__ == '__main__':
    # Initialize the node and name it.
    new_obj = cartesian_controller()

    
    
