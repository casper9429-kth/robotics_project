import rospy
from robp_msgs.msg import DutyCycles
from robp_msgs.msg import Encoders
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from math import pi
import numpy as np
import scipy
# Add do_mpc to path. This is not necessary if it was installed via pip.
import sys
sys.path.append('../../')

# Import do_mpc package:
import do_mpc



class cartesian_controller:
    def __init__(self):
        
        # init a node
        rospy.init_node('cartesian_controller', anonymous = True)

        # init a sleep rate
        self.r = rospy.Rate(10)

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
        self.ticks_per_rev = 360
        self.wheel_radius = 0.0352
        self.base = 0.23

        # Encoder var, will be updated by callback encoder
        self.v_left_enco = 0.0
        self.v_right_enco = 0.0

        # Integral var, will be updated by PI control
        self.integral_v = 0.0
        self.integral_omega = 0.0
        self.integral_left = 0.0
        self.integral_right = 0.0
        
        # Derivative var, will be updated by PID control
        self.derivative_v = []
        self.derivative_omega = []
 
        # Disired velocity, will be updated by callback twist
        self.v_des = 0.0
        self.omega_des = 0.2

        # Desired wheel velocity, will be updated in run main loop
        self.v_left_des = 0.0
        self.v_right_des = 0.0

        # Time since last encoder message
        self.time_enco = rospy.get_time()
        
        # Time since last twist message
        self.time_twist = rospy.get_time()

        # PID omega
        self.Kp_omega = 1
        self.Ki_omega = 0
        self.Kd_omega = 0
        self.PID_omega_list = []


        self.sample_size = 60
        self.error_list_omega = []
        self.error_list_v = []
        self.pre_tune()
        self.twiddle([self.Kp_omega,self.Ki_omega,self.Kd_omega])
        self.run()


    def run(self):
        """
        Main loop of the node
        * transforms desired velocity to desired wheel velocity
        * publishes duty cycle message to /motor/duty_cycles with wheel velocities
        * gets wheel velocities from encoder feedback and PI control
        """
        self.v_des = 0.5
        self.omega_des = 0
        # check if desired speed is to high and warns user
        if self.v_left_des > 0.5 or self.v_right_des > 0.5:
            rospy.loginfo("Desired speed for wheel is too high, max is 0.5 m/s for realiable operation")

        # Create duty cycle message object
        # create inital duty cycle message object, will be used to publish to duty cycle topic
        duty_cycle_msg = DutyCycles()
        duty_cycle_msg.duty_cycle_left = 0.0
        duty_cycle_msg.duty_cycle_right = 0.0


        
        # main loop
        while not rospy.is_shutdown(): # insted of while True, makes sure ros dies when supposed to



            # Edge case tp stop robot and not get oscillations from PI control. Used when no new twist message is received or when desired speed is 0 
            if self.v_des == 0 and self.omega_des == 0 or ((rospy.get_time() - self.time_twist  ) > 0.5):
                duty_cycle_msg.duty_cycle_left = 0.0
                duty_cycle_msg.duty_cycle_right = 0.0
                self.duty_cycle_pub.publish(duty_cycle_msg)
                self.integral_left = 0.0 # reset integral in pi control 
                self.integral_right = 0.0 # reset integral in pi control
                self.r.sleep()
                continue



            # Outerloop pid
            
            # Get current velocity and omega from encoders
            v_enco, omega_enco = self.transform_v_left_v_right_to_v_omega(self.v_left_enco, self.v_right_enco)
            
            # Calculate error
            v_error = self.v_des - v_enco
            omega_error = self.omega_des - omega_enco

            # Calculate pid
            v_pid = self.PID_v(v_error)
            omega_pid = self.PID_omega(omega_error)
            self.error_list_omega.append(omega_error)
            # Calculate desired wheel velocity
            self.v_left_des,self.v_right_des = self.transform_v_omega_to_v_left_v_right(v_pid, omega_pid)
            
            
            if np.abs(self.v_left_des) > 1 or np.abs(self.v_right_des) > 1:
                self.v_left_des = self.v_left_des/np.abs(self.v_left_des)
                self.v_right_des = self.v_right_des/np.abs(self.v_right_des)
                
            duty_cycle_msg.duty_cycle_left = self.v_left_des
            duty_cycle_msg.duty_cycle_right = self.v_right_des

            self.duty_cycle_pub.publish(duty_cycle_msg)

            # print mean error in omega over last 10 samples
            rospy.loginfo("Mean error in omega: " + str(np.mean(self.error_list_omega[-10:])))
            
            # sleep
            self.r.sleep()

    def pre_tune(self):
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
        
        while not rospy.is_shutdown(): # insted of while True, makes sure ros dies when supposed to



            if len(self.PID_omega_list) > 5:
                # Search self.PID_omega for smallest error
                self.Kp_omega,self.Ki_omega,self.Kd_omega = sorted(self.PID_omega_list, key=lambda x: x[3])[0][0:3]
                rospy.loginfo("Kp: %f, Ki: %f, Kd: %f", self.Kp_omega,self.Ki_omega,self.Kd_omega)
                # Print error
                rospy.loginfo("Error: %f", sorted(self.PID_omega_list, key=lambda x: x[3])[0][3])
                return

            if t <5: # Sleep
                self.omega_des = 0.0#np.random.uniform(-0.3,0.3) 
                self.v_des = 0.0#np.random.uniform(0.1,0.5)

            elif t < 35: # Run forward
                self.omega_des = 0#np.random.uniform(-0.5,0.5)
                self.v_des = 0.5#np.random.uniform(0.1,0.5)

            elif t<40:
                self.omega_des = 0.0
                self.v_des = 0.0
            elif t < 70: # Run forward
                self.omega_des = 0#np.random.uniform(-0.5,0.5)
                self.v_des = -0.5#np.random.uniform(0.1,0.5)

            elif t<75:
                self.omega_des = 0.0
                self.v_des = 0.0

            else:
                
                self.auto_tune_omega(0)
                
                t = -1
            
            t += 1                

            # Outerloop pid
            
            # Get current velocity and omega from encoders
            v_enco, omega_enco = self.transform_v_left_v_right_to_v_omega(self.v_left_enco, self.v_right_enco)
            
            # Calculate error
            v_error = self.v_des - v_enco
            omega_error = self.omega_des - omega_enco

            # Calculate pid
            v_pid = self.PID_v(v_error)
            omega_pid = self.PID_omega(omega_error)
            #self.auto_tune_omega(omega_error)
            self.error_list_omega.append(omega_error)            
            
            # Calculate desired wheel velocity
            self.v_left_des,self.v_right_des = self.transform_v_omega_to_v_left_v_right(v_pid, omega_pid)

            
            if np.abs(self.v_left_des) > 1 or np.abs(self.v_right_des) > 1:
                self.v_left_des = self.v_left_des/np.abs(self.v_left_des)
                self.v_right_des = self.v_right_des/np.abs(self.v_right_des)
            
            #duty_cycle_msg.duty_cycle_left = self.PI_left(self.v_left_des, self.v_left_enco)
            #duty_cycle_msg.duty_cycle_right = self.PI_right(self.v_right_des, self.v_right_enco)
            duty_cycle_msg.duty_cycle_left = self.v_left_des
            duty_cycle_msg.duty_cycle_right = self.v_right_des

            self.duty_cycle_pub.publish(duty_cycle_msg)

            v_enco, omega_enco = self.transform_v_left_v_right_to_v_omega(self.v_left_enco, self.v_right_enco)
            #rospy.loginfo("LEFT : Des %s Acc %s Pub %s", self.v_left_des, self.v_left_enco, self.duty_cycle_msg.duty_cycle_left)
            #rospy.loginfo("RIGHT : Des %s Acc %s Pub %s", self.v_right_des, self.v_right_enco, self.duty_cycle_msg.duty_cycle_right)
            #rospy.loginfo("Desired: Omega %s V %s", self.omega_des, self.v_des)
            #rospy.loginfo("Achived: Omega %s V %s", omega_enco,v_enco)


            # sleep
            self.r.sleep()


    def twiddle(self, p, tol=0.1):
        dp = [0.1, 0.1, 0.1]
        best_error = self.run(p)

        while sum(dp) > tol:
            rospy.loginfo("twiddle error: %f", best_error)
            rospy.loginfo("twiddle p: %f, %f, %f", p[0], p[1], p[2])
            for i in range(len(p)):
                p[i] += dp[i]
                error = self.run(p)
                rospy.loginfo("twiddle error: %f", error)

                if error < best_error:
                    best_error = error
                    dp[i] *= 1.05
                    rospy.loginfo("twiddle improved")
                else:
                    p[i] -= 2 * dp[i]
                    error = self.run(p)
                    rospy.loginfo("twiddle error: %f", error)
                    

                    if error < best_error:
                        best_error = error
                        dp[i] *= 1.05
                    else:
                        p[i] += dp[i]
                        dp[i] *= 0.95
        self.Kp_omega = p[0]
        self.Ki_omega = p[1]
        self.Kd_omega = p[2]
        rospy.loginfo("Twiddle finish, Kp: %f, Ki: %f, Kd: %f", self.Kp_omega,self.Ki_omega,self.Kd_omega)
        return 

    def run(self,p):
        """Run omega with pid parameters p"""
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
        error_list_omega = []
        while not rospy.is_shutdown(): # insted of while True, makes sure ros dies when supposed to





            if t <5: # Sleep
                self.omega_des = 0.0#np.random.uniform(-0.3,0.3) 
                self.v_des = 0.0#np.random.uniform(0.1,0.5)

            elif t < 35: # Run forward
                self.omega_des = 0#np.random.uniform(-0.5,0.5)
                self.v_des = 0.5#np.random.uniform(0.1,0.5)

            elif t<40:
                self.omega_des = 0.0
                self.v_des = 0.0
            elif t < 70: # Run forward
                self.omega_des = 0#np.random.uniform(-0.5,0.5)
                self.v_des = -0.5#np.random.uniform(0.1,0.5)

            elif t<75:
                self.omega_des = 0.0
                self.v_des = 0.0

            else:     
                # take mean square error of omega and return it
                return np.sqrt(sum(error_list_omega))/len(error_list_omega)
            
            t += 1                

            # Outerloop pid
            
            # Get current velocity and omega from encoders
            v_enco, omega_enco = self.transform_v_left_v_right_to_v_omega(self.v_left_enco, self.v_right_enco)
            
            # Calculate error
            v_error = self.v_des - v_enco
            omega_error = self.omega_des - omega_enco

            # Calculate pid
            v_pid = self.PID_v(v_error)
            omega_pid = self.PID_omega(omega_error,p)
            #self.auto_tune_omega(omega_error)
            error_list_omega.append(omega_error**2)            
            
            # Calculate desired wheel velocity
            self.v_left_des,self.v_right_des = self.transform_v_omega_to_v_left_v_right(v_pid, omega_pid)

            
            if np.abs(self.v_left_des) > 1 or np.abs(self.v_right_des) > 1:
                self.v_left_des = self.v_left_des/np.abs(self.v_left_des)
                self.v_right_des = self.v_right_des/np.abs(self.v_right_des)
                
            #duty_cycle_msg.duty_cycle_left = self.PI_left(self.v_left_des, self.v_left_enco)
            #duty_cycle_msg.duty_cycle_right = self.PI_right(self.v_right_des, self.v_right_enco)
            duty_cycle_msg.duty_cycle_left = self.v_left_des
            duty_cycle_msg.duty_cycle_right = self.v_right_des

            self.duty_cycle_pub.publish(duty_cycle_msg)

            v_enco, omega_enco = self.transform_v_left_v_right_to_v_omega(self.v_left_enco, self.v_right_enco)
            #rospy.loginfo("LEFT : Des %s Acc %s Pub %s", self.v_left_des, self.v_left_enco, self.duty_cycle_msg.duty_cycle_left)
            #rospy.loginfo("RIGHT : Des %s Acc %s Pub %s", self.v_right_des, self.v_right_enco, self.duty_cycle_msg.duty_cycle_right)
            #rospy.loginfo("Desired: Omega %s V %s", self.omega_des, self.v_des)
            #rospy.loginfo("Achived: Omega %s V %s", omega_enco,v_enco)


            # sleep
            self.r.sleep()

    def PI_right(self,des_spd, cur_spd):
        """PI controller"""
        # PI param
        K_p = 0.95 # 0.95
        K_i = 0.7  # 0.7

        # Implementing auto tuning of PI
        #self.integral_right = self.integral_right + (des_spd - cur_spd) # in callback instead to avoid false addition
        
        
        # Y 
        y = K_p * (des_spd - cur_spd) + K_i * self.integral_right#self.integral_right
        
        # Overload protection        
        if y > 1:
            y = 1
        if y < -1:
            y = -1


        #rospy.loginfo("Received encoder values: %s %s", des_spd, cur_spd)

        return y        

    def PI_left(self,des_spd, cur_spd):
        """PI controller"""
        # PI param
        K_p = 1.2 # 1.2
        K_i = 0.8 # 0.8
        
        # Integral
        #self.integral_left = self.integral_left + (des_spd - cur_spd)   # in callback instead to avoid false infill       
            
        # Y 
        y =  K_p * (des_spd - cur_spd) + K_i * self.integral_left
        
        # overload protection
        if y > 1:
            y = 1
        if y < -1:
            y = -1

        #rospy.loginfo("Received encoder values: %s %s", des_spd, cur_spd)

        return y        


    def transform_v_omega_to_v_left_v_right(self, v, omega):
        """Transforms the desired linear and angular velocity to the desired wheel velocities, angular velocity is in rad/s, speed in m/s"""
        # v = (v_left + v_right) / 2
        # omega = (v_right - v_left) / 2b
        # v_right = v + b * omega 
        # v_left = v - b * omega
        
        v_right = v + self.base * omega
        v_left = v - self.base * omega
        return v_left, v_right

    def PID_v(self, error):
        """PID controller for v"""
        # PID param
        K_p = 1.0
        K_i = 0.5
        K_d = 0.0

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

        # overload protection

        #rospy.loginfo("Received encoder values: %s %s", des_spd, cur_spd)

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
    def auto_tune_v(self,error):
        """Continously tune v PID using Ziegler-Nichols method"""
        # Calculate error for each sample
        self.error_list_v.append(error)

        # Check if enough samples have been collected
        if len(self.error_list_v) > self.sample_size:
            # Calculate the oscillation period
            peak_indices = scipy.signal.find_peaks(self.error_list_v)[0]
            if len(peak_indices) > 1:
                period = peak_indices[-1] - peak_indices[-2]
                # Calculate Ziegler-Nichols tuning parameters
                Ku = 4 * max(self.error_list_v) / (np.pi * period)
                Pu = period * self.r.sleep_dur.to_sec()
                self.Kp_v = 0.6 * Ku
                self.Ki_v = 2 * Ku / Pu
                self.Kd_v = Ku * Pu / 8
            # Reset error list
            self.error_list_v = []

    
    def auto_tune_omega(self,error):
        """Continously tune omega PID using Ziegler-Nichols method"""
        # Calculate error for each sample
        #self.error_list_omega.append(error)
        # Check if enough samples have been collected
        if len(self.error_list_omega) < self.sample_size:
            return 
        # Calculate the oscillation period
        peak_indices = scipy.signal.find_peaks(self.error_list_omega)[0]
        if len(peak_indices) > 1:
            period = peak_indices[-1] - peak_indices[-2]
        else:
            self.error_list_omega = []
            return
        # Check if the period is valid, if not, clear the error list and return
        if period < 2 or period > self.sample_size/2:
            self.error_list_omega = []
            return
        error_pow = np.sqrt(sum([x**2 for x in self.error_list_omega]))/len(self.error_list_omega)
        self.PID_omega_list.append([self.Kp_omega, self.Ki_omega, self.Kd_omega,error_pow])

        #Calculate Ziegler-Nichols tuning parameters
        Ku = 4 * max(self.error_list_omega) / (np.pi * period)
        Pu = period * self.r.sleep_dur.to_sec()
        self.Kp_omega = 0.6 * Ku
        self.Ki_omega = 2 * self.Kp_omega / Pu
        self.Kd_omega = self.Kp_omega * Pu / 8
        # Reset error list and integral
        print("Old omega error sum: {}".format(error_pow)) 
        # Replace with more efficent one liner       
        self.error_list_omega = []
        self.integral_omega = 0
        self.derivative_omega = []
        # Print new tuning parameters

        print("New omega PID tuning parameters: Kp = {}, Ki = {}, Kd = {}".format(self.Kp_omega, self.Ki_omega, self.Kd_omega))

        
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
        
        # append integral
        self.integral_right = self.integral_right + (self.v_right_des - self.v_right_enco)
        self.integral_left = self.integral_left + (self.v_left_des - self.v_left_enco)
        


        #rospy.loginfo("Received encoder values: %s %s", v_left, v_right)
        
        
        
    def callback_twist(self,data):        
        # set time stamp
        self.time_twist = rospy.get_time()
        
        # save in internal var
        #self.v_des = 0.4#data.linear.x
        #self.omega_des = 0.0#data.angular.z
        
        # Debug
        #rospy.loginfo("Received twist values: %s %s", data.linear.x, data.angular.z)
        
        



if __name__ == '__main__':
    # Initialize the node and name it.
    new_obj = cartesian_controller()

    
    
