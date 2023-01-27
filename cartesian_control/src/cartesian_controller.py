import rospy
from robp_msgs.msg import DutyCycles
from robp_msgs.msg import Encoders
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from math import pi

# def of used msgs types

# DutyCycles
#Header header
# Value should be in [-1, 1]
#float64 duty_cycle_left
# Value should be in [-1, 1]
#float64 duty_cycle_right

# Encoders
#Header header
# Total number of ticks
#int64 encoder_left
#int64 encoder_right
# Ticks since last message
#int32 delta_encoder_left
#int32 delta_encoder_right
# Time since last message
#float64 delta_time_left
#float64 delta_time_right

# Twist
#Header header
# Linear velocity in m/s
#float64 linear.x
# Linear velocity in m/s
#float64 linear.y
# Linear velocity in m/s
#float64 linear.z
# Angular velocity in rad/s
#float64 angular.x
# Angular velocity in rad/s
#float64 angular.y
# Angular velocity in rad/s
#float64 angular.z

"""
Your cartesian_controller node should subscribe to the following topics:
/motor_controller/twist (message type: geometry_msgs/Twist): in this topic, the controller will receive the linear and angular velocity at which we wish to move the robot (expressed in the base frame of the robot). The message is a 6D twist (3D for linear velocity and 3D for angular velocity) but since we are controlling the robot in a 2D plane you only need to use one component of the linear velocity (the x-component) and one component of the angular velocity (the z-component). Using the kinematics equations for differential mobile robot configurations, one can then calculate the individual contributions of each wheel (in terms of angular velocity) to achieve the desired twist. To view the complete twist message definition run in a terminal:
rosmsg show geometry_msgs/Twist
/motor/encoders (message type: robp_msgs/Encoders): through this topic, the controller will receive the encoder feedback signals for estimating the angular velocity of the wheels.
Your node should publish to the following topic:
/motor/duty_cycles (message type: robp/DutyCycles): duty cycle signal for controlling the power fed to the motors.
For more details on the encoder and duty cycle topics refer back to the Kobuki description page.
"""


# ticks per rev: 360
# Wheel radius (r): 0.0352 m
# Base (b): 0.23 m


# step 1
# 1. write spd 1 to /motor/duty_cycles
# 2. read encoders
# 3. read motor_controller_twist

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
        self.integral_left = 0.0
        self.integral_right = 0.0
 
        # Disired velocity, will be updated by callback twist
        self.v_des = 0.0
        self.omega_des = 0.0

        # Desired wheel velocity, will be updated in run main loop
        self.v_left_des = 0.0
        self.v_right_des = 0.0


        # Time since last twist message
        self.time_twist = rospy.get_time()



        self.run()



    def run(self):
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

            # Transform desired velocity to desired wheel velocity
            self.v_left_des,self.v_right_des = self.transform_v_omega_to_v_left_v_right(self.v_des, self.omega_des)
            
            # inner loop control each wheel 
            duty_cycle_msg.duty_cycle_left = self.PI_left(self.v_left_des, self.v_left_enco)
            duty_cycle_msg.duty_cycle_right = self.PI_right(self.v_right_des, self.v_right_enco)
            self.duty_cycle_pub.publish(duty_cycle_msg)
        
        
            # Debugging
            v_enco, omega_enco = self.transform_v_left_v_right_to_v_omega(self.v_left_enco, self.v_right_enco)
            #rospy.loginfo("LEFT : Des %s Acc %s Pub %s", self.v_left_des, self.v_left_enco, self.duty_cycle_msg.duty_cycle_left)
            #rospy.loginfo("RIGHT : Des %s Acc %s Pub %s", self.v_right_des, self.v_right_enco, self.duty_cycle_msg.duty_cycle_right)
            rospy.loginfo("Desired: Omega %s V %s", self.omega_des, self.v_des)
            rospy.loginfo("Achived: Omega %s V %s", omega_enco,v_enco)

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
        # Encoders
        #Header header
        
        # Total number of ticks
        #int64 encoder_left
        #int64 encoder_right
        
        # Ticks since last message
        #int32 delta_encoder_left
        #int32 delta_encoder_right
        
        # Time since last message
        #float64 delta_time_left
        #float64 delta_time_right

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

        # append integral
        self.integral_right = self.integral_right + (self.v_right_des - self.v_right_enco)
        self.integral_left = self.integral_left + (self.v_left_des - self.v_left_enco)
        

        #rospy.loginfo("Received encoder values: %s %s", v_left, v_right)
        
    def callback_twist(self,data):        
        # set time stamp
        self.time_twist = rospy.get_time()
        
        # save in internal var
        self.v_des = data.linear.x
        self.omega_des = data.angular.z
        
        # Debug
        #rospy.loginfo("Received twist values: %s %s", data.linear.x, data.angular.z)
        
        



if __name__ == '__main__':
    # Initialize the node and name it.
    #rospy.init_node('cartesian_controller', anonymous = True)
    new_obj = cartesian_controller()
    # Start code
    #WEain()