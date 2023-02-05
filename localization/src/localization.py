#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from robp_msgs.msg import Encoders
from robp_msgs.msg import DutyCycles
import tf_conversions
import tf2_ros
import math
from  math import pi
from aruco_msgs.msg import MarkerArray
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

# Localization node for robot
# state = [x,y,theta,v,omega], from compass, imu and encoders
# control = [v,omega] from cmd_vel


# 1. Print out the state and control, and the time stamp
# 2. Implement IMU Odometry
# 2. Implement UKF


class Localization:
    def __init__(self):
        # Initialize node
        rospy.loginfo('Initializing localization node')
        rospy.init_node('localization')
        
        # Subscribers
        ## Encoders
        self.sub_goal = rospy.Subscriber('/motor/encoders', Encoders, self.encoder_callback)
            
        ## IMU: accelerometer, compass
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback)        
        
        ## control: /motor/duty_cycles
        self.sub_control = rospy.Subscriber('/motor/duty_cycles', DutyCycles, self.control_callback)
        
        
        # Publishers
        ## state
        
        # Robot parameters
        self.ticks_per_rev = 3072
        self.wheel_r = 0.04921
        self.base = 0.3 

        # Create rate var
        self.rate = rospy.Rate(100)

        


        
        # Run loop
        self.run()    
    

    def run(self):
        """
        Main loop
        """
        # Run loop
        rospy.loginfo('Running localization node main loop')
        
        
        while not rospy.is_shutdown():
            
            self.rate.sleep()

    def encoder_callback(self, msg):
        """
        Calculates v, omega from encoder data
        Saves v, omega to self.v_enco, self.omega_enco
        """
        #rospy.loginfo("Encoder callback")
        #rospy.loginfo(msg)
        
        # # Total number of ticks
        # int64 encoder_left
        # int64 encoder_right
        # # The number of ticks since the last reading
        # int32 delta_encoder_left
        # int32 delta_encoder_right
        # # The time elapsed since the last reading in milliseconds
        # float64 delta_time_left
        # float64 delta_time_right
        
        # calculate v_left, v_right
        v_left = (((msg.delta_encoder_left/ self.ticks_per_rev ) * 2*pi * self.wheel_r )/ msg.delta_time_left)*1000
        v_right = (((msg.delta_encoder_right/ self.ticks_per_rev ) * 2*pi * self.wheel_r )/ msg.delta_time_right)*1000

        
        # calculate v, omega
        v, omega = self.transform_v_left_v_right_to_v_omega(v_left, v_right)
        
        self.v_enco = v
        self.omega_enco = omega
        
        #rospy.loginfo("v_left = %f, v_right = %f", v_left, v_right)
        #rospy.loginfo("v = %f, omega = %f", v, omega)

        
    def control_callback(self, msg):        
        """
        Callback for control messages that is also received by the motor controller node
        Calculates v, omega from control data
        saves v, omega to self.v_control, self.omega_control
        """
        v_left = msg.duty_cycle_left
        v_right = msg.duty_cycle_right
        self.v_contro, self.omega_control = self.transform_v_left_v_right_to_v_omega(v_left, v_right)
        
    def imu_callback(self, msg):
        """
        This callback is called when a new IMU message is received
        It calcluates:
        1. self.theta_imu: the orientation of the robot in the IMU frame
        2. self.omega_imu: the angular velocity of the robot in the IMU frame
        3. self.imu_acc_x: the linear acceleration of the robot in the IMU frame
        4. self.imu_acc_y: the linear acceleration of the robot in the IMU frame
        """
        
        #rospy.loginfo("Imu callback")
        #rospy.loginfo(msg)

        # Extract data
        ## orientation
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        
        ### Transform quaternion to euler
        euler = tf.transformations.euler_from_quaternion(q)
        self.theta_imu = euler[2]
        
        ## anuglar velocity
        self.omega_imu = msg.angular_velocity.z
        
        ## linear acceleration
        self.imu_acc_x = msg.linear_acceleration.x
        self.imu_acc_y = msg.linear_acceleration.y
        
        #rospy.loginfo("imu: theta = %f, omega = %f, acc_x = %f, acc_y = %f", self.theta_imu, self.omega_imu, self.imu_acc_x, self.imu_acc_y)


        

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

if __name__ == "__main__":
    rospy.loginfo("Starting localization node")
    try:

        node = Localization()
    except rospy.ROSInterruptException:
        rospy.loginfo("Localization node terminated.")