#!/usr/bin/env python
import rospy
from robp_msgs.msg import Encoders
from math import pi
from nav_msgs.msg import Odometry

class trans_enco_to_odom:
    def __init__(self):
        # Initialize node
        rospy.loginfo('Initializing wheel encoders to odometry_msg node')
        rospy.init_node('wheel_enco_to_odom_msg')
        
        rospy.sleep(1)
        
        # Create subscriber to encoders
        self.sub_goal = rospy.Subscriber('/motor/encoders', Encoders, self.encoder_callback)

        # Create publisher to /odometry/v_omega using Twist message
        self.pub_v_omega = rospy.Publisher('/enco/v_omega', Odometry, queue_size=10)
        

        # Robot parameters
        self.ticks_per_rev = 3072
        self.wheel_r = 0.04921
        self.base = 0.3 
        self.update_rate = 10 





    

    def encoder_callback(self,msg):
        """
        Encoder callback
        When a new encoder message is received, the odometry is updated
        """

        # Calc v_left and v_right
        v_left = (((msg.delta_encoder_left/ self.ticks_per_rev ) * 2*pi * self.wheel_r )/ msg.delta_time_left)*1000
        v_right = (((msg.delta_encoder_right/ self.ticks_per_rev ) * 2*pi * self.wheel_r )/ msg.delta_time_right)*1000

        
        # calculate v, omega
        v, omega = self.transform_v_left_v_right_to_v_omega(v_left, v_right)

        #Odometry message
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega


        # Publish twist message
        self.pub_v_omega.publish(odom)
        

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
    new_odometry = trans_enco_to_odom()
    # Keep it alive
    rospy.spin()                
