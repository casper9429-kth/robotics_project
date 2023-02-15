#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
from  math import pi
import tf
from sensor_msgs.msg import Imu

class Odometry:
    def __init__(self):
        """
        This node uses the encoders to calculate the v of the robot and fuses it with the angle from the imu.
        It then updates the odometry.
        It doesn't use any EKF or UKF explicitly.
        But it uses the imu to correct the odometry, which might be considered as a form of EKF.        
        """
        # Initialize node
        rospy.loginfo('Initializing odometry node')
        rospy.init_node('odometry')
        
        # Create subscriber to encoders
        self.sub_goal = rospy.Subscriber('/motor/encoders', Encoders, self.encoder_callback)
        #self.sub_base_link_init = rospy.Subscriber('odom_init/base_link', TransformStamped, self.base_link_init_callback)

        # Create subscriber to imu/data
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # Subscribe to odom_init/odom
        #self.sub_odom_init = rospy.Subscriber('odom_init/odom', TransformStamped, self.odom_init_callback)
        
        # Robot parameters
        self.ticks_per_rev = 3072
        self.wheel_r = 0.04921
        self.base = 0.3 
        self.update_rate = 20 

        # First imu yaw angle
        self.yaw_offset = None

        # Create rate var
        self.rate = rospy.Rate(self.update_rate)

        # init inital pose
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
        #self.x_init = 0
        #self.y_init = 0
        #self.z_init = 0
        #self.yaw_init = 0
        
        
        # Init a tf broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Init a tf listener
        self.listener = tf.TransformListener()
        
    #def base_link_init_callback(self,msg):
    #    """Extract the initial pose of the robot from the odom_init/base_link topic"""
    #    self.x_init = msg.transform.translation.x
    #    self.y_init = msg.transform.translation.y
    #    self.z_init = msg.transform.translation.z
    #    q = msg.transform.rotation
    #    roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    #    self.yaw_init = yaw
        
        
        

    def imu_callback(self,msg):
        """
        Encoder callback
        When a new encoder message is received, the odometry is updated
        """
    
        # Get orientation in quaternion
        q = msg.orientation
        
        # Convert quaternion to euler
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.yaw_offset == None:
            self.yaw_offset = yaw
                
        # Update yaw
        self.yaw = yaw - self.yaw_offset  
    
    
    def encoder_callback(self,msg):
        """
        Encoder callback
        When a new encoder message is received, the odometry is updated
        """
        #rospy.loginfo('New encoder received:\n%s %s', msg.delta_time_left,msg.delta_time_right)

        # Init transform

    
        # The assumtion that both encoders publish with roughly the same frequency is not correct.
        # But for this exercise it is ok.

        # Get d_rad_left and d_rad_right
        d_rad_left = (msg.delta_encoder_left / self.ticks_per_rev) * 2 * pi
        d_rad_right = (msg.delta_encoder_right / self.ticks_per_rev) * 2 * pi

        # Get D and D_theta
        D = self.wheel_r *  (d_rad_left + d_rad_right) / 2
        D_theta = self.wheel_r * (d_rad_right - d_rad_left) / self.base

        # Calculate new x, y and yaw
        self.x = self.x + D * math.cos(self.yaw)
        self.y = self.y + D * math.sin(self.yaw)
        #self.yaw = self.yaw + D_theta
        
        # Add new x, y and yaw to transform, first cart then rot
        t = TransformStamped()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.header.stamp = msg.header.stamp
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z # z is always 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw) # transform yaw to quaternion
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Publish transform to tf broadcaster init in __init__
        self.br.sendTransform(t)
    





if __name__ == '__main__':
    new_odometry = Odometry()
    rospy.spin()