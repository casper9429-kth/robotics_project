#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
from  math import pi
from aruco_msgs.msg import MarkerArray
import tf


class Odometry:
    def __init__(self):
        # Initialize node
        rospy.loginfo('Initializing odometry node')
        rospy.init_node('odometry')
        
        # Create subscriber to encoders
        self.sub_goal = rospy.Subscriber('/motor/encoders', Encoders, self.encoder_callback)

        # Create a subscriber to the aruco markers
        self.sub_aruco = rospy.Subscriber('/aruco/markers', MarkerArray, self.aruco_callback)

        # Robot parameters
        self.ticks_per_rev = 3072
        self.wheel_r = 0.04921
        self.base = 0.3 
        self.update_rate = 20 

        # Create rate var
        self.rate = rospy.Rate(self.update_rate)

        # init inital pose
        self.x = 0
        self.y = 0
        self.yaw = 0

        # Init a tf broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Init a tf listener
        self.listener = tf.TransformListener()
        
        # Run loop
        self.run()
                


    def run(self):
        """
        Main loop
        """
        # Run loop
        rospy.loginfo('Running odometry node main loop')
        
        
        while not rospy.is_shutdown():
            
            self.rate.sleep()


            
    def aruco_callback(self,msg):
        """ aruco callback """
        # Take the ArUco marker detections, transform them into the map frame in a node and publish 
        # a TF from the map frame to a /aruco/detectedX frame, where X corresponds to the ID of the marker.
        
        # The camera is mounted 0.08987m forward, 0.0175m left, and 0.10456m up with respect to the robot. 
        # Note that the camera coordinate system is such that z is along the optical axis (i.e., forward) and y is pointing down.
        


        for marker in msg.markers:
            from_frame = "camera_link"
            to_frame = "map"
            
            # Define pose stamp and transform to map
            ps = PoseStamped()
            ps.header.stamp = marker.header.stamp
            ps.header.frame_id = from_frame
            ps.pose.position.x = marker.pose.pose.position.x
            ps.pose.position.y = marker.pose.pose.position.y
            ps.pose.position.z = marker.pose.pose.position.z
            roll,pitch,yaw = tf_conversions.transformations.euler_from_quaternion([marker.pose.pose.orientation.x,marker.pose.pose.orientation.y,marker.pose.pose.orientation.z,marker.pose.pose.orientation.w])
            roll = roll - pi/2 
            pitch = pitch 
            yaw = yaw - pi/2           
            q = tf_conversions.transformations.quaternion_from_euler(roll,pitch,yaw)
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]

            self.listener.waitForTransform(from_frame, to_frame,marker.header.stamp,rospy.Duration(4.0) )#rospy.Time(), rospy.Duration(4.0))
            ps_map = self.listener.transformPose("map",ps)

            # publish the transform ps_map to the map frame
            t = TransformStamped()
            t.header.frame_id = "map"
            t.child_frame_id = "aruco/detected" + str(marker.id)
            t.header.stamp = marker.header.stamp
            t.transform.translation.x = ps_map.pose.position.x
            t.transform.translation.y = ps_map.pose.position.y
            t.transform.translation.z = ps_map.pose.position.z
            t.transform.rotation.x = ps_map.pose.orientation.x
            t.transform.rotation.y = ps_map.pose.orientation.y
            t.transform.rotation.z = ps_map.pose.orientation.z
            t.transform.rotation.w = ps_map.pose.orientation.w
            self.br.sendTransform(t)
            
            
            



    def encoder_callback(self,msg):
        """
        Encoder callback
        When a new encoder message is received, the odometry is updated
        """
        #rospy.loginfo('New encoder received:\n%s %s', msg.delta_time_left,msg.delta_time_right)

        # Init transform
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
    
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
        self.yaw = self.yaw + D_theta
        
        # Add new x, y and yaw to transform, first cart then rot
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0 # z is always 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw) # transform yaw to quaternion
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Publish transform to tf broadcaster init in __init__
        self.br.sendTransform(t)
    
    def calc_translation_speed_encoder(self, encoder_data):
        """
        Calculates the translation speed of the wheels in m/s
        input: raw encoder data
        returns v_left, v_right [m/s]
        """
        tic_spd_left = encoder_data.delta_encoder_left / encoder_data.delta_time_left
        tic_spd_right = encoder_data.delta_encoder_right / encoder_data.delta_time_right
        
        # m/tics 
        self.wheel_r
        circ = 2*self.wheel_r*pi
        tics = self.ticks_per_rev
        m_tic = circ/tics 
        
        # tic/s * m/tic = m/s
        v_left = tic_spd_left * m_tic*1000
        v_right = tic_spd_right * m_tic*1000 

        # save in internal var
        return v_left, v_right


    def calc_robot_v_omega_encoder(self,encoder_data):
        """
        Calculates the robot linear and angular velocity in m/s and rad/s        
        """
        v_left, v_right = self.calc_translation_speed_encoder(encoder_data)
        v, omega = self.transform_v_left_v_right_to_v_omega(v_left, v_right)
        return v, omega
    
            
    def transform_v_left_v_right_to_v_omega(self, v_left, v_right):
        """Transform v_left and v_right to v and omega"""
        # v = (v_left + v_right) / 2
        # omega = (v_right - v_left) / b
        
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / (self.base)
        return v, omega



if __name__ == '__main__':
    new_odometry = Odometry()