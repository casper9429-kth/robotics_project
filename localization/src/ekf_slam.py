import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
from  math import pi
import tf
from sensor_msgs.msg import Imu
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from collections import defaultdict

class ekf_odometry():
    def __init__(self):
        """
        Peforms EKF odometry
        """
        rospy.init_node('ekf_odometry')

        # Subscribers
        self.sub_goal = rospy.Subscriber('/motor/encoders', Encoders, self.encoder_callback) # encoders 
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback)                 # imu 
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)        # cmd_vel 
        self.aruco_detect_sub = rospy.Subscriber('aruco/markers', MarkerArray, self.aruco_detect_callback) 
        
        
        # Publisher# Publish the map and the odometry
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50) # Publish the odometry and make it available for other nodes

        # TF Stuff
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(300))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()

        # Target rate
        self.update_rate = 50 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 
        
        
        # Robot parameters
        self.ticks_per_rev = 3072
        self.wheel_r = 0.04921
        self.base = 0.3 

        
        # EKF var
        self.count = 0
        self.mu_bel = np.zeros(2)
        self.simga_bel = np.eye(2)
        self.mu = np.zeros(2)
        self.sigma = np.eye(2)
        
        
        
        # Sensor var, encoder and imu, will be updated by the callback functions
        self.enco = np.zeros(2)
        self.imu = np.zeros(2)
        self.u = np.zeros(2)
        
        # EKF settings
        self.R = np.eye(2) * 100  
        self.Q_enco = np.eye(2)
        self.Q_enco[0,0] = 10
        self.Q_enco[1,1] = 100
        self.Q_imu = np.eye(2)
        self.Q_imu[0,0] = 100000 # Must be very high, because the imu is not very accurate at determining the speed
        self.Q_imu[1,1] = 50
        
        # Odometry var 
        self.x = 0
        self.y = 0
        self.theta = 0
        self.R_odom = np.eye(3) 
        self.simga_bel_odom = np.zeros(3)
        
        # Time var
        self.last_time = rospy.Time.now().to_sec()
        self.current_time = rospy.Time.now().to_sec()


        # Aruco SLAM
        self.aruco_state_space = defaultdict()
        self.seen_ids = set()        
        self.add_to_state_space_list = []        


    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """
        
        # State vector is [v,omega]
        

        if self.count == 0:
            # Initialize state vector
            self.mu = np.zeros(2)
            self.sigma = np.eye(2)
            self.mu_bel = np.zeros(2)
            self.simga_bel = np.eye(2)
            self.count += 1        
            return
        

        # predict step
        self.mu_bel = self.u
        self.simga_bel = np.eye(2) * self.sigma * np.eye(2).T + self.R


        # update step enco
        H = np.eye(2)
        K = self.simga_bel * H.T * np.linalg.inv(H * self.simga_bel * H.T + self.Q_enco)
        self.mu_bel = self.mu_bel + K * (self.enco - self.mu_bel)
        self.simga_bel = (np.eye(2) - K * H) * self.simga_bel        
        
        # update step imu
        H = np.eye(2)
        K = self.simga_bel * H.T * np.linalg.inv(H * self.simga_bel * H.T + self.Q_imu)
        self.mu = self.mu_bel + K * (self.imu - self.mu_bel)
        self.simga = (np.eye(2) - K * H) * self.simga_bel


        # Perform odometry Prediction
        current_time = rospy.Time.now()
        self.current_time = current_time.to_sec()
        dt = self.current_time - self.last_time
        if dt > self.update_dt*1.05:
            rospy.loginfo("dt: %f", dt)
        self.x += self.mu[0] * math.cos(self.theta) * dt
        self.y += self.mu[0] * math.sin(self.theta) * dt
        self.theta += self.mu[1] * dt
        G_0 = [1,0,-self.mu[0] * math.sin(self.theta) * dt]
        G_1 = [0,1,self.mu[0] * math.cos(self.theta) * dt]
        G_2 = [0,0,1]        
        G = [G_0, G_1, G_2]
        R 
        

        
        # Publish the odometry message
        self.publish_odometry(current_time)
        self.last_time = self.current_time

    
    
    def aruco_marker_callback(self, msg):
        """
        Callback function for aruco marker detection
        """
        
        for marker in msg.markers:
            pose_map = PoseStamped()
            pose_map.header.frame_id = msg.header.frame_id
            pose_map.header.stamp = msg.header.stamp
            pose_map.pose = marker.pose.pose                        

            try:
                pose_map = self.tfBuffer.transform("map", pose_map, rospy.Duration(1.0))
                pose_odom = self.tfBuffer.transform("odom", pose_map, rospy.Duration(1.0))

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo("Could not transform aruco marker pose to map frame")
                rospy.logwarn(e)
                continue   



            # SLAM Update step code here

            ## If new marker, add to state space
            if marker.id not in self.seen_ids:
                # Add to state space
                # Define new default dict for the marker
                new_marker_dict = defaultdict()
                new_marker_dict['id'] = marker.id
                new_marker_dict['pose_map'] = pose_map
                new_marker_dict['pose_odom'] = pose_odom
                new_marker_dict['cov'] = self.sigma
                self.aruco_state_space[marker.id] = new_marker_dict
                self.seen_ids.add(marker.id)
                continue


                



            #odom = TransformStamped()
            #odom.header.stamp = msg.header.stamp
            #odom.header.frame_id = "map"
            #odom.child_frame_id = "map_goal"
            #odom.transform.translation.x = pose_map.pose.position.x
            #odom.transform.translation.y = pose_map.pose.position.y
            #odom.transform.translation.z = pose_map.pose.position.z
            #odom.transform.rotation.x = pose_map.pose.orientation.x
            #odom.transform.rotation.y = pose_map.pose.orientation.y
            #odom.transform.rotation.z = pose_map.pose.orientation.z
            #odom.transform.rotation.w = pose_map.pose.orientation.w
            #self.br.sendTransform(odom)

            
    
    def publish_odometry(self,time = None):
        """
        Publish odometry message and transform 
        """
        if time is None:
            time = rospy.Time.now()
            
        odom = Odometry()
        odom.header.stamp = time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"        
        odom.twist.twist.linear.x = self.mu[0]
        odom.twist.twist.angular.z = self.mu[1]
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        self.odom_pub.publish(odom)

        # Publish the transform 
        # Add new x, y and yaw to transform, first cart then rot
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.header.stamp = time
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0 # z is always 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta) # transform yaw to quaternion
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)


    def run(self):
        """
        Run the node. 
        Don't change anything here, change main instead.
        """
        
        # Run as long as node is not shutdown
        while not rospy.is_shutdown():
            self.main()
            self.rate.sleep()


    def imu_callback(self,msg):
        """
        Imu callback
        """

        # Get angular velocity in rad/s
        self.imu = np.array([0,msg.angular_velocity.z])
        


        

    
    
    def encoder_callback(self,msg):
        """
        Encoder callback
        """

        # Calc v_left and v_right
        v_left = (((msg.delta_encoder_left/ self.ticks_per_rev ) * 2*pi * self.wheel_r )/ msg.delta_time_left)*1000
        v_right = (((msg.delta_encoder_right/ self.ticks_per_rev ) * 2*pi * self.wheel_r )/ msg.delta_time_right)*1000

        
        # calculate v, omega
        v, omega = self.transform_v_left_v_right_to_v_omega(v_left, v_right)

        self.enco = np.array([v,omega])        

    def cmd_vel_callback(self, msg):
        """
        This node subscribes to the /cmd_vel topic and converts the linear and angular velocity
        It then updates the internal variables that are used to publish the duty cycle message
        """
        self.u = [msg.linear.x,msg.angular.z]


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

    node=ekf_odometry()
    node.run()
