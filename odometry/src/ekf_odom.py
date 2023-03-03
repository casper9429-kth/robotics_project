# V1.0
# x,y can't be messrured
# theta can be measured

#!/usr/bin/env python
import rospy
from tf2_geometry_msgs import PoseStamped
#from tf2_geometry_msgs import TransformStamped
from tf2_geometry_msgs import PoseStamped
#from tf2_geometry_msgs import TransformStamped
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
from  math import pi
import tf
from sensor_msgs.msg import Imu
import numpy as np
from nav_msgs.msg import Odometry
from robp_msgs.msg import DutyCycles
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Bool
from collections import defaultdict
# from geometry_msgs.msg import PoseStamped

# EKF Aruco Slam, v,omega fusion, odometry in one script
# This node pefors EKF Aruco Slam with odometry, v,omega fusion 
# When the aruco marker with id 3 is seen, the odom is set to the center of the map



class ekf_odom():
    def __init__(self):
        """
        Peforms EKF odometry
        
        The x,y,theta predict and v,omega predict and update is done in run 
        """
        rospy.init_node('ekf_odom')

        # Subscribers
        self.sub_goal = rospy.Subscriber('/motor/encoders', Encoders, self.encoder_callback)
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.duty_cycle_pub = rospy.Subscriber('/motor/duty_cycles', DutyCycles, self.duty_cycle_callback)
                                          
        # Publish the map and the odometry
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_slam_pub = rospy.Publisher("odom_slam", Odometry, queue_size=50) # remove
        


        # TF Stuff
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()

        # Settings
        self.debug = False

        # Define rate
        self.update_rate = 100 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 
        
        # Robot parameters
        self.ticks_per_rev = 3072
        self.wheel_r = 0.04921
        self.base = 0.3 
        
        # EKF var
        self.count = 0
        self.mu_bel = np.array([0,0])
        self.simga_bel = np.eye(2)
        self.mu = np.array([0,0])
        self.sigma = np.eye(2)
        
        # Sensor var
        self.enco = np.array([0,0])
        self.imu = np.array([0,0])
        self.u = np.array([0,0])
        
        # EKF var 
        ## v,omega fusion
        self.R = np.eye(2) * 100  
        self.Q_enco = np.eye(2)
        self.Q_enco[0,0] = 100
        self.Q_enco[1,1] = 10
        self.Q_imu = np.eye(2)
        self.Q_imu[0,0] = 100000000 # Must be very high, because the imu is not very accurate at determining the speed
        self.Q_imu[1,1] = 0.1
        
        # Odometry var 
        self.x = 0
        self.y = 0
        self.theta = 0
        
        # Time var
        self.last_time = rospy.Time.now().to_sec()
        self.current_time_sec = rospy.Time.now().to_sec()
        self.current_time_sec = rospy.Time.now().to_sec()

        


    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function
        write your code here to make it more readable.
        """
        
        
        ########## Init state v,omega:  sensor fusion ##########
        if self.count == 0:
            self.init_sensor_fusion()
            return
        
        self.ekf_v_omega_predict()

        self.ekf_v_omega_update_enco()

        self.ekf_v_omega_update_imu()                

        self.time_calculations_for_odom()

        self.predict_odometry()        

        self.publish_odometry(self.current_time)

        # Update time        
        self.last_time = self.current_time_sec


    def time_calculations_for_odom(self):
        """
        Peform time calculations needed for odometry
        """
        ## Time calculation, used
        self.current_time = rospy.Time.now()
        self.current_time_sec = self.current_time.to_sec()
        self.dt = self.current_time_sec - self.last_time
        ## Time warning
        if self.dt > self.update_dt*1.2 and self.debug:
            rospy.logwarn("refresh rate to high: %f", self.dt)
            rospy.logwarn("refresh rate should be: %f", self.update_dt)



    def init_sensor_fusion(self):
        """
        Init ekf loop
        """
        # Initialize state vector
        self.mu = np.array([0,0])
        self.sigma = np.eye(2)
        self.mu_bel = np.array([0,0])
        self.simga_bel = np.eye(2)
        self.count += 1


    
    def predict_odometry(self):
        """
        Predicts the odometry using the odometry model for the ekf aruco prediction step        
        """
        ## Predict odometry
        self.x += self.mu[0] * math.cos(self.theta) * self.dt
        self.y += self.mu[0] * math.sin(self.theta) * self.dt
        self.theta = self.theta + self.mu[1] * self.dt


    
    
    def ekf_v_omega_predict(self):
        """
        Peforms the predict step in v_omega ekf
        """
        # predict step
        self.mu_bel = self.u
        self.simga_bel = np.eye(2) @ self.sigma @ np.eye(2).T + self.R
        
    def ekf_v_omega_update_imu(self):
        """
        Peforms the update step from imu messurement for the v_omega_ekf
        """
        # update step imu
        H = np.eye(2)
        K = self.simga_bel @ H.T @ np.linalg.inv(H @ self.simga_bel @ H.T + self.Q_imu)
        self.mu = self.mu_bel + K @ (self.imu - self.mu_bel)
        self.simga = (np.eye(2) - K @ H) @ self.simga_bel


    def ekf_v_omega_update_enco(self):
        """
        Peforms the update step from encoder messurement for the v_omega_ekf
        """
        # update step enco
        H = np.eye(2)
        K = self.simga_bel @ H.T @ np.linalg.inv(H @ self.simga_bel @ H.T + self.Q_enco)
        self.mu_bel = self.mu_bel + K @ (self.enco - self.mu_bel)
        self.simga_bel = (np.eye(2) - K @ H) @ self.simga_bel        

    
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
        odom.pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        self.odom_pub.publish(odom)


        # Publish the transform 
        # Add new x, y and yaw to transform, first cart then rot
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.header.stamp = time
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        #t.transform.translation.z = 0 # z is always 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta) # transform yaw to quaternion
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)
        
        # Create pose stampes 
        odom_bl = PoseStamped()
        odom_bl.header.stamp = rospy.Time(0)
        odom_bl.header.frame_id = "odom"
        odom_bl.pose.position.x = self.x
        odom_bl.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom_bl.pose.orientation.x = q[0]
        odom_bl.pose.orientation.y= q[1]
        odom_bl.pose.orientation.z = q[2]
        odom_bl.pose.orientation.w= q[3]

        try:
            map_to_odom_bl = self.tfBuffer.transform(odom_bl, "map")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
            return

        map_SLAM_to_base_link = self.PoseStamped_to_Odometry(map_to_odom_bl,self.mu[0],self.mu[1])
        map_SLAM_to_base_link.header.stamp = time
        self.odom_slam_pub.publish(map_SLAM_to_base_link)
        
    def transform_stamped_to_odom(self,tfs,v,omega):
        """
        Convert tf stamped to Odometry
        """
        odom = Odometry()
        odom.header = tfs.header
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        odom.pose.pose.position.x = tfs.transform.translation.x
        odom.pose.pose.position.y = tfs.transform.translation.y
        odom.pose.pose.position.z = tfs.transform.translation.z
        odom.pose.pose.orientation.x = tfs.transform.rotation.x
        odom.pose.pose.orientation.y = tfs.transform.rotation.y
        odom.pose.pose.orientation.z = tfs.transform.rotation.z
        odom.pose.pose.orientation.w = tfs.transform.rotation.w
        return odom

    def PoseStamped_to_Odometry(self,ps,v,omega):
        """
        Convert Pose stamped to Odometry
        """
        odom = Odometry()
        odom.header = ps.header
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        odom.pose.pose.position.x = ps.pose.position.x
        odom.pose.pose.position.y = ps.pose.position.y
        odom.pose.pose.position.z = ps.pose.position.z
        odom.pose.pose.orientation.x = ps.pose.orientation.x
        odom.pose.pose.orientation.y = ps.pose.orientation.y
        odom.pose.pose.orientation.z = ps.pose.orientation.z
        odom.pose.pose.orientation.w = ps.pose.orientation.w
        return odom                
    def transformed_stamped_to_PoseStamped(self,tfs):
        """
        Convert tf stamped to Pose stamped
        """
        ps = PoseStamped()
        ps.header = tfs.header
        ps.pose.position.x = tfs.transform.translation.x
        ps.pose.position.y = tfs.transform.translation.y
        ps.pose.position.z = tfs.transform.translation.z
        ps.pose.orientation.x = tfs.transform.rotation.x
        ps.pose.orientation.y = tfs.transform.rotation.y
        ps.pose.orientation.z = tfs.transform.rotation.z
        ps.pose.orientation.w = tfs.transform.rotation.w
        return ps
    
    def PoseStamped_to_transformed_stamped(self,ps):
        """
        Convert Pose stamped to tf stamped
        """
        tfs = TransformStamped()
        tfs.header = ps.header
        tfs.transform.translation.x = ps.pose.position.x
        tfs.transform.translation.y = ps.pose.position.y
        tfs.transform.translation.z = ps.pose.position.z
        tfs.transform.rotation.x = ps.pose.orientation.x
        tfs.transform.rotation.y = ps.pose.orientation.y
        tfs.transform.rotation.z = ps.pose.orientation.z
        tfs.transform.rotation.w = ps.pose.orientation.w
        return tfs
                               
    def transform_old_map_pose_into_new_frame(self,pose_in_map, new_frame='odom',time = None):
        """
        Takes old pose in map frame and transforms it into new frame at latest time
        Accepts PoseStamped or TransformStamped
        """
        if time is None:
            time = rospy.Time.now()
        
        if type(pose_in_map) is PoseStamped:
            pose_map_stamped = PoseStamped()
            pose_map_stamped.header.stamp = time
            pose_map_stamped.header.frame_id = 'map'
            pose_map_stamped.pose.position.x = pose_in_map.pose.position.x
            pose_map_stamped.pose.position.y = pose_in_map.pose.position.y
            pose_map_stamped.pose.position.z = pose_in_map.pose.position.z
            pose_map_stamped.pose.orientation.x = pose_in_map.pose.orientation.x
            pose_map_stamped.pose.orientation.y = pose_in_map.pose.orientation.y
            pose_map_stamped.pose.orientation.z = pose_in_map.pose.orientation.z
            pose_map_stamped.pose.orientation.w = pose_in_map.pose.orientation.w



        elif type(pose_in_map) is TransformStamped:
            pose_map_stamped = PoseStamped()
            pose_map_stamped.header.stamp = time
            pose_map_stamped.header.frame_id = 'map'
            pose_map_stamped.pose.position.x = pose_in_map.transform.translation.x
            pose_map_stamped.pose.position.y = pose_in_map.transform.translation.y
            pose_map_stamped.pose.position.z = pose_in_map.transform.translation.z
            pose_map_stamped.pose.orientation.x = pose_in_map.transform.rotation.x
            pose_map_stamped.pose.orientation.y = pose_in_map.transform.rotation.y
            pose_map_stamped.pose.orientation.z = pose_in_map.transform.rotation.z
            pose_map_stamped.pose.orientation.w = pose_in_map.transform.rotation.w
        else:
            return None

        try:
            new_pose = self.tfBuffer.transform(pose_map_stamped,new_frame,rospy.Duration(0.0))
            return new_pose
        except:
            return None





    def duty_cycle_callback(self,msg):
        """
        Duty cycle callback
        """
        
        # Calc v_left and v_right
        v_left = msg.duty_cycle_left
        v_right = msg.duty_cycle_right

        # calculate v, omega
        v, omega = self.transform_v_left_v_right_to_v_omega(v_left, v_right)

        self.u = np.array([v,omega])

    def imu_callback(self,msg):
        """
        Imu callback
        """

        # Get angular velocity in rad/s
        self.imu = np.array([0,-msg.angular_velocity.z])    
    
    def encoder_callback(self,msg):
        """
        Encoder callback
        """

        # Calc v_left and v_right
        mean = 50#(msg.delta_time_left + msg.delta_time_right)/2
        v_left = (((msg.delta_encoder_left/ self.ticks_per_rev ) * 2*pi * self.wheel_r )/ mean)*1000
        v_right = (((msg.delta_encoder_right/ self.ticks_per_rev ) * 2*pi * self.wheel_r )/ mean)*1000

        # calculate v, omega
        v, omega = self.transform_v_left_v_right_to_v_omega(v_left, v_right)

        self.enco = np.array([v,omega])        

    def cmd_vel_callback(self, msg):
        """
        This node subscribes to the /cmd_vel topic and converts the linear and angular velocity
        It then updates the internal variables that are used to publish the duty cycle message
        """
        self.u = np.array([msg.linear.x,msg.angular.z])


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


    def run(self):
        """
        Run the node. 
        Don't change anything here, change main instead.
        """
        
        # Run as long as node is not shutdown
        while not rospy.is_shutdown():
            self.main()
            self.rate.sleep()


if __name__ == "__main__":
    node=ekf_odom()
    node.run()
