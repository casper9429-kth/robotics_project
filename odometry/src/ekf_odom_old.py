# V1.0
# x,y can't be messrured
# theta can be measured

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
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robp_msgs.msg import DutyCycles
from geometry_msgs.msg import PoseWithCovarianceStamped

class ekf_odometry():
    def __init__(self):
        """
        Peforms EKF odometry
        """
        rospy.init_node('ekf_odometry')

        # Subscribers
        self.sub_goal = rospy.Subscriber('/motor/encoders', Encoders, self.encoder_callback)
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        #self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.duty_cycle_pub = rospy.Subscriber('/motor/duty_cycles', DutyCycles, self.duty_cycle_callback)

        
        # Publisher# Publish the map and the odometry
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.pose_with_covariance_pub = rospy.Publisher("pose_with_covariance", PoseWithCovarianceStamped, queue_size=50)


        # TF Stuff
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()



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
        self.current_time = rospy.Time.now().to_sec()

        


    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """
        
        # State vector is [v,omega]
        

        if self.count == 0:
            # Initialize state vector
            self.mu = np.array([0,0])
            self.sigma = np.eye(2)
            self.mu_bel = np.array([0,0])
            self.simga_bel = np.eye(2)
            self.count += 1        
            return
        

        # predict step
        self.mu_bel = self.u
        self.simga_bel = np.eye(2) @ self.sigma @ np.eye(2).T + self.R



        # update step enco
        H = np.eye(2)
        
        K = self.simga_bel @ H.T @ np.linalg.inv(H @ self.simga_bel @ H.T + self.Q_enco)
        self.mu_bel = self.mu_bel + K @ (self.enco - self.mu_bel)
        self.simga_bel = (np.eye(2) - K @ H) @ self.simga_bel        
        
                
        # update step imu
        H = np.eye(2)
        K = self.simga_bel @ H.T @ np.linalg.inv(H @ self.simga_bel @ H.T + self.Q_imu)
        self.mu = self.mu_bel + K @ (self.imu - self.mu_bel)
        self.simga = (np.eye(2) - K @ H) @ self.simga_bel



        # Perform odometry update
        current_time = rospy.Time.now()
        self.current_time = current_time.to_sec()
        dt = self.current_time - self.last_time
        
        
        if dt > self.update_dt*1.1:
            rospy.logwarn("refresh rate to high: %f", dt)
        
        
        self.x += self.mu[0] * math.cos(self.theta) * dt
        self.y += self.mu[0] * math.sin(self.theta) * dt
        self.theta = self.theta + self.mu[1] * dt
        
        
        # Publish the odometry message
        self.publish_odometry(current_time)

        # Update time        
        self.last_time = self.current_time
    
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

        odom_with_cov = PoseWithCovarianceStamped() 
        odom_with_cov.header.stamp = time
        odom_with_cov.header.frame_id = "odom"
        odom_with_cov.pose.pose.position.x = self.x
        odom_with_cov.pose.pose.position.y = self.y
        odom_with_cov
        odom_with_cov.pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        self.pose_with_covariance_pub.publish(odom_with_cov)


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
        #v_left = (((msg.delta_encoder_left/ self.ticks_per_rev ) * 2*pi * self.wheel_r )/ msg.delta_time_left)*1000
        #v_right = (((msg.delta_encoder_right/ self.ticks_per_rev ) * 2*pi * self.wheel_r )/ msg.delta_time_right)*1000

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



if __name__ == "__main__":

    node=ekf_odometry()
    node.run()
