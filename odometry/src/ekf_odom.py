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
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Bool

class ekf_slam():
    def __init__(self):
        """
        Peforms EKF odometry and EKF aruco slam
        
        The x,y,theta predict and v,omega predict and update is done in run 
        
        The x,y,theta update is done in aruco_callback
        """
        rospy.init_node('ekf_slam')

        # Subscribers
        self.sub_goal = rospy.Subscriber('/motor/encoders', Encoders, self.encoder_callback)
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.duty_cycle_pub = rospy.Subscriber('/motor/duty_cycles', DutyCycles, self.duty_cycle_callback)
        #self.aruco_sub = rospy.Subscriber('/aruco/markers', MarkerArray, self.aruco_callback)
        
        self.reset_odom_cov_sub = rospy.Subscriber("odom_updater/reset_odom_cov", Bool,self.odom_cov_reset_callback)
                                          
        # Publisher# Publish the map and the odometry
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)


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
        ## v,omega fusion
        self.R = np.eye(2) * 100  
        self.Q_enco = np.eye(2)
        self.Q_enco[0,0] = 100
        self.Q_enco[1,1] = 10
        self.Q_imu = np.eye(2)
        self.Q_imu[0,0] = 100000000 # Must be very high, because the imu is not very accurate at determining the speed
        self.Q_imu[1,1] = 0.1
        ## aruco slam
        ### x,y,theta predict
        self.odom_sigma = np.eye(3)*0 # Predict cov
        self.reset_odom_sigma = 0 # Reset odom cov when achor is found
        self.R_odom = np.eye(3) 
        self.R_odom[0,0] = 0.00001
        self.R_odom[1,1] = 0.00001
        self.R_odom[2,2] = 0.0000001        
        
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
        
        # Only predict
        self.x += self.mu[0] * math.cos(self.theta) * dt
        self.y += self.mu[0] * math.sin(self.theta) * dt
        self.theta = self.theta + self.mu[1] * dt
        # Calculate/Update G_odom 
        self.G_odom = np.array([[1, 0, -self.mu[0] * math.sin(self.theta) * dt],
                       [0, 1, self.mu[0] * math.cos(self.theta) * dt],
                       [0, 0, 1]])
        
        # If anchor is found, reset odom cov
        if np.abs(self.current_time - self.reset_odom_sigma) < 0.1:
            self.odom_sigma = np.zeros((3,3))
        elif np.abs(self.mu[0])<0.01 and np.abs(self.mu[1])<0.01:
            pass
        else:
            # R_odom should get the error from v_omega fusion            
            # self.sigma[0] is var in v
            # self.sigma[1] is var in omega
            # if v omega is high error, then R_odom should be high
            # if time is high, then R_odom should be high
            # Include linearization

            # error x: (self.mu[0] + v) * math.cos(self.theta + omega*dt) * dt 
            # lin x:   math.cos(self.theta + omega*dt) * dt, -(self.mu[0] + v) * math.sin(self.theta + omega*dt) * dt * dt
            # error y: self.mu[0] * math.sin(self.theta) * dt
            # lin y:  math.sin(self.theta + omega*dt) * dt, self.mu[0] * math.cos(self.theta + omega*dt) * dt * dt
            # error theta: self.mu[1] * dt
            # lin theta: 0, dt
            # x_y_theta_lin = np.array([[math.cos(self.theta + self.mu[1]*dt) * dt, -(self.mu[0] + self.mu[0]) * math.sin(self.theta + self.mu[1]*dt) * dt * dt],[0,dt]])
            # we can disregard the mu[]*dt terms since they are small
            x_y_theta_lin = [[math.cos(self.theta) * dt, -(self.mu[0]) * math.sin(self.theta) * dt * dt],[math.sin(self.theta) * dt, self.mu[0] * math.cos(self.theta) * dt * dt],[0,dt]]
            x_y_theta_lin = np.array(x_y_theta_lin)

            # Include the errors from self.sigma and the error constant term to keep it from becoming zero            
            R_odom = x_y_theta_lin @ self.sigma @ x_y_theta_lin.T + self.R_odom
            
            self.odom_sigma = self.G_odom @ self.odom_sigma @ self.G_odom.T + R_odom
        
        rospy.loginfo(self.odom_sigma)
        
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


    def odom_cov_reset_callback(self,msg):
        """
        Reset odom covariance callback
        """
        self.reset_odom_sigma = rospy.Time.now().to_sec()
        #self.odom_sigma = np.zeros((3,3))#np.eye(3)*0

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

    node=ekf_slam()
    node.run()
