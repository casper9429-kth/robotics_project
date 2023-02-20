#!/usr/bin/env python
import rospy
from tf2_geometry_msgs import PoseStamped
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


# alt 2
# Semi offline EKF SLAM
# collect data from aruco markers and odometry for 0.5 seconds
# associate odometry with aruco markers and create data time line 
# run EKF SLAM on the data time line, and publish the result, correct odometry location with the result




class ekf_slam():
    def __init__(self):
        """
        EKF SLAM
        """
        rospy.init_node('ekf_slam')


        # Subscribe to odometry topic
        self.reset_odom_cov_sub = rospy.Subscriber("odom_updater/reset_odom_cov", Bool,self.odom_cov_reset_callback)
        self.bl_in_map_slam_sub = rospy.Subscriber("odom_slam", Odometry, self.bl_in_map_slam_callback)
        self.aruco_marker_sub = rospy.Subscriber("aruco/markers", MarkerArray, self.aruco_marker_callback)

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

        self.reset_odom_sigma = rospy.Time.now().to_sec()

        # Var to store aruco belief 
        self.aruco_belif_buffer = defaultdict()
        new_marker = defaultdict()
        new_marker['type'] = 'aruco'
        new_marker['x'] = 0
        new_marker['y'] = 0
        new_marker['theta'] =  0# tf.transformations.euler_from_quaternion([new_aruco.transform.rotation.x,new_aruco.transform.rotation.y,new_aruco.transform.rotation.z,new_aruco.transform.rotation.w])[2]
        new_marker['covariance'] = np.eye(3)*1000000000000000000 # high covariance, robot will pin the marker to its location when seen next time
        self.aruco_belif_buffer = defaultdict(lambda: new_marker)

        
        # Anchor id
        self.anchor_id = 3
        
        self.aruco_state_vector = defaultdict()
        self.seen_aruco_ids = set()
        self.slam_cov = np.zeros((2,2))
        self.Q = np.array([[0.025, 0],
                                [0, 0.025]])

        # Odometry var 
        self.odometry_buffer = []
        self.aruco_buffer = []
        self.slam_buffer = []
        self.latest_time_belif = rospy.Time.now()
        self.covariance_belif = np.zeros((3,3))        
        self.R = np.array([[0.0001, 0, 0],
                                [0, 0.0001, 0],
                                [0, 0, 0.0001]])
                
        # Time var
        self.last_time = rospy.Time.now().to_sec()
        self.current_time_sec = rospy.Time.now().to_sec()


    def aruco_marker_callback(self,msg):
        """
        Aruco Callback, lookup aruco marker in map_SLAM frame
        Save the data in a buffer, 
        only allow one update per marker per second
        """
        
        for marker in msg.markers:
            if marker.id == self.anchor_id:
                return

            # if marker is not in the buffer, add it        
            try:
                new_aruco = self.tfBuffer.lookup_transform("map_SLAM", "aruco/detected" + str(marker.id), rospy.Time(0))                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            new_marker = defaultdict()
            new_marker['type'] = 'aruco'
            new_marker['id'] = marker.id
            new_marker['x'] = new_aruco.transform.translation.x
            new_marker['y'] = new_aruco.transform.translation.y
            new_marker['theta'] = tf.transformations.euler_from_quaternion([new_aruco.transform.rotation.x,new_aruco.transform.rotation.y,new_aruco.transform.rotation.z,new_aruco.transform.rotation.w])[2]
            new_marker['t'] = msg.header.stamp.to_sec()
            new_marker['t_stamp'] = rospy.Time.now()
            self.slam_buffer.append(new_marker)


                

    def bl_in_map_slam_callback(self,msg):
        """
        Saves odometry data in a buffer and updates the covariance of the robot pose in the map frame
        """
        # Update step for EKF SLAM
        x_belif = msg.pose.position.x
        y_belif = msg.pose.position.y
        theta_belif = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])[2]
        t = msg.header.stamp.to_sec()
        dt = (msg.header.stamp - self.latest_time_belif).to_sec()
        self.latest_time_belif = msg.header.stamp
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        # Update the covariance of the robot pose in the map frame
        self.covariance_belif = self.calc_odom_sigma_bel(dt,x_belif,y_belif,theta_belif,v,omega,t,self.covariance_belif,self.R)        
        
        # Save the data 
        new_odom = defaultdict()
        new_odom['type'] = "odometry"
        new_odom['x'] = x_belif
        new_odom['y'] = y_belif
        new_odom['theta'] = theta_belif
        new_odom['t'] = t
        new_odom['dt'] = dt
        new_odom['v'] = v
        new_odom['omega'] = omega
        new_odom['covariance'] = self.covariance_belif
        self.odometry_buffer.append(new_odom)
        self.slam_buffer.append(new_odom)

        if self.debug:
            print("bl_in_map_slam_callback")
        

    def calc_odom_sigma_bel(self,dt,x,y,theta,v,omega,time,odom_sigma,R_odom):
        """
        Calculates the sigma for the odometry during the ekf prediction step
        * dt = time since last update
        * x,y,theta = robot pose in map_SLAM frame
        * v,omega = robot velocity
        * time = current time
        * odom_sigma = current odom sigma
        * R_odom = odometry covariance noise matrix/model

        returns the new odom sigma for the ekf prediction step
        """
        
        ### Calculate/Update G_odom, used for odometry covariance
        self.G_odom = np.array([[1, 0, -v * math.sin(theta) * dt],
                       [0, 1, v * math.cos(theta) * dt],
                       [0, 0, 1]])
        
        ### If time since last reset odom cov is less than 0.1, reset odom cov, will be reset when anchor is found 
        if np.abs(time - self.reset_odom_sigma) < 0.1:
            return np.zeros((3,3))
        elif np.abs(v)<0.01 and np.abs(omega)<0.01: ### If robot is not moving, don't update/increase odom cov
            return odom_sigma
        else: ## If robot is moving, update odom cov.             
            return self.G_odom @ odom_sigma @ self.G_odom.T + R_odom


    def odom_cov_reset_callback(self,msg):
        """
        Reset odom covariance callback
        """
        self.reset_odom_sigma = rospy.Time.now().to_sec()
        new_msg = defaultdict()
        new_msg['type'] = "reset_odom_cov"
        new_msg['t'] = msg.header.stamp.to_sec()
        self.slam_buffer.append(new_msg)

    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function
        write your code here to make it more readable.
        """
        
        # threshold for the number of messages in the buffer        
        if len(self.slam_buffer) < 1000:
            return

        # Clear the buffer
        slam_buffer = sorted(self.slam_buffer, key=lambda k: k['t'])
        self.slam_buffer = []


        # Find the first odometry message in the buffer and make it startingpoint for the EKF SLAM
        for i,msg in enumerate(slam_buffer):
            if msg['type'] == 'odometry':
                t = msg['t']
                x = slam_buffer[0]['x']
                y = slam_buffer[0]['y']
                theta = slam_buffer[0]['theta']
                slam_buffer = slam_buffer[i+1:]
                self.slam_cov = self.calc_odom_sigma_bel(msg['dt'],x,y,theta,msg['v'],msg['omega'],t,self.slam_cov,self.R)
                break

        
        # loop through the buffer and run the EKF SLAM
        i = -1
        for i_int,_ in enumerate(slam_buffer):
            i += 1
            msg = slam_buffer[i]
            
            
            if msg['type'] == 'reset_odom_cov':
                # Reset the odometry covariance
                # This is done when the robot finds an anchor
                # Also reset the robot pose in the map_SLAM frame
                # according to the newest odometry message
                self.slam_cov = self.slam_cov * 0.0
                # Find next odometry message
                for new_index,temp_msg in enumerate(slam_buffer[i+1:]):
                    if temp_msg['type'] == 'odometry':
                        latest_t = temp_msg['t']
                        x = temp_msg['x']
                        y = temp_msg['y']
                        theta = temp_msg['theta']
                        i = new_index+1
                        continue
                
                # if we get here, there is no odometry message in the buffer
                # so we have to wait for the next iteration of the main loop
                # to get the next odometry message
                
                # normally at the end, we should have published the new transforms, waited and then wiped the buffer
                # but not this time, we just exit and hopefully another odometry message will come in the next iteration
                # this scenario should be very rare though
                return
                
                
            
            
            if msg['type'] == 'odometry':
                # Run the EKF SLAM prediction step
                latest_t = msg['t']
                dt_belif = msg['dt']
                x = x + msg['v']*math.cos(theta)*dt_belif
                y = y + msg['v']*math.sin(theta)*dt_belif
                theta = theta + msg['omega']*dt_belif 
                self.slam_cov = self.calc_odom_sigma_bel(msg['dt'],x,y,theta,msg['v'],msg['omega'],t,self.slam_cov,self.R)                
                continue
            
            if msg['type'] == 'aruco':
                # Run the EKF SLAM update step

                ## get belif
                mark_belif = self.aruco_belif_buffer[msg['id']]
                m_b = [mark_belif['x'],mark_belif['y']]
                m_b_c = mark_belif['covariance']
                r_b = [x,y]
                r_b_c = self.slam_cov
                z_b = np.array([math.sqrt((m_b[0]-r_b[0])**2 + (m_b[1]-r_b[1])**2),math.atan2(m_b[1]-r_b[1],m_b[0]-r_b[0])])

                ## Linearize the measurement model around the belif
                H = self.calc_H(r_b[0],r_b[1],m_b[0],m_b[1])

                ## get measurement
                m_m = [msg['x'],msg['y']]
                z_m = np.array([math.sqrt((m_m[0]-r_b[0])**2 + (m_m[1]-r_b[1])**2),math.atan2(m_m[1]-r_b[1],m_m[0]-r_b[0])])

                ## Create the state vector
                mu_bel = np.array(r_b+m_b) # x,y,mx,my
                
                ## Create the covariance matrix
                aruco_sigma = np.zeros((4,4))
                aruco_sigma[0:2,0:2] = r_b_c
                aruco_sigma[2:4,2:4] = m_b_c

                ## kalman update step
                aruco_k = aruco_sigma @ H.T @ np.linalg.inv(H @ aruco_sigma @ H.T + self.Q)
                aruco_mu = mu_bel + aruco_k @ (z_m - z_b)
                aruco_sigma = (np.eye(4) - aruco_k @ H) @ aruco_sigma

                # Save new data
                x = aruco_mu[0]
                y = aruco_mu[1]
                self.slam_cov = aruco_sigma[0:2,0:2]
                
                self.aruco_belif_buffer[msg['id']]['x'] = aruco_mu[2]
                self.aruco_belif_buffer[msg['id']]['y'] = aruco_mu[3]
                self.aruco_belif_buffer[msg['id']]['covariance'] = aruco_sigma[2:4,2:4]
                
                

        # When done, update the position of odom to correct the position of the robot in the map_SLAM frame
        
        ## a new belif is created for the robot pose in the map_SLAM frame
        
        ### move odom so that
        # lookup 

        # update frames and all data 

        # a small delay to make sure the data is published and all frames are updated before the next batch of data is added

        # Clear the slam buffer and the aruco buffer and the odometry buffer, no new data is allowed to be collected until the EKF SLAM is done

    def calc_H(self,x,y,mx,my):
        """
        Calculates the linearized messurement model for the aruco marker
        """
        # Linearize the messurement model with respect to x_belif,y_belif,m1x_belif,m1y_belif and construct H
        h_0_0 = (1/np.sqrt((x - mx)**2 + (y - my)**2))*(x - mx)
        h_0_1 = (1/np.sqrt((x - mx)**2 + (y - my)**2))*(y - my)
        h_0_2 = (1/np.sqrt((x - mx)**2 + (y - my)**2))*(-x + mx)
        h_0_3 = (1/np.sqrt((x - mx)**2 + (y - my)**2))*(-y + my)
        h_1_0 = (1/(1+((my - y)/ (mx - x))**2))*((my - y)/ ((mx - x)**2))
        h_1_1 = (1/(1+((my - y)/ (mx - x))**2))*((-y)/ (mx - x))
        h_1_2 =-(1/(1+((my - y)/ (mx - x))**2))*((my - y)/ ((mx - x)**2))
        h_1_3 = (1/(1+((my - y)/ (mx - x))**2))*((my)/ (mx - x))
        H = np.array([[h_0_0,h_0_1,h_0_2,h_0_3],[h_1_0,h_1_1,h_1_2,h_1_3]])
        return H
    


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
    node=ekf_slam()
    node.run()
