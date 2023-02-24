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






class ekf_slam():
    def __init__(self):
        """
        EKF SLAM
        """
        rospy.init_node('ekf_slam')


        # TF Stuff
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(100))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.sbr = tf2_ros.StaticTransformBroadcaster()

        
        # Robot parameters
        self.ticks_per_rev = 3072
        self.wheel_r = 0.04921
        self.base = 0.3 

        self.latest_time = rospy.Time.now()
        self.slam_ready = False
        self.anchor_id = 500
        self.time_threshold = 0.5

        
        # Var to store aruco belief 
        self.aruco_seen = set()
        self.aruco_belif_buffer = defaultdict()
        new_marker = defaultdict()
        new_marker['type'] = 'aruco'
        new_marker['x'] = 0
        new_marker['y'] = 0
        new_marker['z'] = 0
        new_marker['theta'] = 0
        new_marker['first_measurement'] = True
        new_marker['theta'] =  0# tf.transformations.euler_from_quaternion([new_aruco.transform.rotation.x,new_aruco.transform.rotation.y,new_aruco.transform.rotation.z,new_aruco.transform.rotation.w])[2]
        new_marker['covariance'] = np.eye(2)*100000000000000000000000000000000000000 # high covariance, robot will pin the marker to its location when seen next time
        self.aruco_belif_buffer = defaultdict(lambda: new_marker) 
        self.cov = np.zeros((3,3))
        # base 0.2
        self.Q = np.array([[0.02, 0],
                                [0, 0.02]]) 

        # map to odom transform

        self.latest_anchor_time = rospy.Time.now().to_sec()
        self.odom = TransformStamped()
        self.odom.header.stamp = rospy.Time.now() #maybe change this
        self.odom.header.frame_id = "map"
        self.odom.child_frame_id = "odom"
        self.odom.transform.translation.x = 0
        self.odom.transform.translation.y = 0
        self.odom.transform.translation.z = 0
        self.odom.transform.rotation.x = 0
        self.odom.transform.rotation.y = 0
        self.odom.transform.rotation.z = 0
        self.odom.transform.rotation.w = 1
        self.sbr.sendTransform(self.odom)
                
        # Odometry var 
        self.buffer = []
        self.latest_odom_time = rospy.Time.now()
        self.R = np.array([[0.0000001, 0, 0],
                                [0, 0.0000001, 0],
                                [0, 0, 0.00000001]])
                
                
                
        # Subscribe to odometry topic
        self.bl_in_map_slam_sub = rospy.Subscriber("odom_slam", Odometry, self.bl_in_map_slam_callback)
        self.aruco_marker_sub = rospy.Subscriber("aruco/markers", MarkerArray, self.aruco_marker_callback)

                


    def aruco_marker_callback(self,msg):
        """
        Aruco Callback, lookup aruco marker in map_SLAM frame
        Save the data in a buffer, 
        only allow one update per marker per second
        """
        
        for marker in msg.markers:
            
            # check if aruco marker is to far away from robot
            try:
                t_robot_aruco = self.tfBuffer.lookup_transform("base_link", "aruco/detected" + str(marker.id),rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo(e)
                continue
            
            dist_to_robot = np.sqrt(t_robot_aruco.transform.translation.x**2 + t_robot_aruco.transform.translation.y**2 + t_robot_aruco.transform.translation.z**2)
            threshold = 3.0
            if dist_to_robot > threshold:
                continue
            
            
            
            
            

            
            
            # if anchor is seen, update odom transformation
            if marker.id == self.anchor_id:
                # set latest time
                self.latest_time = rospy.Time.now()

                # get latest odom message
                r_b = min(self.buffer, key=lambda x:abs(x['t']-msg.header.stamp.to_sec()))

                # reset cov first time
                self.cov = self.cov*0
                
                # Lookup transform from aruco to odom
                try:
                    t_map_goal_map = self.tfBuffer.lookup_transform("aruco/detected" + str(marker.id), "odom", rospy.Time(0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.loginfo(e)
                    continue
                
                # update odom transform
                self.odom = TransformStamped()
                self.odom.header.stamp = rospy.Time.now()
                self.odom.header.frame_id = "map"
                self.odom.child_frame_id = "odom"
                self.odom.transform.translation.x = t_map_goal_map.transform.translation.x
                self.odom.transform.translation.y = t_map_goal_map.transform.translation.y
                self.odom.transform.translation.z = t_map_goal_map.transform.translation.z
                self.odom.transform.rotation.x = t_map_goal_map.transform.rotation.x
                self.odom.transform.rotation.y = t_map_goal_map.transform.rotation.y
                self.odom.transform.rotation.z = t_map_goal_map.transform.rotation.z
                self.odom.transform.rotation.w = t_map_goal_map.transform.rotation.w
                self.sbr.sendTransform(self.odom)
                
                # set slam ready
                self.slam_ready = True
                                                
                # reset cov second time
                self.cov = self.cov*0                                
                continue


            # if allow slam
            if not self.slam_ready:
                continue

            # if time since latest reading is over threshold
            if rospy.Time.now().to_sec() - self.latest_time.to_sec() < self.time_threshold:                
                continue

            # if survived this far, update slam
            self.latest_time = rospy.Time.now()

            ################## SLAM ####################
            # robot belief
            r_belif = min(self.buffer, key=lambda x:abs(x['t']-msg.header.stamp.to_sec()))
            theta = r_belif['theta']
            time_odom = r_belif['t']
            r_b = [r_belif['x'],r_belif['y']]
            r_cov = self.cov[0:2,0:2]
            
            
            # Lookup measurement transform in map
            try:
                aruco_m = self.tfBuffer.lookup_transform("map", "aruco/detected" + str(marker.id), rospy.Time(0))                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo(e)
                continue

            # make sure if first measurement, then update aruco belief buffer and add to aruco seen set
            if self.aruco_belif_buffer[marker.id]['first_measurement']:
                self.aruco_seen.add(marker.id)
                self.aruco_belif_buffer[marker.id]['first_measurement'] = False
                self.aruco_belif_buffer[marker.id]['x'] = aruco_m.transform.translation.x
                self.aruco_belif_buffer[marker.id]['y'] = aruco_m.transform.translation.y
                self.aruco_belif_buffer[marker.id]['z'] = aruco_m.transform.translation.z
                                                                                             
            # measurement belief 
            mark_belif = self.aruco_belif_buffer[marker.id]
            m_b = [mark_belif['x'],mark_belif['y']]
            m_b_c = mark_belif['covariance']            

            # measurement belief            
            m_m = [aruco_m.transform.translation.x, aruco_m.transform.translation.y]
            

            # z_b and z_m
            z_b = np.array([m_b[0]-r_b[0],m_b[1]-r_b[1]])
            z_m = np.array([m_m[0]-r_b[0],m_m[1]-r_b[1]])



            # H
            H = self.calc_H(r_b[0],r_b[1],m_b[0],m_b[1])

            # N
            N = len(self.aruco_seen)


            # Construct Fx
            aruco_seen_list = list(self.aruco_seen)
            # find index of marker.id in aruco_seen_list
            j = aruco_seen_list.index(marker.id)
            Fx = np.zeros((4,2*N+2))
            Fx[0,0] = 1
            Fx[1,1] = 1

            # Construct mu_bel
            mu_bel = np.zeros((1,2*N+2))
            mu_bel[0,0] = r_b[0]
            mu_bel[0,1] = r_b[1]
            for i,id in enumerate(aruco_seen_list):
                x = self.aruco_belif_buffer[id]['x']
                y = self.aruco_belif_buffer[id]['y']
                mu_bel[0,2*i+2] = x
                mu_bel[0,2*i+3] = y

            Gt = np.eye(2*N+2)
            
            
            # construct sigma_bel
            sigma_bel = np.zeros((2*N+2,2*N+2))
            sigma_bel[0:2,0:2] = r_cov
            for i,id in enumerate(aruco_seen_list):
                sigma_bel[2*i+2:2*i+4,2*i+2:2*i+4] = self.aruco_belif_buffer[id]['covariance']
            
            # Q 
            Q = self.Q
            
            # Agument Fx by j
            Fx[2:4,2*j+2:2*j+4] =  np.eye(2)
            
            # Agument H
            H = H @ Fx
            
            # Calculate Kalman Gain
            K = sigma_bel @ H.T @ np.linalg.inv(H @ sigma_bel @ H.T + self.Q)

            # Calculate mu
            mu = mu_bel + K @ (z_m - z_b)
            
            # Calculate sigma
            sigma = (np.eye(2*N+2) - K @ H) @ sigma_bel 
            
            # Update mu and sigma for robot
            x_r = mu[0]
            y_r = mu[0]
            self.cov = sigma[0:2,0:2]
            
            # Update mu and sigma for aruco markers
            mu = mu[2:]
            sigma = sigma[2:,2:]
            for i,id in enumerate(aruco_seen_list):
                self.aruco_belif_buffer[id]['x'] = mu[2*i]
                self.aruco_belif_buffer[id]['y'] = mu[2*i+1]
                self.aruco_belif_buffer[id]['covariance'] = sigma[2*i:2*i+2,2*i:2*i+2]
            
            # Publish new pose of all aruco markers in map frame
            for id in aruco_seen_list:
                marker_pose = TransformStamped()
                marker_pose.header.stamp = rospy.Time.now()
                marker_pose.header.frame_id = "map"
                marker_pose.child_frame_id = "marker_belif_" + str(id)
                marker_pose.transform.translation.x = self.aruco_belif_buffer[id]['x']
                marker_pose.transform.translation.y = self.aruco_belif_buffer[id]['y']
                marker_pose.transform.translation.z = self.aruco_belif_buffer[id]['z']
                marker_pose.transform.rotation.x = 0
                marker_pose.transform.rotation.y = 0
                marker_pose.transform.rotation.z = 0
                marker_pose.transform.rotation.w = 1
                self.br.sendTransform(marker_pose)
            
            
            # Find transform that corrects map->odom such that base_link is given the new pose mu in map frame
            try:
                map_to_bs = self.tfBuffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo("error at 1")
                rospy.loginfo(e)
                return
            
            time_internal = rospy.Time.now()
            ## a new belif is created for the robot pose in the map_SLAM frame
            desired_base_link = TransformStamped()    
            desired_base_link.header.frame_id = "map"
            desired_base_link.header.stamp = time_internal
            desired_base_link.child_frame_id = "des_base_link"
            desired_base_link.transform.translation.x = x
            desired_base_link.transform.translation.y = y
            desired_base_link.transform.translation.z = map_to_bs.transform.translation.z
            quaterion = tf.transformations.quaternion_from_euler(0,0,theta)
            desired_base_link.transform.rotation.x = quaterion[0]
            desired_base_link.transform.rotation.y = quaterion[1]
            desired_base_link.transform.rotation.z = quaterion[2]
            desired_base_link.transform.rotation.w = quaterion[3]
            self.br.sendTransform(desired_base_link)



            ## look up transfrom from base_link to odom
            try:
                bs_to_odom = self.tfBuffer.lookup_transform("base_link", "odom", rospy.Time(time_odom), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo("error at 2")
                rospy.loginfo(e)
                return
            
            ## apply the transform to des_base_link to get des_odom
            desired_odom =  TransformStamped()
            desired_odom.header.frame_id = "des_base_link"
            desired_odom.child_frame_id = "des_odom"
            desired_odom.header.stamp = rospy.Time(time_odom)
            desired_odom.transform.translation.x = bs_to_odom.transform.translation.x
            desired_odom.transform.translation.y = bs_to_odom.transform.translation.y
            desired_odom.transform.translation.z = bs_to_odom.transform.translation.z
            desired_odom.transform.rotation.x = bs_to_odom.transform.rotation.x
            desired_odom.transform.rotation.y = bs_to_odom.transform.rotation.y
            desired_odom.transform.rotation.z = bs_to_odom.transform.rotation.z
            desired_odom.transform.rotation.w = bs_to_odom.transform.rotation.w
            self.br.sendTransform(desired_odom)


            ## lookup the transform from map to des_odom
            try:
                map_to_new_odom = self.tfBuffer.lookup_transform("map", "des_odom", rospy.Time(time_odom), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo("error at 3")
                rospy.loginfo(e)
                return
            
            ## publish the transform from map to des_odom to odom, make it static
            self.odom_belif = TransformStamped()
            self.odom_belif.header.frame_id = "map"
            self.odom_belif.child_frame_id = "odom"
            self.odom_belif.header.stamp = rospy.Time.now()#rospy.Time(latest_t)
            self.odom_belif.transform.translation.x = map_to_new_odom.transform.translation.x
            self.odom_belif.transform.translation.y = map_to_new_odom.transform.translation.y 
            self.odom_belif.transform.translation.z = map_to_new_odom.transform.translation.z
            self.odom_belif.transform.rotation.x = map_to_new_odom.transform.rotation.x
            self.odom_belif.transform.rotation.y = map_to_new_odom.transform.rotation.y
            self.odom_belif.transform.rotation.z = map_to_new_odom.transform.rotation.z
            self.odom_belif.transform.rotation.w = map_to_new_odom.transform.rotation.w
            self.sbr.sendTransform(self.odom_belif)

                

    def bl_in_map_slam_callback(self,msg):
        """
        Saves odometry data in a buffer and updates the covariance of the robot pose in the map frame
        """
        # Update step for EKF SLAM
        x_belif = msg.pose.pose.position.x
        y_belif = msg.pose.pose.position.y
        theta_belif = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]
        t = msg.header.stamp.to_sec()
        dt = (msg.header.stamp - self.latest_odom_time).to_sec()
        self.latest_odom_time = msg.header.stamp
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        # Update the covariance of the robot pose in the map frame
        self.cov = self.calc_odom_sigma_bel(dt,x_belif,y_belif,theta_belif,v,omega,t,self.cov,self.R)        
        
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
        self.buffer.append(new_odom)
        if len(self.buffer) > 1000:
            self.buffer.pop(0)


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
        if np.abs(v)<0.01 and np.abs(omega)<0.01: ### If robot is not moving, don't update/increase odom cov
            return odom_sigma
        else: ## If robot is moving, update odom cov.             
            return self.G_odom @ odom_sigma @ self.G_odom.T + R_odom





    def calc_H(self,x,y,mx,my):
        """
        Calculates the linearized messurement model for the aruco marker
        """

        # Linearize the messurement model with respect to x_belif,y_belif,m1x_belif,m1y_belif and construct H

        h_0_0 = -1
        h_0_1 = 0
        h_0_2 = 1
        h_0_3 = 0
        h_1_0 = 0
        h_1_1 = -1
        h_1_2 = 0
        h_1_3 = 1
        H = np.array([[h_0_0,h_0_1,h_0_2,h_0_3],[h_1_0,h_1_1,h_1_2,h_1_3]])
        
        return H
    



if __name__ == "__main__":
    node=ekf_slam()
