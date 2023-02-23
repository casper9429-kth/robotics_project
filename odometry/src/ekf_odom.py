# V1.0
# x,y can't be messrured
# theta can be measured

#!/usr/bin/env python
import rospy
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


# Improvements:
# * make the covariance matrix of the aruco markers transform to map frame and back to odom frame when reseen 
# * split script into 3 scripts, one for odometry, one for aruco slam and one for v,omega fusion
# v-omega sensor fusion should publish 
# odometry should publish odom,tf and cov for odom in odom frame, it should reset the cov when the aruco marker with id 3 is seen
# 


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
        self.aruco_sub = rospy.Subscriber('/aruco/markers', MarkerArray, self.aruco_callback)
        self.reset_odom_cov_sub = rospy.Subscriber("odom_updater/reset_odom_cov", Bool,self.odom_cov_reset_callback)
                                          
        # Publish the map and the odometry
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)


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

        # Aruco Slam Setting
        self.anchor_id = 500
        self.seen_aruco_ids = set()
        self.aruco_state_vector = defaultdict()
        
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
        
        ########## EKF v,omega: sensor fusion ##########
        self.ekf_v_omega_predict()
        self.ekf_v_omega_update_enco()
        self.ekf_v_omega_update_imu()                



        ########## Time calculation: used by odom ##########
        self.time_calculations_for_odom()


        ########## EKF odom predict: part 1 aruco slam ##########        
        ## Predict odometry
        self.predict_odometry()        
        
        ########## EKF odom update, part 2 aruco slam ##########
        self.aruco_ekf_slam()
        
        ########## Publish odom ##########
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
        if self.dt > self.update_dt*1.2:
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

    def aruco_ekf_slam(self):
        """
        Aruco slam ekf update
        Iter through all aruco markers, sequentially update the state if new measurement is available
        If new measurement is available, run ekf slam update and save result
        Else, do nothing        
        """
        # iter over all aruco markers, sequentially update the state
        for key in self.aruco_state_vector.keys():
            aruco_state = self.aruco_state_vector[key]
            # If new messurement, run sequential update
            if aruco_state['new_measurement']:

                # Run ekf slam on aruco marker, one at a time
                aruco_mu, aruco_sigma = self.aruco_slam_update(aruco_state)

                # Save the data from the aruco slam update step in the data structures where it should be saved
                self.aruco_slam_save_data(aruco_state,aruco_mu,aruco_sigma,key)                




    def aruco_slam_save_data(self,aruco_state,aruco_mu,aruco_sigma,key):
        """
        Saves the data from the aruco slam update step
        In the data structures where it should be saved
        """
        # Update mu and sigma in the marker and in the robot by splitting the mu_bel and sigma_bel from ekf update
        self.x = aruco_mu[0]
        self.y = aruco_mu[1]
        self.odom_sigma[0:2,0:2] = aruco_sigma[0:2,0:2]
        self.aruco_state_vector[key]['cov'][0:2,0:2] = aruco_sigma[2:,2:]

        # transform the new pose of the marker into map frame
        ## get the new pose in odom frame, because we need to know orientation, z etc which is not estimated by the ekf
        m_odom = self.aruco_state_vector[key]['new_pose_odom'] #marker_odom 
        ## create a pose stamped message and fill it with the new pose in odom frame, but change the state var estimated by the ekf
        m_odom_ps = PoseStamped()
        m_odom_ps.header.stamp = self.current_time
        m_odom_ps.header.frame_id = "odom"
        m_odom_ps.pose.position.x = aruco_mu[2]#m_odom.transform.translation.x
        m_odom_ps.pose.position.y = aruco_mu[3]#m_odom.transform.translation.y
        m_odom_ps.pose.position.z = m_odom.pose.position.z
        m_odom_ps.pose.orientation.x = m_odom.pose.orientation.x
        m_odom_ps.pose.orientation.y = m_odom.pose.orientation.y
        m_odom_ps.pose.orientation.z = m_odom.pose.orientation.z
        m_odom_ps.pose.orientation.w = m_odom.pose.orientation.w
        ## transform the pose to map frame, and save it in the state vector. Use map because odom will change over time
        m_map_ts = self.tfBuffer.transform(m_odom_ps, "map",rospy.Duration(1.0))
        self.aruco_state_vector[key]['pose_map'] = m_map_ts

        # Set new measurement to false so we dont run the ekf update again with the same measurement
        self.aruco_state_vector[key]['new_measurement'] = False



    def aruco_slam_update(self,aruco_state):
        """
        Peforms the aruco slam update step
        """
        # Pre calculate variables
        z_belif = self.calc_z_belif(aruco_state)
        z_mesh = self.calc_z_mesh(aruco_state)
        H = self.calc_linearized_messurement_model(aruco_state)                
        aruco_sigma_bel = self.calc_aruco_sigma_bel(aruco_state)
        aruco_mu_bel = np.array([self.x,self.y,aruco_state['x'],aruco_state['y']])
        Q_aruco = np.eye(2) * 0.01 # Small

        # kalman update step
        aruco_k = aruco_sigma_bel @ H.T @ np.linalg.inv(H @ aruco_sigma_bel @ H.T + Q_aruco)
        aruco_mu = aruco_mu_bel + aruco_k @ (z_mesh - z_belif)
        aruco_sigma = (np.eye(4) - aruco_k @ H) @ aruco_sigma_bel
        return aruco_mu, aruco_sigma    
    
    def predict_odometry(self):
        """
        Predicts the odometry using the odometry model for the ekf aruco prediction step        
        """
        ## Predict odometry
        self.x += self.mu[0] * math.cos(self.theta) * self.dt
        self.y += self.mu[0] * math.sin(self.theta) * self.dt
        self.theta = self.theta + self.mu[1] * self.dt

        ## Predict odom cov after prediction
        self.odom_sigma = self.calc_odom_sigma_bel(self.dt)        

    
    def calc_odom_sigma_bel(self,dt):
        """
        Calculates the sigma for the odometry during the ekf prediction step
        """
                ### Calculate/Update G_odom, used for odometry covariance
        self.G_odom = np.array([[1, 0, -self.mu[0] * math.sin(self.theta) * self.dt],
                       [0, 1, self.mu[0] * math.cos(self.theta) * self.dt],
                       [0, 0, 1]])
        
        ### If time since last reset odom cov is less than 0.1, reset odom cov, will be reset when anchor is found 
        if np.abs(self.current_time_sec - self.reset_odom_sigma) < 0.1:
            return np.zeros((3,3))
        elif np.abs(self.mu[0])<0.01 and np.abs(self.mu[1])<0.01: ### If robot is not moving, don't update/increase odom cov
            return self.odom_sigma
        else: ## If robot is moving, update odom cov. 
            # R_odom should get the error from v_omega fusion            
            # self.sigma[0] is var in v
            # self.sigma[1] is var in omega
            # if v omega is high error, then R_odom should be high
            # if time is high, then R_odom should be high
            # Include linearization

            # error x: (self.mu[0] + v) * math.cos(self.theta + omega*self.dt) * self.dt 
            # lin x:   math.cos(self.theta + omega*self.dt) * self.dt, -(self.mu[0] + v) * math.sin(self.theta + omega*self.dt) * self.dt * self.dt
            # error y: self.mu[0] * math.sin(self.theta) * self.dt
            # lin y:  math.sin(self.theta + omega*self.dt) * self.dt, self.mu[0] * math.cos(self.theta + omega*self.dt) * self.dt * self.dt
            # error theta: self.mu[1] * self.dt
            # lin theta: 0, self.dt
            # x_y_theta_lin = np.array([[math.cos(self.theta + self.mu[1]*self.dt) * self.dt, -(self.mu[0] + self.mu[0]) * math.sin(self.theta + self.mu[1]*self.dt) * self.dt * self.dt],[0,self.dt]])
            # we can disregard the mu[]*self.dt terms since they are small
            x_y_theta_lin = [[math.cos(self.theta) * self.dt, -(self.mu[0]) * math.sin(self.theta) * self.dt * self.dt],[math.sin(self.theta) * self.dt, self.mu[0] * math.cos(self.theta) * self.dt * self.dt],[0,self.dt]]
            x_y_theta_lin = np.array(x_y_theta_lin)

            # Include the errors from self.sigma and the error constant term to keep it from becoming zero            
            R_odom = x_y_theta_lin @ self.sigma @ x_y_theta_lin.T + self.R_odom
            
            
            return self.G_odom @ self.odom_sigma @ self.G_odom.T + R_odom
    
    
    def calc_aruco_sigma_bel(self,aruco_state):
        """
        Calculates the sigma_bel for the aruco marker
        """
        # Construct sigma_bel
        aruco_sigma_bel = np.eye(4)
        aruco_sigma_bel[0:2,0:2] = self.odom_sigma[0:2,0:2]
        aruco_sigma_bel[2:,2:] = aruco_state['cov'][0:2,0:2]
        return aruco_sigma_bel
    
    def calc_linearized_messurement_model(self,aruco_state):
        """
        Calculates the linearized messurement model for the aruco marker
        """
        # Linearize the messurement model with respect to x_belif,y_belif,m1x_belif,m1y_belif and construct H
        h_0_0 = (1/np.sqrt((self.x - aruco_state['x'])**2 + (self.y - aruco_state['y'])**2))*(self.x - aruco_state['x'])
        h_0_1 = (1/np.sqrt((self.x - aruco_state['x'])**2 + (self.y - aruco_state['y'])**2))*(self.y - aruco_state['y'])
        h_0_2 = (1/np.sqrt((self.x - aruco_state['x'])**2 + (self.y - aruco_state['y'])**2))*(-self.x + aruco_state['x'])
        h_0_3 = (1/np.sqrt((self.x - aruco_state['x'])**2 + (self.y - aruco_state['y'])**2))*(-self.y + aruco_state['y'])
        h_1_0 = (1/(1+((aruco_state['y'] - self.y)/ (aruco_state['x'] - self.x))**2))*((aruco_state['y'] - self.y)/ ((aruco_state['x'] - self.x)**2))
        h_1_1 = (1/(1+((aruco_state['y'] - self.y)/ (aruco_state['x'] - self.x))**2))*((- self.y)/ (aruco_state['x'] - self.x))
        h_1_2 = -(1/(1+((aruco_state['y'] - self.y)/ (aruco_state['x'] - self.x))**2))*((aruco_state['y'] - self.y)/ ((aruco_state['x'] - self.x)**2))
        h_1_3 = (1/(1+((aruco_state['y'] - self.y)/ (aruco_state['x'] - self.x))**2))*((aruco_state['y'])/ (aruco_state['x'] - self.x))
        H = np.array([[h_0_0,h_0_1,h_0_2,h_0_3],[h_1_0,h_1_1,h_1_2,h_1_3]])
        return H
    
    
    def calc_z_mesh(self,aruco_state):
        """
        Calculates the z_mesh for the aruco marker
        """
        # calculate z_mesh
        r_mesh = np.sqrt((self.x - aruco_state['new_x'])**2 + (self.y - aruco_state['new_y'])**2)
        bear_mesh = np.arctan2(aruco_state['new_y'] - self.y, aruco_state['new_x'] - self.x)
        z_mesh = np.array([r_mesh,bear_mesh])
        return z_mesh

    def calc_z_belif(self,aruco_state):
        """
        Calculates the z_belif for the aruco marker
        """
        # calculate z_belif
        r_belif = np.sqrt((self.x - aruco_state['x'])**2 + (self.y - aruco_state['y'])**2)
        bear_belif = np.arctan2(aruco_state['y'] - self.y, aruco_state['x'] - self.x)
        z_belif = np.array([r_belif,bear_belif])
        return z_belif

    
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
        t.transform.translation.z = 0 # z is always 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta) # transform yaw to quaternion
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)


    
    def aruco_callback(self,msg):
        """
        aruco callback

        Handles the ekf slam when an aruco marker is seen
        * If new marker, add to the state vector and inherit the covariance from base_link
        * If known marker, flag that a new measurement is available
        * Don't act on the anchor marker 
        \\
        Each aruco marker is saved in self.auco_state_vector dict, with its key being its ID
        each aruco marker has the following keys:
        * id : id of the marker
        * cov : covariance of the marker
        * pose_map : pose of the marker in map frame (transform_stamped)
        * pose_odom : pose of the marker in odom frame (transform_stamped)
        * new_measurement : flag that a new measurement is available of the marker is available
        
        """
        for marker in msg.markers:
            # If anchor, don't do anything
            if marker.id == self.anchor_id:
                return
            
            # If new marker, give it covariance from base_link and add to state vector
            # Check if marker id has already been seen in set()
            if marker.id not in self.seen_aruco_ids:
                
                # look up pose of marker in map_frame
                try:
                    map_to_marker = self.tfBuffer.lookup_transform('map','aruco/detected'+str(marker.id),rospy.Time(0))
                    map_to_marker = self.transformed_stamped_to_PoseStamped(map_to_marker)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    return
                
                # look up pose of marker in odom_frame
                try :
                    odom_to_marker = self.tfBuffer.lookup_transform('odom','aruco/detected'+str(marker.id),rospy.Time(0))
                    odom_to_marker = self.transformed_stamped_to_PoseStamped(odom_to_marker)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    return

                # add id to self.seen_aruco_ids
                self.seen_aruco_ids.add(marker.id)


                
                # add marker to state vector
                new_marker = defaultdict()
                new_marker['id'] = marker.id
                new_marker['cov'] = self.odom_sigma
                new_marker['pose_map'] = map_to_marker
                new_marker['x'] = odom_to_marker.pose.position.x
                new_marker['y'] = odom_to_marker.pose.position.y
                new_marker['new_x'] = None
                new_marker['new_y'] = None
                new_marker['new_measurement'] = False
                new_marker['t_stamp'] = rospy.Time.now()
                self.aruco_state_vector[marker.id] = new_marker
            else: # Already seen marker                
                # Flag that we have a new update information for this marker
                # When used in the main loop, this will be used to update the state vector and covariance
                # After usage, it will be set to False again

                # if aruco marker has been seen in the last 1 second, don't do anything
                if (rospy.Time.now() - self.aruco_state_vector[marker.id]['t_stamp']).to_sec() < 1:
                    return

                # look up pose of the aruco marker in map frame 
                try:
                    map_to_marker = self.tfBuffer.lookup_transform('map','aruco/detected'+str(marker.id),rospy.Time(0))
                    map_to_marker = self.transformed_stamped_to_PoseStamped(map_to_marker)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    return
                
                # look up pose of marker in odom_frame
                try :
                    odom_to_marker = self.tfBuffer.lookup_transform('odom','aruco/detected'+str(marker.id),rospy.Time(0))
                    odom_to_marker = self.transformed_stamped_to_PoseStamped(odom_to_marker)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    return
                
                # the belif of the marker in odom frame might not be the same if odom moved in map frame since last time
                # look up the transform between the old pose in map frame and transform it into the current odom
                # Get old pose in map frame
                
                # correct the old pose in odom frame with the new pose in odom frame using stationary map frame
                old_pose_map = self.aruco_state_vector[marker.id]['pose_map']
                old_pose_new_odom = self.transform_old_map_pose_into_new_frame(old_pose_map, 'odom',marker.header.stamp)
                if old_pose_new_odom == None:
                    return
                self.aruco_state_vector[marker.id]['x'] = old_pose_new_odom.pose.position.x
                self.aruco_state_vector[marker.id]['y'] = old_pose_new_odom.pose.position.y

                # add new messurement to state vector                
                self.aruco_state_vector[marker.id]['new_measurement'] = True                
                self.aruco_state_vector[marker.id]['new_x'] = odom_to_marker.pose.position.x
                self.aruco_state_vector[marker.id]['new_y'] = odom_to_marker.pose.position.y     
                self.aruco_state_vector[marker.id]['new_pose_odom'] = odom_to_marker       
                self.aruco_state_vector[marker.id]['t_stamp'] = rospy.Time.now()
                
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




    def odom_cov_reset_callback(self,msg):
        """
        Reset odom covariance callback
        """
        self.reset_odom_sigma = rospy.Time.now().to_sec()

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
    node=ekf_slam()
    node.run()
