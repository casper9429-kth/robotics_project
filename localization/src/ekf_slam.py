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


        # Subscribe to odometry topic
        self.bl_in_map_slam_sub = rospy.Subscriber("odom_slam", Odometry, self.bl_in_map_slam_callback)
        self.aruco_marker_sub = rospy.Subscriber("aruco/markers", MarkerArray, self.aruco_marker_callback)

        # TF Stuff
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(100))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.reset_odom_time_sec = rospy.Time.now().to_sec()
        
        # Settings
        self.debug = False

        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 
        
        # Robot parameters
        self.ticks_per_rev = 3072
        self.wheel_r = 0.04921
        self.base = 0.3 

        self.aruco_latest_time = defaultdict(lambda: rospy.Time.now().to_sec())

        self.slam_ready = False
        
        # Var to store aruco belief 
        self.aruco_belif_buffer = defaultdict()
        new_marker = defaultdict()
        new_marker['type'] = 'aruco'
        new_marker['x'] = 0
        new_marker['y'] = 0
        new_marker['first_measurement'] = True
        new_marker['theta'] =  0# tf.transformations.euler_from_quaternion([new_aruco.transform.rotation.x,new_aruco.transform.rotation.y,new_aruco.transform.rotation.z,new_aruco.transform.rotation.w])[2]
        new_marker['covariance'] = np.eye(2)*100000000000000000000000000000000000000 # high covariance, robot will pin the marker to its location when seen next time
        self.aruco_belif_buffer = defaultdict(lambda: new_marker)

        
        # Anchor id
        self.anchor_id = 500
        
        self.aruco_state_vector = defaultdict()
        self.seen_aruco_ids = set()
        self.slam_cov = np.zeros((3,3))
        self.Q = np.array([[0.1, 0],
                                [0, 0.1]])

        # map to odom transform

        self.latest_anchor_time = rospy.Time.now().to_sec()
        self.odom = TransformStamped()
        self.odom.header.frame_id = "map"
        self.odom.child_frame_id = "odom"
        self.odom.transform.translation.x = 0
        self.odom.transform.translation.y = 0
        self.odom.transform.translation.z = 0
        self.odom.transform.rotation.x = 0
        self.odom.transform.rotation.y = 0
        self.odom.transform.rotation.z = 0
        self.odom.transform.rotation.w = 1

        self.odom_belif = TransformStamped()
        self.odom_belif.header.frame_id = "map"
        self.odom_belif.child_frame_id = "odom"
        self.odom_belif.transform.translation.x = 0
        self.odom_belif.transform.translation.y = 0
        self.odom_belif.transform.translation.z = 0
        self.odom_belif.transform.rotation.x = 0
        self.odom_belif.transform.rotation.y = 0
        self.odom_belif.transform.rotation.z = 0
        self.odom_belif.transform.rotation.w = 1

        # Odometry var 
        self.odometry_buffer = []
        self.aruco_buffer = []
        self.slam_buffer = []
        self.latest_time_belif = rospy.Time.now()
        self.covariance_belif = np.zeros((3,3))        
        self.R = np.array([[0.00001, 0, 0],
                                [0, 0.00001, 0],
                                [0, 0, 0.0000001]])
                
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
            
            # lookup aruco in robot frame
            try:
                t_robot_aruco = self.tfBuffer.lookup_transform("base_link", "aruco/detected" + str(marker.id), msg.header.stamp)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo("fail")
                continue
            
            # check if aruco marker is to far away from robot
            dist_to_robot = np.sqrt(t_robot_aruco.transform.translation.x**2 + t_robot_aruco.transform.translation.y**2 + t_robot_aruco.transform.translation.z**2)
            threshold = 2
            if dist_to_robot > threshold:
                rospy.loginfo(dist_to_robot)
                continue
            
            

            
            
            
            if marker.id == self.anchor_id:
                # if marker is the anchor, update odom transformation 
                try:
                    t_map_goal_map = self.tfBuffer.lookup_transform("aruco/detected" + str(marker.id), "odom", msg.header.stamp)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    continue
                rospy.loginfo("updated map-odom")
                rospy.loginfo("aruco/detected" + str(marker.id))
                self.slam_ready = True
                new_t_map_odom = TransformStamped()
                new_t_map_odom.header.stamp = t_map_goal_map.header.stamp
                new_t_map_odom.header.frame_id = "map"
                new_t_map_odom.child_frame_id = "odom"
                new_t_map_odom.transform.translation.x = t_map_goal_map.transform.translation.x
                new_t_map_odom.transform.translation.y =  t_map_goal_map.transform.translation.y
                new_t_map_odom.transform.translation.z = t_map_goal_map.transform.translation.z
                q = [t_map_goal_map.transform.rotation.x, t_map_goal_map.transform.rotation.y, t_map_goal_map.transform.rotation.z, t_map_goal_map.transform.rotation.w]
                q = q/np.linalg.norm(q)
                new_t_map_odom.transform.rotation.x = q[0]#t_map_goal_map.transform.rotation.x
                new_t_map_odom.transform.rotation.y = q[1]#t_map_goal_map.transform.rotation.y
                new_t_map_odom.transform.rotation.z = q[2]#t_map_goal_map.transform.rotation.z
                new_t_map_odom.transform.rotation.w = q[3]#t_map_goal_map.transform.rotation.w
                # self.br.sendTransform(new_t_map_odom)

                self.odom = new_t_map_odom
                self.odom_belif = new_t_map_odom 
                self.latest_anchor_time = rospy.Time.now().to_sec()
                rospy.loginfo("reset main odom transform")
                
                # Create reset msg
                new_msg = defaultdict()
                new_msg['type'] = "reset_odom_cov"
                new_msg['t'] = t_map_goal_map.header.stamp.to_sec() # make sure the odometry picked up the new transform
                self.slam_buffer.append(new_msg)
                                
                continue

            if rospy.Time.now().to_sec() - self.aruco_latest_time[marker.id] < 0.5:
                return
                
            try:
                new_aruco = self.tfBuffer.lookup_transform("map", "aruco/detected" + str(marker.id), msg.header.stamp)                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            if self.aruco_belif_buffer[marker.id]['first_measurement']:
                self.aruco_belif_buffer[marker.id]['first_measurement'] = False
                self.aruco_belif_buffer[marker.id]['x'] = new_aruco.transform.translation.x
                self.aruco_belif_buffer[marker.id]['y'] = new_aruco.transform.translation.y
                                

            new_marker = defaultdict()
            new_marker['type'] = 'aruco'
            new_marker['id'] = marker.id
            new_marker['x'] = new_aruco.transform.translation.x
            new_marker['y'] = new_aruco.transform.translation.y
            new_marker['theta'] = 0# tf.transformations.euler_from_quaternion([new_aruco.transform.rotation.x,new_aruco.transform.rotation.y,new_aruco.transform.rotation.z,new_aruco.transform.rotation.w])[2]
            new_marker['t'] = new_aruco.header.stamp.to_sec()
            new_marker['t_stamp'] = rospy.Time.now()
            self.slam_buffer.append(new_marker)
            self.aruco_latest_time[marker.id] = rospy.Time.now().to_sec()

                

    def bl_in_map_slam_callback(self,msg):
        """
        Saves odometry data in a buffer and updates the covariance of the robot pose in the map frame
        """
        # Update step for EKF SLAM
        x_belif = msg.pose.pose.position.x
        y_belif = msg.pose.pose.position.y
        theta_belif = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]
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



    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function
        write your code here to make it more readable.
        """
        




        # if new anchor is found update the map odom transform
        if rospy.Time.now().to_sec() - self.latest_anchor_time < 1.0:
            self.odom.header.stamp = rospy.Time.now()
            self.br.sendTransform(self.odom)
            return
        

        # Check if SLAM is ready, that means that the initial anchor has been found at least once
        # if not, clear the buffer and return 
        # only time it is allowed to publish rospy.Time.now() is when the buffer is cleared. Then all messages afterward are new and have a newer timestamp
        if self.slam_ready == False:
            self.odom.header.stamp = rospy.Time.now()
            self.br.sendTransform(self.odom)
            self.slam_buffer = []
            return


        
        # check if enough new messages have been received to do a SLAM update
        # if not, get the timestamp of the next newest message and publish that
        if len(self.slam_buffer) < 500 or self.slam_buffer[-1]['type'] != 'odometry':
            # sustain the transform
            if len(self.slam_buffer) > 10:
                self.odom_belif.header.stamp = rospy.Time(self.slam_buffer[-2]['t'])
            else:
                self.odom_belif.header.stamp = rospy.Time.now()
            self.br.sendTransform(self.odom_belif)
            return




        # Clear the buffer
        slam_buffer = sorted(self.slam_buffer, key=lambda k: k['t'])
        self.slam_buffer = []


        # Find the first odometry message in the buffer and make it startingpoint for the EKF SLAM
        for i,msg in enumerate(slam_buffer):
            if msg['type'] == 'odometry':
                t = msg['t']
                x = msg['x']
                y = msg['y']
                theta = msg['theta']
                slam_buffer = slam_buffer[i+1:]
                self.slam_cov = self.calc_odom_sigma_bel(msg['dt'],x,y,theta,msg['v'],msg['omega'],t,self.slam_cov,self.R)
                break

        
        # loop through the buffer and run the EKF SLAM
        aruco_seen_ids = set()
        i = -1
        for i_int,_ in enumerate(slam_buffer):
            i += 1
            if i >= len(slam_buffer):
                break
            msg = slam_buffer[i]


            
            
            if msg['type'] == 'reset_odom_cov':
                # Reset the odometry covariance
                # This is done when the robot finds an anchor
                # Also reset the robot pose in the map_SLAM frame
                # according to the newest odometry message
                self.slam_cov = self.slam_cov * 0.0
                # Find next odometry message
                found_new_odom = False
                for new_index,temp_msg in enumerate(slam_buffer[i:]):
                    if temp_msg['type'] == 'odometry':
                        latest_t = temp_msg['t']
                        x = temp_msg['x']
                        y = temp_msg['y']
                        theta = temp_msg['theta']
                        i = i + new_index
                        found_new_odom = True
                        break
                
                

                if found_new_odom:
                    continue
                # should not be possible to get here
                raise Exception("No odometry message found after reset_odom_cov " + str(i) + " : "+ str(len(slam_buffer)) + str(msg))
                
                
            
            
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
                # EKF update
                rospy.loginfo("aruco update")
                ## get belif

                if msg['id'] in aruco_seen_ids:
                    continue
                else: 
                    aruco_seen_ids.add(msg['id'])
                
                mark_belif = self.aruco_belif_buffer[msg['id']]
                m_b = [mark_belif['x'],mark_belif['y']]
                m_b_c = mark_belif['covariance']
                r_b = [x,y]
                r_b_c = self.slam_cov[0:2,0:2]


                z_b = np.array([math.sqrt((m_b[0]-r_b[0])**2 + (m_b[1]-r_b[1])**2),np.arctan2(m_b[1]-r_b[1],m_b[0]-r_b[0])])

                rospy.loginfo("belif: r %s m %s",r_b,m_b)
                
                

                ## get measurement
                m_m = [msg['x'],msg['y']]
                rospy.loginfo("measurement: %s",m_m)
                z_m = np.array([math.sqrt((m_m[0]-r_b[0])**2 + (m_m[1]-r_b[1])**2),np.arctan2(m_m[1]-r_b[1],m_m[0]-r_b[0])])

                ## Linearize the measurement model around the belif
                H = self.calc_H(r_b[0],r_b[1],m_b[0],m_b[1])


                ## Create the state vector
                mu_bel = np.array([r_b[0],r_b[1],m_b[0],m_b[1]]) # x,y,mx,my
                
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
                self.slam_cov[0:2,0:2] = aruco_sigma[0:2,0:2]
                
                self.aruco_belif_buffer[msg['id']]['x'] = aruco_mu[2]
                self.aruco_belif_buffer[msg['id']]['y'] = aruco_mu[3]
                self.aruco_belif_buffer[msg['id']]['covariance'] = aruco_sigma[2:4,2:4]
                
                ## new belif is created for the robot pose in the map_SLAM frame
                rospy.loginfo("new belif: r %s m %s",aruco_mu[0:2],aruco_mu[2:4])

        # When done, update the position of odom to correct the position of the robot in the map_SLAM frame
        
        # Print 
        rospy.loginfo("############# SLAM BATCH UPDATE #############")
        rospy.loginfo("SLAM: " + str(x) + " " + str(y) + " " + str(theta))
        
        try:
            map_to_bs = self.tfBuffer.lookup_transform("map", "base_link", rospy.Time(latest_t), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        
        
        ## a new belif is created for the robot pose in the map_SLAM frame
        desired_base_link = TransformStamped()    
        desired_base_link.header.frame_id = "map"
        desired_base_link.header.stamp = rospy.Time(latest_t)
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
            bs_to_odom = self.tfBuffer.lookup_transform("base_link", "odom", rospy.Time(latest_t), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        
        ## apply the transform to des_base_link to get des_odom
        desired_odom =  TransformStamped()
        desired_odom.header.frame_id = "des_base_link"
        desired_odom.child_frame_id = "des_odom"
        desired_odom.header.stamp = rospy.Time(latest_t)
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
            map_to_new_odom = self.tfBuffer.lookup_transform("map", "des_odom", rospy.Time(latest_t), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        
        ## publish the transform from map to des_odom to odom
        #self.anchor_transform = defaultdict()
        self.odom_belif = TransformStamped()
        self.odom_belif.header.frame_id = "map"
        self.odom_belif.child_frame_id = "odom"
        self.odom_belif.header.stamp = rospy.Time(latest_t)
        self.odom_belif.transform.translation.x = map_to_new_odom.transform.translation.x
        self.odom_belif.transform.translation.y = map_to_new_odom.transform.translation.y 
        self.odom_belif.transform.translation.z = map_to_new_odom.transform.translation.z
        self.odom_belif.transform.rotation.x = map_to_new_odom.transform.rotation.x
        self.odom_belif.transform.rotation.y = map_to_new_odom.transform.rotation.y
        self.odom_belif.transform.rotation.z = map_to_new_odom.transform.rotation.z
        self.odom_belif.transform.rotation.w = map_to_new_odom.transform.rotation.w
        self.br.sendTransform(self.odom_belif)

        
        
        # wait for the new position of odom to be published
        rospy.sleep(0.05)
                
        # Clear the slam buffer and the aruco buffer and the odometry buffer, no new data is allowed to be collected until the EKF SLAM is done
        self.slam_buffer = []
        


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
