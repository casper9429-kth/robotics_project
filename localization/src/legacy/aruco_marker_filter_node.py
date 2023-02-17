#!/usr/bin/env python3
import rospy
from aruco_msgs.msg import MarkerArray
from collections import defaultdict
from geometry_msgs.msg import PoseStamped
import tf
from tf2_geometry_msgs import PoseStamped
import tf2_ros
from collections import defaultdict
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
# Read in aruco markers, prefilter the data and save the data

class aruco_marker_filter_node():
    def __init__(self):
        """
        ROS node: aruco_marker_filter_node\\
            
        Functionality: \\
        * Listens after aruco markers at "aruco/markers" and saves the data.
        
        * Filters the data and publishes the filtered data at "aruco/detectedx/belif" and aruco/detected_array/belif"
        
        * Keeps publishing the aruco markers, even when not seen
                
        * If aruco marker is seen again, publish the filtered data at "aruco/detected_array/update"

        Purpose: \\
        
        * To keep track of the aruco markers, even when not seen.    
        """
        rospy.init_node('aruco_marker_filter_node')

        # Subscribers 
        ## Subscribe to aruco/markers and save the data
        self.sub_aruco_markers = rospy.Subscriber("aruco/markers", MarkerArray, self.callback_aruco_markers)

        ## Define publisher that publishes the filtered data as a MarkerArray
        self.aruco_belif_pub = rospy.Publisher("aruco/detected_array/belif", PoseArray, queue_size=50)
        self.aruco_update_pub = rospy.Publisher("aruco/detected_array/update", PoseArray, queue_size=50)

        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 

        # TF
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()

        # Variables
        self.aruco_init_data = defaultdict(list)
        self.aruco_init_time = defaultdict(lambda: 0)
        self.aruco_latest_time = defaultdict(lambda: 0)
        self.aruco_latest_data = defaultdict(list)
        self.aruco_ref_established = defaultdict(lambda: False)
        self.aruco_ref_data = defaultdict()

    def callback_aruco_markers(self, msg):
        """
        callback_aruco_markers: 
        Saves the first 50 aruco markers and publishes the filtered data.
        """
                
    
        
        # Save the data
        stamp = msg.header.stamp
        frame_id = msg.header.frame_id


        for marker in msg.markers:
            pose_map = PoseStamped()
            pose_map.header.frame_id = frame_id
            pose_map.header.stamp = stamp


            id = marker.id
            pose = marker.pose

            # Transform pose from camera_color_optical_frame to map 
            pose_map.pose.orientation = pose.pose.orientation
            pose_map.pose.position = pose.pose.position
            
            
            try:
                pose_map = self.tfBuffer.transform(pose_map, "map", rospy.Duration(1.0))
                rospy.loginfo("tf ok")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(e)
                return   




            # Aruco init start position
            if stamp.secs - self.aruco_init_time[id] < 2 and len(self.aruco_init_data[id]) < 50:
                self.aruco_init_data[id].append(pose_map)
                self.aruco_init_time[id] = stamp.secs
                rospy.loginfo("aruco marker id: %s is appended", id)
            elif len(self.aruco_init_data[id]) < 50:
                self.aruco_init_data[id].append(pose_map)
                self.aruco_init_time[id] = stamp.secs
                rospy.loginfo("aruco marker id: %s is appended", id)
            else:
                rospy.loginfo("aruco marker id: %s not appended", id)
                
                
                
            # Aruco latest seen: keep the 100 latest aruco data and time of the latest aruco data
            self.aruco_latest_time[id] = stamp.secs
            self.aruco_latest_data[id].append(pose_map)
            if len(self.aruco_latest_data[id]) > 100:
                self.aruco_latest_data[id].pop(0)
            
            


        

    def publish_aruco_marker_belief(self):
        """
        Publish the aruco marker belief of the aruco markers that are established
        """

        # Publish the aruco marker belief of the aruco markers that are established
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()
        array = []
        
        for key in self.aruco_ref_established.keys():
            if self.aruco_ref_established[key] == True:
                # Publish the aruco marker belief of the aruco markers that are established in map frame using tf transform at time.now
                pose_map = self.aruco_ref_data[key]
                new_pose = Pose()

                t = TransformStamped()
                t.header.frame_id = "map"
                t.child_frame_id = "aruco/detected" + str(key) + "/belief" 
                t.header.stamp = rospy.Time.now()
                array.append(pose_map.pose)

                t.transform.rotation = pose_map.pose.orientation
                t.transform.translation = pose_map.pose.position
                self.br.sendTransform(t)
        
        # If poses in pose array pub it
        if len(array) > 0:
            pose_array.poses = array
            self.aruco_belif_pub.publish(pose_array)
                
    def process_aruco_init_data(self):
        """
        process_aruco_init_data:
        
        Process the aruco init data, calculate the mean and std of the aruco init data and save it as the aruco reference data
        Used to pin done the aruco marker position in the map frame.
        This is done to define the aruco marker position in the map frame to act like an landmark.         
        """
        
        # iterate through all aruco markers
        for id in self.aruco_init_data.keys():

            # Run the init code only if the aruco marker is not established or the init data is less than 100
            if self.aruco_ref_established[id] == False:
                # calculate the mean and std of the aruco init data
                x = [pose.pose.position.x for pose in self.aruco_init_data[id]]
                y = [pose.pose.position.y for pose in self.aruco_init_data[id]]
                z = [pose.pose.position.z for pose in self.aruco_init_data[id]]                
                roll = [tf.transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[0] for pose in self.aruco_init_data[id]]
                pitch = [tf.transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[1] for pose in self.aruco_init_data[id]]
                yaw = [tf.transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[2] for pose in self.aruco_init_data[id]]
                
                
                

                x_mean = np.mean(x)
                y_mean = np.mean(y)
                z_mean = np.mean(z)
                yaw_mean = np.mean(yaw)
                pitch_mean = np.mean(pitch)
                roll_mean = np.mean(roll)
                
                x_std = np.std(x)
                y_std = np.std(y)
                z_std = np.std(z)
                yaw_std = np.std(yaw)
                pitch_std = np.std(pitch)
                roll_std = np.std(roll)

                # Peform outlier detection on the aruco init data and save the mean
                x = x[np.abs(x - x_mean) < 1 * x_std]
                y = y[np.abs(y - y_mean) < 1 * y_std]
                z = z[np.abs(z - z_mean) < 1 * z_std]
                yaw = yaw[np.abs(yaw - yaw_mean) < 1 * yaw_std]
                pitch = pitch[np.abs(pitch - pitch_mean) < 1 * pitch_std]
                roll = roll[np.abs(roll - roll_mean) < 1 * roll_std]    

                # Save the mean of the aruco init data and print that the aruco marker is established
                if len(self.aruco_init_data[id]) >= 50:
                    rospy.loginfo("aruco marker id: %s is established permanently", id)
                    self.aruco_ref_established[id] = True
                
                # create the aruco ref pose
                self.aruco_ref_data[id] = PoseStamped()
                self.aruco_ref_data[id].pose.position.x = np.mean(x)
                self.aruco_ref_data[id].pose.position.y = np.mean(y)
                self.aruco_ref_data[id].pose.position.z = np.mean(z)
                self.aruco_ref_data[id].pose.orientation = tf.transformations.quaternion_from_euler(np.mean(roll), np.mean(pitch), np.mean(yaw))
                self.aruco_ref_data[id].header.frame_id = "map"
                self.aruco_ref_data[id].header.stamp = rospy.Time.now()
                
                rospy.loginfo("aruco marker id: %s is established, but can still be updated", id)
            
            
    def publish_aruco_marker_update(self):
        """
        If an aruco marker is seen whithin a time threshold, publish the aruco marker update.
        This is where the robot thinks it sees the aruco marker now in map frame.

        * Will publish at aruco/detectedx/update
        
        """
        
        # Creat pose array
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()
        array = []
        
        threshold_time = 0.5
        threshold_samples = 10
        
        time = rospy.get_time().to_sec()
        # iterate through all aruco markers
        for id in self.aruco_init_data.keys():

            # if the aruco marker is established
            if self.aruco_ref_established[id] == True:

                # if the aruco marker is seen recently
                if time - self.aruco_latest_time[id] < threshold_time:
                    
                    # Calculate the mean of the latest 10 observations
                    x = [pose.pose.position.x for pose in self.aruco_latest_data[id]]
                    y = [pose.pose.position.y for pose in self.aruco_latest_data[id]]
                    z = [pose.pose.position.z for pose in self.aruco_latest_data[id]]
                    
                    roll = [tf.transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[0] for pose in self.aruco_latest_data[id]]
                    pitch = [tf.transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[1] for pose in self.aruco_latest_data[id]]
                    yaw = [tf.transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[2] for pose in self.aruco_latest_data[id]]                

                    # Filter the latest 10 observations
                    if len(self.aruco_latest_data[id]) > threshold_samples:
                        x = x[-threshold_samples:]
                        y = y[-threshold_samples:]
                        z = z[-threshold_samples:]
                        yaw = yaw[-threshold_samples:]
                        pitch = pitch[-threshold_samples:]
                        roll = roll[-threshold_samples:]

                    # Do outlier detection on the latest 0:10 observations
                    x = x[np.abs(x - np.mean(x)) < 1 * np.std(x)]
                    y = y[np.abs(y - np.mean(y)) < 1 * np.std(y)]
                    z = z[np.abs(z - np.mean(z)) < 1 * np.std(z)]
                    yaw = yaw[np.abs(yaw - np.mean(yaw)) < 1 * np.std(yaw)]
                    pitch = pitch[np.abs(pitch - np.mean(pitch)) < 1 * np.std(pitch)]
                    roll = roll[np.abs(roll - np.mean(roll)) < 1 * np.std(roll)]    
                    
                    # Create the aruco update pose
                    aruco_update_data = PoseStamped()
                    aruco_update_data.pose.position.x = np.mean(x)
                    aruco_update_data.pose.position.y = np.mean(y)
                    aruco_update_data.pose.position.z = np.mean(z)
                    aruco_update_data.pose.orientation = tf.transformations.quaternion_from_euler(np.mean(roll), np.mean(pitch), np.mean(yaw))
                    aruco_update_data.header.frame_id = "map"
                    aruco_update_data.header.stamp = rospy.Time.now()
                    
                    # Add to pose array
                    array.append(aruco_update_data.pose)
                    
                    
                    t = TransformStamped()
                    t.header.frame_id = "map"
                    t.child_frame_id = "aruco/detected" + str(id) + "/update" 
                    t.header.stamp = rospy.Time.now()
                    t.transform.rotation = aruco_update_data.pose.orientation
                    t.transform.translation = aruco_update_data.pose.position
                    self.br.sendTransform(t)
        
        # Publish the pose array
        if len(array) > 0:
            pose_array.poses = array
            self.aruco_update_pub.publish(pose_array)
            
                    
                    
                    

    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """

        # Firt time a aruco marker is seen, save the data
        # aruco init data processing, calculate and save the mean of the init data
        self.process_aruco_init_data()
        
        # Keep publishing the aruco, when not seen as an belif
        self.publish_aruco_marker_belief()
        
        # If aruco marker is seen, publish the aruco marker update.
        self.publish_aruco_marker_update()
        
        

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

    node=aruco_marker_filter_node()
    node.run()
