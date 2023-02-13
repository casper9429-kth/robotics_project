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


class odom_init():
    def __init__(self):
        """
        odom_updater
        
        Functionality
        * Init odom at origo
        * If aruco marker with id is detected, move the odom so the aruco marker is in the center of the map frame
        """
        rospy.init_node('odom_updater')

        # Subscribers 
        self.aruco_markers_sub = rospy.Subscriber("aruco/markers", MarkerArray, self.aruco_markers_callback, queue_size=1)
                
        # Publishers
        self.odom_established_pub = rospy.Publisher("odom_init/odom", TransformStamped, queue_size=1)
        #self.base_link_init_pub = rospy.Publisher("odom_init/base_link", TransformStamped, queue_size=1)
                
        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 
        
        # TF Stuff
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()


        self.odom_established_pose_list = []
        self.odom_established_pose = None
        
        # States 
        self.odom_aruco_id = 3
        self.maintain = False # If true, maintain the odom that is established        

    def aruco_markers_callback(self,msg):
        """
        Callback aruco markers \\
        
        Functionality:
        * Saves the pose of the aruco marker with the odom id in map fraPoseStampedme
        """
                
        for marker in msg.markers:
            if marker.id == self.odom_aruco_id:                
                pose_map = PoseStamped()
                pose_map.header.frame_id = msg.header.frame_id
                pose_map.header.stamp = msg.header.stamp
                pose_map.pose = marker.pose.pose
                            
                            
                try:
                    pose_map = self.tfBuffer.transform(pose_map, "map", rospy.Duration(1.0))
                    rospy.loginfo("tf ok")
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(e)
                    return   



                if marker.id == self.odom_aruco_id:
                    self.odom_established_pose_list.append(pose_map.pose)
                    if len(self.odom_established_pose_list) > 100:
                        self.odom_established_pose_list.pop(0)


    def establish_odom(self):
        """
        Establishes odom if possible\\
        If odom is established, it returns True\\
        If odom is not established, it returns False\\
        Will return True if odom is established
        """


        # Filer odom_established_pose_list
        if len(self.odom_established_pose_list) > 50:
            pose_list = self.odom_established_pose_list[-50:]
        else:
            return False            

        # Get list of x and y
        x = [pose.position.x for pose in pose_list]
        y = [pose.position.y for pose in pose_list]
        z = [pose.position.z for pose in pose_list]
        roll = [tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[0] for pose in pose_list]
        pitch = [tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[1] for pose in pose_list]
        yaw = [tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2] for pose in pose_list]
        
        # Get mean of x,y,z,roll,pitch,yaw and std of x,y,z,roll,pitch,yaw
        x_m = np.mean(x)
        y_m = np.mean(y)
        z_m = np.mean(z)
        roll_m = np.mean(roll)
        pitch_m = np.mean(pitch)
        yaw_m = np.mean(yaw)
        try:
            x_std = np.std(x)
            y_std = np.std(y)
            z_std = np.std(z)
            roll_std = np.std(roll)
            pitch_std = np.std(pitch)
            yaw_std = np.std(yaw)
        except:
            x_std = 0
            y_std = 0
            z_std = 0
            roll_std = 0
            pitch_std = 0
            yaw_std = 0
        
        
        # Filter x,y,z,roll,pitch,yaw based on std
        x = [x[i] for i in range(len(x)) if abs(x[i]-x_m) < 1*x_std]
        y = [y[i] for i in range(len(y)) if abs(y[i]-y_m) < 1*y_std]
        z = [z[i] for i in range(len(z)) if abs(z[i]-z_m) < 1*z_std]
        roll = [roll[i] for i in range(len(roll)) if abs(roll[i]-roll_m) < 1*roll_std]
        pitch = [pitch[i] for i in range(len(pitch)) if abs(pitch[i]-pitch_m) < 1*pitch_std]
        yaw = [yaw[i] for i in range(len(yaw)) if abs(yaw[i]-yaw_m) < 1*yaw_std]
        
        # Get mean of x,y,z,roll,pitch,yaw
        x = np.mean(x)
        y = np.mean(y)
        z = np.mean(z)
        roll = np.mean(roll)
        pitch = np.mean(pitch)
        yaw = np.mean(yaw)
        
        
        self.odom_established_pose = Pose()
        self.odom_established_pose.position.x = x
        self.odom_established_pose.position.y = y
        self.odom_established_pose.position.z = z
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.odom_established_pose.orientation.x = q[0]
        self.odom_established_pose.orientation.y = q[1]
        self.odom_established_pose.orientation.z = q[2]
        self.odom_established_pose.orientation.w = q[3]
        
        
        
        
        
        # If odom is established, publish the established odom
        odom = TransformStamped()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "map_goal"
        odom.transform.translation.x = x
        odom.transform.translation.y = y
        odom.transform.translation.z = z
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        odom.transform.rotation.x = q[0]
        odom.transform.rotation.y = q[1]
        odom.transform.rotation.z = q[2]
        odom.transform.rotation.w = q[3]
        self.br.sendTransform(odom)
            
        return True
        


    def main(self): # Do main stuff here    
        """
        * If no odom is established, do nothing, alt pub zero odom
        * If odom is established, publish the new odom
        """
        rospy.loginfo("RUNNING main")
        

        
        # If odom not is established, publish it as zero, also publish base_link as zero to make eveything works
        if not self.establish_odom():
            ## Publish zero odom
            odom = TransformStamped()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"
            odom.child_frame_id = "odom"
            odom.transform.translation.x = 0
            odom.transform.translation.y = 0
            odom.transform.translation.z = 0
            odom.transform.rotation.x = 0
            odom.transform.rotation.y = 0
            odom.transform.rotation.z = 0
            odom.transform.rotation.w = 1
            self.br.sendTransform(odom)


        if self.establish_odom():            
            
            try:    
                t_map_goal_map = self.tfBuffer.lookup_transform("map_goal", "map", rospy.Time(0),rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return None
            
            new_base_link = TransformStamped()
            new_base_link.header.stamp = rospy.Time.now()
            new_base_link.header.frame_id = "odom"
            new_base_link.child_frame_id = "odom"
            new_base_link.transform.translation.x = t_map_goal_map.transform.translation.x
            new_base_link.transform.translation.y = t_map_goal_map.transform.translation.y
            new_base_link.transform.translation.z = t_map_goal_map.transform.translation.z
            new_base_link.transform.rotation.x = t_map_goal_map.transform.rotation.x
            new_base_link.transform.rotation.y = t_map_goal_map.transform.rotation.y
            new_base_link.transform.rotation.z = t_map_goal_map.transform.rotation.z
            new_base_link.transform.rotation.w = t_map_goal_map.transform.rotation.w
            self.br.sendTransform(new_base_link)
            self.trans_established = new_base_link 
            
            self.odom_pose = self.tfBuffer.lookup_transform("map", "odom", rospy.Time(0),rospy.Duration(1.0))
            
            
            
            
            

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

    node=odom_init()
    node.run()



# Everything in mapping should be defined in reference to odom

# The robot should when odom is seen wit