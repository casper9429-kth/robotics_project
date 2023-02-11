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


class odom_updater():
    def __init__(self):
        """
        odom_updater
        
        Functionality
        * When an odom is not established keep odom as zero
        
        * When an odom is established, change the odom to the estimated odom
        from geometry_msgs.msg import MarkerArray
        
        * If odom is changed, publish the new odom
        """
        rospy.init_node('odom_updater')

        # Subscribers 
        self.aruco_update_sub = rospy.Subscriber("aruco/detected_list/update", MarkerArray, self.aruco_update_callback, queue_size=1)
        self.aruco_belif_sub = rospy.Subscriber("aruco/detected_list/belif", MarkerArray, self.aruco_belif_callback, queue_size=1)
                
        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 
        
        # TF Stuff
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.odom_established_pose_new = Pose()
        self.odom_established_pose = Pose()
        self.odom_update_pose = None
        
        # States 
        self.odom_aruco_id = 0
        
        # No established odom
        self.odom_established = False
        # Odom Changed 
        self.odom_updated = False


    def aruco_update_callback(self,msg):
        """
        Read in the update of the aruco marker with the odom id
        """
        for marker in msg.markers:
            if marker.id == self.odom_aruco_id and self.odom_updated == False:
                self.odom_updated = True
                self.odom_update_pose = marker.pose.pose
            elif marker.id == self.odom_aruco_id and self.odom_updated == True:
                # Check distance between odom_update_pose and new odom
                if np.sqrt((self.odom_update_pose.position.x - marker.pose.pose.position.x)**2 + (self.odom_update_pose.position.y - marker.pose.pose.position.y)**2) > 0.01:
                    self.odom_update_pose = marker.pose.pose
                



    
    def aruco_belif_callback(self,msg):
        """
        Read in the belif of the aruco marker with the odom id
        """
        # Iter through all markers
        for marker in msg.markers:
            if marker.id == self.odom_aruco_id and self.odom_established == False:
                self.odom_established = True
                self.odom_established_pose = marker.pose.pose
            elif marker.id == self.odom_aruco_id and self.odom_established == True:
                # Check distance between current and new odom
                dist = np.sqrt((self.odom_established_pose.position.x - marker.pose.pose.position.x)**2 + (self.odom_established_pose.position.y - marker.pose.pose.position.y)**2)
                if dist > 0.01:
                    self.odom_established_pose = marker.pose.pose            
                
                
                                

    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """

        # No established odom, publish zero odom in map frame
        if not self.odom_established:
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
            return

        
        # if odom is established but hasn't been updated, publish the established odom
        if not self.odom_updated:
            odom = TransformStamped()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"
            odom.child_frame_id = "odom"
            odom.transform.translation.x = self.odom_established_pose.position.x
            odom.transform.translation.y = self.odom_established_pose.position.y
            odom.transform.translation.z = self.odom_established_pose.position.z
            odom.transform.rotation.x = self.odom_established_pose.orientation.x
            odom.transform.rotation.y = self.odom_established_pose.orientation.y
            odom.transform.rotation.z = self.odom_established_pose.orientation.z
            odom.transform.rotation.w = self.odom_established_pose.orientation.w
            self.br.sendTransform(odom)
            return


        if self.odom_updated: # if odom is established and updated, publish the updated odom
            odom = TransformStamped()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"
            odom.child_frame_id = "odom"
            odom.transform.translation.x = self.odom_update_pose.position.x
            odom.transform.translation.y = self.odom_update_pose.position.y
            odom.transform.translation.z = self.odom_update_pose.position.z
            odom.transform.rotation.x = self.odom_update_pose.orientation.x
            odom.transform.rotation.y = self.odom_update_pose.orientation.y
            odom.transform.rotation.z = self.odom_update_pose.orientation.z
            odom.transform.rotation.w = self.odom_update_pose.orientation.w
            self.br.sendTransform(odom)
            return            
            
        


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

    node=odom_updater()
    node.run()



# Robot has defined odom frame and it is fixed in the map frame

# Don't save data in odom frame, it will move