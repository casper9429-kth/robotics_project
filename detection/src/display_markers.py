#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
import tf_conversions
import tf2_ros
import aruco_ros
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker
import math
import tf
from std_msgs.msg import String

from tf2_geometry_msgs import PoseStamped


class Display_Markers():
    def __init__(self):
        
        rospy.init_node('display_markers')

        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.sub_goal = rospy.Subscriber('/aruco/markers', MarkerArray, self.aruco_callback)
        self.sub_goal = rospy.Subscriber('/aruco_anchor/markers', MarkerArray, self.aruco_anchor_callback)
        self.pub = rospy.Publisher('/aruco/markers/transformed_pose', Marker, queue_size=1)

        self.speaker_pub = rospy.Publisher("/speaker/speech", String, queue_size=10)
        
        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 
        
        self.box_1_detected = False
        self.box_2_detected = False
        self.box_3_detected = False

    def aruco_callback(self, msg):
        
        stamp = msg.header.stamp
        frame_id = msg.header.frame_id

        for marker in msg.markers:
            id = marker.id
            if id != 500:
                pose_map = PoseStamped()
                pose_map.header.frame_id = frame_id
                pose_map.header.stamp = stamp

                id = marker.id
                pose = marker.pose

                transformed_pose = Marker()
                transformed_pose.header.stamp = stamp
                transformed_pose.id = id
                transformed_pose.header.frame_id = "base_link"
                transformed_pose.confidence = marker.confidence
        
                pose_safe = PoseStamped()
                pose_safe.header.frame_id = frame_id
                pose_safe.header.stamp = stamp
                pose_safe.pose.orientation.x = pose.pose.orientation.x
                pose_safe.pose.orientation.y = pose.pose.orientation.y
                pose_safe.pose.orientation.z = pose.pose.orientation.z
                pose_safe.pose.orientation.w = pose.pose.orientation.w
                pose_safe.pose.position.x = pose.pose.position.x
                pose_safe.pose.position.y = pose.pose.position.y
                pose_safe.pose.position.z = pose.pose.position.z - 0.15

                # Transform pose from camera_color_optical_frame to map 
                pose_map.pose.orientation = pose.pose.orientation
                pose_map.pose.position = pose.pose.position
                
                
                try:
                    transformed_pose.pose.pose = self.tfBuffer.transform(pose_map, "base_link", rospy.Duration(1.0)).pose
                    pose_map = self.tfBuffer.transform(pose_map, "map", rospy.Duration(1.0)) 
                    pose_safe_map = self.tfBuffer.transform(pose_safe, "map", rospy.Duration(1.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(e)
                    return   
                
                self.pub.publish(transformed_pose)

                # Publish new tranform to aruco/detectedX
                
                br = tf2_ros.TransformBroadcaster()

                t = TransformStamped()
                t.header.frame_id = "map"
                t.child_frame_id = "aruco/detected" 
                t.child_frame_id = t.child_frame_id + str(id)

                t.header.stamp = stamp
                
                # rospy.loginfo(msg)
            
                t.transform.rotation = pose_map.pose.orientation
                t.transform.translation = pose_map.pose.position
                br.sendTransform(t)

                
    
                t2 = TransformStamped()
                t2.header.frame_id = "map"
                t2.child_frame_id = "aruco/detected" + str(id) + "_safe"
                t2.header.stamp = stamp
                t2.transform.rotation = pose_safe_map.pose.orientation
                t2.transform.translation = pose_safe_map.pose.position

                br.sendTransform(t2)
                
                if id == 1 and not self.box_1_detected:
                    to_speech = "Cubes box detected"
                    self.speaker_pub.publish(to_speech)
                    self.box_1_detected = True
                elif id == 2 and not self.box_2_detected:
                    to_speech = "Balls box detected"
                    self.speaker_pub.publish(to_speech)
                    self.box_2_detected = True
                elif id == 3 and not self.box_3_detected:
                    to_speech = "Animals box detected "
                    self.speaker_pub.publish(to_speech)
                    self.box_3_detected = True

    def aruco_anchor_callback(self, msg):
        
        stamp = msg.header.stamp
        frame_id = msg.header.frame_id

        for marker in msg.markers:
            id = marker.id
            if id == 500:
                pose_map = PoseStamped()
                pose_map.header.frame_id = frame_id
                pose_map.header.stamp = stamp

                id = marker.id
                pose = marker.pose

                transformed_pose = Marker()
                transformed_pose.header.stamp = stamp
                transformed_pose.id = id
                transformed_pose.header.frame_id = "base_link"
                transformed_pose.confidence = marker.confidence

                # Transform pose from camera_color_optical_frame to map 
                pose_map.pose.orientation = pose.pose.orientation
                pose_map.pose.position = pose.pose.position
                
                
                try:
                    transformed_pose.pose.pose = self.tfBuffer.transform(pose_map, "base_link", rospy.Duration(1.0)).pose
                    pose_map = self.tfBuffer.transform(pose_map, "map", rospy.Duration(1.0)) 
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(e)
                    return   
                
                self.pub.publish(transformed_pose)

                # Publish new tranform to aruco/detectedX
                
                br = tf2_ros.TransformBroadcaster()

                t = TransformStamped()

                t.header.frame_id = "map"
                t.child_frame_id = "aruco/detected" 
                t.child_frame_id = t.child_frame_id + str(id)

                t.header.stamp = stamp
            
                t.transform.rotation = pose_map.pose.orientation
                t.transform.translation = pose_map.pose.position
                br.sendTransform(t)


    def run(self):
        """
        Run the node. 
        Don't change anything here, change main instead.
        """
        
        # Run as long as node is not shutdown
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":

    display_markers = Display_Markers()
    display_markers.run()


    