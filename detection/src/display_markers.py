#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
import tf_conversions
import tf2_ros
import aruco_ros
from aruco_msgs.msg import MarkerArray
import math
import tf


from tf2_geometry_msgs import PoseStamped

rospy.init_node('display_markers')

tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
listener = tf2_ros.TransformListener(tfBuffer)

aruco_pose_pub = rospy.Publisher('/aruco_pose', PoseStamped, queue_size=10) #Added for testing purposes

def aruco_callback(msg):
    #rospy.loginfo('New aruco marker detected:\n%s', msg)
    
    stamp = msg.header.stamp
    frame_id = msg.header.frame_id

    pose_map = PoseStamped()
    pose_map.header.frame_id = frame_id
    pose_map.header.stamp = stamp

    for marker in msg.markers:
        id = marker.id
        pose = marker.pose

        # Transform pose from camera_color_optical_frame to map 
        pose_map.pose.orientation = pose.pose.orientation
        pose_map.pose.position = pose.pose.position
        
        
        try:
            pose_map = tfBuffer.transform(pose_map, "map", rospy.Duration(1.0))
            rospy.loginfo("tf ok")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)
            aruco_pose_pub.publish(pose_map) #Added for testing purposes
            return   
        

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

sub_goal = rospy.Subscriber('/aruco/markers', MarkerArray, aruco_callback)

if __name__ == '__main__':
    rospy.spin()