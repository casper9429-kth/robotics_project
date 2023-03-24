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


from tf2_geometry_msgs import PoseStamped

rospy.init_node('display_markers')

tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
listener = tf2_ros.TransformListener(tfBuffer)

def aruco_callback(msg):
    
    stamp = msg.header.stamp
    frame_id = msg.header.frame_id

    for marker in msg.markers:
        if int(marker.id) != int(id):
            continue
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
            transformed_pose.pose.pose = tfBuffer.transform(pose_map, "base_link", rospy.Duration(1.0)).pose
            pose_map = tfBuffer.transform(pose_map, "map", rospy.Duration(1.0)) 
            #rospy.loginfo("tf ok")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)
            return   
        
        pub.publish(transformed_pose)

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


if __name__ == '__main__':
    # from launch file import param for id
    id = int(rospy.get_param("id"))
    name_or_path = rospy.get_param("aruco_name")
    sub_goal = rospy.Subscriber('/'+str(name_or_path)+'/markers', MarkerArray, aruco_callback)
    pub = rospy.Publisher('/aruco/markers/transformed_pose', Marker, queue_size=1)
    rospy.spin()