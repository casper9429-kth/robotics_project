#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CameraNode:
    def __init__(self):
        # Initialize the node
        self.node_name = "arm_camera"
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/camera/image", Image, queue_size=1)
        

    def start(self):
        rospy.init_node(self.node_name)
        cap = cv2.VideoCapture(6)
        
        cap.device_id()
        # Set camera device properties
                
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("Publishing image")
            ret, frame = cap.read()
            if ret:
                try:
                    # 
                    
                    image_message = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.image_pub.publish(image_message)
                except CvBridgeError as e:
                    print(e)
            rate.sleep()

if __name__ == "__main__":
    rospy.loginfo("Starting camera node")
    try:

        node = CameraNode()
        node.start()
    except rospy.ROSInterruptException:
        rospy.loginfo("Camera node terminated.")
