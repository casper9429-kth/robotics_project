#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Twist # and other stuff that might be needed
import math

# Import the task specific pakages HERE


class Node_name():
    def __init__(self) -> None:
        rospy.init_node('Node_name')

        # Subscribers 
        self.sub_topic = rospy.rospy.Subscriber("topic", type, self.callback_topic)
        # Publisher
        self.message_pub = rospy.Publisher("topic", type, queue_size=10)

        
        self.rate = rospy.Rate(10) # put frequency here

        # Tf 
        self.tf_buffer = tf2_ros.Buffer()
        self.br = tf2_ros.TransformBroadcaster()
        self.listner = tf2_ros.TransformListener(self.tf_buffer)

        # Paramethers HERE
        
    def callback_topic(self): 
        # do callback stuff
        pass

    def publish(self):

        self.message_pub.publish('')


    def spin(self): # Do main stuff here    
        pass

    def main(self):
        try:
            while not rospy.is_shutdown():
                self.spin()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr()

if __name__ == "__main__":
    try:
        node=Node_name()
        node.main()
    except:
        rospy.logerr()