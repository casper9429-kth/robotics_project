#!/usr/bin/env python3
import rospy
from detection.msg import BoundingBox

class Object_computations():
    def __init__(self):
        """ Put the node name here, and description of the node"""
        rospy.init_node('object_computations')

        # Subscribers 
        self.sub_topic = rospy.Subscriber("detection/bounding_boxes", BoundingBox, self.bb_callback)
        
        # Publisher
        # self.message_pub = rospy.Publisher("topic", type, queue_size=10)

        # Define rate
        # self.update_rate = 10 # [Hz] Change this to the rate you want
        # self.update_dt = 1.0/self.update_rate # [s]
        # self.rate = rospy.Rate(self.update_rate) 

        # Tf 
        # self.tf_buffer = tf2_ros.Buffer()
        # self.br = tf2_ros.TransformBroadcaster()
        # self.listner = tf2_ros.TransformListener(self.tf_buffer)

        # Paramethers HERE

    ###### All your callbacks here ######
        
    def bb_callback(self): 
        """Callback function for the topic"""
        # do callback stuff
        pass

    def color_object(self):
        pass

    def main(self): 

        pass



if __name__ == "__main__":

    object_computations = Object_computations()
    rospy.spin()