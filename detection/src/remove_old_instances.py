#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from detection.msg import ObjectInstanceArray

class RemoveOldInstances():
    def __init__(self):
        """ Put the node name here, and description of the node"""
        rospy.init_node('remove_old_instances')

        # Subscribers 
        self.instances_sub = rospy.Subscriber("/detection/object_instances", ObjectInstanceArray, self.callback_instances)
        
        # Publisher
        self.delete_instance_pub = rospy.Publisher("/detection/remove_instance", String, queue_size=10)

        self.expiration_time = 5


    
    def callback_instances(self, msg):
        """ Callback for the instances topic"""
        for instance in msg.instances:
            stamp = rospy.Time.now()
            if instance.latest_stamp < stamp - rospy.Duration(self.expiration_time):
                self.delete_instance_pub.publish(String(instance.instance_name))

    def run(self):
        """
        Run the node. 
        Don't change anything here, change main instead.
        """
        rospy.spin()


if __name__ == "__main__":

    node=RemoveOldInstances()
    node.run()