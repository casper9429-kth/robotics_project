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

        self.expiration_time = rospy.get_param('~expiration_time')
        self.instance_list = None


    
    def callback_instances(self, msg):
        """ Callback for the instances topic"""
        self.instance_list = msg.instances
        
    def remove_old_instances(self):
        if not self.instance_list is None:
            stamp = rospy.Time.now()
            for instance in self.instance_list:
                if instance.latest_stamp < stamp - rospy.Duration(self.expiration_time):
                    self.delete_instance_pub.publish(String(instance.instance_name))
    
    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.5)
            self.remove_old_instances()
        


if __name__ == "__main__":

    node=RemoveOldInstances()
    node.run()