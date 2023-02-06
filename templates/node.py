#!/usr/bin/env python3
import rospy


class Node_name():
    def __init__(self) -> None:
        """ Put the node name here, and description of the node"""
        rospy.init_node('Node_name')

        # Subscribers 
        # self.sub_topic = rospy.Subscriber("topic", type, self.callback_topic)
        
        # Publisher
        # self.message_pub = rospy.Publisher("topic", type, queue_size=10)


        # Publisher
        # self.message_pub = rospy.Publisher("topic", type, queue_size=10)


        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 

        # Tf 
        # self.tf_buffer = tf2_ros.Buffer()
        # self.br = tf2_ros.TransformBroadcaster()
        # self.listner = tf2_ros.TransformListener(self.tf_buffer)

        # Paramethers HERE

    ###### All your callbacks here ######
        
    # def callback_topic(self): 
    #     """Callback function for the topic"""
    #     # do callback stuff
    #     pass

    ###### All your other methods here #######

    # def publish(self):
    #     """Publish your messages here"""
    # 
    #     pass
    #     # self.message_pub.publish('')


    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """
        pass

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

    node=Node_name()
    node.run()
