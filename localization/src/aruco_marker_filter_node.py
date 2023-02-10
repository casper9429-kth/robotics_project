#!/usr/bin/env python3
import rospy
from aruco_msgs.msg import MarkerArray
from collections import defaultdict
# Read in aruco markers, prefilter the data and save the data

class aruco_marker_filter_node():
    def __init__(self):
        """
        ROS node: aruco_marker_filter_node\\
            
        Functionality: \\
        * Listens after aruco markers at "topic1" and saves the data.
        
        * Filters the data and publishes the filtered data at "topic2"
        
        * Keeps publishing the aruco markers if seen and not detected.
        
        * If seen again, it will update the position of the marker when the new position is more accurate. 
        
        * If seen again but at a different location, it will flag that the the robot has drifted and the position is not accurate.       
        
        Purpose: \\
            
        
        Put the node name here, and description of the node
        """
        rospy.init_node('aruco_marker_filter_node')

        # Subscribers 
        ## Subscribe to aruco/markers and save the data
        self.sub_aruco_markers = rospy.Subscriber("aruco/markers", MarkerArray, self.callback_aruco_markers)
        # self.sub_topic = rospy.Subscriber("topic", type, self.callback_topic)
        
        # Publisher
        # self.message_pub = rospy.Publisher("topic", type, queue_size=10)

        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 

        # dict to save aruco markers
        self.aruco_data_dict = defaultdict(list)

        # Tf 
        # self.tf_buffer = tf2_ros.Buffer()
        # self.br = tf2_ros.TransformBroadcaster()
        # self.listner = tf2_ros.TransformListener(self.tf_buffer)

        # Paramethers HERE

    def callback_aruco_markers(self, msg):
        """
        callback_aruco_markers: 
        """
                
        
        # Logic to determine if the marker data should be saved or not
        ## If the marker has been seen less than 100 times, save the data
        
        ## OR
        
        ## If it was less than 5 seconds ago since the marker was seen, save the data 

        
        # Pre filter the data
        
        # Save the data
        
        
        
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

    node=aruco_marker_filter_node()
    node.run()
