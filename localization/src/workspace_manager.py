#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import tf2_ros
import tf2_geometry_msgs



class workspace_manager():
    def __init__(self) -> None:
        """ Put the node name here, and description of the node"""
        rospy.init_node('workspace_manager')

        # Subscribers 
        #self.sub_topic = rospy.Subscriber("topic", type, self.callback_topic)
        
        # Publisher
        self.message_pub = rospy.Publisher("/workspace_poses/pose_array", PoseArray, queue_size=10)
        self.message_pub_point = rospy.Publisher("/workspace_poses/pose", PoseStamped, queue_size=10)

        # Define rate
        self.update_rate = 1/2 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 
        

        # Tf 
        # self.tf_buffer = tf2_ros.Buffer()
        # self.br = tf2_ros.TransformBroadcaster()
        # self.listner = tf2_ros.TransformListener(self.tf_buffer)

        # Paramethers HERE
        self.pose_array = PoseArray()
        
    ###### All your callbacks here ######
        
    # def callback_topic(self): 
    #     """Callback function for the topic"""seStamped' object has no attribute 'position'

    #     # do callback stuff
    #     pass

    ###### All your other methods here #######

    # def publish(self):
    #     """Publish your messages here"""
    # 
    #     pass
    #     # self.message_pub.publish('')

    def is_valid_decimal(self,num):
        try:
            float(num)
        except ValueError:
            return False
        else:
            return True

    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """
        filepath = '/home/robot/dd2419_ws/src/localization/src/test_workspace.tsv'
        with open(filepath) as file:
            for line in file:
                value = line.strip().split('\t')
                #print(value)
                point = Pose()

                if self.is_valid_decimal(value[0]) and self.is_valid_decimal(value[1]):
                    point.position.x = float(value[0])
                    point.position.y = float(value[1])
                    self.pose_array.poses.append(point)
                    #self.message_pub_point.publish(point)
                    
                    #print(point)
        #print(workspace_points)

        #self.pose_array = PoseArray()
        self.pose_array.header.frame_id = 'map'
        #self.pose_array.poses = self.workspace_points
        #print(self.pose_array)
        #print(1)
        self.message_pub.publish(self.pose_array)
        self.pose_array = PoseArray()
        #print(2)

    def run(self):
        """
        Run the node. 
        Don't change anything here, change main instead.
        """
        
        # Run as long as node is not shutdown
        try:
            while not rospy.is_shutdown():
                self.main()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":

    node=workspace_manager()
    node.run()
