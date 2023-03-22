#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray



class workspace_manager():
    def __init__(self) -> None:
        """ Put the node name here, and description of the node"""
        rospy.init_node('workspace_manager')

        # Subscribers 
        #self.sub_topic = rospy.Subscriber("topic", type, self.callback_topic)
        
        # Publisher
        self.message_pub = rospy.Publisher("/workspace_poses/pose_array", PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher("/workspace_poses/marker", Marker, queue_size=10)
        #self.message_pub_point = rospy.Publisher("/workspace_poses/pose", PoseStamped, queue_size=10)

        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 
        

        # Paramethers HERE
        self.pose_array = PoseArray()
        
    ###### All your callbacks here ######
        
    # def callback_topic(self): 
    #     """Callback function for the topic"""seStamped' object has no attribute 'position'

    #     # do callback stuff
    #     pass

    ###### All your other methods here #######

    def is_valid_decimal(self,num):
        try:
            float(num)
        except ValueError:
            return False
        else:
            return True
        


    def visualize_point(self):
        poselist = [(pose.position.x, pose.position.y, pose.position.z) for pose in self.pose_array.poses]

        mark = Marker()
        mark.header.frame_id = "map"
        mark.header.stamp = rospy.Time.now()
        mark.type = mark.LINE_STRIP
        mark.action = mark.ADD
        mark.scale.x = 0.01
        mark.color.a = 1.0
        mark.color.r = 0.0
        mark.color.g = 1.0
        mark.color.b = 0.0
        mark.pose.orientation.w = 1.0
        mark.pose.position.x = 0.0#poselist[0][0]
        mark.pose.position.y = 0.0#poselist[0][1]
        mark.pose.position.z = 0.0#poselist[0][2]

        mark.id = 0
        mark.lifetime = rospy.Duration(0)
        mark.points = [Point(x=x, y=y, z=z) for x, y, z in poselist]
        mark.points.append(Point(x=poselist[0][0], y=poselist[0][1], z=poselist[0][2]))

        self.marker_pub.publish(mark)


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
                
        self.pose_array.header.frame_id = 'map'
        self.message_pub.publish(self.pose_array)
        self.visualize_point()

        self.pose_array = PoseArray()

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
