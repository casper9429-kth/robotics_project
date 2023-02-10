#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
import tf2_ros
import tf2_geometry_msgs

# Recives poses from path_planner and transforms them to the robot frame

class path_tracker:
    def __init__(self):
        rospy.init_node('path_tracker')
        print('path_tracker node initalized')
        self.path = []
        self.rate = rospy.Rate(10)
        # subscribers
        self.robot_frame = 'base_link'
        self.path_sub = rospy.Subscriber('/path', PoseStamped, self.path_callback)                      # Might change depending on where the coordinates of the objects are published
        self.transform_sub = rospy.Subscriber('/transform', TransformStamped, self.transform_callback)
        
        print('Subscribers initalized')
        
        #publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # tf stuff
        self.br = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        print('Tf2 stuff initialized')

    def path_callback(self, msg):
        rospy.loginfo(msg)
        self.path.append(msg)
        #print(self.path)

    def transform_callback(self, msg):
        pass

    def transform_to_robot_frame(self, pose):
        #print(self.path)
        stamp = pose.header.stamp  
        try:
            transform = self.tfBuffer.lookup_transform(self.robot_frame,'map',stamp,rospy.Duration(0.5)) # The transform that relate map fram to base link frame
        except:
            print('No transform found')
            return pose
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        t = TransformStamped()
        t.header.stamp = pose.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'goal'
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        # t.transform.rotation.x = pose.pose.orientation.x
        # t.transform.rotation.y = pose.pose.orientation.y
        # t.transform.rotation.z = pose.pose.orientation.z
        # t.transform.rotation.w = pose.pose.orientation.w
        self.br.sendTransform(t)


        """t.transform.translation.x = transformed_pose.pose.position.x
        t.transform.translation.y = transformed_pose.pose.position.y
        t.transform.translation.z = transformed_pose.pose.position.z
        t.transform.rotation.x = transformed_pose.pose.orientation.x
        t.transform.rotation.y = transformed_pose.pose.orientation.y
        t.transform.rotation.z = transformed_pose.pose.orientation.z
        t.transform.rotation.w = transformed_pose.pose.orientation.w
        self.br.sendTransform(t)"""

        return transformed_pose

        

    def math(self,pose):
        turn =  3 *   math.atan2(pose.pose.position.y,pose.pose.position.x)
        forward = 1 * math.hypot(pose.pose.position.y,pose.pose.position.x)
        print('turn: ',turn)
        print('forward: ',forward)
        return [forward,turn]

    def publish_twist(self, msg):
        message = Twist()
        message.linear.x = msg[0]
        message.angular.z = msg[1]
        self.cmd_pub.publish(message)


    def spin(self):
        if len(self.path) > 0:
            pose = self.path.pop(0)
            pose = self.transform_to_robot_frame(pose)
            twist = self.math(pose)
            self.publish_twist(twist)
            t = TransformStamped()
            
        
        
    def main(self):
        try:
            while not rospy.is_shutdown():
                self.spin()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    node = path_tracker()
    node.main()