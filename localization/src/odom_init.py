#!/usr/bin/env python3
import rospy
from aruco_msgs.msg import MarkerArray
from collections import defaultdict
from geometry_msgs.msg import PoseStamped
import tf
from tf2_geometry_msgs import PoseStamped
import tf2_ros
from collections import defaultdict
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose


class odom_init():
    def __init__(self):
        """
        odom_updater
        
        Functionality
        * Initialize the odom in the map frame using an aruco marker. 
        * Publish to odom_init/odom and odom_init/base_link when the odom is established and maintained
        """
        rospy.init_node('odom_updater')

        # Subscribers 
        self.aruco_markers_sub = rospy.Subscriber("aruco/markers", MarkerArray, self.aruco_markers_callback, queue_size=1)
                
        # Publishers
        self.odom_established_pub = rospy.Publisher("odom_init/odom", TransformStamped, queue_size=1)
        self.base_link_init_pub = rospy.Publisher("odom_init/base_link", TransformStamped, queue_size=1)
                
        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 
        
        # TF Stuff
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()


        self.odom_established_pose_list = []
        self.odom_established_pose = None
        
        # States 
        self.odom_aruco_id = 3
        self.maintain = False # If true, maintain the odom that is established        

    def aruco_markers_callback(self,msg):
        """
        Callback aruco markers \\
        
        Functionality:
        * Saves the pose of the aruco marker with the odom id in map fraPoseStampedme
        """
                
        for marker in msg.markers:
            if marker.id == self.odom_aruco_id:                
                pose_map = PoseStamped()
                pose_map.header.frame_id = msg.header.frame_id
                pose_map.header.stamp = msg.header.stamp
                pose_map.pose = marker.pose.pose
                            
                            
                try:
                    pose_map = self.tfBuffer.transform(pose_map, "map", rospy.Duration(1.0))
                    rospy.loginfo("tf ok")
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(e)
                    return   



                if marker.id == self.odom_aruco_id:
                    self.odom_established_pose_list.append(pose_map.pose)
                    if len(self.odom_established_pose_list) > 100:
                        self.odom_established_pose_list.pop(0)


    def establish_odom(self):
        """
        Establishes odom if possible\\
        If odom is established, it returns True\\
        If odom is not established, it returns False\\
        Will return True if odom is established
        """

        if self.odom_established_pose != None:

            x = self.odom_established_pose.position.x
            y = self.odom_established_pose.position.y
            z = self.odom_established_pose.position.z
            q0 = self.odom_established_pose.orientation.x
            q1 = self.odom_established_pose.orientation.y
            q2 = self.odom_established_pose.orientation.z
            q3 = self.odom_established_pose.orientation.w
        
            # If odom is established, publish the established odom
            odom = TransformStamped()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"
            odom.child_frame_id = "odom_established"
            odom.transform.translation.x = x
            odom.transform.translation.y = y
            odom.transform.translation.z = z
            odom.transform.rotation.x = q0
            odom.transform.rotation.y = q1
            odom.transform.rotation.z = q2
            odom.transform.rotation.w = q3
            self.br.sendTransform(odom)
                
            return True








        
        if len(self.odom_established_pose_list) == 0:
            return False
        
        
        # Filer odom_established_pose_list
        if len(self.odom_established_pose_list) > 50:
            pose_list = self.odom_established_pose_list[-50:]
        else:
            return False            

        # Get list of x and y
        x = [pose.position.x for pose in pose_list]
        y = [pose.position.y for pose in pose_list]
        z = [pose.position.z for pose in pose_list]
        roll = [tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[0] for pose in pose_list]
        pitch = [tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[1] for pose in pose_list]
        yaw = [tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2] for pose in pose_list]
        
        # Get mean of x,y,z,roll,pitch,yaw and std of x,y,z,roll,pitch,yaw
        x_m = np.mean(x)
        y_m = np.mean(y)
        z_m = np.mean(z)
        roll_m = np.mean(roll)
        pitch_m = np.mean(pitch)
        yaw_m = np.mean(yaw)
        try:
            x_std = np.std(x)
            y_std = np.std(y)
            z_std = np.std(z)
            roll_std = np.std(roll)
            pitch_std = np.std(pitch)
            yaw_std = np.std(yaw)
        except:
            x_std = 0
            y_std = 0
            z_std = 0
            roll_std = 0
            pitch_std = 0
            yaw_std = 0
        
        
        # Filter x,y,z,roll,pitch,yaw based on std
        x = [x[i] for i in range(len(x)) if abs(x[i]-x_m) < 1*x_std]
        y = [y[i] for i in range(len(y)) if abs(y[i]-y_m) < 1*y_std]
        z = [z[i] for i in range(len(z)) if abs(z[i]-z_m) < 1*z_std]
        roll = [roll[i] for i in range(len(roll)) if abs(roll[i]-roll_m) < 1*roll_std]
        pitch = [pitch[i] for i in range(len(pitch)) if abs(pitch[i]-pitch_m) < 1*pitch_std]
        yaw = [yaw[i] for i in range(len(yaw)) if abs(yaw[i]-yaw_m) < 1*yaw_std]
        
        # Get mean of x,y,z,roll,pitch,yaw
        x = np.mean(x)
        y = np.mean(y)
        z = np.mean(z)
        roll = np.mean(roll)
        pitch = np.mean(pitch)
        yaw = np.mean(yaw)
        
        
        self.odom_established_pose = Pose()
        self.odom_established_pose.position.x = x
        self.odom_established_pose.position.y = y
        self.odom_established_pose.position.z = z
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.odom_established_pose.orientation.x = q[0]
        self.odom_established_pose.orientation.y = q[1]
        self.odom_established_pose.orientation.z = q[2]
        self.odom_established_pose.orientation.w = q[3]
        
        
        
        
        
        # If odom is established, publish the established odom
        odom = TransformStamped()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "odom_established"
        odom.transform.translation.x = x
        odom.transform.translation.y = y
        odom.transform.translation.z = z
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        odom.transform.rotation.x = q[0]
        odom.transform.rotation.y = q[1]
        odom.transform.rotation.z = q[2]
        odom.transform.rotation.w = q[3]
        self.br.sendTransform(odom)
            
        return True
        


    def main(self): # Do main stuff here    
        """
        * If no odom is established, do nothing, alt pub zero odom
        * If odom is established, publish the new odom
        """
        rospy.loginfo("RUNNING main")
        
        if self.maintain:
            # If odom is established, publish the established odom and base_link
            # At topic /odom_init
            # At topic /base_link_init        
            self.odom_established_pub.publish(self.base_link_pose)
            self.base_link_init_pub.publish(self.odom_pose)
            trana = self.trans_established
            trana.header.stamp = rospy.Time.now()
            
            self.br.sendTransform(trana)

            return None
        
        # If odom not is established, publish it as zero, also publish base_link as zero to make eveything works
        if not self.establish_odom():
            ## Publish zero odom
            #odom = TransformStamped()
            #odom.header.stamp = rospy.Time.now()
            #odom.header.frame_id = "map"
            #odom.child_frame_id = "odom"
            #odom.transform.translation.x = 0
            #odom.transform.translation.y = 0
            #odom.transform.translation.z = 0
            #odom.transform.rotation.x = 0
            #odom.transform.rotation.y = 0
            #odom.transform.rotation.z = 0
            #odom.transform.rotation.w = 1
            #self.br.sendTransform(odom)


            # Publish zero base_link
            base_link = TransformStamped()
            base_link.header.stamp = rospy.Time.now()
            base_link.header.frame_id = "odom"
            base_link.child_frame_id = "base_link"
            base_link.transform.translation.x = 0
            base_link.transform.translation.y = 0
            base_link.transform.translation.z = 0
            base_link.transform.rotation.x = 0
            base_link.transform.rotation.y = 0
            base_link.transform.rotation.z = 0
            base_link.transform.rotation.w = 1
            self.br.sendTransform(base_link)

        if self.establish_odom():            
            # Move the robot so that the aruco marker is at the same position as the odom
            ## Get the position of the aruco marker in regard to the odom
            ## Get the position of the robot in regard to the aruco marker
            ## Move the robot so that the aruco marker is at the same position as the odom


            # Get transform from odom_established to base_link
            try:    
                trans = self.tfBuffer.lookup_transform("odom_established", "odom", rospy.Time(0),rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return None
         
            # apply the transform to base_link to move the aruco marker to the same position as the odom
            new_base_link = TransformStamped()
            new_base_link.header.stamp = rospy.Time.now()
            new_base_link.header.frame_id = "odom"
            new_base_link.child_frame_id = "base_link"
            new_base_link.transform.translation.x = trans.transform.translation.x
            new_base_link.transform.translation.y = trans.transform.translation.y
            new_base_link.transform.translation.z = trans.transform.translation.z
            new_base_link.transform.rotation.x = trans.transform.rotation.x
            new_base_link.transform.rotation.y = trans.transform.rotation.y
            new_base_link.transform.rotation.z = trans.transform.rotation.z
            new_base_link.transform.rotation.w = trans.transform.rotation.w
            self.br.sendTransform(new_base_link)
            self.trans_established = new_base_link 
            
            # Save the base_link position and the odom position in map frame
            self.base_link_pose = self.tfBuffer.lookup_transform("map", "base_link", rospy.Time(0),rospy.Duration(1.0))
            self.odom_pose = self.tfBuffer.lookup_transform("map", "odom", rospy.Time(0),rospy.Duration(1.0))
            
            self.maintain = True
            
            
            
            

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

    node=odom_init()
    node.run()



# Everything in mapping should be defined in reference to odom

# The robot should when odom is seen wit