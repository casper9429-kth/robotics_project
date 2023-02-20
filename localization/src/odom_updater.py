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
from std_msgs.msg import Bool

class odom_updater():
    def __init__(self):
        """
        odom_updater node \\
        
        Functionality:
        * Make aruco marker with id, the center of the map
        * If it hasn't been seen, set map and odom to zero
        * If seen, set odom so that the aruco marker is in the center of the map
        """
        rospy.init_node('odom_updater')

        # Subscribers 
        self.aruco_markers_sub = rospy.Subscriber("aruco/markers", MarkerArray, self.aruco_markers_callback, queue_size=1)
                
        # Publishers
        self.odom_established_pub = rospy.Publisher("odom_updater/odom", TransformStamped, queue_size=1)

        # Define a publisher that sends a bool msg 
        self.reset_odom_cov_pub = rospy.Publisher("odom_updater/reset_odom_cov", Bool, queue_size=1)

        # Define a publisher that sends a bool msg slam ready 
        self.slam_ready_pub = rospy.Publisher("odom_updater/slam_ready", Bool, queue_size=1)


                
        # Rotation from map to odom first time:
        self.map_odom_quat = None
                
        # Define rate
        self.update_rate = 100 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 
        
        # TF Stuff
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.map_goal = None

        
        # States 
        self.odom_aruco_id = 3

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

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.loginfo("Could not transform aruco marker pose to map frame")
                    rospy.logwarn(e)
                    return   

                odom = TransformStamped()
                odom.header.stamp = msg.header.stamp
                odom.header.frame_id = "map"
                odom.child_frame_id = "map_goal"
                odom.transform.translation.x = pose_map.pose.position.x
                odom.transform.translation.y = pose_map.pose.position.y
                odom.transform.translation.z = pose_map.pose.position.z
                odom.transform.rotation.x = pose_map.pose.orientation.x
                odom.transform.rotation.y = pose_map.pose.orientation.y
                odom.transform.rotation.z = pose_map.pose.orientation.z
                odom.transform.rotation.w = pose_map.pose.orientation.w
                self.br.sendTransform(odom)

                self.new_aruco_marker = True
                self.map_goal = pose_map.pose




    
        


    def main(self): # Do main stuff here    
        """
        * If no aruco marker has been seen, set map and odom to zero
        * If aruco marker has been seen, set odom so that the aruco marker is in the center of the map
        * If not seen, set odom so that the aruco marker is in the center of the map the last time it was seen
        """
        #rospy.loginfo("RUNNING main")
        
        # If a new map pos is read
        if self.map_goal != None:
            # Calc diff between map_goal and map
            x_diff = self.map_goal.position.x
            y_diff = self.map_goal.position.y
            z_diff = self.map_goal.position.z
            dist = np.sqrt(x_diff**2 + y_diff**2 + z_diff**2)                
            #rospy.loginfo("dist: {}".format(dist))
            # if dist is bigger than threshold.
            if dist > 0.0 and self.new_aruco_marker:
                # Get transform from map to aruco marker
                # Get transform from map to odom
                try:    
                    t_map_goal_map = self.tfBuffer.lookup_transform("map_goal", "odom",  rospy.Time(),rospy.Duration(2.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    return None
                
                # And make them one transform from map to odom
                new_t_map_odom = TransformStamped()
                new_t_map_odom.header.stamp = rospy.Time.now()
                new_t_map_odom.header.frame_id = "map"
                new_t_map_odom.child_frame_id = "odom"
                new_t_map_odom.transform.translation.x = t_map_goal_map.transform.translation.x
                new_t_map_odom.transform.translation.y =  t_map_goal_map.transform.translation.y
                new_t_map_odom.transform.translation.z = t_map_goal_map.transform.translation.z
                
                # Normalize quaternion
                q = [t_map_goal_map.transform.rotation.x, t_map_goal_map.transform.rotation.y, t_map_goal_map.transform.rotation.z, t_map_goal_map.transform.rotation.w]
                q = q/np.linalg.norm(q)

                # Set the rotation of the final map_odom
                new_t_map_odom.transform.rotation.x = q[0]
                new_t_map_odom.transform.rotation.y = q[1]
                new_t_map_odom.transform.rotation.z = q[2]
                new_t_map_odom.transform.rotation.w = q[3]

                # Reset odom covariance
                reset_odom_cov_msg = Bool()
                reset_odom_cov_msg.data = True
                self.reset_odom_cov_pub.publish(reset_odom_cov_msg)
                
                #rospy.loginfo("Publishing new odom")
                self.new_aruco_marker = False
                self.br.sendTransform(new_t_map_odom)

                # Save the quaternion of the transform from first map to odom
                if self.map_odom_quat is None:
                    self.map_odom_quat = q

                # Publish slam ready message, if seen once means slam is ready to go
                temp = Bool()
                temp.data = True
                self.slam_ready_pub.publish(temp)

                # Publish map_SLAM frame to visualize the map in rviz, it should be static and its xy plane should be parallel to odom xy plane
                t_map_mapSLAM = TransformStamped()
                t_map_mapSLAM.header.stamp = rospy.Time.now()
                t_map_mapSLAM.header.frame_id = "map"
                t_map_mapSLAM.child_frame_id = "map_SLAM"
                t_map_mapSLAM.transform.translation.x = 0
                t_map_mapSLAM.transform.translation.y = 0
                t_map_mapSLAM.transform.translation.z = 0
                t_map_mapSLAM.transform.rotation.x = self.map_odom_quat[0]
                t_map_mapSLAM.transform.rotation.y = self.map_odom_quat[1]
                t_map_mapSLAM.transform.rotation.z = self.map_odom_quat[2]
                t_map_mapSLAM.transform.rotation.w = self.map_odom_quat[3]
                self.br.sendTransform(t_map_mapSLAM)
                


                return None
            else:
                # Look up previous odom pose and republish it
                try:    
                    t_map_goal_map = self.tfBuffer.lookup_transform("map", "odom",  rospy.Time(),rospy.Duration(2.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    return None
                
                #rospy.loginfo("No new aruco marker pose, republishing old odom")
                
                # And make them one transform from map to odom
                new_t_map_odom = TransformStamped()
                new_t_map_odom.header.stamp = rospy.Time.now()
                new_t_map_odom.header.frame_id = "map"
                new_t_map_odom.child_frame_id = "odom"
                new_t_map_odom.transform.translation.x = t_map_goal_map.transform.translation.x
                new_t_map_odom.transform.translation.y =  t_map_goal_map.transform.translation.y
                new_t_map_odom.transform.translation.z = t_map_goal_map.transform.translation.z
                
                # Normalize quaternion
                q = [t_map_goal_map.transform.rotation.x, t_map_goal_map.transform.rotation.y, t_map_goal_map.transform.rotation.z, t_map_goal_map.transform.rotation.w]
                q = q/np.linalg.norm(q)

                # Set the rotation of the final map_odom
                new_t_map_odom.transform.rotation.x = q[0]
                new_t_map_odom.transform.rotation.y = q[1]
                new_t_map_odom.transform.rotation.z = q[2]
                new_t_map_odom.transform.rotation.w = q[3]

                
                self.br.sendTransform(new_t_map_odom)
                
                
                t_map_mapSLAM = TransformStamped()
                t_map_mapSLAM.header.stamp = rospy.Time.now()
                t_map_mapSLAM.header.frame_id = "map"
                t_map_mapSLAM.child_frame_id = "map_SLAM"
                t_map_mapSLAM.transform.translation.x = 0
                t_map_mapSLAM.transform.translation.y = 0
                t_map_mapSLAM.transform.translation.z = 0
                t_map_mapSLAM.transform.rotation.x = self.map_odom_quat[0]
                t_map_mapSLAM.transform.rotation.y = self.map_odom_quat[1]
                t_map_mapSLAM.transform.rotation.z = self.map_odom_quat[2]
                t_map_mapSLAM.transform.rotation.w = self.map_odom_quat[3]
                self.br.sendTransform(t_map_mapSLAM)

                
                
                return None
                
                
                         
        #rospy.loginfo("No new map goal, publishing zero odom")
        odom = TransformStamped()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "odom"
        odom.transform.translation.x = 0
        odom.transform.translation.y = 0
        odom.transform.translation.z = 0
        odom.transform.rotation.x = 0
        odom.transform.rotation.y = 0
        odom.transform.rotation.z = 0
        odom.transform.rotation.w = 1
        self.br.sendTransform(odom)


        odom = TransformStamped()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "map_SLAM"
        odom.transform.translation.x = 0
        odom.transform.translation.y = 0
        odom.transform.translation.z = 0
        odom.transform.rotation.x = 0
        odom.transform.rotation.y = 0
        odom.transform.rotation.z = 0
        odom.transform.rotation.w = 1
        self.br.sendTransform(odom)


        return None
            
            

            
            

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

    node=odom_updater()
    node.run()



# Everything in mapping should be defined in reference to odom

# The robot should when odom is seen wit