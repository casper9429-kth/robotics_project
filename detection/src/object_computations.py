#!/usr/bin/env python3
import rospy
from detection.msg import BoundingBox, BoundingBoxArray
import tf2_ros
from tf2_geometry_msgs import  PointStamped
from geometry_msgs.msg import TransformStamped, Point, Quaternion
import message_filters 



class Object_computations():
    def __init__(self):
        """ Put the node name here, and description of the node"""
        rospy.init_node('object_computations')

       
        # Tf 
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
        listener = tf2_ros.TransformListener(self.tfBuffer)

        # Parameters
        self.objects_dict = None

        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 


        # Subscribers 
        #self.sub_topic = rospy.Subscriber("detection/bounding_boxes", BoundingBoxArray)
        # self.sub = message_filters.Subscriber("detection/bounding_boxes", BoundingBoxArray)
        # self.cache = message_filters.Cache(self.sub, 100)
        #self.cache.registerCallback(myCallback)

    # def myCallback(posemsg):
    #     print posemsg



    def filter(self, objects):
        #filter
            # cluster position
            # apply mean position
            # take max class

        #populate or update dict

        # notify if new object detected


        # publish tf


        pass


    def publish_tf(self, msg): 
        
        #rospy.loginfo('New object detected:\n%s', msg.category_name)
        for bb in msg.bounding_boxes:
            #stamp = msg.header.stamp # This generate an annoying warning in the terminal but would be better!
            stamp = rospy.Time.now()
            frame_id = msg.header.frame_id
            
            pose_map = PointStamped()
            pose_map.header.frame_id = frame_id
            pose_map.header.stamp = stamp

            point = Point(bb.bb_center.x, bb.bb_center.y, bb.bb_center.z)

            pose_map.point = point
            
            try:
                pose_map = self.tfBuffer.transform(pose_map, "map", rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(e)
                return   
            

            # Publish new tranform to object/detected/category_name : do not deal with several objects at the same time
            br = tf2_ros.TransformBroadcaster()

            t = TransformStamped()
            t.header.frame_id = "map"
            t.child_frame_id = "object/detected/"+bb.category_name

            t.header.stamp = stamp
            
            t.transform.rotation = Quaternion(0,0,0,1)
            t.transform.translation = pose_map.point
            br.sendTransform(t)

    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """
        
        rospy.loginfo("hey")
        # batch = self.cache.getInterval(rospy.Time.now()-rospy.Duration.from_sec(1), rospy.Time.now())
        # rospy.loginfo(batch)
        #rospy.loginfo(self.cache.getOldestTime())
        

    def run(self):
        """
        Run the node. 
        Don't change anything here, change main instead.
        """
        
        # Run as long as node is not shutdown
        while not rospy.is_shutdown():
            self.main()
            rospy.spin()


if __name__ == "__main__":

    object_computations = Object_computations()
    object_computations.run()