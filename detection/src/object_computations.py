#!/usr/bin/env python3
import rospy
from detection.msg import  BoundingBoxArray, ObjectInstanceArray, ObjectInstance
import tf2_ros
from tf2_geometry_msgs import  PointStamped
from geometry_msgs.msg import TransformStamped, Point, Quaternion
import message_filters 
from scipy.cluster.hierarchy import linkage, fcluster
from scipy.spatial.distance import pdist
from matplotlib import pyplot as plt
import numpy as np
from collections import Counter
import math



class Object_computations():
    def __init__(self):
        """ Put the node name here, and description of the node"""
        rospy.init_node('object_computations')

       
        # Tf 
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(60))
        listener = tf2_ros.TransformListener(self.tfBuffer)

        # Parameters
        self.objects_dict = {}
        self.frame_id = "camera_color_optical_frame"

        # Define rate
        self.update_rate = 1 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 


        # Subscribers 
        #self.sub_topic = rospy.Subscriber("detection/bounding_boxes", BoundingBoxArray)
        self.sub = message_filters.Subscriber("detection/bounding_boxes", BoundingBoxArray)
        self.cache = message_filters.Cache(self.sub, 100)
        
        # Publisher
        self.instances_pub = rospy.Publisher("/detection/object_instances", ObjectInstanceArray, queue_size=10)
        
        rospy.Rate(3).sleep()
        
        

    def filter(self, batch, time):

        nb_msgs = len(batch)
        #rospy.loginfo("objects len: %s", nb_msgs)
        if nb_msgs > 0:
            # cluster on position
            X = []
            bb_list = []
            for i in range(nb_msgs):
                curr_msg = batch[i]
                nb_bb = len(curr_msg.bounding_boxes)
                for bb in curr_msg.bounding_boxes:
                    bb_list.append(bb)
                    X.append([bb.bb_center.x, bb.bb_center.y, bb.bb_center.z])
            #rospy.loginfo(X)
            Y = pdist(X, 'euclidean')
            #rospy.loginfo(Y)
            if len(Y) > 0:
                Z = linkage(Y, method='single', metric='euclidean')
            
                clusters = fcluster(Z, t=0.05, criterion='distance')


                # keep clusters with more than 10 bb detected
                bbs_by_cluster = []
                for i in np.unique(clusters):
                    bb_cluster = []
                    a = [j for j in range(len(clusters)) if clusters[j] == i ]
                    for index in a:
                        bb_cluster.append(bb_list[index])
                    #rospy.loginfo("Cluster: %s, nb points in cluster: %s",bb_list[a[0]].category_name,len(bb_cluster))
                    if len(bb_cluster)>12:
                        bbs_by_cluster.append(bb_cluster)
    
                    
                # take mean position and maximum category_name
                for cluster in bbs_by_cluster:
                    category_names = [o.category_name for o in cluster]
                    x = [o.bb_center.x for o in cluster]
                    y = [o.bb_center.y for o in cluster]
                    z = [o.bb_center.z for o in cluster]
                    
                    time = time + rospy.Duration.from_sec(0.05)
                    occurence_count = Counter(category_names)
                    category_name = occurence_count.most_common(1)[0][0]
                    x = np.mean(x)
                    y = np.mean(y)
                    z = np.mean(z)
                    
                    self.save_instances((category_name, x, y, z), time)
                    #rospy.loginfo("category_name:%s, x=%s, y=%s, z=%s",category_name,x,y,z)


    def save_instances(self, new_instance, time):
        
        # convert coordinates to map frame
        point_map = PointStamped()
        
        try:
            point_map = self.instance_to_point_map(new_instance, time)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)
            return   
        #rospy.loginfo("recherche none: %s, %s",point_map, new_instance)
        # test if there is already an instance of that category in the list
        instances = [item for item in self.objects_dict if new_instance[0] in item]
        
        nb_instances = len(instances)

        if nb_instances == 0:
            # add instance to dict 
            instance_key = new_instance[0]+str(1)
            self.objects_dict[instance_key] = (new_instance[0], point_map.point.x, point_map.point.y, point_map.point.z)   

            # notify if new object detected
            rospy.loginfo("New object detected: %s. Position in map: %s", instance_key,point_map.point)

            # publish tf
            self.publish_tf(instance_key, point_map)

            # publish instances msg 
            self.publish_instances()

        else:
            found_close = 0
            
            for old_instance_key in instances:
                instance = self.objects_dict[old_instance_key]
                # rospy.loginfo("current instance: %s", old_instance_key)
                # rospy.loginfo("old: %s, new: %s",instance,point_map.point)
                dist = math.sqrt((point_map.point.x - float(instance[1]))**2 + (point_map.point.y - float(instance[2]))**2 + (point_map.point.z - float(instance[3]))**2 )
                # rospy.loginfo("dist = %s",dist)
                if dist < 0.05: #TODO: choose good threshold
                    #TODO: add check category ?
                    
                    # update
                    self.objects_dict[old_instance_key] = (new_instance[0], (point_map.point.x +float(instance[1]))/2, (point_map.point.y +float(instance[2]))/2, (point_map.point.z +float(instance[3]))/2)  
                    
                    # publish tf
                    self.publish_tf(old_instance_key, point_map)

                    # publish instances msg 
                    self.publish_instances()
                    
                    found_close = 1
               
            if not found_close:
                # add instance to dict 
                instance_key = new_instance[0]+str(nb_instances+1)
                self.objects_dict[instance_key] = (new_instance[0], point_map.point.x, point_map.point.y, point_map.point.z)  

                # notify if new object detected
                rospy.loginfo("New object detected: %s. Position in map: %s", instance_key,point_map.point)

                # publish tf
                self.publish_tf(instance_key, point_map)

                # publish instances msg 
                self.publish_instances()

    
    #TODO: remove instances

    def publish_instances(self):
        instances_list_msg = ObjectInstanceArray()
        instances_list_msg.header.stamp = rospy.Time.now()
        instances_list_msg.header.frame_id = "map"
        for instance_key in self.objects_dict:
            instance = self.objects_dict[instance_key]
            instance_msg = ObjectInstance()
            instance_msg.category_name = instance[0]
            instance_msg.instance_name = instance_key
            point = Point()
            point.x = float(instance[1])
            point.y = float(instance[2])
            point.z = float(instance[3])
            instance_msg.object_position = point
            instances_list_msg.instances.append(instance_msg)

        self.instances_pub.publish(instances_list_msg)



    def instance_to_point_map(self, instance, time):

        point_map = PointStamped()
        point_map.header.frame_id = self.frame_id
        point_map.header.stamp = time 

        point = Point(instance[1], instance[2], instance[3])

        point_map.point = point
        
        try:
            point_map = self.tfBuffer.transform(point_map, "map", rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)
            return   
        
        return point_map


    def publish_tf(self, instance_key, point_map): 
        
        # Publish new tranform to object/detected/instance_key
        br = tf2_ros.TransformBroadcaster()

        t = TransformStamped()
        t.header.frame_id = "map"
        t.child_frame_id = "object/detected/"+instance_key

        t.header.stamp = point_map.header.stamp 
        
        t.transform.rotation = Quaternion(0,0,0,1)
        t.transform.translation = point_map.point
        br.sendTransform(t)



    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """
        
        if rospy.Time.now() > rospy.Time():
            batch = self.cache.getInterval(rospy.Time.now()-rospy.Duration.from_sec(2), rospy.Time.now())
            if len(batch)>0:
                self.filter(batch, rospy.Time.now()-rospy.Duration.from_sec(1))
        #rospy.loginfo(self.cache.getOldestTime())
        

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

    object_computations = Object_computations()
    object_computations.run()