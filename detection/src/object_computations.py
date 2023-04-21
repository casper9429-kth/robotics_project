#!/usr/bin/env python3
import rospy
from detection.msg import  BoundingBoxArray, ObjectInstanceArray, ObjectInstance
import tf2_ros
from tf2_geometry_msgs import  PointStamped
from geometry_msgs.msg import TransformStamped, Point, Quaternion
import message_filters 
from scipy.cluster.hierarchy import linkage, fcluster
from scipy.spatial.distance import pdist
import numpy as np
from collections import Counter
import math
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
from std_msgs.msg import String

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
        self.temp_dict = {}

        self.directory = "/home/robot/dd2419_ws/src/detection/src/saved_instances"
        self.bridge = CvBridge()
        self.threshold = 8
        self.id = 0
        
        self.reduce_categories = rospy.get_param('reduce_categories')
        self.mapping_animals = ["Binky", "Hugo", "Slush", "Muddles", "Kiki", "Oakie", "Cube", "Sphere"]
        self.mapping_cubes = ["Red_cube", "Green_cube", "Blue_cube", "Wooden_cube"]
        self.mapping_spheres = ["Red_ball", "Green_ball", "Blue_ball"]

        # Define rate
        self.update_rate = 1 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 


        # Subscribers 
        #self.sub_topic = rospy.Subscriber("detection/bounding_boxes", BoundingBoxArray)
        self.sub = message_filters.Subscriber("detection/bounding_boxes", BoundingBoxArray)
        self.sub_image = message_filters.Subscriber("/camera/color/image_raw", Image) 
        self.sub_remove_instance = rospy.Subscriber("/detection/remove_instance", String, self.remove_instance_callback)
        self.cache = message_filters.Cache(self.sub, 100)
        self.cache_image = message_filters.Cache(self.sub_image, 100)
        
        # Publisher
        self.instances_pub = rospy.Publisher("/detection/object_instances", ObjectInstanceArray, queue_size=10)
        self.speaker_pub = rospy.Publisher("/speaker/speech", String, queue_size=10)
        rospy.Rate(4).sleep()
        
        

    def filter(self, batch, time):

        nb_msgs = len(batch)
        # rospy.loginfo("objects len: %s", nb_msgs)
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
            
            Y = pdist(X, 'euclidean')
            
            if len(Y) > 0:
                Z = linkage(Y, method='single', metric='euclidean')
            
                clusters = fcluster(Z, t=0.05, criterion='distance')


                # keep clusters with more than threshold bb detected
                bbs_by_cluster = []
                for i in np.unique(clusters):
                    bb_cluster = []
                    a = [j for j in range(len(clusters)) if clusters[j] == i ]
                    for index in a:
                        bb_cluster.append(bb_list[index])
                    
                    #rospy.loginfo("cluster size: %s" % len(bb_cluster))
                    if len(bb_cluster) > self.threshold:
                        bbs_by_cluster.append(bb_cluster)

                    #rospy.loginfo("cluster size: %s", nb_msgs)
                    
                # take mean position and maximum category_name
                instances_to_save = []
                for cluster in bbs_by_cluster:
                   
                    category_names = [o.category_name for o in cluster]
                    x = [o.bb_center.x for o in cluster]
                    y = [o.bb_center.y for o in cluster]
                    z = [o.bb_center.z for o in cluster]
                    
                    # avoid TF repeated timestamp warning
                    time = time + rospy.Duration.from_sec(0.05)
                    occurence_count = Counter(category_names)
                    category_name = occurence_count.most_common(1)[0][0]
                    x = np.mean(x)
                    y = np.mean(y)
                    z = np.mean(z)
                    
                    bb = cluster[int(self.threshold/2)]
                    stamp = bb.stamp
                    image = self.cache_image.getElemAfterTime(stamp)
                    #self.save_instances((category_name, x, y, z), time, (bb.x, bb.y, bb.width, bb.height, image))
                    instances_to_save.append([(category_name, x, y, z), time, (bb.x, bb.y, bb.width, bb.height, image)])
    
                self.save_instances(instances_to_save)


    def save_instances(self, list_instances):

        
        #rospy.loginfo("number of instances detected in batch: %s", len(list_instances))

        for instance in list_instances:
            
            new_instance = instance[0]
            
            if self.reduce_categories:
                new_instance_name = self.reduce_category(new_instance[0])
                new_instance = (new_instance_name, new_instance[1], new_instance[2], new_instance[3])
                rospy.logdebug("Reduced category: %s", new_instance[0])
            
            time = instance[1]
            bb_info = instance[2]

            # convert coordinates to map frame
            point_map = PointStamped()
            try:
                point_map = self.instance_to_point_map(new_instance, time)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(e)
                return   
            
            # test if there is already an instance of that category in the list
            instances = [item for item in self.objects_dict if new_instance[0] in item]
            nb_instances = len(instances)

            if nb_instances == 0:
                
                # Is there an object instance closer than 5cm to the new instance ?
                found_close, old_instance_key = self.found_close(list(self.objects_dict.keys()), point_map, 0.05, self.objects_dict)
                new_instance_key = new_instance[0]+str(1)

                if not found_close:
                    # add instance to dict 
                    self.objects_dict[new_instance_key] = (new_instance[0], point_map.point.x, point_map.point.y, point_map.point.z, 1, self.id)  
                    self.id += 1 
                    # notify if new object detected
                    rospy.loginfo("New object detected: %s. Position in map: %s", new_instance_key,point_map.point)
                    to_speech = "New object detected: " + str(new_instance_key)
                    self.speaker_pub.publish(to_speech)
                    self.save_instance_image(new_instance_key, bb_info)
                   
                    # publish tf
                    self.publish_tf(new_instance_key, point_map)


                else:
                    # Goal: keep only one in the long term memory. Keep the one with the largest number of detections
                    temp_instances = [item for item in self.temp_dict if new_instance[0] in item]
                    
                    if len(temp_instances) > 0:
                        
                        # check if there is a close instance in the temp memory
                        found_close, tmp_old_instance_key = self.found_close(temp_instances, point_map, 0.05, self.temp_dict)

                        if found_close:
                            instance_temp = self.temp_dict[tmp_old_instance_key]
                            self.temp_dict[tmp_old_instance_key] = (instance_temp[0], (point_map.point.x +float(instance_temp[1]))/2, (point_map.point.y +float(instance_temp[2]))/2, (point_map.point.z +float(instance_temp[3]))/2, int(instance_temp[4])+1) 
                            new_instance_key = tmp_old_instance_key
                               

                    else:
                        self.temp_dict[new_instance_key] = (new_instance[0], point_map.point.x, point_map.point.y, point_map.point.z, 1)

                    # compare temp and long term memory 
                    if self.objects_dict[old_instance_key][4] <= self.temp_dict[new_instance_key][4]:
                        old_id = self.objects_dict[old_instance_key][5]
                        del self.objects_dict[old_instance_key]
                        self.objects_dict[new_instance_key] = (self.temp_dict[new_instance_key][0], self.temp_dict[new_instance_key][1],self.temp_dict[new_instance_key][2], self.temp_dict[new_instance_key][3], self.temp_dict[new_instance_key][4], old_id)
                        del self.temp_dict[new_instance_key]

                        # notify if new object detected
                        rospy.loginfo("New object detected: %s. Position in map: %s", new_instance_key,point_map.point)
                        to_speech = "New object detected: " + str(new_instance_key)
                        self.speaker_pub.publish(to_speech)
                        self.save_instance_image(new_instance_key, bb_info)

                        # publish tf
                        self.publish_tf(new_instance_key, point_map)
                    
                        # delete other image
                        old_instance_path = self.directory+"/"+old_instance_key+".jpg"
                        if os.path.exists(old_instance_path):
                            os.remove(old_instance_path)

            else:
                
               
                # Is the old instance closer than 30cm to the new one ?
                found_close, old_instance_key = self.found_close(instances, point_map, 0.3, self.objects_dict)
            
                if found_close: 

                    # update
                    instance = self.objects_dict[old_instance_key]
                    point_map.point.x = (point_map.point.x +float(instance[1]))/2
                    point_map.point.y = (point_map.point.y +float(instance[2]))/2
                    point_map.point.z = (point_map.point.z +float(instance[3]))/2
                    self.objects_dict[old_instance_key] = (new_instance[0], point_map.point.x, point_map.point.y, point_map.point.z, int(instance[4])+1, instance[5])  
                    
                    # publish tf
                    self.publish_tf(old_instance_key, point_map)

                else:
                    # add instance to dict 
                    instance_key = new_instance[0]+str(nb_instances+1)
                    self.objects_dict[instance_key] = (new_instance[0], point_map.point.x, point_map.point.y, point_map.point.z, 1, self.id)  
                    self.id += 1

                    # notify if new object detected
                    rospy.loginfo("New object detected: %s. Position in map: %s", instance_key,point_map.point)
                    to_speech = "New object detected: " + str(instance_key)
                    self.speaker_pub.publish(to_speech)
                    self.save_instance_image(instance_key, bb_info)

                    # publish tf
                    self.publish_tf(instance_key, point_map)



    def found_close(self, instances, point_map, threshold, dictionnary):
        
        found_close = 0
        instance_key = None
        for old_instance_key in instances:
            instance = dictionnary[old_instance_key]
            
            dist = math.sqrt((point_map.point.x - float(instance[1]))**2 + (point_map.point.y - float(instance[2]))**2 + (point_map.point.z - float(instance[3]))**2 )
            # rospy.loginfo("dist = %s",dist)
            if dist < threshold: 
                found_close = 1
                instance_key = old_instance_key
                break

        return found_close, instance_key


    def save_instance_image(self, instance_key, bb_info):
        # save image of the instance
        image = bb_info[4]
        x_min = bb_info[0]
        y_min = bb_info[1]
        width = bb_info[2]
        height = bb_info[3]
        
        start_point = (int(x_min), int(y_min))
        end_point = (int(x_min+width), int(y_min+height))
        color = (0, 0, 255)
        thickness = 2

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            cv_image = cv2.rectangle(cv_image, start_point, end_point, color, thickness)
            cv_image = cv2.putText(cv_image, instance_key, (start_point[0]-10, start_point[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, thickness, cv2.LINE_AA)
            path = self.directory+"/"+instance_key+".jpg"
            #rospy.loginfo("Saving image at %s", path)
            cv2.imwrite(path, cv_image)
        except CvBridgeError as e:
            print(e)


    def publish_instances(self):

        # Publish list of current instances on topic /detection/object_instances
        instances_list_msg = ObjectInstanceArray()
        stamp = rospy.Time.now()
        instances_list_msg.header.stamp = stamp
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
            instance_msg.latest_stamp = stamp
            instance_msg.nb_detections = int(instance[4])
            instance_msg.id = instance[5]
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


    def remove_instance_callback(self, msg):
        instance_key = msg.data
        # delete instance from dict
        try:
            del self.objects_dict[instance_key]
        except KeyError as e:
            rospy.logwarn(e)

    def reduce_category(self, new_instance):
        if new_instance in self.mapping_animals:
            new_instance = "animal"
        elif new_instance in self.mapping_cubes:
            new_instance = "cube"
        elif new_instance in self.mapping_spheres:
            new_instance = "sphere"
        return new_instance

    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """
        
        if rospy.Time.now() > rospy.Time():
            batch = self.cache.getInterval(rospy.Time.now()-rospy.Duration.from_sec(2), rospy.Time.now())
            if len(batch)>0:
                self.filter(batch, rospy.Time.now()-rospy.Duration.from_sec(1))


            # publish instances msg 
            self.publish_instances()
        

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