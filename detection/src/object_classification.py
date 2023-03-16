#!/usr/bin/env python3
import rospy

from typing import Dict, List
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Point

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import torch
import os
from PIL import Image as pil_image, ImageDraw

from detection.detector import Detector
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from detection.msg import BoundingBox, BoundingBoxArray
import time

from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh



class Object_classifier():

    def __init__(self):
        """ Put the node name here, and description of the node"""
        rospy.init_node('object_classifier')

       
         # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 

        # Paramethers 
        self.device = "cuda"
        self.detector = Detector().to(self.device)
        #model_path = "/home/robot/dd2419_ws/src/detection/src/dl_detection/det_2023-03-09_16-47-53-692698.pt" #for robot
        model_path = "/home/robot/dd2419_ws/src/detection/src/dl_detection/det_2023-03-15_14-32-40-347854.pt" #for robot
        #model_path = "/home/sleepy/dd2419_ws/src/detection/src/dl_detection/det_2023-03-09_16-47-53-692698.pt" #for computer
        model= self.load_model(self.detector, model_path, self.device)
        self.detector.eval()
        
        #self.cloud = None

        self.bridge = CvBridge()

        self.mapping = ["Binky", "Hugo", "Slush", "Muddles", "Kiki", "Oakie", "Cube", "Sphere"]
        self.sub_mapping_cube = ["Red_cube", "Green_cube", "Blue_cube", "Wooden_cube"]
        self.sub_mapping_sphere = ["Red_ball", "Green_ball", "Blue_ball"]
        self.depth = None
        
        self.camera_info = [None, None, None, None]
        
        # Subscribers 
        self.sub_image = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback) 
        self.sub_depth= rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)
        self.sub_camera_info= rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        #self.sub_cloud = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.cloud_callback)
  
        
        # Publisher
        self.bb_pub = rospy.Publisher("/detection/bounding_boxes", BoundingBoxArray, queue_size=10)
        self.image_bb_pub = rospy.Publisher("/detection/image_with_bounding_boxes", Image, queue_size=10)
        #self.pub = rospy.Publisher('/camera/depth/color/ds_points', PointCloud2, queue_size=1)



        
    def image_callback(self, msg): 
        """Callback function for the topic"""
        try:
            #t0 = time.time()
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            if self.depth is not None:
                self.compute_bb(msg.header.stamp, msg.header.frame_id, self.depth, cv_image) 

        except CvBridgeError as e:
            print(e)
            


    def depth_image_callback(self, msg): 
        """Callback function for the topic"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            np_arr = np.asarray(cv_image)
            self.depth = np_arr

        except CvBridgeError as e:
            print(e)



    def camera_info_callback(self, msg): 
        """Callback function for the topic"""
        
        self.camera_info[0] = msg.K[0] #fx
        self.camera_info[1] = msg.K[4] #fy
        self.camera_info[2] = msg.K[2] #cx
        self.camera_info[3] = msg.K[5] #cy
        
        # **Intrinsic camera matrix for the raw (distorted) images.**
            #      [fx  0 cx]
            #  K = [ 0 fy cy]
            #      [ 0  0  1]
            #  Projects 3D points in the camera coordinate frame to 2D pixel
            #  coordinates using the focal lengths (fx, fy) and principal point
            # (cx, cy).



    # def cloud_callback(self, msg: PointCloud2):
    #     self.cloud = msg



    def load_model(self,model: torch.nn.Module, path: str, device: str) -> torch.nn.Module:
        """Load model weights from disk.

        Args:
            model: The model to load the weights into.
            path: The path from which to load the model weights.
            device: The device the model weights should be on.

        Returns:
            The loaded model (note that this is the same object as the passed model).
        """
        state_dict = torch.load(path, map_location=device)
        model.load_state_dict(state_dict)
        return model



    def compute_bb(self, stamp, frame_id, depth_image, cv_image):     
        
        np_arr = np.asarray(cv_image)
        image = pil_image.fromarray(np_arr)
        
        test_images = []
        torch_image, _  = self.detector.input_transform(image, [])
        test_images.append(torch_image)
        test_images = torch.stack(test_images)
        test_image = test_images.to(self.device)
        
        with torch.no_grad():
            out = self.detector(test_image).cpu()
            bbs = self.detector.decode_output(out, 0.5)

            
            bb_list_msg = BoundingBoxArray()
            bb_list_msg.header.stamp = stamp
            bb_list_msg.header.frame_id = frame_id
           
            #rospy.loginfo(bbs[0])
            for bb in bbs[0]:
                
                x = int(bb["x"])
                y = int(bb["y"])
                bb_msg = BoundingBox()
                bb_msg.x = x
                bb_msg.y = y
                bb_msg.width = bb["width"]
                bb_msg.height = bb["height"]
                bb_msg.category_id = bb["category"]
                
                if bb["category"] == 6 or bb["category"] == 7:
                    category_color = self.compute_color(bb, np_arr)
                    if category_color is not None:
                        bb_msg.category_name = category_color
                    else:
                        bb_msg.category_name = self.mapping[bb["category"]]
                        
                else:
                    bb_msg.category_name = self.mapping[bb["category"]]
                
                # visualize image with bb in Rviz
                start_point = (x, y)
                end_point = (int(x+bb["width"]), int(y+bb["height"]))
                color = (255, 0, 0)
                thickness = 2
                cv_image = cv2.rectangle(cv_image, start_point, end_point, color, thickness)
                cv_image = cv2.putText(cv_image, self.mapping[bb["category"]], (start_point[0]-10, start_point[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, color, thickness, cv2.LINE_AA)
                imgMsg = self.bridge.cv2_to_imgmsg(cv_image, "rgb8")
                self.image_bb_pub.publish(imgMsg)
                
                
                # Call function to compute point and depth
                x,y,depth = self.compute_point(depth_image, bb, depth_image)
                point = Point()
                point.x = x
                point.y = y
                point.z = depth
                bb_msg.bb_center = point
                bb_list_msg.bounding_boxes.append(bb_msg)

                
            self.bb_pub.publish(bb_list_msg)
              
            # tinfer = time.time() - t0
            # rospy.loginfo(tinfer)



    def compute_point(self, depth, bb, depth_image):
        depth = depth_image[int(bb["y"]+bb["height"]/2),int(bb["x"]+bb["width"]/2)]/1000

        x=0
        y=0
        if self.camera_info is not None:      
            x = depth*(int(bb["x"]+bb["width"]/2) - self.camera_info[2]) / self.camera_info[0] 
            y = depth*(int(bb["y"]+bb["height"]/2) - self.camera_info[3]) / self.camera_info[1]
        
        return x, y, depth
    


    def compute_color_pc(self, x,y,z, category_id):
        # Convert ROS -> Open3D
        cloud = o3drh.rospc_to_o3dpc(self.cloud)

        #Crop according to BB TODO

        cropped = cloud.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=np.array([x-0.2, y-0.2, z-0.2]), max_bound=np.array([x+0.2, y+0.2, z+0.2])))

        # Downsample the point cloud to 1 cm
        ds_cloud = cropped.voxel_down_sample(voxel_size=0.01)
        #ds_cloud = cropped.voxel_down_sample(voxel_size=0.01)

        # # Convert Open3D -> NumPy
        points = np.asarray(ds_cloud.points)
        colors = np.asarray(ds_cloud.colors)
        
        # Filter Colors
        #TODO: remove loginfo after tests

        filter_r, filter_b, filter_g, filter_w = [], [], [], []
        count_red, count_blue, count_green, count_wooden = 0,0,0,0
        for color in colors:
        # if the element is red, green or blue, set the value to True, otherwise False:
            if color[0] > 0.75 and color[0] > (color[1] + color[2]): 
            #if color[0] > 1.2*(color[1] + color[2]) and color[0] > 0.75 : 
                filter_r.append(True)
                filter_g.append(False)
                filter_b.append(False)
                filter_w.append(False)
                
                count_red+=1
            
            elif color[1] > 0.5*(color[0] + color[2]) and color[1] > 0.6:
            # elif color[1] > (color[0] + color[2]) and color[1] > 0.4:
                filter_r.append(False)
                filter_g.append(True)
                filter_b.append(False)
                filter_w.append(False)
                
                count_green+=1
            
            elif color[2] > (color[0] + color[1])and color[1] > 0.5:
                filter_r.append(False)
                filter_g.append(False)
                filter_b.append(True)
                filter_w.append(False)
                
                count_blue+=1
            elif color[1] > (1.2*color[2]) and color[0] > 0.75: # TODO: wooden color to define!
                filter_r.append(False)
                filter_g.append(False)
                filter_b.append(False)
                filter_w.append(True)
                count_wooden+=1
            else:
                filter_r.append(False)
                filter_g.append(False)
                filter_b.append(False)
                filter_w.append(False)
        
        max_color = max(count_red, count_green, count_blue, count_wooden)
        mapping = None
        if category_id == 6:
            mapping = self.sub_mapping_cube
        else:
            mapping = self.sub_mapping_sphere

        category_name = ""
        if max_color == 0:
            return None

        if max_color == count_red:
            rospy.loginfo("RED Object detected")
            category_name = mapping[0]
            ds_cloud.points = o3d.utility.Vector3dVector(points[filter_r])
            ds_cloud.colors = o3d.utility.Vector3dVector(colors[filter_r])
            # Convert Open3D -> ROS
            out_msg = o3drh.o3dpc_to_rospc(ds_cloud)
            out_msg.header = self.cloud.header
            self.pub.publish(out_msg)
        elif max_color == count_green:
            rospy.loginfo("GREEN Object detected")
            category_name = mapping[1]
            ds_cloud.points = o3d.utility.Vector3dVector(points[filter_g])
            ds_cloud.colors = o3d.utility.Vector3dVector(colors[filter_g])
            # Convert Open3D -> ROS
            out_msg = o3drh.o3dpc_to_rospc(ds_cloud)
            out_msg.header = self.cloud.header
            self.pub.publish(out_msg)
        elif max_color == count_blue:
            rospy.loginfo("BLUE Object detected")
            category_name = mapping[2]
            ds_cloud.points = o3d.utility.Vector3dVector(points[filter_b])
            ds_cloud.colors = o3d.utility.Vector3dVector(colors[filter_b])
            # Convert Open3D -> ROS
            out_msg = o3drh.o3dpc_to_rospc(ds_cloud)
            out_msg.header = self.cloud.header
            self.pub.publish(out_msg)
        elif max_color == count_wooden and category_id== 6:
            rospy.loginfo("WOODEN Object detected")
            category_name = mapping[3]
            ds_cloud.points = o3d.utility.Vector3dVector(points[filter_w])
            ds_cloud.colors = o3d.utility.Vector3dVector(colors[filter_w])
            # Convert Open3D -> ROS
            out_msg = o3drh.o3dpc_to_rospc(ds_cloud)
            out_msg.header = self.cloud.header
            self.pub.publish(out_msg)

        return category_name

    def compute_color(self, bb, image):
        
        # Crop image
        cropped_image = image[int(bb["y"]):int(bb["y"]+bb["height"]),int(bb["x"]):int(bb["x"]+bb["width"])]
        count_red, count_blue, count_green, count_wooden = 0,0,0,0
        count_red = np.sum(cropped_image[:,:,0]>180)
        count_green = np.sum(cropped_image[:,:,1]>160)
        count_blue = np.sum(cropped_image[:,:,2]>160)
    
        category_id = bb["category"]
       
        max_color = max(count_red, count_green, count_blue, count_wooden)
        mapping = None
        if category_id == 6:
            mapping = self.sub_mapping_cube
        else:
            mapping = self.sub_mapping_sphere

        category_name = ""
        total_sum = count_red + count_green + count_blue
        if  max_color == count_red and count_red>0.5*(count_green +count_blue) and total_sum>100:
            # rospy.loginfo("RED Object detected")
            category_name = mapping[0]
        elif category_id== 6 and np.std([count_red, count_green, count_blue])<130 and total_sum>100:
            # rospy.loginfo(np.std([count_red, count_green, count_blue]))
            # rospy.loginfo("%s, %s, %s",count_red, count_green, count_blue)
            # rospy.loginfo("WOODEN Object detected")
            category_name = mapping[3]
        elif max_color == count_green and total_sum>100:
            # rospy.loginfo("GREEN Object detected")
            #rospy.loginfo("%s, %s, %s",count_red, count_green, count_blue)
            category_name = mapping[1]
        elif max_color == count_blue and total_sum>100:
            # rospy.loginfo("BLUE Object detected")
            category_name = mapping[2]
        
        
        return category_name

        
           


                
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

    classifier = Object_classifier()
    #classifier.run()
    rospy.spin()








