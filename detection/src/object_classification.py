#!/usr/bin/env python3
import rospy

from typing import Dict, List
from sensor_msgs.msg import Image

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import torch
import os
from PIL import Image as pil_image, ImageDraw

from detection.detector import Detector
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from detection.msg import BoundingBox

class Object_classifier():

    def __init__(self):
        """ Put the node name here, and description of the node"""
        rospy.init_node('object_classifier')

        # Subscribers 
        # Or use compressed image?
        self.sub_image = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback) 
        
        # Publisher
        self.bb_pub = rospy.Publisher("/detection/bounding_boxes", BoundingBox, queue_size=10)

         # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate) 

        # Paramethers HERE

        self.device = "cuda"
        self.detector = Detector().to(self.device)
        model_path = "/home/robot/dd2419_ws/src/detection/src/dl_detection/det_2023-02-18_15-46-29-649082.pt"
        #model_path = "/home/sleepy/dd2419_ws/src/detection/src/dl_detection/det_2023-02-18_15-46-29-649082.pt"
        model= self.load_model(self.detector, model_path, self.device)
        self.detector.eval()

        self.bridge = CvBridge()

        self.mapping = ["Binky", "Hugo", "Slush", "Muddles", "Kiki", "Oakie", "Cube", "Sphere"]

        #self.count = 0
        
        


        
    def image_callback(self, msg): 
        """Callback function for the topic"""
        rospy.loginfo("image received")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            np_arr = np.asarray(cv_image)
            im = pil_image.fromarray(np_arr)
            self.compute_bb(im, msg.header.stamp, msg.header.frame_id)

        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)

        
       


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



    def compute_bb(self,image, stamp, frame_id): # Do main stuff here    
        
        test_images = []
        torch_image, _  = self.detector.input_transform(image, [])
        test_images.append(torch_image)
        test_images = torch.stack(test_images)
        test_image = test_images.to(self.device)
        
        with torch.no_grad():
            out = self.detector(test_image).cpu()
            bbs = self.detector.decode_output(out, 0.5)
            # rospy.loginfo(bbs)
            # if self.count % 15 == 0:
            #     image1 = ImageDraw.Draw(image)
            #     for bb in bbs[0]:
            #         shape = [(bb["x"], bb["y"]), ( bb["x"]+bb["width"], bb["y"]+bb["height"])]
            #         image1.rectangle(shape, outline ="red")
            #         rospy.loginfo("category =")
            #         rospy.loginfo(bb["category"])
            #     image.show()

            # self.count+=1

            #publish bb
            for bb in bbs[0]:
                bb_msg = BoundingBox()
                bb_msg.header.stamp = stamp
                bb_msg.header.frame_id = frame_id
                bb_msg.x = bb["x"]
                bb_msg.y = bb["y"]
                bb_msg.width = bb["width"]
                bb_msg.height = bb["height"]
                bb_msg.category_id = bb["category"]
                bb_msg.category_name = self.mapping[bb["category"]]
                self.bb_pub.publish(bb_msg)




                
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








