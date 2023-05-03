#!/usr/bin/env python3
import rospy
from rospy import Service
import numpy as np
import cv2 # OpenCV
import random as rn
import rembg
from std_srvs.srv import Trigger, TriggerResponse
from rgb_detection.rgb_detection import find_object_morph,find_object_rembg,find_object_derivatives

# find_object_morph : find object using morphological operations, input image is a numpy array in BGR format
# find_object_rembg : find object using rembg u2net deep learning model, input image is a numpy array in BGR format
# find_object_derivatives : find object using derivatives, input image is a numpy array in BGR format


class ArmCameraServices:
    def __init__(self):
        rospy.init_node('arm_camera_services')

        # State
        self.is_running = False

        # Services
        self.start_service = Service('arm_camera_services/start', Trigger, self.callback)
        self.stop_service = Service('arm_camera_services/stop', Trigger, self.callback)
        self.is_running_service = Service('arm_camera_services/is_running', Trigger, self.callback)

    def run(self):
        rospy.spin()

    def get_ellipse(self,method = "morph"):
        """
        Get the ellipse of the object in the image
        If no object is found, return None
        method = "morph" : find object using morphological operations
        method = "rembg" : find object using rembg u2net deep learning model
        method = "derivatives" : find object using derivatives
        """
        # Take picture from the camera
        camera_device = "/dev/video0" # run "v4l2-ctl --list-devices" to find the camera device
        image = self.take_picture(camera_device)

        # Find object using morphological operations
        if method == "morph":
            ellipse = find_object_morph(image)
        # Find object using rembg u2net deep learning mode
        elif method == "rembg":
            ellipse = find_object_rembg(image)
        # Find object using derivatives
        elif method == "derivatives":
            ellipse = find_object_derivatives(image)
        else:
            ellipse = None

        return ellipse


    def take_picture(camera_device="/dev/video0",debug=False):
        """
        Take picture from the camera
        To determine the camera device, run the following command in the terminal:
        v4l2-ctl --list-devices
        This command will list all the devices connected to the computer.
        For laptop, the camera device is usually /dev/video0
        For the NUC, the arm camera device is usually /dev/video6
        Will return the image as a numpy array in BGR format uint8
        """
        # v4l2-ctl --list-devices
        cap = cv2.VideoCapture(camera_device)
        ret, frame = cap.read()
        if debug:
            cv2.imwrite("image.png", frame)
        return frame

    def transform_point(self,point):
        """
        Transform the point to camera frame
        """
        # Camera matrix
        camera_matrix = [[517.03632655, 0.0, 312.03052029], [0.0, 516.70216219, 252.01727667], [0.0, 0.0, 1.0]]

        # Distortion coefficients
        dist_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Transform the point to camera frame
        point = np.array(point).reshape(1, 1, 2)
        point = cv2.undistortPoints(point, camera_matrix, dist_coeffs)
        point = point[0][0]
        return point
        


if __name__ == "__main__":
    node = ArmCameraServices()
    node.run()
