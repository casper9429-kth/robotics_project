#!/usr/bin/env python3
import rospy
from rospy import Service

from std_srvs.srv import Trigger, TriggerResponse
from rgb_detection.rgb_detection import RGBDetection

class ArmCameraServices:
    def __init__(self):
        rospy.init_node('arm_camera_services')

        rgb_detection = RGBDetection()

        # State
        self.is_running = False

        # Services
        self.start_service = Service('arm_camera_services/start', Trigger, self.callback)
        self.stop_service = Service('arm_camera_services/stop', Trigger, self.callback)
        self.is_running_service = Service('arm_camera_services/is_running', Trigger, self.callback)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = ArmCameraServices()
    node.run()
