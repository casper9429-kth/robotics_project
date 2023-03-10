#!/usr/bin/env python3
import rospy
from rospy import ServiceProxy
from arm.srv import SetPickUpTarget, SetPickUpTargetResponse, ArmTrigger, ArmTriggerResponse


class ArmActions():
    def __init__(self) -> None:
        """ Node exposing action server used to control the arm on a high level. """
        rospy.init_node('arm_actions')

        # Service Proxies

        # TODO wait for services to be available

        self.straight = ServiceProxy('arm/poses/straight', ArmTrigger, self.straight_service_callback)
        self.default = ServiceProxy('arm/poses/default', ArmTrigger, self.default_service_callback)
        self.observe = ServiceProxy('arm/poses/observe', ArmTrigger, self.observe_service_callback)
        self.prepare_to_pick_up = ServiceProxy('arm/poses/prepare_to_pick_up', ArmTrigger, self.prepare_to_pick_up_service_callback)
        self.pick_up = ServiceProxy('arm/poses/pick_up', ArmTrigger, self.pick_up_service_callback)

        self.open_gripper = ServiceProxy('arm/poses/open_gripper', ArmTrigger, self.open_gripper_service_callback)
        self.close_gripper = ServiceProxy('arm/poses/close_gripper', ArmTrigger, self.close_gripper_service_callback)

        self.set_pick_up_target = ServiceProxy('arm/poses/set_target', SetPickUpTarget, self.set_pick_up_target_service_callback)

        # Define rate
        self.update_rate = 10 # [Hz]
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate)

        # Parameters

    ###### All your callbacks here ######

    def some_callback(self, req):
        """
        Some callback.
        """
        pass
        
    ###### All your other methods here #######
    
    def main(self): # Do main stuff here
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """
        pass

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
    node = ArmActions()
    node.run()
