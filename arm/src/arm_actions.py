#!/usr/bin/env python3
import rospy
from rospy import ServiceProxy
from arm.srv import SetPickUpTarget, ArmTrigger


class ArmActions():
    def __init__(self) -> None:
        """ Node exposing action server used to control the arm on a high level. """
        rospy.init_node('arm_actions')

        # Service Proxies

        # TODO wait for services to be available

        rospy.wait_for_service('arm/poses/straight')
        self.straight = ServiceProxy('arm/poses/straight', ArmTrigger)
        rospy.wait_for_service('arm/poses/default')
        self.default = ServiceProxy('arm/poses/default', ArmTrigger)
        rospy.wait_for_service('arm/poses/observe')
        self.observe = ServiceProxy('arm/poses/observe', ArmTrigger)
        rospy.wait_for_service('arm/poses/prepare_to_pick_up')
        self.prepare_to_pick_up = ServiceProxy('arm/poses/prepare_to_pick_up', ArmTrigger)
        rospy.wait_for_service('arm/poses/pick_up')
        self.pick_up = ServiceProxy('arm/poses/pick_up', ArmTrigger)

        rospy.wait_for_service('arm/poses/open_gripper')
        self.open_gripper = ServiceProxy('arm/poses/open_gripper', ArmTrigger)
        rospy.wait_for_service('arm/poses/close_gripper')
        self.close_gripper = ServiceProxy('arm/poses/close_gripper', ArmTrigger)

        rospy.wait_for_service('arm/poses/set_target')
        self.set_pick_up_target = ServiceProxy('arm/poses/set_target', SetPickUpTarget)

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
