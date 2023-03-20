#!/usr/bin/env python3
import rospy
from rospy import ServiceProxy
from actionlib import SimpleActionServer
from arm.srv import SetPickUpTarget, ArmTrigger
from arm.msg import ArmAction, ArmGoal, ArmResult, ArmFeedback


class ArmActions():
    def __init__(self) -> None:
        """ Node exposing action server used to control the arm on a high level. """
        rospy.init_node('arm_actions')

        # Service Proxies

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

        # Action Server
        action_server = SimpleActionServer('arm_actions', ArmAction, self.execute_callback, auto_start=False)
        # TODO if a new goal is received, cancel the current goal
        self.running = False
        action_server.start()

        # Define rate
        self.update_rate = 10 # [Hz]
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate)

        # Parameters

    ###### All your callbacks here ######

    def execute_callback(self, goal: ArmGoal):
        """
        Some callback.
        """
        # TODO check if goal is valid
        # TODO check if goal is already running
        # TODO check if goal is already finished

        # Set running to true
        self.running = True

        # TODO execute goal

        # Set running to false
        self.running = False

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
