#!/usr/bin/env python3
import rospy
from rospy import ServiceProxy
from actionlib import SimpleActionServer
from arm.srv import Target, ArmTrigger
from arm.msg import ArmAction, ArmGoal, ArmResult, ArmFeedback


class ArmActions():
    def __init__(self) -> None:
        """ Node exposing action server used to control the arm on a high level. """
        rospy.init_node('arm_actions')

        # Service Proxies

        rospy.wait_for_service('arm/steps/straight')
        self.straight = ServiceProxy('arm/steps/straight', ArmTrigger)
        rospy.wait_for_service('arm/steps/default')
        self.default = ServiceProxy('arm/steps/default', ArmTrigger)
        rospy.wait_for_service('arm/steps/observe')
        self.observe = ServiceProxy('arm/steps/observe', ArmTrigger)
        rospy.wait_for_service('arm/steps/hover_target')
        self.hover_target = ServiceProxy('arm/steps/hover_target', ArmTrigger)
        rospy.wait_for_service('arm/steps/on_target')
        self.on_target = ServiceProxy('arm/steps/on_target', ArmTrigger)

        rospy.wait_for_service('arm/steps/open_gripper')
        self.open_gripper = ServiceProxy('arm/steps/open_gripper', ArmTrigger)
        rospy.wait_for_service('arm/steps/close_gripper')
        self.close_gripper = ServiceProxy('arm/steps/close_gripper', ArmTrigger)

        rospy.wait_for_service('arm/steps/set_target')
        self.set_pick_up_target = ServiceProxy('arm/steps/set_target', Target)
        rospy.wait_for_service('arm/steps/target_is_valid')
        self.target_is_valid_service = ServiceProxy('arm/steps/target_is_valid', Target, self.target_is_valid_service_callback)

        # Action Server
        action_server = SimpleActionServer('arm_actions', ArmAction, self.execute_callback, auto_start=False)
        # TODO if a new goal is received, cancel the current goal
        # TODO on cancel, return to straight position
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
