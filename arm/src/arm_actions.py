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
        self.set_target = ServiceProxy('arm/steps/set_target', Target)
        rospy.wait_for_service('arm/steps/target_is_valid')
        self.target_is_valid = ServiceProxy('arm/steps/target_is_valid', Target)

        # Action Server
        self.action_server = SimpleActionServer('arm_actions', ArmAction, self.goal_received_callback, auto_start=False)

        # Parameters
        self.steps = {
            'pick_up': [
                self.open_gripper,
                self.straight,
                self.hover_target,
                self.on_target,
                self.close_gripper,
                self.hover_target,
                self.straight
            ],
            'drop_off': [
                self.straight,
                self.hover_target,
                self.open_gripper,
                self.straight
            ]
        }


    ###### All your callbacks here ######

    def goal_received_callback(self, goal: ArmGoal):
        """
        This function is called when the action server receives a new goal.
        """
        if goal.action not in self.steps.keys():
            self.action_server.set_aborted(text=f"Invalid action: '{goal.action}', valid actions are: {list(self.steps.keys())}")
            return
        
        validation = self.target_is_valid(goal.type, goal.x, goal.y, goal.z, goal.yaw)
        if not validation.valid:
            self.action_server.set_aborted(text=f"Invalid target: {validation.message}")
            return
        
        response = self.set_target(goal.type, goal.x, goal.y, goal.z, goal.yaw)
        feedback = ArmFeedback(response.valid, response.message, 0)
        self.action_server.publish_feedback(feedback)
        if not response.valid:
            self.action_server.set_aborted()
            return
        
        for step in self.steps[goal.action]:
            response = step()
            feedback = ArmFeedback(response.success, response.message, response.duration)
            self.action_server.publish_feedback(feedback)
            if not response.success:
                self.action_server.set_aborted()
                return
            rospy.sleep(response.duration)

        result = ArmResult(message="Action completed successfully", success=True)
        self.action_server.set_succeeded(result)
    
    def run(self):
        """
        Run the node. 
        Don't change anything here, change main instead.
        """
        
        self.action_server.start()
        rospy.spin()


if __name__ == "__main__":
    node = ArmActions()
    node.run()
