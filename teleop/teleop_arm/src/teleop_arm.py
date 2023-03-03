#!/usr/bin/env python3
import rospy
from rospy import ServiceProxy, wait_for_service
from sensor_msgs.msg import Joy
from arm.srv import ArmTrigger


class TeleopArm():
    def __init__(self) -> None:
        """ A node to control the arm using the joycon. """
        rospy.init_node('teleop_arm')
        
        wait_for_service('/arm/poses/default')
        wait_for_service('/arm/poses/observe')
        wait_for_service('/arm/poses/prepare_to_pick_up')
        wait_for_service('/arm/poses/pick_up')
        wait_for_service('/arm/poses/open_gripper')
        wait_for_service('/arm/poses/close_gripper')

        # Subscribers 
        self.joy_subscriber = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
        
        # Services
        self.default = ServiceProxy('arm/poses/default', ArmTrigger)
        self.observe = ServiceProxy('arm/poses/observe', ArmTrigger)
        self.prepare_to_pick_up = ServiceProxy('arm/poses/prepare_to_pick_up', ArmTrigger)
        self.pick_up = ServiceProxy('arm/poses/pick_up', ArmTrigger)
        self.open_gripper = ServiceProxy('arm/poses/open_gripper', ArmTrigger)
        self.close_gripper = ServiceProxy('arm/poses/close_gripper', ArmTrigger)

        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate)

    ###### All your callbacks here ######
        
    def joy_callback(self, joy_msg: Joy): 
        """ Callback to handle joy messages. """
        open_button = joy_msg.buttons[4] # LB
        should_open = open_button == 1
        close_button = joy_msg.buttons[5] # RB
        should_close = close_button == 1
        up_down_axes = joy_msg.axes[-1] # left stick pad up/down
        should_default = up_down_axes == 1
        should_pick_up = up_down_axes == -1
        left_right_axes = joy_msg.axes[-2] # left stick left/right
        should_prepare_to_pick_up = left_right_axes == 1 # left
        should_observe = left_right_axes == -1 # right
        
        if should_open:
            self.open_gripper()
        elif should_close:
            self.close_gripper()
        elif should_default:
            self.default()
        elif should_observe:
            self.observe()
        elif should_prepare_to_pick_up:
            self.prepare_to_pick_up()
        elif should_pick_up:
            self.pick_up()
        
    ###### All your other methods here #######

    # def call_service(self):
    #     """Publish your messages here"""
    # 
    #     pass
    #     # self.message_pub.publish('')


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

    node=TeleopArm()
    node.run()
