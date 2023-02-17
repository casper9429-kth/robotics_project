#!/usr/bin/env python3
from math import pi, atan2, acos, asin, sqrt

import rospy
from hiwonder_servo_msgs.msg import CommandDuration
from std_srvs.srv import Trigger, TriggerResponse
from rospy import Service


class Joints:
    def __init__(self, joint1=0, joint2=0, joint3=0, joint4=0, joint5=0):
        self.joint1 = joint1
        self.joint2 = joint2
        self.joint3 = joint3
        self.joint4 = joint4
        self.joint5 = joint5

    def __str__(self):
        # format the output to 2 decimal places
        return f"JointState(joint1={self.joint1:.2f}, joint2={self.joint2:.2f}, joint3={self.joint3:.2f}, joint4={self.joint4:.2f}, joint5={self.joint5:.2f})"


class Arm():
    def __init__(self) -> None:
        """ Services for controlling the arm. """
        rospy.init_node('arm')
        
        # Publishers
        self.joint1_pub = rospy.Publisher("/joint1_controller/command_duration", CommandDuration, queue_size=10)
        self.joint2_pub = rospy.Publisher("/joint2_controller/command_duration", CommandDuration, queue_size=10)
        self.joint3_pub = rospy.Publisher("/joint3_controller/command_duration", CommandDuration, queue_size=10)
        self.joint4_pub = rospy.Publisher("/joint4_controller/command_duration", CommandDuration, queue_size=10)
        self.joint5_pub = rospy.Publisher("/joint5_controller/command_duration", CommandDuration, queue_size=10)
        self.gripper_pub = rospy.Publisher("/r_joint_controller/command_duration", CommandDuration, queue_size=10)

        # Subscribers
        self.pick_up_target_sub = rospy.Subscriber("/arm/pick_up_target", Joints, self.pick_up_target_callback)

        # Services
        self.straight_service = Service('arm/poses/straight', Trigger, self.straight_service_callback)
        self.observe_service = Service('arm/poses/observe', Trigger, self.observe_service_callback)
        self.prepare_to_pick_up_service = Service('arm/poses/prepare_to_pick_up', Trigger, self.prepare_to_pick_up_service_callback)
        self.pick_up_service = Service('arm/poses/pick_up', Trigger, self.pick_up_service_callback)
        self.open_gripper_service = Service('arm/poses/open_gripper', Trigger, self.open_gripper_service_callback)
        self.close_gripper_service = Service('arm/poses/close_gripper', Trigger, self.close_gripper_service_callback)

        # Define rate
        self.update_rate = 10 # [Hz]
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate)

        # Parameters

        # Measurements
        self.d_12 = 18e-3 # [m]
        self.d_23 = 100e-3 # [m]
        self.d_34 = 97e-3 # [m]
        self.d_5e = 55e-3 # [m] TODO: Find this value

        # Duration of the motion
        self.joint_duration = 1500 # [ms] TODO: make dynamic
        self.gripper_duration = 1000 # [ms]

        # gripper
        self.gripper_open = -1.5
        self.gripper_close = { 'cube': -0.0, 'sphere': -0.5, 'animal': None } # TODO: Figure out animal gripper value

        # Joints
        self.joints_straight = Joints()
        self.joints_observe = Joints(joint1=0, joint2=0, joint3=-pi/2, joint4=-pi/2, joint5=0)
        self.joints_prepare_to_pick_up = None
        self.joints_observe = None

        # Target type
        self.target_type = None

    ###### All your callbacks here ######

    def straight_service_callback(self, _):
        """ Callback function for the initial position service. """
        self.publish_joints(self.joints_straight)
        return TriggerResponse(True, 'Initial')
    
    def observe_service_callback(self, _):
        """ Callback function for the testing service. """
        self.publish_joints(self.joints_observe)
        return TriggerResponse(True, 'Testing')

    def prepare_to_pick_up_service_callback(self, _): 
        """ Callback function for the prepare to pick up service. """
        self.publish_joints(self.joints_prepare_to_pick_up)
        return TriggerResponse(True, 'Preparing to pick up')

    def pick_up_service_callback(self, _):
        """ Callback function for the prepare pick up service. """
        object_type = 'cube'
        if object_type == 'cube':
            self.publish_joints(self.joints_pick_up_cube)
            return TriggerResponse(True, 'Pick up cube')
        elif object_type == 'sphere':
            self.publish_joints(self.joints_pick_up_sphere)
            return TriggerResponse(True, 'Picked up sphere')
        else:
            return TriggerResponse(False, 'object_type must be either "cube" or "sphere".')
        
    def open_gripper_service_callback(self, _):
        """ Callback function for the close gripper service. """
        self.publish_gripper(self.gripper_open)
        return TriggerResponse(True, 'Opened gripper')
    
    def close_gripper_service_callback(self, _):
        """ Callback function for the close gripper service."""
        object_type = 'cube'
        if object_type == 'cube':
            self.publish_gripper(self.gripper_close_cube)
            return TriggerResponse(True, 'Closed gripper')
        elif object_type == 'sphere':
            self.publish_gripper(self.gripper_close_sphere)
            return TriggerResponse(True, 'Closed gripper')
        else:
            return TriggerResponse(False, 'object_type must be either "cube" or "sphere".')
    
    def pick_up_target_callback(self, pick_up_target):
        """ Callback function for the pick up target subscriber. """
        self.type = pick_up_target.type
        x = pick_up_target.target.x
        y = pick_up_target.target.y
        z = pick_up_target.target.z
        self.publish_joints(req)
        
    ###### All your other methods here #######

    def inverse_kinematics(self, x, y, z=0):
        """ Calculates the inverse kinematics for the arm. """
        theta1 = atan2(y, x) % (2 * pi) - pi
        theta5 = theta1

        h = z
        d = sqrt(x**2 + y**2)
        a = self.d_23
        b = self.d_34
        cos_beta = (d**2 + h**2 - a**2 - b**2) / (2 * a * b)
        beta = acos(cos_beta)
        A = a + b * cos_beta
        B = b * sqrt(1 - cos_beta**2)
        alpha = asin(d / sqrt(A**2 + B**2)) - atan2(B, A)
        theta2 = -alpha
        theta3 = -beta
        theta4 = pi - theta2 - theta3
        if theta4 > pi:
            theta4 -= 2 * pi

        return Joints(theta1, theta2, theta3, theta4, theta5)

    def publish_joints(self, joints):
        """ Publishes the current joint states. """
        self.joint1_pub.publish(CommandDuration(data=joints.joint1, duration=self.joint_duration))
        self.joint2_pub.publish(CommandDuration(data=joints.joint2, duration=self.joint_duration))
        self.joint3_pub.publish(CommandDuration(data=joints.joint3, duration=self.joint_duration))
        self.joint4_pub.publish(CommandDuration(data=joints.joint4, duration=self.joint_duration))
        self.joint5_pub.publish(CommandDuration(data=joints.joint5, duration=self.joint_duration))
    
    def publish_gripper(self, gripper):
        """ Publishes the current gripper state. """
        self.gripper_pub.publish(CommandDuration(data=gripper, duration=self.gripper_duration))

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
    node = Arm()
    node.run()
