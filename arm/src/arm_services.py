#!/usr/bin/env python3
from math import pi, atan2, acos, asin, sqrt

import rospy
from hiwonder_servo_msgs.msg import CommandDuration
from std_srvs.srv import Trigger, TriggerResponse
from rospy import Service
from sensor_msgs.msg import JointState

from arm.srv import SetPickUpTarget, SetPickUpTargetResponse, ArmTrigger, ArmTriggerResponse

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

    def greatest_difference(self, other: 'Joints'):
        # return the greatest difference between two joint states
        return max(abs(self.joint1 - other.joint1), abs(self.joint2 - other.joint2), abs(self.joint3 - other.joint3), abs(self.joint4 - other.joint4), abs(self.joint5 - other.joint5))


class MissingJointsError(Exception):
    pass


class ArmServices():
    def __init__(self) -> None:
        """ Services for controlling the arm. """
        rospy.init_node('arm_services')

        # Subscribers
        self.joints_sub = rospy.Subscriber("/joint_states", JointState, self.joints_callback, queue_size=10)
        
        # Publishers
        self.joint1_pub = rospy.Publisher("/joint1_controller/command_duration", CommandDuration, queue_size=10)
        self.joint2_pub = rospy.Publisher("/joint2_controller/command_duration", CommandDuration, queue_size=10)
        self.joint3_pub = rospy.Publisher("/joint3_controller/command_duration", CommandDuration, queue_size=10)
        self.joint4_pub = rospy.Publisher("/joint4_controller/command_duration", CommandDuration, queue_size=10)
        self.joint5_pub = rospy.Publisher("/joint5_controller/command_duration", CommandDuration, queue_size=10)
        self.gripper_pub = rospy.Publisher("/r_joint_controller/command_duration", CommandDuration, queue_size=10)

        # Services
        self.straight_service = Service('arm/poses/straight', ArmTrigger, self.straight_service_callback)
        self.default_service = Service('arm/poses/default', ArmTrigger, self.default_service_callback)
        self.observe_service = Service('arm/poses/observe', ArmTrigger, self.observe_service_callback)
        self.prepare_to_pick_up_service = Service('arm/poses/prepare_to_pick_up', ArmTrigger, self.prepare_to_pick_up_service_callback)
        self.pick_up_service = Service('arm/poses/pick_up', ArmTrigger, self.pick_up_service_callback)

        self.open_gripper_service = Service('arm/poses/open_gripper', ArmTrigger, self.open_gripper_service_callback)
        self.close_gripper_service = Service('arm/poses/close_gripper', ArmTrigger, self.close_gripper_service_callback)

        self.set_pick_up_target_service = Service('arm/poses/set_target', SetPickUpTarget, self.set_pick_up_target_service_callback)

        # Define rate
        self.update_rate = 10 # [Hz]
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate)

        # Parameters

        # Measurements
        self.d_12 = 30e-3 # [m]
        self.d_23 = 100e-3 # [m]
        self.d_34 = 96e-3 # [m]
        self.d_5e = 180e-3 # [m] this is to the end of the end effector with a closed cube grip (approximately)
        self.d_floor_to_base = 131e-3 # [m]
        
        self.pick_up_margin = 6e-3 # [m]
        self.prepare_to_pick_up_margin = 55e-3 # [m]

        # Duration of the motion
        self.max_joint_speed = 0.8 # [rad/s]
        self.gripper_duration = 1 # [s]

        # gripper
        self.gripper_open = -1.8
        self.gripper_close = { 'cube': 0, 'sphere': -0.45, 'animal': 0 }

        # Joints
        self.joints_straight = Joints()
        self.joints_default = Joints(0, 0.52, -1.36, -1.76, 0)
        self.joints_observe = Joints(0, 0, -pi/2, -pi/2, 0)
        self.joints_prepare_to_pick_up = None
        self.joints_pick_up = None

        # Current state
        self.joints = None
        self.gripper = None

        # Target type
        self.target_type = None
        self.allowed_target_types = ['cube', 'sphere', 'animal']

        # Error messages
        self.error_messages = {
            'missing joints': 'No joint states received. Is anyone publishing to /joint_states?',
            'missing target': 'No pick up target received.',
            'bad target type': '''Bad target type. Legal values are 'cube', 'sphere' and 'animal'.''',
            'target out of reach': 'Pick up target out of reach',
        }


    ###### All your callbacks here ######

    def joints_callback(self, msg):
        self.joints = Joints(msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4]) 
        self.gripper = msg.position[5]

    def straight_service_callback(self, _):
        duration = None
        try:
            duration = self.calculate_duration(self.joints, self.joints_straight)
        except MissingJointsError:
            return ArmTriggerResponse(False, self.error_messages['missing joints'], 0)
        self.publish_joints(self.joints_straight, duration)
        return ArmTriggerResponse(True, 'Arm straightening', duration)
    
    def default_service_callback(self, _):
        duration = None
        try:
            duration = self.calculate_duration(self.joints, self.joints_default)
        except MissingJointsError:
            return ArmTriggerResponse(False, self.error_messages['missing joints'], 0)
        self.publish_joints(self.joints_default, duration)
        return ArmTriggerResponse(True, 'Arm defaulting', duration)
    
    def observe_service_callback(self, _):
        duration = None
        try:
            duration = self.calculate_duration(self.joints, self.joints_observe)
        except MissingJointsError:
            return ArmTriggerResponse(False, self.error_messages['missing joints'], 0)
        self.publish_joints(self.joints_observe, duration)
        return ArmTriggerResponse(True, 'Arm observing', duration)

    def prepare_to_pick_up_service_callback(self, _):
        if self.joints_prepare_to_pick_up is None:
            return ArmTriggerResponse(False, self.error_messages['missing target'], 0)
        duration = None
        try:
            duration = self.calculate_duration(self.joints, self.joints_prepare_to_pick_up)
        except MissingJointsError:
            return ArmTriggerResponse(False, self.error_messages['missing joints'], 0)
        self.publish_joints(self.joints_prepare_to_pick_up, duration)
        return ArmTriggerResponse(True, 'Arm preparing to pick up', duration)

    def pick_up_service_callback(self, _):
        if self.joints_pick_up is None:
            return ArmTriggerResponse(False, self.error_messages['missing target'], 0)
        duration = None
        try:
            duration = self.calculate_duration(self.joints, self.joints_pick_up)
        except MissingJointsError:
            return ArmTriggerResponse(False, self.error_messages['missing joints'], 0)
        self.publish_joints(self.joints_pick_up, duration)
        return ArmTriggerResponse(True, 'Arm picking up', duration)
        
    def open_gripper_service_callback(self, _):
        self.publish_gripper(self.gripper_open, self.gripper_duration)
        return ArmTriggerResponse(True, 'Opening gripper', self.gripper_duration)
    
    def close_gripper_service_callback(self, _):
        if self.target_type is None:
            return ArmTriggerResponse(False, self.error_messages['missing target'], 0)
        try:
            self.publish_gripper(self.gripper_close[self.target_type], self.gripper_duration)
            return ArmTriggerResponse(True, 'Closing gripper', self.gripper_duration)
        except KeyError:
            return ArmTriggerResponse(False, self.error_messages['bad target type'], 0)
    
    def set_pick_up_target_service_callback(self, pick_up_target):
        if pick_up_target.type not in self.allowed_target_types:
            return SetPickUpTargetResponse(False, self.error_messages['bad target type'])
        self.target_type = pick_up_target.type
        x = pick_up_target.x
        y = pick_up_target.y
        z = pick_up_target.z
        yaw = pick_up_target.yaw
        try:
            self.joints_prepare_to_pick_up = self.inverse_kinematics(x, y, z + self.prepare_to_pick_up_margin, yaw)
            self.joints_pick_up = self.inverse_kinematics(x, y, z + self.pick_up_margin, yaw)
        except ValueError:
            return SetPickUpTargetResponse(False, self.error_messages['target out of reach'])
        return SetPickUpTargetResponse(True, 'Pick up target set')
        
    ###### All your other methods here #######

    def inverse_kinematics(self, x, y, z, yaw):
        """ Calculates the inverse kinematics for the arm. """
        theta1 = atan2(y, x) % (2 * pi) - pi
        yaw = (yaw + pi / 2) % pi - pi / 2
        theta5 = theta1 - yaw
        
        # The inverse kinematics works from the second joint to the fourth joint.
        z = z - self.d_12 + self.d_5e

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
    
    def calculate_duration(self, start_joints, end_joints):
        """ Calculates the duration of a motion. """
        if self.joints is None:
            raise MissingJointsError('No joints received yet.')
        greatest_change = start_joints.greatest_difference(end_joints)
        duration = greatest_change / self.max_joint_speed # [s]
        return duration

    def publish_joints(self, joints, duration):
        """ Publishes the current joint states. """
        duration *= 1000 # [s] -> [ms]
        self.joint1_pub.publish(CommandDuration(data=joints.joint1, duration=duration))
        self.joint2_pub.publish(CommandDuration(data=joints.joint2, duration=duration))
        self.joint3_pub.publish(CommandDuration(data=joints.joint3, duration=duration))
        self.joint4_pub.publish(CommandDuration(data=joints.joint4, duration=duration))
        self.joint5_pub.publish(CommandDuration(data=joints.joint5, duration=duration))
    
    def publish_gripper(self, gripper, duration):
        """ Publishes the current gripper state. """
        duration *= 1000 # [s] -> [ms]
        self.gripper_pub.publish(CommandDuration(data=gripper, duration=duration))

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
    node = ArmServices()
    node.run()
