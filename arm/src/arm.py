
#!/usr/bin/env python3
import rospy
from hiwonder_servo_msgs.msg import CommandDuration
from std_srvs.srv import Trigger, TriggerResponse
from rospy import Service


class JointState:
    def __init__(self, joint1=0, joint2=0, joint3=0, joint4=0, joint5=0):
        self.joint1 = joint1
        self.joint2 = joint2
        self.joint3 = joint3
        self.joint4 = joint4
        self.joint5 = joint5


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

        # Services
        self.initial_position_service = Service('arm/poses/initial', Trigger, self.initial_service_callback)
        self.prepare_pick_up_service = Service('arm/poses/prepare_to_pick_up', Trigger, self.prepare_to_pick_up_service_callback)
        # self.pick_up_service = Service('arm/pick_up', TODO, self.pick_up_service_callback)
        self.open_gripper_service = Service('arm/poses/open_gripper', Trigger, self.open_gripper_service_callback)
        self.close_gripper_service = Service('arm/poses/close_gripper', Trigger, self.close_gripper_service_callback)

        # Define rate
        self.update_rate = 10 # [Hz]
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate)

        # Parameters

        # Duration of the motion
        self.joint_duration = 2000 # [ms]
        self.gripper_duration = 1000 # [ms]

        # gripper
        self.gripper_open = -1.5
        self.gripper_close_sphere = -0.5
        self.gripper_close_cube = -0.0

        # joints initial position
        self.joints_initial = JointState()

        # joints pick up
        self.joints_prepare_pick_up = JointState(0, -1, -0.8, -1.3, -0.2)

    ###### All your callbacks here ######

    def initial_service_callback(self, req):
        """ Callback function for the initial position service. """
        self.publish_joints(self.joints_initial)
        return TriggerResponse(True, 'Initial')
    
    def prepare_to_pick_up_service_callback(self, req): 
        """ Callback function for the prepare to pick up service. """
        self.publish_joints(self.joints_prepare_pick_up)
        return TriggerResponse(True, 'Preparing to pick up')

    def pick_up_service_callback(self, req):
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
        
    def open_gripper_service_callback(self, req):
        """ Callback function for the close gripper service. """
        self.publish_gripper(self.gripper_open)
        return TriggerResponse(True, 'Opened gripper')
    
    def close_gripper_service_callback(self, req):
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
        
    ###### All your other methods here #######

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
