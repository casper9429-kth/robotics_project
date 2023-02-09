
#!/usr/bin/env python3
import rospy
from hiwonder_servo_msgs.msg import CommandDuration
from std_srvs.srv import Trigger, TriggerResponse
from rospy import Service


class JointState:
    def __init__(self, joint1=0, joint2=0, joint3=0, joint4=0, joint5=0, gripper=0):
        self.joint1 = joint1
        self.joint2 = joint2
        self.joint3 = joint3
        self.joint4 = joint4
        self.joint5 = joint5
        self.gripper = gripper


class Yeet():
    def __init__(self) -> None:
        """ Yeets a cube. """
        rospy.init_node('arm')
        
        # Publisher
        self.joint1_pub = rospy.Publisher("/joint1_controller/command_duration", CommandDuration, queue_size=10)
        self.joint2_pub = rospy.Publisher("/joint2_controller/command_duration", CommandDuration, queue_size=10)
        self.joint3_pub = rospy.Publisher("/joint3_controller/command_duration", CommandDuration, queue_size=10)
        self.joint4_pub = rospy.Publisher("/joint4_controller/command_duration", CommandDuration, queue_size=10)
        self.joint5_pub = rospy.Publisher("/joint5_controller/command_duration", CommandDuration, queue_size=10)
        self.gripper_pub = rospy.Publisher("/r_joint_controller/command_duration", CommandDuration, queue_size=10)

        self.initial_position_service = Service('arm/initial', Trigger, self.initial_service_callback)
        self.prepare_pick_up_service = Service('arm/prepare_pick_up', Trigger, self.prepare_pick_up_service_callback)
        self.pick_up_service = Service('arm/pick_up', Trigger, self.pick_up_service_callback)
        self.hold_service = Service('arm/hold', Trigger, self.hold_service_callback)

        # Define rate
        self.update_rate = 10 # [Hz]
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate)

        # Tf 
        # self.tf_buffer = tf2_ros.Buffer()
        # self.br = tf2_ros.TransformBroadcaster()
        # self.listner = tf2_ros.TransformListener(self.tf_buffer)

        # Parameters HERE

        # Duration of the motion
        self.duration = 2000 # [ms]

        # gripper
        self.gripper_open = -1.5
        self.gripper_close_sphere = -0.5
        self.gripper_close_cube = -0.0

        # joints initial position
        self.joints_initial = JointState(gripper=self.gripper_open)

        # joints pick up
        self.joints_prepare_pick_up = JointState(0, -1, -0.8, -1.3, -0.2, self.gripper_open)
        self.joints_pick_up_cube = JointState(0, -1, -0.8, -1.3, -0.2, self.gripper_close_cube)
        self.joints_pick_up_sphere = JointState(0, -1, -0.8, -1.3, -0.2, self.gripper_close_sphere)

        # joints hold
        self.joints_hold_cube = JointState(gripper=self.gripper_close_cube)
        self.joints_hold_sphere = JointState(gripper=self.gripper_close_sphere)

        # Variables HERE
        self.target_joints = self.joints_initial

    ###### All your callbacks here ######

    def initial_service_callback(self, req):
        """ Callback function for the initial position service. """
        self.target_joints = self.joints_initial
        self.publish()
        return TriggerResponse(True, 'Initial')
    
    def prepare_pick_up_service_callback(self, req): 
        """ Callback function for the prepare pick up service. """
        self.target_joints = self.joints_prepare_pick_up
        self.publish()
        return TriggerResponse(True, 'Prepare pick up')

    def pick_up_service_callback(self, req):
        """ Callback function for the prepare pick up service. """
        type = 'cube'
        if type == 'cube':
            self.target_joints = self.joints_pick_up_cube
            self.publish()
            return TriggerResponse(True, 'Pick up cube')
        elif type == 'sphere':
            self.target_joints = self.joints_pick_up_sphere
            self.publish()
            return TriggerResponse(True, 'Pick up sphere')
        else:
            return TriggerResponse(False, 'Type must be either "cube" or "sphere".')
        
    def hold_service_callback(self, req):
        """ Callback function for the prepare pick up service. """
        type = 'cube'
        if type == 'cube':
            self.target_joints = self.joints_hold_cube
            self.publish()
            return TriggerResponse(True, 'Hold cube')
        elif type == 'sphere':
            self.target_joints = self.joints_hold_sphere
            self.publish()
            return TriggerResponse(True, 'Hold sphere')
        else:
            return TriggerResponse(False, 'Type must be either "cube" or "sphere".')
        
    ###### All your other methods here #######

    def publish(self):
        """ Publishes the current joint states. """
        self.joint1_pub.publish(CommandDuration(data=self.target_joints.joint1, duration=self.duration))
        self.joint2_pub.publish(CommandDuration(data=self.target_joints.joint2, duration=self.duration))
        self.joint3_pub.publish(CommandDuration(data=self.target_joints.joint3, duration=self.duration))
        self.joint4_pub.publish(CommandDuration(data=self.target_joints.joint4, duration=self.duration))
        self.joint5_pub.publish(CommandDuration(data=self.target_joints.joint5, duration=self.duration))
        self.gripper_pub.publish(CommandDuration(data=self.target_joints.gripper, duration=self.duration))

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

    node = Yeet()
    node.run()
