
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerResponse
from rospy import Service


class JointState:
    def __init__(self):
        pass


class Yeet():
    def __init__(self) -> None:
        """ Yeets a cube. """
        rospy.init_node('yeet')
        
        # Publisher
        self.joint1_pub = rospy.Publisher("/joint1_controller/command", Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher("/joint2_controller/command", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("/joint3_controller/command", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("/joint4_controller/command", Float64, queue_size=10)
        self.joint5_pub = rospy.Publisher("/joint5_controller/command", Float64, queue_size=10)
        self.gripper_pub = rospy.Publisher("/r_joint_controller/command", Float64, queue_size=10)

        self.initial_position_service = Service('initial_position', Trigger, self.initial_position_service_callback)
        self.pick_up_service = Service('pick_up', Trigger, self.pick_up_service_callback)
        self.yeet_service = Service('yeet', Trigger, self.yeet_service_callback)

        # Define rate
        self.update_rate = 10 # [Hz]
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate)

        # Tf 
        # self.tf_buffer = tf2_ros.Buffer()
        # self.br = tf2_ros.TransformBroadcaster()
        # self.listner = tf2_ros.TransformListener(self.tf_buffer)

        # Parameters HERE

        # gripper
        self.gripper_open = -1.5
        self.gripper_close_sphere = -0.5
        self.gripper_close_cube = -0.2

        # joints initial position
        self.joint1_initial = 0
        self.joint2_initial = 0
        self.joint3_initial = 0
        self.joint4_initial = 0
        self.joint5_initial = 0

        # joints pick up
        self.joint1_pick_up = 0
        self.joint2_pick_up = -1
        self.joint3_pick_up = -0.8
        self.joint4_pick_up = -1.3
        self.joint5_pick_up = -0.2

        # joints yeet
        self.joint1_yeet = 0
        self.joint2_yeet = 0
        self.joint3_yeet = 0
        self.joint4_yeet = 0
        self.joint5_yeet = 0

        # Variables HERE
        self.should_gripper_move = False
        self.target_gripper = 0
        self.target_joint1 = 0
        self.target_joint2 = 0
        self.target_joint3 = 0
        self.target_joint4 = 0
        self.target_joint5 = -1.5

    ###### All your callbacks here ######

    def initial_position_service_callback(self, req):
        """ Callback function for the initial position service. """
        self.initial_position()
        return TriggerResponse(True, "Initial position")
        
    def pick_up_service_callback(self, req): 
        """ Callback function for the pick up service. """
        self.pick_up(type='cube')
        return TriggerResponse(True, "Pick up")

    def yeet_service_callback(self, req):
        """ Callback function for the yeet service. """
        self.yeet()
        return TriggerResponse(True, "Yeet")

    ###### All your other methods here #######

    def initial_position(self):
        """ Sets joints commands to be published for the initial position. """
        self.target_joint1 = self.joint1_initial
        self.target_joint2 = self.joint2_initial
        self.target_joint3 = self.joint3_initial
        self.target_joint4 = self.joint4_initial
        self.target_joint5 = self.joint5_initial
        self.target_gripper = self.gripper_open

    def pick_up(self, type):
        """ Sets joints commands to be published for a pick up. """
        self.target_joint1 = self.joint1_pick_up
        self.target_joint2 = self.joint2_pick_up
        self.target_joint3 = self.joint3_pick_up
        self.target_joint4 = self.joint4_pick_up
        self.target_joint5 = self.joint5_pick_up
        if type == "sphere":
            self.target_gripper = self.gripper_close_sphere
        elif type == "cube":
            self.target_gripper = self.gripper_close_cube
    
    def yeet(self):
        """ Sets joints commands to be published for a yeet. """
        self.target_joint1 = self.joint1_yeet
        self.target_joint2 = self.joint2_yeet
        self.target_joint3 = self.joint3_yeet
        self.target_joint4 = self.joint4_yeet
        self.target_joint5 = self.joint5_yeet
        self.target_gripper = self.gripper_open

    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """
        self.joint1_pub.publish(Float64(self.target_joint1))
        self.joint2_pub.publish(Float64(self.target_joint2))
        self.joint3_pub.publish(Float64(self.target_joint3))
        self.joint4_pub.publish(Float64(self.target_joint4))
        self.joint5_pub.publish(Float64(self.target_joint5))
        if self.should_gripper_move:
            self.gripper_pub.publish(Float64(self.target_gripper))

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
