
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerResponse
from rospy import Service


class JointState:
    def __init__(self, joint1=0, joint2=0, joint3=0, joint4=0, joint5=0, gripper=-1.5):
        self.joint1 = joint1
        self.joint2 = joint2
        self.joint3 = joint3
        self.joint4 = joint4
        self.joint5 = joint5
        self.gripper = gripper

    def interpolate_path(self, target, steps):
        """ Interpolates a path from current state to target state. """
        path = []
        for i in range(1, steps):
            path.append(JointState(
                self.joint1 + (target.joint1 - self.joint1) * i / steps,
                self.joint2 + (target.joint2 - self.joint2) * i / steps,
                self.joint3 + (target.joint3 - self.joint3) * i / steps,
                self.joint4 + (target.joint4 - self.joint4) * i / steps,
                self.joint5 + (target.joint5 - self.joint5) * i / steps,
                self.gripper + (target.gripper - self.gripper) * i / steps
            ))
        path.append(target)
        return path


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
        self.update_rate = 100 # [Hz]
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
        self.joints_initial = JointState()

        # joints pick up
        self.joints_pick_up_cube = JointState(0, -1, -0.8, -1.3, -0.2, -0.0)
        self.joints_pick_up_sphere = JointState(0, -1, -0.8, -1.3, -0.2, -0.5)

        # joints yeet
        self.joints_yeet = JointState(0, 0, 0, 0, 0, -1.5)

        # Variables HERE
        self.path = [self.joints_initial]
        self.should_publish = True

    ###### All your callbacks here ######

    def initial_position_service_callback(self, req):
        """ Callback function for the initial position service. """
        self.initial_position()
        return TriggerResponse(True, "Initial position")
        
    def pick_up_service_callback(self, req): 
        """ Callback function for the pick up service. """
        try:
            self.pick_up(type='cube')
            return TriggerResponse(True, "Pick up")
        except ValueError as e:
            return TriggerResponse(False, str(e))

    def yeet_service_callback(self, req):
        """ Callback function for the yeet service. """
        self.yeet()
        return TriggerResponse(True, "Yeet")

    ###### All your other methods here #######

    def initial_position(self):
        """ Sets target and path joint states to be published for the initial position. """
        self.path = self.path[0].interpolate_path(self.joints_initial, int(self.update_rate))
        for i in range(len(self.path)):
            self.path[i].gripper = self.gripper_open
        self.should_publish = True

    def pick_up(self, type):
        """ Sets target and path joint states to be published for a pick up. """
        if type == "cube":
            self.path = self.path[0].interpolate_path(self.joints_pick_up_cube, int(self.update_rate))
        elif type == "sphere":
            self.path = self.path[0].interpolate_path(self.joints_pick_up_sphere, int(self.update_rate))
        else:
            raise ValueError("Type must be either 'cube' or 'sphere'")
        # Keep the gripper open until the end of the motion
        for i in range(len(self.path) - 1):
            self.path[i].gripper = self.gripper_open
        self.should_publish = True
    
    def yeet(self):
        """ Sets target and path joint states to be published for a yeet. """
        current_gripper = self.path[0].gripper
        self.path = [self.joints_yeet] * (self.update_rate // 5)
        for i in range(len(self.path)):
            if i > len(self.path) // 4 * 3:
                self.path[i].gripper = current_gripper
        self.should_publish = True

    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """

        current_joints: JointState = self.path[0]
        
        if self.should_publish:
            self.joint1_pub.publish(Float64(current_joints.joint1))
            self.joint2_pub.publish(Float64(current_joints.joint2))
            self.joint3_pub.publish(Float64(current_joints.joint3))
            self.joint4_pub.publish(Float64(current_joints.joint4))
            self.joint5_pub.publish(Float64(current_joints.joint5))
            self.gripper_pub.publish(Float64(current_joints.gripper))
        
        if len(self.path) > 1:
            self.path.pop(0)
        else:
            self.should_publish = False

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
