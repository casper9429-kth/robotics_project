#!/usr/bin/env python3
import rospy

import numpy as np
from numpy import pi, cos, sin


class Node_name():
    def __init__(self) -> None:
        """ Put the node name here, and description of the node"""
        rospy.init_node('Node_name')

        # Subscribers 
        # self.sub_topic = rospy.Subscriber("topic", type, self.callback_topic)
        
        # Publisher
        # self.message_pub = rospy.Publisher("topic", type, queue_size=10)

        # Define rate
        self.update_rate = 10 # [Hz] Change this to the rate you want
        self.update_dt = 1.0/self.update_rate # [s]
        self.rate = rospy.Rate(self.update_rate)

        # Tf 
        # self.tf_buffer = tf2_ros.Buffer()
        # self.br = tf2_ros.TransformBroadcaster()
        # self.listner = tf2_ros.TransformListener(self.tf_buffer)

        # Parameters

        # Measurements
        self.l_12 = 18e-3 # [m]
        self.l_23 = 100e-3 # [m]
        self.l_34 = 97e-3 # [m]
        # self.l_5e = 100e-3 # [m] TODO: Find this value

        # Joint angles
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.theta4 = 0
        self.theta5 = 0

    ###### All your callbacks here ######
        
    # def callback_topic(self): 
    #     """Callback function for the topic"""
    #     # do callback stuff
    #     pass

    ###### All your other methods here #######

    # def publish(self):
    #     """Publish your messages here"""
    # 
    #     pass
    #     # self.message_pub.publish('')
    
    def dh_parameters(self):
        """
        Return the DH parameters for the arm.
        The parameters are given in the order r, alpha, d, theta.
        The first row is the first joint to the second joint.
        The last row is the fifth joint to the end effector.
        """
        DH = [[0,         pi/2,  self.l_12, self.theta1       ],
              [self.l_23, 0,     0,         self.theta2 - pi/2],
              [self.l_34, 0,     0,         self.theta3       ],
              [0,         -pi/2, 0,         self.theta4 + pi/2],
              [0,         pi/2,  self.l_5e, self.theta5       ]]
    
    def forward_kinematics(self):
        """
        Return the pose of the end effector in the base frame as a homogeneous transformation matrix.
        """
        DH = self.dh_parameters()
        T = np.eye(4)
        for i in range(len(DH)):
            r, alpha, d, theta = DH[i]
            ct, st, ca, sa = cos(theta), sin(theta), cos(alpha), sin(alpha)
            T_i = np.array([[ct, -st*ca, st*sa,  r*ct],
                            [st, ct*ca,  -ct*sa, r*st],
                            [0,  sa,     ca,     d   ],
                            [0,  0,      0,      1   ]])
            T = T @ T_i
        return T

    def inverse_kinematics(self, T):
        """
        Return the joint angles for the end effector pose T, where T is a homogeneous transformation matrix.
        """
        pass
        

    def main(self): # Do main stuff here    
        """
        Main loop, instead of changing run function,
        write your code here to make it more readable.
        """
        print('funny message')

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

    node=Node_name()
    node.run()
