#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from robp_msgs.msg import Encoders
from robp_msgs.msg import DutyCycles
import tf_conversions
import tf2_ros
import math
from  math import pi
from aruco_msgs.msg import MarkerArray
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np



class Localization:
    def __init__(self):
        # Initialize node
        rospy.loginfo('Initializing localization node')
        rospy.init_node('localization')
        
        # Initial sleep to allow roscore to start
        rospy.sleep(5)
        
        
        # Subscribers
        ## Encoders
        self.sub_goal = rospy.Subscriber('/motor/encoders', Encoders, self.encoder_callback)
            
        ## IMU: accelerometer, compass
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback)        
        
        ## control: /motor/duty_cycles
        self.sub_control = rospy.Subscriber('/motor/duty_cycles', DutyCycles, self.control_callback)
        
                
        ## state, updated by ukf_estimate function
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0
        self.omega = 0
        
        # global variables used by callbacks
        self.v_control = 0
        self.omega_control = 0
        self.v_enco = 0
        self.omega_enco = 0
        self.omega_imu = 0
        self.theta_imu = 0
        self.imu_acc_x = 0
        self.imu_acc_y = 0

        # imu encoder callback
        self.imu_callback_list = []
        self.control_callback_list = []
        self.encoder_callback_list = []    
        
        # Robot parameters
        self.ticks_per_rev = 3072
        self.wheel_r = 0.04921
        self.base = 0.3 

        # Create rate var
        self.dt = 0.01
        self.rate = rospy.Rate(1/self.dt)

        


        
        # Run loop
        self.ukf_estimate()    
    
        
    def ukf_estimate(self):
        """
        Use UKF and IMU, encoder, control data to estimate state as long as ROS is running.
        Continuously save result to self.x, self.y, self.theta, self.v, self.omega.
        """
        rospy.loginfo("ukf_estimate")
        
        # Initialize the UKF state and state covariance matrix
        x = np.array([self.v, self.omega,self.theta])
        P = np.eye(3)*0.001
        
        while not rospy.is_shutdown():
            # Predict the next state of the robot
            x, P = self.predict_state(x, P, self.v_control, self.omega_control)
            
            # Get the measurement updates from the encoders and IMU
            z = np.array([self.v_enco, self.omega_enco, self.omega_imu,self.theta_imu])
            
            # Update the state estimate using the measurement updates
            x, P = self.update_state(x, P, z)
            
            # Save the updated state estimate
            self.v, self.omega,self.theta = x[0], x[1], x[2]
            
            rospy.loginfo("theta: %f, v: %f, omega: %f", self.theta, self.v, self.omega)
            
            self.rate.sleep()
            
    def predict_state(self, x, P, v_control, omega_control):
        """
        Predict the next state of the robot based on the control inputs and current state.
        """
        # Number of states
        n = x.shape[0]

        # Process noise standard deviation
        std_v = 0.1
        std_omega = 0.1

        # Control input
        u = np.array([v_control, omega_control])

        # Generate sigma points
        W, X = self.generate_sigma_points(x, P)

        # Propagate sigma points through the process model
        for i in range(2 * n + 1):
            X[:, i] = self.process_model(X[:, i], u, std_v, std_omega)

        # Recalculate the mean and covariance of the predicted state
        x_pred = X[:,i]*W[i]
        P_pred = np.zeros((n, n))

        for i in range(2 * n + 1):
            #P_pred[i,i] = #W[i]* np.outer(X[:, i] - x_pred, X[:, i] - x_pred)
            P_pred +=  W[i]* np.outer(X[:, i] - x_pred, X[:, i] - x_pred)

        return x_pred, P_pred
        
    def process_model(self, x, u, std_v, std_omega):
        """
        Model the motion of the robot based on the control inputs and previous state.
        """
        # Unpack the state and control inputs
        v_prev,omega_prev,theta_prev = x
        v, omega = u

        # Update the state using the robot's kinematics
        theta = theta_prev + omega * self.dt
        v = v_prev + np.random.normal(0, std_v)
        omega = omega_prev + np.random.normal(0, std_omega)
        
        # Pack the updated state
        x = np.array([v, omega,theta])
        
        return x
        

    def update_state(self, x, P, z):
        n = x.shape[0]
        m = z.shape[0]

        # Measurement noise covariance
        R = np.eye(m)*0.1

        # Generate sigma points
        W, X = self.generate_sigma_points(x, P)

        # Propagate sigma points through the measurement model
        Z = np.zeros((m, 2 * n + 1))
        for i in range(2 * n + 1):
            Z[:, i] = self.measurement_model(X[:, i])

        # Recalculate the mean and covariance of the measurement
        z_hat = np.sum(W * Z, axis=1)
        Z_diff = Z - z_hat[:, np.newaxis]
        P_zz = np.dot(Z_diff * W, Z_diff.T) + R

        # Calculate the cross-covariance of the state and measurement
        X_diff = np.array(X - x[:, np.newaxis])
        
        #P_xz = np.einsum('i,ji->j',X_diff*W,Z_diff.T)
            
        P_xz = np.dot(X_diff * W, Z_diff.T)

        # Calculate the Kalman gain
        K = np.dot(P_xz, np.linalg.inv(P_zz))

        # Update the state estimate
        x = x + np.dot(K, z - z_hat)

        P = P - np.dot(K, P_zz) @ K.T
        return x, P



    def generate_sigma_points(self, x, P):
        n = x.shape[0]
        W = np.zeros(2 * n + 1)
        X = np.zeros((n, 2 * n + 1))

        # Set the weights for the sigma points
        W[0] = 1/n#np.power(n, -1)
        W[1:] = np.power(2.0 * n, -1)

        # Generate the sigma points
        input_matrix = n*P + 0.1*np.eye(n)
        U = np.linalg.cholesky(input_matrix)

        X[:, 0] = x
        for i in range(n):
            X[:, i + 1] = x + U[i, :]
            X[:, i + n + 1] = x - U[i, :]
        return W, X

    def encoder_callback(self, msg):
        """
        Calculates v, omega from encoder data
        Saves v, omega to self.v_enco, self.omega_enco
        """
        # calculate v_left, v_right
        t = msg.header.stamp.to_sec()
        v_left = (((msg.delta_encoder_left/ self.ticks_per_rev ) * 2*pi * self.wheel_r )/ msg.delta_time_left)*1000
        v_right = (((msg.delta_encoder_right/ self.ticks_per_rev ) * 2*pi * self.wheel_r )/ msg.delta_time_right)*1000

        
        # calculate v, omega
        v, omega = self.transform_v_left_v_right_to_v_omega(v_left, v_right)
        
        self.v_enco = v
        self.omega_enco = omega
        self.encoder_callback_list.append({'t':t, 'v':self.v_enco, 'omega':self.omega_enco})
        
        
        
    def measurement_model(self, x):
            """
            Transforms the state estimate into a measurement space
            """
            v = x[0]
            omega = x[1]
            theta = x[2]

            # Calculate the encoder readings
            v_enco = v
            omega_enco = omega

            # Calculate the IMU readings
            omega_imu = omega
            theta_imu = theta

            return np.array([v_enco, omega_enco, omega_imu, theta_imu])


    def control_callback(self, msg):        
        """
        Callback for control messages that is also received by the motor controller node
        Calculates v, omega from control data
        saves v, omega to self.v_control, self.omega_control
        """
        t = msg.header.stamp.to_sec()
        v_left = msg.duty_cycle_left
        v_right = msg.duty_cycle_right
        self.v_control, self.omega_control = self.transform_v_left_v_right_to_v_omega(v_left, v_right)
        self.control_callback_list.append({'t':t, 'v':self.v_control, 'omega':self.omega_control})
        
    def imu_callback(self, msg):
        """
        This callback is called when a new IMU message is received
        It calcluates:
        1. self.theta_imu: the orientation of the robot in the IMU frame
        2. self.omega_imu: the angular velocity of the robot in the IMU frame
        3. self.imu_acc_x: the linear acceleration of the robot in the IMU frame
        4. self.imu_acc_y: the linear acceleration of the robot in the IMU frame
        """
        # Get time stamp as a float
        t = msg.header.stamp.to_sec()
        
        # Extract data
        ## orientation
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        
        ### Transform quaternion to euler
        euler = tf.transformations.euler_from_quaternion(q)
        self.theta_imu = euler[2]
        
        ## anuglar velocity
        self.omega_imu = msg.angular_velocity.z
        
        ## linear acceleration
        self.imu_acc_x = msg.linear_acceleration.x
        self.imu_acc_y = msg.linear_acceleration.y

        self.imu_callback_list.append({'t': t, 'theta': self.theta_imu, 'omega': self.omega_imu, 'acc_x': self.imu_acc_x, 'acc_y': self.imu_acc_y})


    def transform_v_omega_to_v_left_v_right(self, v, omega):
        """Transforms the desired linear and angular velocity to the desired wheel velocities, angular velocity is in rad/s, speed in m/s"""
        v_right = (2*v + self.base * omega)/2
        v_left = (2*v - self.base * omega)/2
        return v_left, v_right



    def transform_v_left_v_right_to_v_omega(self, v_left, v_right):
        """Transforms the desired wheel velocities to the desired linear and angular velocity, angular velocity is in rad/s, speed in m/s"""
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self.base
        return v, omega

if __name__ == "__main__":
    rospy.loginfo("Starting localization node")
    try:

        node = Localization()
    except rospy.ROSInterruptException:
        rospy.loginfo("Localization node terminated.")