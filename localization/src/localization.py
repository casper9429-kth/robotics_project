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
        self.dt = 0.04
        self.rate = rospy.Rate(1/self.dt)

        # Uncertainty parameters
        self.P_init = 1.0        
        self.std_omega = 0.01
        self.R = np.eye(3)*0.01
        
        # Init a tf broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Init a tf listener
        self.listener = tf.TransformListener()
    
        # Save new returned values from callbacks
        self.omega_control_list = []
        self.omega_imu_list = []
        self.omega_enco_list = []
        self.theta_imu_list = []
        
        # Run loop
        self.ukf_estimate()    
    
        
    def ukf_estimate(self):
        """
        Use UKF and IMU, encoder, control data to estimate state as long as ROS is running.
        Continuously save result to self.v, self.omega.
        """
        rospy.loginfo("ukf_estimate")
        
        # Initialize the UKF state and state covariance matrix
        x = np.array([self.omega,self.theta])
        P = np.eye(2)*self.P_init
        
        while not rospy.is_shutdown():
            
            # Get the impults form all sensors and empty the lists
            if self.omega_control_list:
                omega_control = np.mean(self.omega_control_list)
                self.omega_control_list = [omega_control]
            else:
                omega_control = 0
    
            if self.omega_imu_list:        
                omega_imu = np.mean(self.omega_imu_list)
                self.omega_imu_list = [omega_imu]
            else:
                omega_imu = 0

            if self.omega_enco_list:
                omega_enco = np.mean(self.omega_enco_list)
                self.omega_enco_list = [omega_enco]
            else:
                omega_enco = 0

            if self.theta_imu_list:
                theta_imu = np.mean(self.theta_imu_list)
                self.theta_imu_list = [theta_imu]
            else:
                theta_imu = 0

            # Predict the next state of the robot
            x, P = self.predict_state(x, P, omega_control)
            
            # Get the measurement updates from the encoders and IMU
            z = np.array([omega_imu,omega_enco,theta_imu])
            
            # Update the state estimate using the measurement updates
            x, P = self.update_state(x, P, z)
            
            # Save the updated state estimate
            self.omega,self.theta = x[0], x[1]
            
            # Print the state estimate
            rospy.loginfo("theta: %f,  omega: %f", self.theta,  self.omega)
            self.publish_state(self.theta)

        
            
            self.rate.sleep()
            
            
    def publish_state(self,yaw):
            t = TransformStamped()
            t.header.frame_id = "odom"
            t.child_frame_id = "test_link"
                    # Add new x, y and yaw to transform, first cart then rot
            t.header.stamp = rospy.Time.now()
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0 # z is always 0
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw) # transform yaw to quaternion
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            # Publish transform to tf broadcaster init in __init__
            self.br.sendTransform(t)
    
    def predict_state(self, x, P, omega_control):
        """
        Predict the next state of the robot based on the control inputs and current state.
        """
        # Number of states
        n = x.shape[0]

        # Process noise standard deviation
        std_omega = self.std_omega

        # Control input
        u = np.array([omega_control])

        # Generate sigma points
        W, X = self.generate_sigma_points(x, P)

        # Propagate sigma points through the process model
        for i in range(2 * n + 1):
            X[:, i] = self.process_model(X[:, i], u, std_omega)

        # Recalculate the mean and covariance of the predicted state
        x_pred = X[:,i]*W[i]
        P_pred = np.zeros((n, n))

        for i in range(2 * n + 1):
            P_pred +=  W[i]* np.outer(X[:, i] - x_pred, X[:, i] - x_pred)

        return x_pred, P_pred
        
    def process_model(self, x, u, std_omega):
        """
        Model the motion of the robot based on the control inputs and previous state.
        """
        # Unpack the state and control inputs
        omega_prev,theta_prev = x
        omega = u[0]

        # Update the state using the robot's kinematics
        theta = theta_prev + omega * self.dt
        omega = omega_prev + np.random.normal(0, std_omega)
        
        # Pack the updated state
        x = np.array([omega,theta])
        
        return x
        

    def update_state(self, x, P, z):
        n = x.shape[0]
        m = z.shape[0]

        # Measurement noise covariance

        R = self.R

        
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
        W[0] = 1.0/n#np.power(n, -1)
        W[1:] = np.power(2.0 * n, -1)

        # Generate the sigma points
        c = 0
        input_matrix = n*P 

        while True:
            try:
                U = np.linalg.cholesky(input_matrix + 0.05*np.eye(n)*c)
                break
            except:
                c += 1
                
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
        #self.encoder_callback_list.append({'t':t, 'v':self.v_enco, 'omega':self.omega_enco})
        self.omega_enco_list.append(self.omega_enco)
        
        
    def measurement_model(self, x):
            """
            Transforms the state estimate into a measurement space
            """
            omega = x[0]
            theta = x[1]

            # Calculate the encoder readings
            omega_enco = omega

            # Calculate the IMU readings
            omega_imu = omega
            theta_imu = theta

            return np.array([omega_enco, omega_imu, theta_imu])


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
        #self.control_callback_list.append({'t':t, 'v':self.v_control, 'omega':self.omega_control})
        self.omega_control_list.append(self.omega_control)
        
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
        self.omega_imu = -msg.angular_velocity.z
        
        ## linear acceleration
        self.imu_acc_x = msg.linear_acceleration.x
        self.imu_acc_y = msg.linear_acceleration.y

        #self.imu_callback_list.append({'t': t, 'theta': self.theta_imu, 'omega': self.omega_imu, 'acc_x': self.imu_acc_x, 'acc_y': self.imu_acc_y})
        #rospy.loginfo("IMU callback %s", self.imu_callback_list[-1])
        self.omega_imu_list.append(self.omega_imu)
        self.theta_imu_list.append(self.theta_imu)

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