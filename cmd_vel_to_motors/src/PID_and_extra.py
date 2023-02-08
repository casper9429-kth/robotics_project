import rospy
import math
class PIDController:
    # Initialize the PID controller with the proportional, integral, and derivative gains
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = 0.0
        self.previous_error = 0.0
    # Update the PID controller with the new error and dt (time elapsed since the last update)
    def update(self, error, dt):
        # Integrate the error over time
        self.error_sum += error * dt
        # Calculate the derivative of the error
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        # Calculate the output of the PID controller as a weighted sum of the error, the integral of the error, and the derivative of the error
        return self.kp * error + self.ki * self.error_sum + self.kd * derivative



class DifferentialWheeledRobot:
    # Initialize the robot with its wheel radius, wheel separation, maximum linear speed, and maximum angular speed
    def __init__(self, wheel_radius, wheel_separation, max_linear_speed, max_angular_speed):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0
    # Set the linear speed and angular speed of the robot
    def set_speed(self, linear_speed, angular_speed):
        # Clip the linear speed and angular speed to the maximum values
        self.linear_speed = max(-self.max_linear_speed, min(self.max_linear_speed, linear_speed))
        self.angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))
        # Calculate the left and right wheel speeds based on the linear speed and angular speed
        self.left_wheel_speed = (2 * self.linear_speed - self.angular_speed * self.wheel_separation) / (2 * self.wheel_radius)
        self.right_wheel_speed = (2 * self.linear_speed + self.angular_speed * self.wheel_separation) / (2 * self.wheel_radius)
# The main control loop
def control_loop(robot, setpoint, dt):
    # Calculate the linear speed error and angular speed error
    linear_error = setpoint[0] - robot.linear_speed
    angular_error = setpoint[1] - robot.angular_speed
    # Create two PID controllers for the linear speed and angular speed
    linear_speed_pid = PIDController(0.1, 0.01, 0.005)
    angular_speed_pid = PIDController(0.1, 0.01, 0.005)
    # Update the PID
    linear_speed_output = linear_speed_pid.update(linear_error, dt)
    angular_speed_output = angular_speed_pid.update(angular_error, dt)
    robot.set_speed(linear_speed_output, angular_speed_output)
if __name__ == '__main__':
    robot = DifferentialWheeledRobot(0.1, 0.2, 0.5, 1.0)
    setpoint = (0.4, 0.2)
    dt = 0.01
    for i in range(100):
        control_loop(robot, setpoint, dt)
        print('Linear speed:', robot.linear_speed, 'Angular speed:', robot.angular_speed)
        # for loop to simulate the robot moving