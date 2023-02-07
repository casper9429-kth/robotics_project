#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0


def move(path):
    global control_client, robot_frame_id, pub
    #baselink = robot_frame_id
    path1 = path.path
    gain = path.gain
    
    # Call service client with path
    point_request = GetSetpointRequest(path1)
    
    return_point = control_client(point_request)
    setpoint = return_point.setpoint
    new_path = return_point.new_path
    #rospy.loginfo("The frame is: %s", setpoint.header.frame_id)
    twist_msg = Twist()
    rospy.Rate(10)
    
    while new_path.poses:
        #print(new_path)
    # Transform Setpoint from service client
        transform = tf_buffer.lookup_transform(robot_frame_id,'map',rospy.Time()) # The transform that relate map fram to base link frame
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform) # Transforms the setpoint to the base link frame

        #twist_msg = Twist()
        
        turn =  3 * atan2(transformed_setpoint.point.y,transformed_setpoint.point.x)
        forward = 1 * hypot(transformed_setpoint.point.y,transformed_setpoint.point.x)
        
        if turn >= max_angular_velocity:
            turn = max_angular_velocity
        if forward >= max_linear_velocity:
            forward = max_linear_velocity
            
        twist_msg.angular.z = turn
        twist_msg.linear.x = forward
        pub.publish(twist_msg)
        
        point_request = GetSetpointRequest(new_path)
        return_point = control_client(point_request)
        setpoint = return_point.setpoint
        new_path = return_point.new_path
        #rospy.sleep()
    #quitting
    print('Yay, found a goal')
    twist_msg.angular.z = 0
    twist_msg.linear.x = 0
    pub.publish(twist_msg)    
    
    #get_path()

def get_path():
    global goal_client
    #print(goal_client)
#while
    # Get path from action server
    goal_client.wait_for_server()
    msg = irob_assignment_1.msg.GetNextGoalResult()
    goal_client.send_goal(msg)
    goal_client.wait_for_result()
    return_msg = goal_client.get_result()

    # Call move with path from action server
    move(return_msg)
    
    if return_msg.gain != 0.0:
        print('Finding new path\n')
        print('Gain:',return_msg.gain)
        get_path()
    
    print('Gain:',return_msg.gain)
    print('Finished')
    
    
if __name__ == "__main__":
    # Init node
    rospy.init_node("controller")
    # create tf buffer and listner
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Init publisher
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

    # Init simple action client
    actionclient = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)
    goal_client = actionclient
    
    # Init service client
    control_client = rospy.ServiceProxy('get_setpoint',GetSetpoint)
    
    # Call get path
    
    get_path()
    # Spin
    rospy.spin()
    # Init stuff


