#Create a package named open_loop_control in the catkin workspace (~/dd2419_ws/src) folder.
#In the package, write a Python node called open_loop_controller which publishes full power commands (duty cycle 1.0) 
#to the DC motors through the /motor/duty_cycles topic for the wheels every 100 milliseconds (10 Hz).

#Play around with your open-loop controller node and try to send different duty cycle values (between [-1.0, 1.0]) 
#to the motors to see what happens to the trajectory of the robot.


# First import the necessary libraries
import rospy
from robp_msgs.msg import DutyCycles

# main in rospy package to run the code
def main():
    #rospy.init_node('open_loop_controller')
    r = rospy.Rate(10) # 10 hz
    
    # rospy print debug message
    #while True:
    #    r.sleep()
    #    print("lol")
    #    rospy.loginfo("lol")
    

    # init a publisher to the /motor/duty_cycles topic
    duty_cycle_pub = rospy.Publisher('/motor/duty_cycles', DutyCycles, queue_size=1)
    duty_cycle_msg = DutyCycles()
    duty_cycle_msg.duty_cycle_left = 1.0
    duty_cycle_msg.duty_cycle_right = 1.0

    while not rospy.is_shutdown(): # insted of while True, makes sure ros dies when supposed to
        rospy.loginfo("Publishing duty cycle values: %s %s", duty_cycle_msg.duty_cycle_left, duty_cycle_msg.duty_cycle_right)
        # create duty cycle message object
        duty_cycle_pub.publish(duty_cycle_msg)
        r.sleep()
    
    

# Main function.
if __name__ == '__main__':
    rospy.init_node('open_loop_controller', anonymous = True)
    main()
    #r = rospy.Rate(10) # 10 hz
    #
    ## rospy print debug message
    #while True:
    #    r.sleep()
    #    rospy.loginfo("lol")
