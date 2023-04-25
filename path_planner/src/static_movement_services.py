#!/usr/bin/env python3
import rospy
from rospy import Service, Publisher, Timer
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Twist
from path_planner.srv import Bool, BoolResponse

class StaticMovementServicesNode:
    def __init__(self):
        rospy.init_node('static_movement_services')
        
        self.reverse_pick_up_service = Service('move/reverse/pick_up', Trigger, self.handle_reverse_pick_up)
        self.reverse_pick_up_service_animal = Service('move/reverse/pick_up_animal', Trigger, self.handle_reverse_pick_up_animal)
        self.forward_pick_up_service = Service('move/forward/pick_up', Trigger, self.handle_forward_pick_up)
        self.box_service = Service('move/reverse/box', Trigger, self.handle_box)
        self.cancel_service = Service('move/cancel', Trigger, self.handle_cancel)
        self.is_running = Service('move/is_running', Bool, self.handle_is_running)
        self.cmd_vel_publisher = Publisher('cmd_vel', Twist, queue_size=10)
        
        self.forward_pick_up_duration = .75 # [s]
        self.reverse_pick_up_duration = .5 # [s]
        self.reverse_pick_up_duration_animal = .25 # [s]
        self.box_duration = 2.6 # [s]
        linear_x = .1
        self.forward_twist = Twist()
        self.forward_twist.linear.x = linear_x # TODO: might be different when using battery
        self.reverse_twist = Twist()
        self.reverse_twist.linear.x = -linear_x # TODO: might be different when using battery
        
        self.timer = None

    def handle_reverse_pick_up(self, _):
        if self.timer:
            return TriggerResponse(success=False, message='Timer already running')
        self.cmd_vel_publisher.publish(self.reverse_twist)
        self.timer = Timer(rospy.Duration(self.reverse_pick_up_duration), self.timer_callback, oneshot=True)
        return TriggerResponse(success=True, message='Backing away from object')
    
    def handle_reverse_pick_up_animal(self, _):
        if self.timer:
            return TriggerResponse(success=False, message='Timer already running')
        self.cmd_vel_publisher.publish(self.reverse_twist)
        self.timer = Timer(rospy.Duration(self.reverse_pick_up_duration_animal), self.timer_callback, oneshot=True)
        return TriggerResponse(success=True, message='Backing away from animal')
    
    def handle_forward_pick_up(self, _):
        if self.timer:
            return TriggerResponse(success=False, message='Timer already running')
        self.cmd_vel_publisher.publish(self.forward_twist)
        self.timer = Timer(rospy.Duration(self.forward_pick_up_duration), self.timer_callback, oneshot=True)
        return TriggerResponse(success=True, message='Moving forward to object')
    
    def handle_box(self, _):
        if self.timer:
            return TriggerResponse(success=False, message='Timer already running')
        self.cmd_vel_publisher.publish(self.reverse_twist)
        self.timer = Timer(rospy.Duration(self.box_duration), self.timer_callback, oneshot=True)
        return TriggerResponse(success=True, message='Backing away from box')
    
    def handle_cancel(self, _):
        self.timer.shutdown()
        self.timer_callback(None)
        return TriggerResponse(success=True, message='Timer cancelled')
    
    def handle_is_running(self, _):
        return BoolResponse(bool(self.timer))
    
    def timer_callback(self, _):
        self.cmd_vel_publisher.publish(Twist())
        self.timer = None
    
    def run(self):
        rospy.spin()
    
    
if __name__ == '__main__':
    node = StaticMovementServicesNode()
    node.run()